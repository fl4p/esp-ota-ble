import json,subprocess
from bench_driver import ROOT,PY,IDENTITIES,install
from native_helpers import native
from run_native_tuning import tuned

results=[]
for label in ('XipCore1','Graceful','Core1Grace'):
    install('final-'+label+'-install',label)
    for chunk in ((495,) if label=='XipCore1' else (495,244)):
        name='final-'+label+'-chunk'+str(chunk)
        row=tuned(name,label,'--chunk',str(chunk))
        results.append((row['kB_s'],label,chunk))
        install(name+'-restore',label)
for label in ('Coc251','Coc1000'):
    install('final-'+label+'-install',label)
    row=native('final-'+label+'-raw',label,coc=True)
    results.append((row['kB_s'],label,None))

best=max(results)
(ROOT/'final-candidate-selection.json').write_text(json.dumps(dict(candidates=results,selected=best),indent=2)+'\n')
_,label,chunk=best
print('SELECTED FOR INDEPENDENT REPEATS',best,flush=True)
install('repeat-best-install',label)
repeats=[]
for repeat in range(1,4):
    name='repeat-best-'+str(repeat)
    row=(native(name,label,coc=True) if chunk is None else tuned(name,label,'--chunk',str(chunk)))
    repeats.append(row['kB_s'])
    install(name+'-restore',label)
print('RAW REPEATS',repeats,'mean',sum(repeats)/len(repeats),flush=True)
(ROOT/'final-repeat-summary.json').write_text(json.dumps(dict(label=label,chunk=chunk,rates=repeats,mean=sum(repeats)/len(repeats)),indent=2)+'\n')
for transform in ('tamp','delta'):
    name='native-best-'+transform
    path=ROOT/(name+'.log');assert not path.exists(),path
    with path.open('w') as f:
        proc=subprocess.run([PY,str(ROOT/'run_native_xform.py'),label,name,transform],
                            stdout=f,stderr=subprocess.STDOUT)
    rows=[json.loads(s[7:]) for s in path.read_text().splitlines() if s.startswith('RESULT ')]
    if proc.returncode or not rows or not rows[-1]['verified'] or not rows[-1]['real_write']:
        print(path.read_text()[-2000:],flush=True);raise RuntimeError(path)
    row=rows[-1]
    print(name,{k:row[k] for k in ('kB_s','image_kB_s','elapsed','verified','real_write')},flush=True)
    install(name+'-restore',label)
