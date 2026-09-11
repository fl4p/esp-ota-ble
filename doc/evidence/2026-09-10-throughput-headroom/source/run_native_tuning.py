import json,subprocess
from bench_driver import ROOT,PY,IDENTITIES,call,install

def tuned(name,label,*opts):
    path=ROOT/(name+'.log');assert not path.exists(),path
    with path.open('w') as f:
        proc=subprocess.run([PY,str(ROOT/'run_native_tuned.py'),label,name,*opts],
                            stdout=f,stderr=subprocess.STDOUT)
    rows=[json.loads(s[7:]) for s in path.read_text().splitlines() if s.startswith('RESULT ')]
    if proc.returncode or not rows or not rows[-1]['verified'] or not rows[-1]['real_write']:
        print(path.read_text()[-2000:],flush=True);raise RuntimeError(path)
    row=rows[-1]
    print(name,{k:row.get(k) for k in ('kB_s','elapsed','verified','real_write','burst','chunk','interval_request','nosleep_requested')},flush=True)
    return row

if __name__=='__main__':
    for suffix,opts in [('nosleep',['--nosleep']),('i6',['--interval','6']),
                        ('i7',['--interval','7']),('b2',['--burst','2']),
                        ('b4',['--burst','4']),('b8',['--burst','8']),
                        ('chunk244',['--chunk','244']),('chunk512',['--chunk','512'])]:
        name='native-tune-'+suffix
        tuned(name,'Ce2',*opts)
        install(name+'-restore','Ce2')
    for transform in ('tamp','delta'):
        name='optimized-transform-'+transform
        call(name,'/tmp/node_Y.bin','--xform',transform,'--setup','link',
             '--expect-base',IDENTITIES['Ce2']['image_sha'])
        install(name+'-restore','Ce2')
