from pathlib import Path
import json
import subprocess
import time

ROOT=Path(__file__).resolve().parent
PY='/tmp/otavenv/bin/python'
BENCH=str(ROOT/'node/tools/bench_ble_ota.py')
IDENTITIES=json.loads((ROOT/'build-identities.json').read_text())

def call(name,image,*extra):
    for retry in range(4):
        path=ROOT/(name+('' if retry==0 else '-retry'+str(retry))+'.log')
        if path.exists():raise RuntimeError('refusing to replace evidence '+str(path))
        with path.open('w') as log:
            run=subprocess.run([PY,BENCH,str(image),'--chunk','495','--results',str(ROOT/'flash-matrix.jsonl'),*extra],
                               stdout=log,stderr=subprocess.STDOUT,timeout=240)
        content=path.read_text()
        rows=[json.loads(s[7:]) for s in content.splitlines() if s.startswith('RESULT ')]
        if not rows and 'bench device not found' in content and 'BEFORE ' not in content:
            print(name,'discovery retry; transfer did not start',flush=True)
            time.sleep(3);continue
        record=rows[-1] if rows else None
        print(name,'exit',run.returncode,{k:record.get(k) for k in ['wire','elapsed','kB_s','image_kB_s','verified','real_write','skip','flash']} if record else 'NO RESULT',flush=True)
        if not record or not record['verified']:raise RuntimeError('unverified image; stop')
        if run.returncode:raise RuntimeError('transfer succeeded but measurement did not qualify')
        return record
    raise RuntimeError('device unavailable')

def install(name,label):
    return call(name,ROOT/('node_'+label+'.bin'),'--delta')

if __name__=='__main__':
    install('profile-install','Profile')
    call('profile-raw','/tmp/node_W.bin','--setup','link','--expect-base',IDENTITIES['Profile']['image_sha'])
    for label in ['Blocks','Seq','Upfront']:
        install(label+'-install',label)
        call(label+'-raw','/tmp/node_X.bin','--setup','link','--expect-base',IDENTITIES[label]['image_sha'])
    install('blocks-transform-install','Blocks')
    for wire in ['tamp','delta']:
        call('blocks-'+wire,'/tmp/node_Y.bin','--xform',wire,'--setup','link','--expect-base',IDENTITIES['Blocks']['image_sha'])
        install('blocks-'+wire+'-restore','Blocks')
    print('FLASH MATRIX COMPLETE; Blocks receiver left installed',flush=True)
