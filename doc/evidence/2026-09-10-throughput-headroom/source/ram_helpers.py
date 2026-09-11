import json,subprocess,time
from bench_driver import ROOT,PY

def ram(name,label,*args):
    for retry in range(4):
        path=ROOT/(name+('' if not retry else '-retry'+str(retry))+'.log')
        assert not path.exists(),path
        with path.open('w') as log:
            proc=subprocess.run([PY,str(ROOT/'run_ram.py'),label,*args],
                stdout=log,stderr=subprocess.STDOUT,timeout=240)
        content=path.read_text()
        if proc.returncode and 'bench device not found' in content and 'BENCH RAM ready' not in content:
            print(name,'discovery failed before transfer, retrying',flush=True)
            time.sleep(3);continue
        rows=[json.loads(s[7:]) for s in content.splitlines() if s.startswith('RESULT ')]
        assert proc.returncode==0 and rows and rows[-1]['verified'],path
        row=rows[-1]
        print(name,{k:row.get(k) for k in ['receiver_kB_s','host_kB_s','post_link','verified']},flush=True)
        return row
    raise RuntimeError('discovery failed for '+name)
