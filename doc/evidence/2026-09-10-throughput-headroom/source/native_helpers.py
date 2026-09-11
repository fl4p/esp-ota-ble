import json,subprocess
from bench_driver import ROOT,PY

def native(name,label,coc=False):
    path=ROOT/(name+'.log');assert not path.exists(),path
    with path.open('w') as f:
        run=subprocess.run([PY,str(ROOT/'run_native.py'),label,name,*(['--coc'] if coc else [])],
                           stdout=f,stderr=subprocess.STDOUT)
    rows=[json.loads(s[7:]) for s in path.read_text().splitlines() if s.startswith('RESULT ')]
    if run.returncode or not rows or not rows[-1]['verified'] or not rows[-1]['real_write']:
        print(path.read_text()[-2000:],flush=True);raise RuntimeError(path)
    row=rows[-1];print(name,{k:row[k] for k in ['elapsed','kB_s','verified','real_write','backend']},flush=True)
    return row
