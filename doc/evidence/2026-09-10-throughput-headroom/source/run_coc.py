import subprocess
from bench_driver import ROOT,PY,install
install('coc-first-install','Coc')
name='coc-first-native'
path=ROOT/(name+'.log');assert not path.exists(),path
with path.open('w') as f:
    subprocess.run([PY,str(ROOT/'run_native.py'),'Coc',name,'--coc'],
                   stdout=f,stderr=subprocess.STDOUT,check=True)
print(name,path.read_text().splitlines()[-1],flush=True)
install('coc-first-restore','Coc')
