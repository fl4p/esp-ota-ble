import subprocess
from bench_driver import ROOT,PY,install
from native_helpers import native

for label in ('Coc247','Coc5k','Coc16k'):
    install('coc-'+label+'-install',label)
    native('coc-'+label+'-raw',label,coc=True)
install('coc-variants-final-restore','Coc16k')
subprocess.run([PY,str(ROOT/'run_ce_matrix.py')],check=True)
