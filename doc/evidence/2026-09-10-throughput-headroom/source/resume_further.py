import subprocess
from ram_helpers import ram
from bench_driver import ROOT,PY

for interval in (48,100,104):
    ram('resume-further-ram-i'+str(interval),'XipQuiet','--bytes','524288','--interval',str(interval))
subprocess.run([PY,str(ROOT/'after_sweep.py')],check=True)
subprocess.run([PY,str(ROOT/'build_ce.py')],check=True)
