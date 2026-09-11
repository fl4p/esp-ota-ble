import subprocess
from bench_driver import ROOT,PY,IDENTITIES,install,call
from ram_helpers import ram

for label in ('Ce0','Ce1','Ce2'):
    install('controller-'+label+'-install',label)
    for interval in (8,24,100):
        ram('controller-'+label+'-ram-i'+str(interval),label,'--bytes','262144',
            '--interval',str(interval))
    # Native sender always requests eight units; it checks every write's readiness.
    name='controller-'+label+'-native'
    path=ROOT/(name+'.log');assert not path.exists(),path
    with path.open('w') as f:
        subprocess.run([PY,str(ROOT/'run_native.py'),label,name],
                       stdout=f,stderr=subprocess.STDOUT,check=True)
    print(name,path.read_text().splitlines()[-1],flush=True)

# Leave the final receiver installed for additional sleep/payload trials.
install('controller-final-restore','Ce2')
