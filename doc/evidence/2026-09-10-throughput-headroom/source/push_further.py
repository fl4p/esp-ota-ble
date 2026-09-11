import subprocess
from bench_driver import ROOT,PY,IDENTITIES,install,call

install('further-XipQuiet-install','XipQuiet')
for name,args in [
    ('raw-b1',[]),('raw-b2',['--burst','2']),
    ('timeout20',['--setup','timeout 20']),
    ('timeout10',['--setup','timeout 10']),
]:
    call('further-'+name,'/tmp/node_X.bin',*args,'--setup','link',
         '--setup','buffers','--setup','dleinfo',
         '--expect-base',IDENTITIES['XipQuiet']['image_sha'])
    install('further-'+name+'-restore','XipQuiet')
for interval in [8,10,12,16,24,48,100,104]:
    path=ROOT/('further-ram-i'+str(interval)+'.log')
    assert not path.exists(),path
    with path.open('w') as log:
        subprocess.run([PY,str(ROOT/'run_ram.py'),'XipQuiet','--bytes','524288',
                        '--interval',str(interval)],stdout=log,stderr=subprocess.STDOUT,
                       timeout=180,check=True)
    print(path.name,path.read_text().splitlines()[-1],flush=True)
