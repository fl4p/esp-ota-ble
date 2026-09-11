import subprocess
from bench_driver import ROOT,PY,IDENTITIES,install,call

install('deep-Quiet-burst1-restore','Quiet')
call('deep-Quiet-burst2','/tmp/node_X.bin','--burst','2','--setup','link',
     '--setup','buffers','--setup','dleinfo','--expect-base',IDENTITIES['Quiet']['image_sha'])
install('deep-Quiet-burst2-restore','Quiet')
install('deep-Xip-install','Xip')
for name,args in [('deep-Xip-burst1',[]),('deep-Xip-burst2',['--burst','2']),
                  ('deep-Xip-ready',['--ready-event'])]:
    row=call(name,'/tmp/node_X.bin',*args,'--setup','link','--setup','buffers','--setup','dleinfo',
        '--expect-base',IDENTITIES['Xip']['image_sha'])
    if '--ready-event' in args:assert row['ready_callbacks'] and row['ready_callbacks']>0,row
    install(name+'-restore','Xip')
for tag,args in [('default',[]),('ready',['--ready-event'])]:
    with (ROOT/('xip-ram-'+tag+'.log')).open('w') as f:
        subprocess.run([PY,str(ROOT/'run_ram.py'),'Xip','--bytes','1048576',*args],
            stdout=f,stderr=subprocess.STDOUT,check=True,timeout=180)
    print('xip-ram-'+tag,(ROOT/('xip-ram-'+tag+'.log')).read_text().splitlines()[-1],flush=True)
print('XIP/READINESS COMPLETE; Xip installed',flush=True)
