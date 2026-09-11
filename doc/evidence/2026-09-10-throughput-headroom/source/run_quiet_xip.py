import subprocess
from bench_driver import ROOT,PY,IDENTITIES,install,call

for label in ['Quiet','Xip']:
    install('deep-'+label+'-install',label)
    for burst in [1,2]:
        name='deep-'+label+'-burst'+str(burst)
        row=call(name,'/tmp/node_X.bin','--burst',str(burst),'--setup','link',
             '--setup','buffers','--setup','dleinfo','--expect-base',IDENTITIES[label]['image_sha'])
        install(name+'-restore',label)
print('QUIET/XIP MATRIX COMPLETE; Xip installed',flush=True)
