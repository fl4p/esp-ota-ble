import json,subprocess,time
from bench_driver import ROOT,PY,IDENTITIES,install,call

install('wide-diag-install','WideDiag')
call('wide-diag-raw','/tmp/node_X.bin','--setup','link','--setup','buffers',
     '--setup','dleinfo','--setup','flashinfo','--expect-base',IDENTITIES['WideDiag']['image_sha'])
for label in ['Write1k','Write2k','Write8k','Write16k']:
    install('batch-'+label+'-install',label)
    call('batch-'+label+'-raw','/tmp/node_X.bin','--setup','link',
         '--setup','buffers','--setup','dleinfo','--expect-base',IDENTITIES[label]['image_sha'])
install('wide-matrix-restore','WideDiag')
print('WIDE/BATCH MATRIX COMPLETE; WideDiag running',flush=True)
