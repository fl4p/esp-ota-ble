from ram_helpers import ram
from bench_driver import ROOT,PY,IDENTITIES,call,install

for burst in (2,4,8,16):
    ram('queue-ram-b'+str(burst),'XipQuiet','--bytes','1048576','--burst',str(burst))
for interval in (6,7):
    ram('queue-ram-i'+str(interval),'XipQuiet','--bytes','524288','--interval',str(interval))
for burst in (4,8,16):
    name='queue-raw-b'+str(burst)
    call(name,'/tmp/node_X.bin','--burst',str(burst),'--setup','link',
         '--expect-base',IDENTITIES['XipQuiet']['image_sha'])
    install(name+'-restore','XipQuiet')
for transform in ('tamp','delta'):
    name='xip-transform-'+transform
    call(name,'/tmp/node_Y.bin','--xform',transform,'--setup','link',
         '--expect-base',IDENTITIES['XipQuiet']['image_sha'])
    install(name+'-restore','XipQuiet')
