from remote_helpers import remote
from bench_driver import install,IDENTITIES
for host,adapter,tag in [('rpi.local','hci1','rpi-internal'),('farmgw','hci0','farmgw')]:
    remote(tag+'-raw-explicit-adapter',host,['node/tools/bench_ble_ota.py','node_X.bin',
        '--adapter',adapter,'--chunk','400','--write-fd','--setup','interval 6',
        '--setup','link','--setup','dleinfo','--expect-base',IDENTITIES['XipQuiet']['image_sha']])
    install(tag+'-explicit-restore','XipQuiet')
