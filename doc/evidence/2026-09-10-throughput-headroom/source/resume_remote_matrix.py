from remote_helpers import remote
from bench_driver import install,IDENTITIES

remote('rpi-internal-ram','rpi.local',['run_ram.py','XipQuiet','--bytes','262144',
    '--adapter','hci1','--chunk','400','--interval','6','--write-fd'])
for mode in ('dbus','fd'):
    remote('farmgw-ram-'+mode,'farmgw',['run_ram.py','XipQuiet','--bytes','262144',
        '--adapter','hci0','--chunk','400','--interval','6',
        *(['--write-fd'] if mode=='fd' else [])])
for host,adapter,chunk,tag in [('rpi.local','hci0',495,'rpi-usb'),
                             ('rpi.local','hci1',400,'rpi-internal'),
                             ('farmgw','hci0',400,'farmgw')]:
    remote(tag+'-raw',host,['node/tools/bench_ble_ota.py','node_X.bin','--adapter',adapter,
        '--chunk',str(chunk),'--write-fd','--setup','interval 6','--setup','link',
        '--setup','dleinfo','--expect-base',IDENTITIES['XipQuiet']['image_sha']])
    install(tag+'-restore','XipQuiet')
