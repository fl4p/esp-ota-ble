from bench_driver import install,IDENTITIES
from remote_helpers import remote

install('native-first-restore','XipQuiet')
for interval in (8,6,12,24,100):
    remote('rpi-fd-ram-i'+str(interval),'rpi.local',[
        'run_ram.py','XipQuiet','--bytes','524288','--adapter','hci0',
        '--interval',str(interval),'--write-fd'])
for mode in ('dbus','fd'):
    remote('farmgw-ram-'+mode,'farmgw',[
        'run_ram.py','XipQuiet','--bytes','524288','--adapter','hci0','--chunk','400',
        '--interval','8',*(['--write-fd'] if mode=='fd' else [])])
