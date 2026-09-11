from pathlib import Path
import hashlib,json,subprocess
R=Path(__file__).resolve().parent
names=['run_ram.py','node/tools/bench_ble_ota.py','esp-ota-ble/host/esp_ota_ble.py']
for host in ['rpi.local','farmgw']:
    for name in names:
        subprocess.run(['scp','-i',str(Path.home()/'.ssh/id_ed25519'),str(R/name),
                        'fab@'+host+':ota-throughput-20260911/'+name],check=True)
    result=subprocess.run(['ssh','-o','BatchMode=yes','-i',str(Path.home()/'.ssh/id_ed25519'),
         'fab@'+host,'cd ~/ota-throughput-20260911 && sha256sum '+ ' '.join(names)],
         check=True,capture_output=True,text=True)
    print(host,result.stdout,flush=True)
(R/'remote-sources.json').write_text(json.dumps({name:hashlib.sha256((R/name).read_bytes()).hexdigest()
                                              for name in names},indent=2)+'\n')
