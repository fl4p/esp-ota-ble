import json
import hashlib
import os
from pathlib import Path
import shlex
import subprocess

R = Path(__file__).resolve().parent
def trial(name, args, **env):
    path = R / (name + '.log')
    if path.exists():
        raise RuntimeError('refusing to replace ' + str(path))
    command = ('cd /home/fab/ota-throughput-20260911 && sha256sum bumble_entry.py node/tools/bench_ble_ota.py esp-ota-ble/host/esp_ota_ble.py && sudo -n env ' +
               shlex.join([k+'='+str(v) for k,v in env.items()]) +
               ' timeout --signal=INT 150s venv/bin/python bumble_entry.py ' + shlex.join(args))
    with path.open('w') as out:
        rc = subprocess.run(['ssh','-o','BatchMode=yes','-i',str(Path.home()/'.ssh/id_ed25519'),
                             'fab@rpi.local',command],stdout=out,stderr=subprocess.STDOUT).returncode
    rows = [json.loads(s[7:]) for s in path.read_text().splitlines() if s.startswith('RESULT ')]
    row = rows[-1] if rows else None
    print(name, rc, {k:row.get(k) for k in ('receiver_kB_s','kB_s','verified','real_write')} if row else path.read_text()[-1400:], flush=True)
    with (R/'rpi-bumble-runs.jsonl').open('a') as out:
        out.write(json.dumps(dict(name=name,env=env,args=args,exit_code=rc,result=row,
                                 local_wrapper_sha256=hashlib.sha256((R/'bumble_entry.py').read_bytes()).hexdigest()))+'\n')
    if rc or not row or not row['verified']:
        raise RuntimeError('unverified trial: '+name)
    return row

if __name__ == '__main__':
    for interval,ce,chunk in ((7.5,7.5,495),(30,30,495),(125,125,495),(7.5,0,209),(7.5,0,244),(7.5,0,512)):
        trial('rpi-bumble-i%s-ce%s-c%s'%(interval,ce,chunk),
              ['run_ram.py','Graceful','--bytes','131072','--adapter','hci0','--interval','6','--chunk',str(chunk)],
              BENCH_INTERVAL_MS=interval,BENCH_CE_MS=ce)
