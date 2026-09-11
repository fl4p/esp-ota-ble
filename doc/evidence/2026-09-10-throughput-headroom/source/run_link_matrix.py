#!/usr/bin/env python3
"""Sequential physical-bench runs; restore the same receiver after each push."""
from pathlib import Path
import json
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent
PY='/tmp/otavenv/bin/python'
BENCH='/Users/fab/o2p-push/nodehead/tools/bench_ble_ota.py'
FAST='/tmp/node_Fast.bin'
cases=[('baseline',12,1),('burst2',12,2),('burst4',12,4),('burst8',12,8),
       ('burst16',12,16),('interval30',24,1),('interval45',36,1),
       ('interval60',48,1),('interval90',72,1),('interval130',104,1)]
def call(name,image,*extra):
    path=ROOT/(name+'.log')
    attempt=0
    while path.exists():
        attempt+=1
        path=ROOT/(name+'-attempt'+str(attempt)+'.log')
    with path.open('w') as log:
        run=subprocess.run([PY,BENCH,image,'--chunk','495','--results',str(ROOT/'link-matrix.jsonl'),*extra],
                           stdout=log,stderr=subprocess.STDOUT,timeout=180)
    rows=[json.loads(s[7:]) for s in path.read_text().splitlines() if s.startswith('RESULT ')]
    record=rows[-1] if rows else None
    if not record and 'bench device not found' in path.read_text() and attempt<3:
        print(name,'discovery retry (no transfer started)',flush=True)
        time.sleep(3)
        return call(name,image,*extra)
    print(name,'exit',run.returncode, {k:record[k] for k in ['kB_s','verified','real_write','skip']} if record else 'NO RESULT',flush=True)
    return run.returncode,record
for i,(name,interval,burst) in enumerate(cases):
    if i < (int(sys.argv[1]) if len(sys.argv)>1 else 0):continue
    payload='/tmp/node_Y.bin' if i%2==0 else '/tmp/node_X.bin'
    rc,record=call(name,payload,'--burst',str(burst),'--setup','interval '+str(interval),'--setup','link')
    if not record or not record['verified']:
        raise RuntimeError('unverified trial; stop before assuming the running image')
    restore_rc,restore=call(name+'-restore',FAST,'--delta')
    if restore_rc or not restore or not restore['verified']:
        raise RuntimeError('restore failed')
print('MATRIX COMPLETE',flush=True)
