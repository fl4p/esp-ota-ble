"""Recheck real positive/negative observations and time existing verifiers."""
import ast
import hashlib
import json
import platform
import time
from pathlib import Path

R=Path(__file__).resolve().parent
def load_function(path,name):
    tree=ast.parse(path.read_text())
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace={'hashlib':hashlib}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
    return namespace[name]

ram=load_function(R/'run_ram.py','verified_ram')
flash=load_function(R/'node/tools/bench_ble_ota.py','flash_full_rewrite')
rows=[json.loads(line) for line in (R/'ram-matrix.jsonl').read_text().splitlines()]
long=next(row for row in reversed(rows) if row['bytes']==6291456 and row['verified'])
bad=next(row for row in reversed(rows) if row['bytes']==8192 and not row['verified'])
data=hashlib.shake_256(b'RAM throughput unique byte stream 2026-09-10').digest(long['bytes'])
assert ram(long['fields'],data)
bad_data=hashlib.shake_256(b'RAM throughput unique byte stream 2026-09-10').digest(8192)
assert int(bad['fields']['bytes'])==len(bad_data) and not ram(bad['fields'],bad_data)
for key in ('bytes','us','sha'):
    fields=dict(long['fields']);del fields[key]
    assert not ram(fields,data)
for n in (-10**12,0,len(data)-1,len(data)+1,10**12):
    assert not ram(dict(long['fields'],bytes=n),data)
for n in (-10**12,0):
    assert not ram(dict(long['fields'],us=n),data)
assert not ram(None,data)
raw=[json.loads(line) for line in (R/'rpi-bumble-flash.jsonl').read_text().splitlines()]
full=[row for row in raw if row.get('wire')=='raw' and row.get('real_write') is True]
assert len(full)>=3
for row in full:
    assert flash(row['flash'],row['bytes']) is True
    assert flash({},row['bytes']) is None
    for n in (-10**12,0,row['bytes']-1,row['bytes']+1,10**12):
        assert flash(dict(row['flash'],program_bytes=n),row['bytes']) is False
    for n in (-10**12,0,int(row['flash']['erase_bytes'])-1,int(row['flash']['erase_bytes'])+1,10**12):
        assert flash(dict(row['flash'],erase_bytes=n),row['bytes']) is False

start=time.perf_counter()
for _ in range(100):
    assert ram(long['fields'],data)
ram_ms=(time.perf_counter()-start)*10
start=time.perf_counter()
for _ in range(10000):
    assert flash(full[-1]['flash'],full[-1]['bytes']) is True
flash_us=(time.perf_counter()-start)*100
print(json.dumps(dict(host=platform.node(),verified=True,
    known_bad='502-byte HCI, correct length but wrong SHA rejected',
    ram_6MiB_ms_per_check=ram_ms,flash_us_per_check=flash_us,
    source_sha256={name:hashlib.sha256((R/name).read_bytes()).hexdigest()
                   for name in ('run_ram.py','node/tools/bench_ble_ota.py','check_rpi_verifiers.py')}),indent=2))
