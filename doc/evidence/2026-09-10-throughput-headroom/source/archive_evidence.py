"""Copy an explicit, credential-free whitelist of this experiment's evidence."""
from pathlib import Path
import csv,gzip,hashlib,io,json,shutil,subprocess
R=Path(__file__).resolve().parent
D=Path('/Users/fab/dev/pv/esp-ota-ble/doc/evidence/2026-09-10-throughput-headroom')
D.mkdir(parents=True,exist_ok=True)
def copy(src,dst):
    dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
for p in sorted(R.glob('*.log')):
    if p.name.startswith('build-') or p.stat().st_size>1024*1024:
        q=D/'logs'/(p.name+'.gz');q.parent.mkdir(parents=True,exist_ok=True)
        q.write_bytes(gzip.compress(p.read_bytes(),mtime=0))
        (D/'logs'/p.name).unlink(missing_ok=True)
    else:copy(p,D/'logs'/p.name)
for n in ['link-matrix.jsonl','flash-matrix.jsonl','ram-matrix.jsonl','native-matrix.jsonl','build-identities.json',
          'air-capture-summary.json','fast-air-observed-summary.json','xip-late-recovery.json',
          'queue-air-summary.json','remote-sources.json','mac-controller-metadata.json',
          'native-b4-abort-recovery.json','native-wrong-base-after.json',
          'graceful-sdk-provenance.json','final-candidate-selection.json','final-repeat-summary.json',
          'priority-summary.json','priority-queue-summary.json','dependency-versions.json','coc-air-signals.json']:
    if (R/n).exists():copy(R/n,D/n)
for n in ['node-starting.patch','esp-ota-ble-starting.patch']:
    copy(R/n,D/'source'/n)
for p in sorted(R.glob('*.py')):copy(p,D/'source'/p.name)
for p in sorted(R.glob('*-v*.cpp')):copy(p,D/'source'/'versions'/p.name)
for p in sorted(R.glob('*-v*.h')):copy(p,D/'source'/'versions'/p.name)
for p in sorted(R.glob('*-v*.swift')):copy(p,D/'source'/'versions'/p.name)
for n in ['credit_cost.cpp','native_push.swift','native_push_tuned.swift','native_push_xform.swift','native_push_priority.swift','native_push_priority_queue.swift']:
    if (R/n).exists():copy(R/n,D/'source'/n)
for n in ['bleota-profile.cpp','main-core1.cpp']:
    copy(R/n,D/'source'/'versions'/n)
for n in ['node/platformio.ini','node/src/bleota.cpp','node/src/main.cpp',
          'node/include/bench_msys.h','node/tools/bench_ble_ota.py','node/tools/bench_ble_sniff.py',
          'esp-ota-ble/src/ota_ble.cpp','esp-ota-ble/src/ota_ble.h',
          'esp-ota-ble/src/ota_xform.cpp','esp-ota-ble/src/ota_xform.h',
          'esp-ota-ble/host/esp_ota_ble.py','esp-ota-ble/test/test_tamp_compat.py',
          'esp-ota-ble/test/host-stub/bench-strategy-test.cpp',
          'esp-ota-ble/test/host-stub/tamp-compat-test.cpp',
          'esp-ota-ble/test/host-stub/credit-tail-test.cpp',
          'esp-ota-ble/test/host-stub/esp_heap_caps.h']:
    copy(R/n,D/'source'/n)
air_provenance={}
for name in ['link-matrix-air.csv','fast-link-air.csv','queue-air.csv','coc-air.csv']:
    air=R/name
    if not air.exists():continue
    raw=air.read_bytes();reader=csv.DictReader(io.StringIO(raw.decode()))
    fields=[k for k in reader.fieldnames if k!='payload_hex']+['ram_start','ram_end']
    out=io.StringIO();writer=csv.DictWriter(out,fieldnames=fields);writer.writeheader();count=0
    head=hashlib.shake_256(b'RAM throughput unique byte stream 2026-09-10').digest(16)
    for row in reader:
        payload=bytes.fromhex(row.pop('payload_hex'))
        valid=row['crc_ok']=='1' and row['llid']=='2'
        row['ram_start']=int(valid and len(payload)>=23 and payload[4]==0x52 and payload[7:23]==head)
        row['ram_end']=int(valid and len(payload)>7 and payload[7:].startswith(b'ramend'))
        writer.writerow(row);count+=1
    (D/(name+'.gz')).write_bytes(gzip.compress(out.getvalue().encode(),mtime=0))
    air_provenance[name]=dict(raw_sha256=hashlib.sha256(raw).hexdigest(),rows=count,
        archived='metadata and derived RAM markers; payload bytes remain in private bench directory')
(D/'air-metadata-provenance.json').write_text(json.dumps(air_provenance,indent=2)+'\n')
for p in (R/'xip-sdk-evidence').glob('*'):
    if p.is_file():copy(p,D/'source'/'xip-sdk-evidence'/p.name)
for p in (R/'build-sources').rglob('*'):
    if p.is_file():copy(p,D/'source'/'build-sources'/p.relative_to(R/'build-sources'))
rows=[]
for p in sorted(R.glob('*.log')):
    for line in p.read_text(errors='replace').splitlines():
        if line.startswith('RESULT '):
            row=json.loads(line[7:]);row['log']='logs/'+p.name+('.gz' if p.name.startswith('build-') or p.stat().st_size>1024*1024 else '');rows.append(row)
(D/'results.json').write_text(json.dumps(rows,indent=2,sort_keys=True)+'\n')
manifest={str(p.relative_to(D)):hashlib.sha256(p.read_bytes()).hexdigest()
          for p in sorted(D.rglob('*')) if p.is_file() and p.name!='SHA256SUMS.json'}
(D/'SHA256SUMS.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
print('Archived',len(manifest),'files;',len(rows),'results; bytes',sum(p.stat().st_size for p in D.rglob('*') if p.is_file()))
