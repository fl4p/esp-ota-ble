from pathlib import Path
import hashlib,json,shutil,sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'esp-ota-ble/host'))
import esp_ota_ble as O
label,env=sys.argv[1:]
ids=json.loads((ROOT/'build-identities.json').read_text())
src=ROOT/'node/.pio/build'/env
for suffix in ['bin','elf']:
    shutil.copy2(src/('firmware.'+suffix),ROOT/('node_'+label+'.'+suffix))
dest=ROOT/('node_'+label+'.bin');data=dest.read_bytes();shutil.copy2(dest,'/tmp/'+dest.name)
ids[label]=dict(bytes=len(data),image_sha=O.image_id(data),file_sha=hashlib.sha256(data).hexdigest())
safe=['node/platformio.ini','node/src/bleota.cpp','node/src/main.cpp',
      'node/include/bench_msys.h','esp-ota-ble/src/ota_ble.cpp',
      'esp-ota-ble/src/ota_xform.cpp','esp-ota-ble/src/ota_ble.h']
snapshot=ROOT/'build-sources'/label
fingerprints={}
for name in safe:
    source=ROOT/name;target=snapshot/name
    target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    fingerprints[name]=hashlib.sha256(source.read_bytes()).hexdigest()
sdk=ROOT/'pio-xip/packages/framework-arduinoespressif32-libs/esp32s3/qio_opi/include/sdkconfig.h'
if sdk.exists() and (env.startswith('esp32s3_throughput_ce') or 'xip' in env or 'coc' in env or 'graceful' in env):
    shutil.copy2(sdk,snapshot/'sdkconfig.h')
    fingerprints['sdkconfig.h']=hashlib.sha256(sdk.read_bytes()).hexdigest()
ids[label]['environment']=env
ids[label]['source_sha256']=fingerprints
print(label,ids[label],flush=True)
(ROOT/'build-identities.json').write_text(json.dumps(ids,indent=2)+'\n')
