import hashlib,importlib.metadata,json,platform,re,subprocess
from pathlib import Path
R=Path(__file__).resolve().parent
meta=dict(python=platform.python_version(),macos=platform.mac_ver()[0],
          machine=platform.machine(),cpu=subprocess.check_output(['sysctl','-n','machdep.cpu.brand_string'],text=True).strip())
meta['python_packages']={p:importlib.metadata.version(p) for p in ('bleak','detools','tamp','pyobjc-core','pyserial')}
meta['swift']=subprocess.check_output(['xcrun','swiftc','--version'],text=True,stderr=subprocess.STDOUT).strip()
props=R/'node/.pio/libdeps/esp32s3_throughput_coc1000/NimBLE-Arduino/library.properties'
meta['NimBLE-Arduino']=re.search(r'^version=(.+)$',props.read_text(),re.M)[1]
meta['ESP-IDF']=(R/'pio-xip/packages/framework-espidf/version.txt').read_text().strip()
meta['current_native_binaries']={}
for name in ('native_push','native_push_tuned','native_push_xform','native_push_priority','native_push_priority_queue'):
    binary=R/name;source=R/(name+'.swift')
    if source.stat().st_mtime>binary.stat().st_mtime:
        raise RuntimeError('Source newer than binary: '+name)
    meta['current_native_binaries'][name]=dict(binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
packet=Path('/Users/fab/dev/ha/farming/.sniffer/nrf_sniffer/extcap/SnifferAPI/Packet.py')
meta['nrf_sniffer']=dict(package_firmware_filename='sniffer_nrf52840dongle_nrf52840_4.1.1.hex',
    device_firmware_readback=None,packet_parser_sha256=hashlib.sha256(packet.read_bytes()).hexdigest(),
    note='Packet.py calls LL header bit 2 sn and bit 3 nesn. The capture script exports them by their actual meaning: bit 3 SN, bit 2 NESN.')
(R/'dependency-versions.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps(meta,indent=2))
