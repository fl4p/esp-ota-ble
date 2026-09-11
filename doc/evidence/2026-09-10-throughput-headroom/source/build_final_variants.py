import os,subprocess
from pathlib import Path
R=Path(__file__).resolve().parent
env=dict(os.environ,PLATFORMIO_CORE_DIR=str(R/'pio-xip'))
with (R/'build-native-xform.log').open('w') as f:
    subprocess.run(['xcrun','swiftc','-O','native_push_xform.swift','-o','native_push_xform'],
                   cwd=R,stdout=f,stderr=subprocess.STDOUT,check=True)
for label,target in [('XipCore1','esp32s3_throughput_xip_core1'),
                     ('Graceful','esp32s3_throughput_graceful'),
                     ('Core1Grace','esp32s3_throughput_core1_graceful'),
                     ('Coc251','esp32s3_throughput_coc251'),
                     ('Coc1000','esp32s3_throughput_coc1000')]:
    path=R/('build-'+label+'.log');assert not path.exists(),path
    with path.open('w') as f:
        subprocess.run(['/Users/fab/.local/bin/pio','run','-e',target],cwd=R/'node',
            env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['/tmp/otavenv/bin/python',str(R/'save_build.py'),label,target],check=True)
