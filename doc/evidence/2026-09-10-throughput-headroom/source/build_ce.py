import os,subprocess
from pathlib import Path
R=Path(__file__).resolve().parent
env=dict(os.environ,PLATFORMIO_CORE_DIR=str(R/'pio-xip'))
for n in (0,1,2):
    label='Ce'+str(n);target='esp32s3_throughput_ce'+str(n)
    log=R/('build-'+label+'.log')
    assert not log.exists(),log
    with log.open('w') as f:
        subprocess.run(['/Users/fab/.local/bin/pio','run','-e',target],cwd=R/'node',
                       env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['/tmp/otavenv/bin/python',str(R/'save_build.py'),label,target],check=True)
with (R/'build-native.log').open('w') as f:
    subprocess.run(['xcrun','swiftc','-O',str(R/'native_push.swift'),'-o',str(R/'native_push')],
                   stdout=f,stderr=subprocess.STDOUT,check=True)
