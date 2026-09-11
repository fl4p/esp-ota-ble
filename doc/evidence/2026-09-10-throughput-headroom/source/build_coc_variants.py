import os,subprocess
from pathlib import Path
R=Path(__file__).resolve().parent
env=dict(os.environ,PLATFORMIO_CORE_DIR=str(R/'pio-xip'))
for label,target in [('Coc247','esp32s3_throughput_coc247'),('Coc5k','esp32s3_throughput_coc5k'),
                     ('Coc16k','esp32s3_throughput_coc16k')]:
    path=R/('build-'+label+'.log');assert not path.exists(),path
    with path.open('w') as f:
        subprocess.run(['/Users/fab/.local/bin/pio','run','-e',target],cwd=R/'node',
            env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['/tmp/otavenv/bin/python',str(R/'save_build.py'),label,target],check=True)
