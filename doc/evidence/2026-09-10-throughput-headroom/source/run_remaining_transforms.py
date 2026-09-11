import subprocess
from bench_driver import ROOT,PY,IDENTITIES,install,call

for wire in ['tamp','delta']:
    call('blocks2-'+wire,'/tmp/node_Y.bin','--xform',wire,'--setup','interval 8','--setup','link',
         '--expect-base',IDENTITIES['Blocks2']['image_sha'])
    install('blocks2-'+wire+'-restore','Blocks2')
subprocess.run([PY,str(ROOT/'run_small_updates.py')],check=True)
print('TRANSFORM AND SMALL UPDATE TRIALS COMPLETE',flush=True)
