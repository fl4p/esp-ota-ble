import asyncio,json,subprocess
from types import SimpleNamespace
from bench_driver import ROOT,PY,IDENTITIES,call,install
from run_native_tuning import tuned
import sys
sys.path.insert(0,str(ROOT/'node/tools'))
import bench_ble_ota as B

async def query(name):
    args=SimpleNamespace(name='farmnode-202A29',auth='BEEF',chunk=495,burst=1,
                         setup=['link','buffers'],ready_event=False,poll_s=None)
    link=await B.connect(args)
    try: row=await B.O.query_info(link)
    finally:await link.release()
    assert row['base']==IDENTITIES['Ce2']['image_sha'],row
    (ROOT/(name+'.json')).write_text(json.dumps(row,indent=2)+'\n')
    return row

before=asyncio.run(query('native-b4-abort-recovery'))
path=ROOT/'native-wrong-base-control.log';assert not path.exists()
with path.open('w') as f:
    proc=subprocess.run([str(ROOT/'native_push_tuned'),'/tmp/node_X.bin','0'*64,
                         'gatt','1','495','8','sleep'],stdout=f,stderr=subprocess.STDOUT,timeout=200)
content=path.read_text()
assert proc.returncode!=0 and 'receiver identity/slot unverified' in content
assert 'OTAB READY ' not in content and 'NATIVE_RESULT' not in content
after=asyncio.run(query('native-wrong-base-after'))
assert before==after
print('Native wrong-base control rejected before BEGIN; receiver unchanged',flush=True)

for chunk in (244,512):
    name='native-tune-chunk'+str(chunk)
    tuned(name,'Ce2','--chunk',str(chunk))
    install(name+'-restore','Ce2')
for transform in ('tamp','delta'):
    name='optimized-transform-'+transform
    call(name,'/tmp/node_Y.bin','--xform',transform,'--setup','link',
         '--expect-base',IDENTITIES['Ce2']['image_sha'])
    install(name+'-restore','Ce2')

# Measurements stop while compiling the next three variants.
subprocess.run([PY,str(ROOT/'build_final_variants.py')],check=True)
