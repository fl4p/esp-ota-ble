import asyncio
import json
import re
import sys
from types import SimpleNamespace
from bench_driver import ROOT,IDENTITIES,install,call
sys.path.insert(0,str(ROOT/'node/tools'))
import bench_ble_ota as B

async def probe(interval):
    args=SimpleNamespace(name='farmnode-202A29',auth='BEEF',chunk=495,burst=1,setup=['interval '+str(interval),'link'])
    for attempt in range(4):
        try:link=await B.connect(args);break
        except Exception:
            if attempt==3:raise
            await asyncio.sleep(3)
    try:
        info=await B.O.query_info(link)
        if info['base']!=IDENTITIES['Blocks2']['image_sha']:raise RuntimeError('unexpected probe receiver')
        intervals=[int(m.group(1)) for s in link.setup_lines if (m:=re.search(r'BENCH LINK interval=(\d+)',s))]
        if not intervals:raise RuntimeError('no negotiated interval evidence')
        row=dict(requested=interval,accepted=intervals[-1],lines=link.setup_lines,info=info)
        (ROOT/('interval-probe-'+str(interval)+'.json')).write_text(json.dumps(row,indent=2)+'\n')
        print('INTERVAL PROBE',row,flush=True)
        return intervals[-1]==interval
    finally:await link.release()

install('blocks-v2-install','Blocks2')
for interval in [6,8]:
    if asyncio.run(probe(interval)):
        call('blocks-v2-interval'+str(interval),'/tmp/node_X.bin','--setup','interval '+str(interval),
             '--setup','link','--expect-base',IDENTITIES['Blocks2']['image_sha'])
        install('blocks-v2-interval'+str(interval)+'-restore','Blocks2')
for chunk,burst in [(244,1),(400,1),(512,1),(495,2),(495,4),(495,8)]:
    name='blocks-v2-chunk'+str(chunk)+'-burst'+str(burst)
    call(name,'/tmp/node_X.bin','--chunk',str(chunk),'--burst',str(burst),'--setup','link',
         '--expect-base',IDENTITIES['Blocks2']['image_sha'])
    install(name+'-restore','Blocks2')
install('buffered-install','Buffered')
for burst in [1,4]:
    call('buffered-burst'+str(burst),'/tmp/node_X.bin','--burst',str(burst),'--setup','link',
         '--expect-base',IDENTITIES['Buffered']['image_sha'])
    install('buffered-burst'+str(burst)+'-restore','Buffered')
print('V2 MATRIX COMPLETE; Buffered receiver left installed',flush=True)
