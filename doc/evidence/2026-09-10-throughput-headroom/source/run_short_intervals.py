import asyncio
import json
import re
import sys
from types import SimpleNamespace
from bench_driver import ROOT,IDENTITIES,install,call
sys.path.insert(0,str(ROOT/'node/tools'))
import bench_ble_ota as B

async def probe(label,interval):
    args=SimpleNamespace(name='farmnode-202A29',auth='BEEF',chunk=495,burst=1,setup=['interval '+str(interval),'link'])
    for attempt in range(4):
        try:link=await B.connect(args);break
        except Exception:
            if attempt==3:raise
            await asyncio.sleep(3)
    try:
        info=await B.O.query_info(link)
        assert info['base']==IDENTITIES[label]['image_sha'],info
        values=[int(m.group(1)) for s in link.setup_lines if (m:=re.search(r'BENCH LINK interval=(\d+)',s))]
        if not values:raise RuntimeError('accepted interval unavailable')
        row=dict(requested=interval,accepted=values[-1],lines=link.setup_lines,info=info)
        (ROOT/('short-probe-'+label+'-'+str(interval)+'.json')).write_text(json.dumps(row,indent=2)+'\n')
        print('SHORT INTERVAL PROBE',row,flush=True)
        return values[-1]
    finally:await link.release()

for label in ['Buffered','Blocks2']:
    install('short-'+label+'-install',label)
    accepted=asyncio.run(probe(label,7))
    interval=7 if accepted==7 else 8
    for burst in [1,2]:
        name='short-'+label+'-interval'+str(interval)+'-burst'+str(burst)
        call(name,'/tmp/node_X.bin','--burst',str(burst),'--setup','interval '+str(interval),'--setup','link',
             '--expect-base',IDENTITIES[label]['image_sha'])
        install(name+'-restore',label)
    # Compare against the known accepted 10 ms setting if 8.75 ms worked.
    if interval==7:
        name='short-'+label+'-interval8-burst2'
        call(name,'/tmp/node_X.bin','--burst','2','--setup','interval 8','--setup','link',
             '--expect-base',IDENTITIES[label]['image_sha'])
        install(name+'-restore',label)
print('SHORT INTERVAL MATRIX COMPLETE; Blocks2 receiver left installed',flush=True)
