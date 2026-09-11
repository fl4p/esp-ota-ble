import asyncio,json,subprocess,sys
from types import SimpleNamespace
from bench_driver import ROOT,PY,IDENTITIES,install
sys.path.insert(0,str(ROOT/'node/tools'))
import bench_ble_ota as B

def trial(name,chunk):
    path=ROOT/(name+'.log');assert not path.exists(),path
    with path.open('w') as f:
        proc=subprocess.run([PY,str(ROOT/'run_native_priority_queue.py'),'Graceful',name,'--chunk',str(chunk)],
                            stdout=f,stderr=subprocess.STDOUT)
    rows=[json.loads(s[7:]) for s in path.read_text().splitlines() if s.startswith('RESULT ')]
    if proc.returncode or not rows or not rows[-1]['verified'] or not rows[-1]['real_write']:
        print(path.read_text()[-2000:],flush=True);raise RuntimeError(path)
    row=rows[-1]
    assert row['delegate_qos_is_userInteractive'],row
    print(name,{k:row.get(k) for k in ('kB_s','elapsed','verified','real_write','delegate_qos_is_userInteractive')},flush=True)
    install(name+'-restore','Graceful')
    return row['kB_s']

rates=[]
for chunk in (244,495):rates.append((trial('priority-queue-chunk'+str(chunk),chunk),chunk))
best,chunk=max(rates)
repeats=[]
if best>88.282:
    for i in range(1,4):repeats.append(trial('priority-queue-repeat-'+str(i),chunk))

async def query():
    cfg=SimpleNamespace(name='farmnode-202A29',auth='BEEF',chunk=495,burst=1,
        setup=['link','buffers','dleinfo','flashinfo','rssi'],ready_event=False,poll_s=None)
    link=await B.connect(cfg)
    try:
        info=await B.O.query_info(link)
        assert info['base']==IDENTITIES['Graceful']['image_sha'],info
        return dict(info=info,setup=link.setup_lines)
    finally:await link.release()

final=asyncio.run(query())
summary=dict(pilots=rates,repeat_chunk=chunk,repeat_rates=repeats,
             repeat_mean=sum(repeats)/len(repeats) if repeats else None,final=final)
(ROOT/'priority-queue-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('PRIORITY SUMMARY',json.dumps(summary),flush=True)
