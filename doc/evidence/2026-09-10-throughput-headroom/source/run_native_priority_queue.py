import argparse,asyncio,hashlib,json,re,subprocess,sys,time
from pathlib import Path
from types import SimpleNamespace
R=Path(__file__).resolve().parent
sys.path.insert(0,str(R/'node/tools'))
import bench_ble_ota as B

async def verify():
    args=SimpleNamespace(name='farmnode-202A29',auth='BEEF',chunk=495,burst=1,
                         setup=[],ready_event=False,poll_s=None)
    await asyncio.sleep(4)
    for attempt in range(4):
        try:
            link=await B.connect(args,setup=False)
            try:return await B.O.query_info(link)
            finally:await link.release()
        except Exception as exc:
            print('VERIFY RETRY',repr(exc),flush=True);await asyncio.sleep(2)
    return None

ap=argparse.ArgumentParser();ap.add_argument('label');ap.add_argument('name')
ap.add_argument('--image',default='/tmp/node_X.bin');ap.add_argument('--coc',action='store_true')
ap.add_argument('--burst',type=int,choices=(1,2,4,8),default=1)
ap.add_argument('--chunk',type=int,default=495)
ap.add_argument('--interval',type=int,default=8)
ap.add_argument('--nosleep',action='store_true');args=ap.parse_args()
ids=json.loads((R/'build-identities.json').read_text())
image=Path(args.image).read_bytes();assert B.O.image_id(image)
path=R/(args.name+'-native.log');assert not path.exists(),path
with path.open('w') as f:
    proc=subprocess.run([str(R/'native_push_priority_queue'),args.image,ids[args.label]['image_sha'],
                        'coc' if args.coc else 'gatt',str(args.burst),str(args.chunk),str(args.interval),
                        'nosleep' if args.nosleep else 'sleep'],
                        stdout=f,stderr=subprocess.STDOUT,timeout=200)
content=path.read_text();print(content,flush=True)
rows=[json.loads(s[14:]) for s in content.splitlines() if s.startswith('NATIVE_RESULT ')]
assert proc.returncode==0 and len(rows)==1,path
native=rows[0];after=asyncio.run(verify())
lines=native['lines']
def fields(prefix):
    matches=[s for _,s in lines if s.startswith(prefix)]
    return dict(re.findall(r'(\w+)=(\S+)',matches[0])) if len(matches)==1 else {}
flash=fields('OTAB FLASH ')
verified=B.O.image_is_running(after,image) is True and after['run']!=native['before']['run']
real=B.flash_full_rewrite(flash,len(image))
row=dict(native,image=args.image,after=after,verified=verified,real_write=real,
         flash=flash,stats=fields('OTAB STAT '),timing=fields('OTAB TIME '),
         skip=fields('OTAB SKIP '),wire='raw',wire_bytes=len(image),chunk=None if args.coc else args.chunk,
         stream_write_max=4096 if args.coc else None,burst=args.burst,
         kB_s=len(image)/native['elapsed']/1000,image_kB_s=len(image)/native['elapsed']/1000,
         purpose='measure',source_sha256={name:hashlib.sha256((R/name).read_bytes()).hexdigest()
            for name in ('native_push_priority_queue.swift','run_native_priority_queue.py','node/tools/bench_ble_ota.py')})
print('RESULT',json.dumps({k:v for k,v in row.items() if k!='lines'},sort_keys=True),flush=True)
with (R/'native-matrix.jsonl').open('a') as f:f.write(json.dumps(row,sort_keys=True)+'\n')
assert verified and real and native['payload_sha256']==hashlib.sha256(image).hexdigest(),row
