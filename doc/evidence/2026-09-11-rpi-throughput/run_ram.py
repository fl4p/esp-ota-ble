import argparse, asyncio, hashlib, json, re, sys, time, platform
from pathlib import Path
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'node/tools'))
import bench_ble_ota as B

def verified_ram(fields,data):
    try:
        return (int(fields['bytes'])==len(data)>0 and int(fields['us'])>0
                and fields['sha']==hashlib.sha256(data).hexdigest())
    except (KeyError,TypeError,ValueError):return False

async def run(args):
    cfg=SimpleNamespace(name='farmnode-202A29',auth='BEEF',chunk=495,
        burst=args.burst,poll_s=args.poll_s,ready_event=args.ready_event,
        adapter=args.adapter,write_fd=args.write_fd,
        setup=['interval '+str(args.interval),'link','buffers','dleinfo'])
    link=await B.connect(cfg)
    info=await B.O.query_info(link)
    ids=json.loads((ROOT/'build-identities.json').read_text())
    assert info and info['base']==ids[args.label]['image_sha'],info
    q=asyncio.Queue()
    def line(s):
        print(s,flush=True);q.put_nowait(s)
    link.set_line_handler(line)
    async def response(prefix):
        while True:
            s=await asyncio.wait_for(q.get(),timeout=15)
            if s.startswith(prefix):return s
    async def trial(data,want=None):
        await link.write_cmd('ram '+str(len(data) if want is None else want))
        ready=await response('BENCH RAM ')
        assert ready=='BENCH RAM ready',ready
        t=time.monotonic()
        for offset in range(0,len(data),args.chunk):
            await link.write_fw(data[offset:offset+args.chunk])
        await link.write_cmd('ramend')
        answer=await response('BENCH RAM ')
        elapsed=time.monotonic()-t
        fields=dict(re.findall(r'(\w+)=(\S+)',answer))
        return fields,answer,elapsed
    try:
        if args.calibrate:
            for n in (0,6*1024*1024+1):
                await link.write_cmd('ram '+str(n))
                assert 'FAIL' in await response('BENCH RAM ')
            for data,want in ((b'a'*7,8),(b'a'*9,8)):
                fields,answer,elapsed=await trial(data,want)
                assert 'FAIL' in answer and not verified_ram(fields,data),answer
            data=bytes(range(256))*4
            fields,answer,elapsed=await trial(data)
            assert verified_ram(fields,data),answer
            assert not verified_ram(fields,data[:-1]+b'x')
            for field in ('bytes','us','sha'):
                bad=dict(fields);bad.pop(field);assert not verified_ram(bad,data)
            for n in (-100000,0,len(data)-1,len(data)+100000):
                bad=dict(fields,bytes=n);assert not verified_ram(bad,data)
            started=time.perf_counter()
            for _ in range(10000):verified_ram(fields,data)
            print('CALIBRATION OK us_per_check',(time.perf_counter()-started)*100,flush=True)
        data=hashlib.shake_256(b'RAM throughput unique byte stream 2026-09-10').digest(args.bytes)
        fields,answer,elapsed=await trial(data)
        verified=verified_ram(fields,data)
        after=await B.O.query_info(link)
        verified=verified and after==info
        await link.write_cmd('link')
        post_link=await response('BENCH LINK ')
        row=dict(label=args.label,purpose='ram-only',bytes=len(data),chunk=args.chunk,
            host=platform.node(),adapter=args.adapter,write_fd=args.write_fd,
            burst=args.burst,poll_s=args.poll_s,ready_event=args.ready_event,interval=args.interval,
            ready_callbacks=getattr(getattr(link,'_ready_delegate',None),'_bench_ready_count',None),
            host_elapsed=elapsed,fields=fields,verified=verified,
            host_kB_s=len(data)/elapsed/1000,before=info,after=after,post_link=post_link,setup=link.setup_lines)
        if verified:row['receiver_kB_s']=len(data)/int(fields['us'])*1000
        print('RESULT',json.dumps(row,sort_keys=True),flush=True)
        with (ROOT/'ram-matrix.jsonl').open('a') as f:f.write(json.dumps(row,sort_keys=True)+'\n')
        assert verified,answer
    finally:await link.release()

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('label');ap.add_argument('--bytes',type=int,default=6*1024*1024)
    ap.add_argument('--chunk',type=int,default=495);ap.add_argument('--burst',type=int,default=1)
    ap.add_argument('--calibrate',action='store_true')
    ap.add_argument('--poll-s',type=float,default=None)
    ap.add_argument('--ready-event',action='store_true')
    ap.add_argument('--interval',type=int,default=8)
    ap.add_argument('--adapter',default=None)
    ap.add_argument('--write-fd',action='store_true')
    asyncio.run(run(ap.parse_args()))
