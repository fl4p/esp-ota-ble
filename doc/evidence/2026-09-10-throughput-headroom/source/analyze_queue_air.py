import collections,csv,hashlib,json,statistics
from pathlib import Path
R=Path(__file__).resolve().parent
rows=list(csv.DictReader((R/'queue-air.csv').open()))
head=hashlib.shake_256(b'RAM throughput unique byte stream 2026-09-10').digest(16)
starts=[]
for i,r in enumerate(rows):
    b=bytes.fromhex(r.get('payload_hex',''))
    marker=(r.get('ram_start')=='1' if 'ram_start' in r else
            r['crc_ok']=='1' and r['llid']=='2' and len(b)>=23 and b[4]==0x52 and b[7:23]==head)
    if marker:
        if not starts or i-starts[-1]>10:starts.append(i)
results=[]
for start in starts:
    end=None
    for i in range(start+1,len(rows)):
        r=rows[i];b=bytes.fromhex(r.get('payload_hex',''))
        marker=(r.get('ram_end')=='1' if 'ram_end' in r else
                r['crc_ok']=='1' and r['llid']=='2' and len(b)>7 and b[7:].startswith(b'ramend'))
        if marker:
            end=i;break
    if end is None:continue
    events=collections.defaultdict(list)
    for r in rows[start:end]:
        if r['crc_ok']=='1':events[int(r['event'])].append(r)
    active=[];spacings=[];previous=None
    for event,items in events.items():
        central=[r for r in items if r['direction']=='1' and int(r['length'])>200]
        if not central:continue
        active.append(central)
        first=int(items[0]['timestamp_us'])
        if previous and event==previous[0]+1:spacings.append((first-previous[1])/1000)
        previous=(event,first)
    results.append(dict(start_packet=start,end_packet=end,
        median_interval_ms=statistics.median(spacings) if spacings else None,
        active_events=len(active),observed_large_pdu_per_event=dict(collections.Counter(len(x) for x in active)),
        final_large_central_md=dict(collections.Counter(x[-1]['md'] for x in active)),
        note='Captured transmissions include retransmissions; incomplete capture is not packet-loss evidence.'))
(R/'queue-air-summary.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
