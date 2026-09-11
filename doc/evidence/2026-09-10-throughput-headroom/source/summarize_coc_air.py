import csv,hashlib,json,struct
from pathlib import Path
R=Path(__file__).resolve().parent
path=R/'coc-air.csv';rows=[]
with path.open() as f:
    for index,row in enumerate(csv.DictReader(f)):
        if row['crc_ok']!='1' or row['llid']!='2':continue
        p=bytes.fromhex(row['payload_hex'])
        if len(p)<8:continue
        size,cid=struct.unpack_from('<HH',p)
        if cid!=5:continue
        # Only complete LE CoC connection signalling is summarized here.
        # Firmware and application data stay out of this derived archive.
        code,ident,n=struct.unpack_from('<BBH',p,4)
        if code not in (0x14,0x15):continue
        if n!=10 or size!=14 or len(p)<18:
            raise RuntimeError('Incomplete CoC connection signal at packet '+str(index))
        names=('psm','scid','mtu','mps','credits') if code==0x14 else ('dcid','mtu','mps','credits','result')
        fields=dict(zip(names,struct.unpack_from('<HHHHH',p,8)))
        rows.append(dict(packet=index,timestamp_us=row['timestamp_us'],event=row['event'],
                         direction=row['direction'],kind='request' if code==0x14 else 'response',
                         identifier=ident,fields=fields))
out=dict(raw_capture_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),signals=rows,
         note='Observed complete connection signals only; capture is not a complete packet-loss census.')
(R/'coc-air-signals.json').write_text(json.dumps(out,indent=2)+'\n')
print('Summarized',len(rows),'CoC connection signals')
