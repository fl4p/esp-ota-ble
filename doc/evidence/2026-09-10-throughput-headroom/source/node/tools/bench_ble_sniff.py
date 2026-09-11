#!/usr/bin/env python3
"""Capture nRF sniffer air timestamps and LL fields; no host-time rate inference."""
import argparse
import csv
from pathlib import Path
import sys
import time

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--api',default=str(Path.home()/'dev/ha/farming/.sniffer/nrf_sniffer/extcap'))
ap.add_argument('--port',default='/dev/cu.usbmodem21301')
ap.add_argument('--mac',default='7c:4f:ad:20:2a:29')
ap.add_argument('--seconds',type=float,default=70)
ap.add_argument('--out',required=True)
a=ap.parse_args()
sys.path.insert(0,a.api)
from SnifferAPI import Sniffer

s=Sniffer.Sniffer(portnum=a.port,baudrate=1000000)
try:
    s.start();s.scan()
    deadline=time.monotonic()+25
    dev=None
    while time.monotonic()<deadline:
        for d in s._devices.asList():
            if ':'.join('%02x'%b for b in d.address[:6])==a.mac:
                dev=d;break
        if dev:break
        time.sleep(.1)
    if not dev:raise RuntimeError('target not found before deadline')
    s.follow(dev)
    print('FOLLOWING',a.mac,flush=True)
    deadline=time.monotonic()+a.seconds
    count=0
    with open(a.out,'w') as f:
        w=csv.writer(f)
        w.writerow(['timestamp_us','event','direction','phy','crc_ok','llid','sn','nesn','md','length','payload_hex'])
        while time.monotonic()<deadline:
            for p in s.getPackets():
                bp=getattr(p,'blePacket',None)
                if getattr(p,'channel',37)>=37 or bp is None:continue
                # Packet.py labels SN/NESN oppositely to the LL header layout.
                # Export its raw fields by their true bit meaning: bit 3 is SN.
                w.writerow([p.timestamp,p.eventCounter,int(p.direction),p.phy,int(p.crcOK),
                            bp.llid,bp.nesn,bp.sn,bp.md,bp.length,bytes(bp.payload[:bp.length]).hex()])
                count+=1
            f.flush()
            time.sleep(.005)
    print('CAPTURED',count,a.out,flush=True)
finally:
    s.doExit()
