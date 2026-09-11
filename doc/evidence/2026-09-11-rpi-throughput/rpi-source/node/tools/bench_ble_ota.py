#!/usr/bin/env python3
"""Bench-only raw BLE OTA comparison. Records receiver identity and real erase cost."""
import argparse
import asyncio
import json
from pathlib import Path
import re
import platform
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'esp-ota-ble/host'))
import esp_ota_ble as O

SVC = 'e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb'
CTRL = 'b0e0d1a4-7f52-4a3e-9c61-2d8f5b3ae741'
DATA = 'b0e0d1a5-7f52-4a3e-9c61-2d8f5b3ae741'


def full_rewrite(skip, image_bytes):
    """True/False/None: final sector accounting, or explicitly unverified."""
    if not skip:
        return None
    try:
        kept, wrote, erases, erase_ms = (int(skip[k]) for k in ('kept','wrote','erases','erase_ms'))
    except (KeyError, TypeError, ValueError):
        return None
    sectors = (image_bytes + 4095) // 4096
    return (image_bytes > 0 and kept == 0 and wrote == sectors and
            erases == sectors and erase_ms >= 5000)


def flash_full_rewrite(flash, image_bytes):
    """Bench instrumentation counts successful writes, not reconstructed bytes."""
    if not flash or flash.get('mode') not in ('blocks','buffered','internal','upfront','sequential'):
        return None
    try:
        programmed=int(flash['program_bytes'])
        erased=int(flash['erase_bytes'])
    except (KeyError,TypeError,ValueError):
        return None
    if image_bytes<=0 or programmed!=image_bytes:return False
    if flash['mode']=='sequential':
        # IDF owns erase accounting; each successful esp_ota_write was counted.
        return erased==0
    return erased==((image_bytes+4095)//4096)*4096


def accounted_sector_update(skip, image_bytes):
    try:
        kept,wrote,erases=(int(skip[k]) for k in ('kept','wrote','erases'))
    except (KeyError,TypeError,ValueError):
        return False
    return image_bytes>0 and kept>=0 and wrote>0 and erases==wrote and kept+wrote==(image_bytes+4095)//4096


async def connect(args, setup=True):
    if getattr(args,'adapter',None):
        from bleak import BleakScanner
        def matches(d,a):
            found=(a.local_name or d.name)==args.name
            if found:print('DISCOVERY adapter=%s address=%s rssi=%s' % (args.adapter,d.address,a.rssi),flush=True)
            return found
        dev=await BleakScanner.find_device_by_filter(
            matches,
            timeout=20,adapter=args.adapter)
    else:
        dev = await O.find_device(name_prefix=args.name, service_uuid=SVC)
    if not dev:
        raise RuntimeError('bench device not found')
    link = O.BleOtaLink(CTRL, CTRL, DATA, chunk=args.chunk)
    await link.open(dev,adapter=getattr(args,'adapter',None))
    link.setup_lines=[]
    def setup_line(s):
        link.setup_lines.append(s)
        print('SETUP',s,flush=True)
    link.set_line_handler(setup_line)
    if getattr(args,'adapter',None):
        setup_line('BENCH HOST adapter_path='+str(getattr(link._cli._backend,'_device_path',None)))
    if getattr(args,'ready_event',False):
        if getattr(args,'poll_s',None) is not None:raise ValueError('choose one readiness mechanism')
        import objc
        from bleak.backends.corebluetooth.PeripheralDelegate import ObjcPeripheralDelegate
        from bleak.backends._utils import external_thread_callback,try_call_soon_threadsafe
        @external_thread_callback
        def ready_callback(native,peripheral):
            delegate=native.py_delegate()
            if delegate is None:return
            callback=getattr(delegate,'_bench_ready_callback',None)
            if callback is not None:try_call_soon_threadsafe(delegate.event_loop,callback)
        objc.classAddMethods(ObjcPeripheralDelegate,[objc.selector(ready_callback,
            selector=b'peripheralIsReadyToSendWriteWithoutResponse:',signature=b'v@:@')])
        delegate=link._cli._backend._delegate
        link._ready_delegate=delegate
        event=asyncio.Event();delegate._bench_ready_count=0
        def wake():
            delegate._bench_ready_count+=1;event.set()
        delegate._bench_ready_callback=wake
        peripheral=link._cb_peripheral()
        if peripheral is None:raise RuntimeError('readiness callback requires CoreBluetooth')
        async def await_event():
            deadline=time.monotonic()+2
            while True:
                event.clear()
                if peripheral.canSendWriteWithoutResponse():return
                remaining=deadline-time.monotonic()
                if remaining<=0:raise RuntimeError('CoreBluetooth readiness callback timed out')
                await asyncio.wait_for(event.wait(),remaining)
        link._await_writable=await_event
    if getattr(args,'poll_s',None) is not None:
        peripheral=link._cb_peripheral()
        if peripheral is None:raise RuntimeError('pacing experiment requires CoreBluetooth')
        async def poll_writable():
            deadline=time.monotonic()+2
            while not peripheral.canSendWriteWithoutResponse():
                if time.monotonic()>=deadline:raise RuntimeError('CoreBluetooth remained unwritable')
                await asyncio.sleep(args.poll_s)
        link._await_writable=poll_writable
    if args.burst > 1:
        counter = 0
        async def burst_write(data):
            nonlocal counter
            if counter % args.burst == 0:
                await link._await_writable()
            await link._cli.write_gatt_char(link.fw_uuid, data, response=False)
            counter += 1
        link.write_fw = burst_write
    if args.auth:
        await link.write_cmd('auth ' + args.auth)
    await asyncio.sleep(1.5)
    for cmd in (args.setup if setup else []):
        await link.write_cmd(cmd)
        await asyncio.sleep(2)
    if getattr(args,'write_fd',False):
        if not sys.platform.startswith('linux'):raise RuntimeError('AcquireWrite requires BlueZ')
        import socket
        from dbus_fast import Message
        from bleak.backends.bluezdbus import defs
        from bleak.backends.bluezdbus.utils import assert_reply
        backend=link._cli._backend
        ch=link._cli.services.get_characteristic(DATA)
        reply=await backend._bus.call(Message(destination=defs.BLUEZ_SERVICE,path=ch.obj[0],
            interface=defs.GATT_CHARACTERISTIC_INTERFACE,member='AcquireWrite',
            signature='a{sv}',body=[{}]))
        assert_reply(reply)
        sock=socket.socket(fileno=reply.unix_fds[0]);sock.setblocking(False)
        mtu=int(reply.body[1]);link.mtu=backend._mtu_size=mtu
        link._mtu_acquired=True
        setup_line('BENCH HOST AcquireWrite mtu=%d socket_type=%d' % (mtu,sock.type))
        original_release=link.release
        async def release_fd():
            sock.close();await original_release()
        link.release=release_fd
        async def write_fd(data):
            if len(data)>min(512,mtu-3):raise RuntimeError('AcquireWrite value exceeds capacity')
            loop=asyncio.get_running_loop()
            while True:
                try:
                    n=sock.send(data)
                    if n!=len(data):raise RuntimeError('AcquireWrite partial record')
                    return
                except BlockingIOError:
                    event=asyncio.Event();fd=sock.fileno()
                    loop.add_writer(fd,event.set)
                    try:await asyncio.wait_for(event.wait(),2)
                    finally:loop.remove_writer(fd)
        link.write_fw=write_fd
    return link


async def run(args):
    data = Path(args.image).read_bytes()
    if not O.image_id(data):
        raise RuntimeError('invalid target image SHA')
    link = await connect(args)
    lines = []
    t0 = time.monotonic()
    def log(line):
        lines.append((time.monotonic()-t0, line))
        if not line.startswith(('OTAB PROG', 'OTAB CRED')):
            print('[%.3f] %s' % lines[-1], flush=True)
    try:
        before = await O.query_info(link)
        if not before or not before.get('base'):
            raise RuntimeError('running image is unverified')
        if args.expect_base and before['base']!=args.expect_base:
            raise RuntimeError('receiver differs from the expected benchmark image')
        maximum = link._max_write()
        if sys.platform.startswith('linux') and maximum==0 and link._mtu_acquired:
            # Explicit bench limit: this receiver's characteristic capacity is
            # 512, and AcquireWrite supplied the actual negotiated ATT MTU.
            maximum=min(512,link.mtu-3)
        if maximum < args.chunk:
            raise RuntimeError('chunk %d exceeds characteristic limit %d' % (args.chunk,maximum))
        print('BEFORE',json.dumps(before,sort_keys=True),flush=True)
        print('LINK mtu=%d max_write=%d chunk=%d' % (link.mtu,maximum,args.chunk),flush=True)
        payload_started=time.monotonic()
        wire, payload = ('raw', data)
        if args.delta or args.xform!='raw':
            wire, payload = O.choose_payload(data, before, prefer='delta' if args.delta else args.xform,
                base_dirs=[str(Path(args.image).resolve().parent),'/tmp'],on_note=print)
        payload_build_s=time.monotonic()-payload_started
        t0 = time.monotonic()
        ok = await O.push_image(link,payload,xform=wire,out_size=len(data) if wire!='raw' else None,on_line=log,flush_timeout=40,
                                credit_timeout=10,credit_retries=2)
        # Keep the original benchmark endpoint even if firmware now drains its
        # final OK before reboot. Do not gain speed by changing the stopwatch.
        if not link.disconnected.is_set():
            await asyncio.wait_for(link.disconnected.wait(), timeout=8)
        elapsed = time.monotonic()-t0
        paced = link.paced_writes
    finally:
        await link.release()
    await asyncio.sleep(4)
    after = None
    for attempt in range(4):
        try:
            peer = await connect(args, setup=False)
            try:
                after = await O.query_info(peer)
            finally:
                await peer.release()
            break
        except Exception as exc:
            print('VERIFY RETRY', repr(exc), flush=True)
            await asyncio.sleep(2)
    stat = [s for _,s in lines if s.startswith('OTAB STAT')]
    fields = dict(re.findall(r'(\w+)=(\d+)', ' '.join(stat)))
    erase_ms = int(fields['erase_ms']) if 'erase_ms' in fields else None
    verified = (O.image_is_running(after,data) is True and after.get('run') != before.get('run'))
    skip_lines = [s for _,s in lines if s.startswith('OTAB SKIP')]
    skip = dict(re.findall(r'(\w+)=(\S+)', ' '.join(skip_lines))) if len(skip_lines)==1 else {}
    real_write = full_rewrite(skip, len(data))
    flash_lines=[s for _,s in lines if s.startswith('OTAB FLASH ')]
    flash=dict(re.findall(r'(\w+)=(\S+)',flash_lines[0])) if len(flash_lines)==1 else {}
    timing_lines=[s for _,s in lines if s.startswith('OTAB TIME ')]
    timing=dict(re.findall(r'(\w+)=(\S+)',timing_lines[0])) if len(timing_lines)==1 else {}
    if flash.get('mode') in ('blocks','buffered','internal','upfront','sequential'):
        real_write=flash_full_rewrite(flash,len(data))
    result = dict(image=args.image, name=args.name, before=before,after=after,
                  host=platform.node(),adapter=args.adapter,write_fd=args.write_fd,
                  bytes=len(data),wire=wire,wire_bytes=len(payload),chunk=args.chunk,burst=args.burst,setup=args.setup,elapsed=elapsed,kB_s=len(payload)/elapsed/1000,
                  paced_writes=paced,poll_s=args.poll_s,ready_event=args.ready_event,
                  ready_callbacks=getattr(getattr(link,'_ready_delegate',None),'_bench_ready_count',None),
                  transfer_hint=ok,verified=verified,real_write=real_write,
                  stats=fields,skip=skip,flash=flash,timing=timing,lines=lines,
                  negotiated_setup=link.setup_lines,
                  payload_build_s=payload_build_s,
                  purpose='install' if args.delta or args.install else 'measure-update' if args.allow_skips else 'measure',
                  image_kB_s=len(data)/elapsed/1000)
    print('RESULT',json.dumps({k:v for k,v in result.items() if k!='lines'},sort_keys=True),flush=True)
    if args.results:
        with open(args.results,'a') as f: f.write(json.dumps(result,sort_keys=True)+'\n')
    qualifies=real_write or (args.allow_skips and accounted_sector_update(skip,len(data)))
    if not verified or (not (args.delta or args.install) and not qualifies):
        raise RuntimeError('run cannot count: image verification or real-write evidence failed')


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('image')
    ap.add_argument('--name',default='farmnode-202A29')
    ap.add_argument('--auth',default='BEEF')
    ap.add_argument('--chunk',type=int,default=244)
    ap.add_argument('--burst',type=int,default=1,help='experimental queue submissions per pacing check')
    ap.add_argument('--poll-s',type=float,default=None,help='experimental CoreBluetooth readiness poll sleep')
    ap.add_argument('--ready-event',action='store_true',help='experimental native CoreBluetooth readiness callback')
    ap.add_argument('--results')
    ap.add_argument('--adapter',help='explicit BlueZ adapter for isolated bench tests')
    ap.add_argument('--write-fd',action='store_true',help='experimental BlueZ AcquireWrite socket')
    ap.add_argument('--setup',action='append',default=[])
    ap.add_argument('--delta',action='store_true',help='install a bench change, excluded from raw throughput comparison')
    ap.add_argument('--xform',choices=['raw','tamp','delta'],default='raw')
    ap.add_argument('--install',action='store_true',help='installation only; never counted as a full-rewrite measurement')
    ap.add_argument('--expect-base',help='required identity of the receiver before this measurement')
    ap.add_argument('--allow-skips',action='store_true',help='measure a normal update with complete sector accounting; never counted as a full rewrite')
    asyncio.run(run(ap.parse_args()))
