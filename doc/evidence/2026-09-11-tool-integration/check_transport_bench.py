"""Integration bench: shipping DirectLink, real flash accounting, exact boot check."""
import argparse
import asyncio
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import time

sys.path.insert(0, str(Path(__file__).parent / 'esp-ota-ble/host'))
import esp_ota_ble as O
import esp_ota_ble_transport as T

CTRL = 'b0e0d1a4-7f52-4a3e-9c61-2d8f5b3ae741'
DATA = 'b0e0d1a5-7f52-4a3e-9c61-2d8f5b3ae741'

async def connect(a):
    link = T.DirectLink(CTRL, CTRL, DATA, options=T.from_arguments(a))
    try:
        dev = await link.find_device(name_prefix='farmnode-202A29', timeout=20)
        await link.open(dev)
        authed = asyncio.Event()
        def auth(line):
            print(line, flush=True)
            if line == 'OTAB AUTH ok': authed.set()
        link.set_line_handler(auth)
        await link.write_cmd('auth BEEF')
        await asyncio.wait_for(authed.wait(), 5)
        return link
    except BaseException:
        await link.release()
        raise

async def main(a):
    data = Path(a.image).read_bytes()
    want = O.image_id(data)
    if not want: raise RuntimeError('invalid image')
    link = await connect(a)
    lines = []
    def note(line):
        lines.append(line)
        if not line.startswith(('OTAB CRED', 'OTAB PROG')): print(line, flush=True)
    try:
        before = await O.query_info(link)
        print('IDENTITY', json.dumps(before), flush=True)
        if a.probe:
            await link.prepare_transfer()
            after = await O.query_info(link)
            if not before or before != after: raise RuntimeError('probe identity changed/missing')
            print('PROBE VERIFIED', json.dumps(after), flush=True)
            return
        if not before or not before.get('run') or not before.get('base'):
            raise RuntimeError('unverified receiver')
        if a.expect_base and before['base'] != a.expect_base: raise RuntimeError('unexpected receiver')
        link.set_line_handler(note)
        if a.mac_interval:
            await link.write_cmd('interval 8')
            await asyncio.sleep(1)
        await link.write_cmd('link')
        await asyncio.sleep(.3)
        print('BEFORE', json.dumps(before), flush=True)
        wire, payload = O.choose_payload(data, before, prefer='delta' if a.install else 'raw',
                                         base_dirs=[str(Path(a.image).parent)], on_note=print)
        start = time.monotonic()
        ok = await O.push_image(link, payload, xform=wire,
                                out_size=len(data) if wire != 'raw' else None,
                                on_line=note, flush_timeout=40)
        if not link.disconnected.is_set(): await asyncio.wait_for(link.disconnected.wait(), 8)
        elapsed = time.monotonic() - start
    finally:
        await link.release()
    await asyncio.sleep(4)
    after = None
    for attempt in range(4):
        try:
            peer = await connect(a)
            try: after = await O.query_info(peer)
            finally: await peer.release()
            if after and after.get('base') == want and after.get('run') and after['run'] != before['run']:
                break
        except Exception as exc: print('VERIFY RETRY', repr(exc), flush=True)
        await asyncio.sleep(1)
    verified = bool(after and after.get('base') == want and after.get('run') and after['run'] != before['run'])
    flash_lines = [s for s in lines if s.startswith('OTAB FLASH ')]
    flash = dict(re.findall(r'(\w+)=(\S+)', flash_lines[0])) if len(flash_lines) == 1 else {}
    full = (flash.get('program_bytes') == str(len(data))
            and flash.get('erase_bytes') == str((len(data)+4095)//4096*4096))
    result = dict(options=vars(a), wire=wire, wire_bytes=len(payload), bytes=len(data),
                  before=before, after=after, verified=verified, full_rewrite=full,
                  flash=flash, elapsed=elapsed, kB_s=len(payload)/elapsed/1000,
                  source_sha=hashlib.sha256(Path(T.__file__).read_bytes()).hexdigest(), lines=lines)
    print('RESULT', json.dumps(result, sort_keys=True), flush=True)
    if not ok or not verified or (not a.install and not full): raise RuntimeError('run not verified')

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('image')
    ap.add_argument('--install', action='store_true')
    ap.add_argument('--probe', action='store_true', help='read-only identity/negotiation check; no OTA')
    ap.add_argument('--expect-base')
    ap.add_argument('--mac-interval', action='store_true', help='bench Graceful only')
    T.add_arguments(ap)
    asyncio.run(main(ap.parse_args()))
