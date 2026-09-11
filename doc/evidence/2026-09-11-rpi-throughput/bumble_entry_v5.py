"""Private Bumble OTA/RAM experiment; retains the existing receiver verifiers."""
import asyncio
import json
import os
from pathlib import Path
import runpy
import sys

import bumble_bleak.shadow
from bumble.hci import Phy, OwnAddressType
from bumble.device import Device
from bumble_bleak import _backend
from bumble_bleak import BleakScanner

if os.environ.get('BENCH_MAC_DISCOVERY') == '1':
    original_find = BleakScanner.find_device_by_filter
    async def find_mac(cls, predicate, **kwargs):
        def matches(device, advertisement):
            if device.address.upper() != '7C:4F:AD:20:2A:29':
                return False
            print('DISCOVERY exact_mac', device.address, repr(device.name),
                  repr(advertisement.local_name), flush=True)
            return True
        return await original_find(matches, **kwargs)
    BleakScanner.find_device_by_filter = classmethod(find_mac)

if os.environ.get('BENCH_PUBLIC') == '1':
    _backend.OWN_ADDRESS_TYPE = OwnAddressType.PUBLIC
if os.environ.get('BENCH_LEGACY_SCAN') == '1':
    start_scan, stop_scan = Device.start_scanning, Device.stop_scanning
    async def start_legacy(self, **kwargs):
        kwargs['legacy'] = True
        kwargs['scan_interval'] = 60
        kwargs['scan_window'] = 60
        return await start_scan(self, **kwargs)
    async def stop_legacy(self, **kwargs):
        kwargs['legacy'] = True
        return await stop_scan(self, **kwargs)
    Device.start_scanning, Device.stop_scanning = start_legacy, stop_legacy

if os.environ.get('BENCH_SCAN_DIAGNOSTIC') == '1':
    adv_original = _backend._Backend._dispatch_advertisement
    def adv(self, advertisement):
        print('SCAN', advertisement.address, advertisement.rssi, flush=True)
        return adv_original(self, advertisement)
    _backend._Backend._dispatch_advertisement = adv

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'node/tools'))
import bench_ble_ota as B

original_connect = B.connect

async def connect(args, setup=True):
    # Keep one adapter reference across scan -> connect. Otherwise the facade
    # powers the controller off between these operations.
    keeper = await _backend.get_backend(args.adapter)
    await keeper.acquire()
    try:
        link = await original_connect(args, setup=setup)
    finally:
        await keeper.release()
    try:
        conn = link._cli._connection
        queue = conn.device.host.get_data_packet_queue(conn.handle)
        # This facade omits bleak's maximum-write property. Use the known
        # bench receiver's 512-byte characteristic capacity, bounded by the
        # actual exchanged MTU, rather than pretending the property exists.
        char = link._cli.services.get_characteristic(B.DATA)
        if char is None or 'write-without-response' not in char.properties:
            raise RuntimeError('bench data characteristic is unverified')
        if not 23 < link.mtu <= 517 or link.mtu != conn.att_mtu:
            raise RuntimeError('negotiated ATT MTU is unverified')
        limit_bytes = min(512, link.mtu - 3)
        link._max_write = lambda: limit_bytes
        print('BUMBLE explicit_bench_characteristic_capacity=512 mtu=%s max_value=%s'
              % (link.mtu, limit_bytes), flush=True)
        interval = float(os.environ.get('BENCH_INTERVAL_MS', '7.5'))
        ce = float(os.environ.get('BENCH_CE_MS', '0'))
        phy = int(os.environ.get('BENCH_PHY', '2'))
        await conn.set_data_length(251, 2120)
        await conn.set_phy([Phy(phy)], [Phy(phy)])
        await conn.update_parameters(interval, interval, 0, 2000, ce, ce)
        await asyncio.sleep(1)
        diagnostic = dict(transport=link._cli._backend.adapter,
                          hci_packet_bytes=queue.max_packet_size,
                          hci_in_flight=queue.max_in_flight,
                          interval_requested_ms=interval, ce_requested_ms=ce,
                          phy=str(await conn.get_phy()), parameters=str(conn.parameters))
        print('BUMBLE', json.dumps(diagnostic, sort_keys=True), flush=True)
        for cmd in ('link', 'dleinfo', 'rssi'):
            await link.write_cmd(cmd)
            await asyncio.sleep(.2)
        # Bound host buffering independently of controller HCI credits. Never
        # enlarge the controller's advertised packet size or credit count.
        limit = int(os.environ.get('BENCH_QUEUE_PACKETS', '256'))
        if limit < 1 or limit > 8192:
            raise ValueError('invalid host queue limit')
        event = asyncio.Event()
        queue.on('flow', event.set)
        async def writable():
            while queue.pending >= limit:
                event.clear()
                if link.disconnected.is_set():
                    raise RuntimeError('disconnected while waiting for HCI queue')
                await asyncio.wait_for(event.wait(), 5)
        link._await_writable = writable
        original_cmd = link.write_cmd
        async def command(text):
            # RAM end must follow delivery, not just host enqueue. OTA end
            # additionally waits for the receiver's final PROG in push_image.
            if text == 'ramend':
                while queue.pending:
                    event.clear()
                    await asyncio.wait_for(event.wait(), 5)
            await original_cmd(text)
        link.write_cmd = command
        return link
    except BaseException:
        await link.release()
        raise

B.connect = connect
target = sys.argv.pop(1)
if target not in ('run_ram.py', 'node/tools/bench_ble_ota.py'):
    raise ValueError('unknown experiment')
if target.endswith('bench_ble_ota.py'):
    # runpy gives __main__ a separate global namespace; inject our connect
    # after executing only its definitions, then reuse its CLI unchanged.
    source = (ROOT / target).read_text()
    definitions, cli = source.split("if __name__=='__main__':", 1)
    namespace = dict(B.__dict__)
    exec("if True:" + cli, namespace)
else:
    runpy.run_path(str(ROOT / target), run_name='__main__')
