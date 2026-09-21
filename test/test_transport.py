"""Transport boundary tests: fail closed, preserve credits, and order READY/setup/data."""
import argparse
import asyncio
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import AsyncMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import esp_ota_ble as O
import esp_ota_ble_transport as T


class OptionsTests(unittest.TestCase):
    def test_public_arguments_roundtrip_and_defaults(self):
        ap = argparse.ArgumentParser()
        T.add_arguments(ap)
        self.assertEqual(T.from_arguments(ap.parse_args([])), T.Options())
        args = ap.parse_args(['--ble-backend', 'bumble', '--adapter', T.TESTED_UB500,
                             '--ble-chunk', '495', '--ble-interval-ms', '12.5',
                             '--ble-phy', '2', '--experimental-hci-packet-size', '251'])
        self.assertEqual(T.from_arguments(args), T.Options('bumble', T.TESTED_UB500, 495, 12.5, 2, 251))

    def test_invalid_options_and_far_tail(self):
        for value in (-10**9, -1, 513, 10**9, None, True, 244.5):
            with self.subTest(chunk=value), self.assertRaises(ValueError): T.Options(chunk=value)
        for value in (-1e99, 0, 7.4, 8, 4001, 1e99, float('nan'), float('inf')):
            with self.subTest(interval=value), self.assertRaises(ValueError):
                T.Options(backend='bumble', interval_ms=value)
        for options in [dict(phy=2), dict(interval_ms=10), dict(backend='native', adapter='hci0'),
                        dict(backend='bumble', phy=True), dict(experimental_hci_packet_size=251),
                        dict(backend='bumble', experimental_hci_packet_size=251),
                        dict(backend='bumble', adapter='hci0', experimental_hci_packet_size=502)]:
            with self.subTest(options=options), self.assertRaises(ValueError): T.Options(**options)


class Queue:
    max_packet_size = 27
    max_in_flight = 8
    pending = 0
    def on(self, _, callback): self.callback = callback
    def remove_listener(self, _, callback): self.callback = None


class TransportTests(unittest.IsolatedAsyncioTestCase):
    def link(self, **options):
        link = T.DirectLink('cmd', 'notify', 'fw', options=T.Options(**options), on_note=lambda _: None)
        link._cli = NS(disconnect=AsyncMock(), write_gatt_char=AsyncMock())
        link._capacity = 512
        return link

    async def bumble(self, experimental=False, address=T.TESTED_UB500, report=(27, 8), owners=1):
        link = self.link(backend='bumble', adapter=T.TESTED_UB500,
                         experimental_hci_packet_size=251 if experimental else None)
        queue = Queue()
        host = NS(get_data_packet_queue=lambda _: queue,
                  send_sync_command=AsyncMock(return_value=NS(
                      le_acl_data_packet_length=report[0], total_num_le_acl_data_packets=report[1])))
        device = NS(host=host, public_address=NS(to_string=lambda _: address), connections=list(range(owners)))
        link._cli._connection = NS(device=device, handle=1)
        return link, queue

    async def configure(self, link):
        with patch.dict(sys.modules, {'bumble.hci': NS(HCI_LE_Read_Buffer_Size_Command=lambda: None)}):
            await link._configure_bumble()

    async def test_default_does_not_override_even_on_tested_adapter(self):
        link, queue = await self.bumble()
        await self.configure(link)
        self.assertEqual((queue.max_packet_size, queue.max_in_flight), (27, 8))
        await link.release()

    async def test_experiment_preserves_credits_and_restores_on_disconnect_failure(self):
        link, queue = await self.bumble(experimental=True)
        await self.configure(link)
        self.assertEqual((queue.max_packet_size, queue.max_in_flight), (251, 8))
        link._cli.disconnect.side_effect = OSError('link already gone')
        with self.assertRaises(OSError): await link.release()
        self.assertEqual((queue.max_packet_size, queue.max_in_flight), (27, 8))
        self.assertIsNone(queue.callback)
        self.assertIsNone(link._cli)

    async def test_wrong_controller_report_and_ownership_fail_before_override(self):
        for changes in [dict(address='00:00:00:00:00:00'), dict(report=(251, 8)),
                        dict(report=(27, 7)), dict(owners=0), dict(owners=2), dict(owners=1000)]:
            link, queue = await self.bumble(experimental=True, **changes)
            with self.subTest(changes=changes), self.assertRaises(O.OtaBleError):
                await self.configure(link)
            self.assertEqual((queue.max_packet_size, queue.max_in_flight), (27, 8))
            await link.release()

    async def test_stalled_hci_queue_never_writes_and_flow_resumes(self):
        link = self.link()
        link._queue = queue = Queue()
        queue.pending = 256
        task = asyncio.create_task(link.write_fw(b'a'))
        await asyncio.sleep(.01)
        link._cli.write_gatt_char.assert_not_awaited()
        queue.pending = 255
        link._flow.set()
        await asyncio.wait_for(task, .5)
        link._cli.write_gatt_char.assert_awaited_once()
        queue.pending = 1000000
        with patch.object(T.time, 'monotonic', side_effect=[0, 6]):
            with self.assertRaises(O.OtaBleError): await link.write_fw(b'a')
        self.assertEqual(link._cli.write_gatt_char.await_count, 1)
        link.disconnected.set()
        with self.assertRaises(O.OtaBleError): await link.write_fw(b'a')

    async def configure_with(self, capacity, *, bluez=True, mtu=23, acquired=False, **options):
        """_configure_capacity() against a characteristic reporting `capacity`."""
        notes = []
        link = self.link(**options)
        link.note = notes.append
        link._capacity = 0
        link.mtu = mtu
        link._mtu_acquired = acquired
        if acquired:
            link._mtu_status = 'acquired (MTU %d)' % mtu
        char = NS(properties=['write-without-response'], max_write_without_response_size=capacity)
        link._cli.services = NS(get_characteristic=lambda _: char)
        with patch.object(T.sys, 'platform', 'linux' if bluez else 'darwin'):
            await link._configure_capacity()
        return link, ' '.join(notes)

    async def test_unreportable_capacity_degrades_to_the_safe_minimum_and_says_so(self):
        # bleak can hand back anything here: getattr()'s default also absorbs an
        # AttributeError raised inside the backend's property.
        for capacity in (None, 0, -1, True, '244', object()):
            with self.subTest(capacity=capacity):
                link, notes = await self.configure_with(capacity)
                self.assertEqual((link._capacity, link.chunk), (O.SAFE_MIN_FW_WRITE, 20))
                self.assertIn('FALLING BACK', notes)
                self.assertIn('not attempted', notes)
                await link.write_fw(b'x' * 20)
                with self.assertRaises(O.OtaBleError): await link.write_fw(b'x' * 21)

    async def test_bleak3_bluez_reports_mtu_minus_3_above_the_att_maximum(self):
        # bleak 3.x/BlueZ 5.62+ computes this live as char MTU - 3, so a link at
        # the maximal ATT MTU of 517 reports 514 -- above the 512 an ATT value
        # can hold, and exactly the size that silently corrupted an image.
        link, notes = await self.configure_with(514)
        self.assertEqual((link._capacity, link.chunk), (O.BLUEZ_MAX_FW_WRITE, 244))
        self.assertIn('CLAMPED', notes)
        link, _ = await self.configure_with(514, bluez=False)
        self.assertEqual((link._capacity, link.chunk), (512, 244))

    async def test_plain_report_is_used_and_an_oversized_override_still_refuses(self):
        link, notes = await self.configure_with(244)
        self.assertEqual((link._capacity, link.chunk), (244, 244))
        self.assertIn('backend reported 244', notes)
        await link.write_fw(b'x' * 244)
        with self.assertRaises(O.OtaBleError): await link.write_fw(b'x' * 245)
        with self.assertRaises(O.OtaBleError): await self.configure_with(244, chunk=495)
        link = self.link(chunk=495)
        link._capacity = 0
        for data in (b'', b'x' * 496, b'x' * 100000):
            with self.assertRaises(O.OtaBleError): await link.write_fw(data)
        link._cli.write_gatt_char.assert_not_awaited()

    async def test_acquired_mtu_rescues_a_stale_or_absent_report(self):
        for capacity in (20, None):
            with self.subTest(capacity=capacity):
                link, notes = await self.configure_with(capacity, mtu=517, acquired=True)
                self.assertEqual((link._capacity, link.chunk), (O.BLUEZ_MAX_FW_WRITE, 244))
                self.assertIn('acquired MTU 517', notes)
                self.assertIn('acquired (MTU 517)', notes)
        # A real 20 on a link nobody acquired is authoritative, not stale.
        link, _ = await self.configure_with(20, bluez=False)
        self.assertEqual(link._capacity, 20)

    async def test_corebluetooth_unwritable_is_failure(self):
        link = self.link()
        link._cli._backend = NS(_delegate=NS(peripheral=NS(canSendWriteWithoutResponse=lambda: False)))
        with patch.object(T.time, 'monotonic', side_effect=[0, 3]):
            with self.assertRaises(O.OtaBleError): await link.write_fw(b'x')
        link._cli.write_gatt_char.assert_not_awaited()

    async def test_backend_reference_survives_scan_and_releases_on_failure(self):
        keeper = NS(acquire=AsyncMock(), release=AsyncMock())
        link = self.link(backend='bumble')
        finder = AsyncMock(side_effect=RuntimeError('failed scan'))
        with patch.dict(sys.modules, {'bumble_bleak._backend': NS(get_backend=AsyncMock(return_value=keeper))}), \
             patch.object(link, 'scanner_class', return_value=object):
            with self.assertRaises(RuntimeError): await link.scan(finder)
        keeper.acquire.assert_awaited_once()
        keeper.release.assert_awaited_once()

    async def test_prepare_after_ready_failure_sends_no_firmware(self):
        events = []
        class Link:
            disconnected = asyncio.Event()
            chunk = 244
            def set_line_handler(self, fn): self.line = fn
            async def write_cmd(self, text):
                events.append('begin')
                self.line('OTAB READY')
                events.append('ready')
            async def prepare_transfer(self):
                events.append('prepare')
                raise O.OtaBleError('not negotiated')
            async def write_fw(self, data): events.append('data')
        with self.assertRaises(O.OtaBleError): await O.push_image(Link(), b'image')
        self.assertEqual(events, ['begin', 'ready', 'prepare'])

    async def test_actual_parameters_must_match_not_only_command_acceptance(self):
        for actual, expected in [(12.5, True), (15, False), (4000, False)]:
            link = self.link(backend='bumble', interval_ms=12.5, phy=2)
            conn = NS(set_data_length=AsyncMock(), set_phy=AsyncMock(), update_parameters=AsyncMock(),
                      get_phy=AsyncMock(return_value=NS(tx_phy=2, rx_phy=2)),
                      parameters=NS(connection_interval=actual, peripheral_latency=0))
            link._cli._connection = conn
            with self.subTest(actual=actual), \
                 patch.dict(sys.modules, {'bumble.hci': NS(Phy=lambda n: n)}), \
                 patch.object(T.time, 'monotonic', side_effect=[0, 4]):
                if expected: await link.prepare_transfer()
                else:
                    with self.assertRaises(O.OtaBleError): await link.prepare_transfer()

    async def test_attached_client_preserves_subscriptions_and_disconnect_ownership(self):
        owner_event = asyncio.Event()
        char = NS(properties=['write-without-response'], max_write_without_response_size=244)
        client = NS(is_connected=True, mtu_size=247, services=NS(get_characteristic=lambda _: char),
                    disconnect=AsyncMock(), start_notify=AsyncMock(), write_gatt_char=AsyncMock())
        link = T.DirectLink('cmd', 'notify', 'fw', on_note=lambda _: None)
        with patch.object(link, '_acquire_mtu_if_default', AsyncMock()):
            await link.attach(client, disconnected=owner_event)
        lines = []
        link.set_line_handler(lines.append)
        link.feed_notification(b'OTAB REA')
        link.feed_notification(b'DY\n')
        self.assertEqual(lines, ['OTAB READY'])
        await link.write_fw(b'payload')
        client.write_gatt_char.assert_awaited_once_with('fw', b'payload', response=False)
        await link.release()
        client.disconnect.assert_not_awaited()
        client.start_notify.assert_not_awaited()
        self.assertFalse(owner_event.is_set())
        with self.assertRaises(O.OtaBleError): await link.write_fw(b'after release')

    async def test_attach_rejects_unverified_connection(self):
        for client, event in [(NS(is_connected=False), asyncio.Event()), (NS(is_connected=True), None)]:
            link = T.DirectLink('cmd', 'notify', 'fw')
            with self.assertRaises(O.OtaBleError): await link.attach(client, disconnected=event)

    async def test_failed_backend_start_is_closed(self):
        keeper = NS(acquire=AsyncMock(side_effect=OSError('HCI startup failed')), release=AsyncMock())
        link = self.link(backend='bumble')
        with patch.dict(sys.modules, {'bumble_bleak._backend': NS(get_backend=AsyncMock(return_value=keeper))}):
            with self.assertRaises(OSError): await link.scan(AsyncMock())
        keeper.release.assert_awaited_once()
        self.assertIsNone(link._keeper)

    async def test_guard_cost(self):
        link = self.link(backend='bumble')
        start = time.perf_counter()
        for _ in range(10000): await link.write_fw(b'x'*244)
        elapsed = time.perf_counter() - start
        print('write guard plus AsyncMock: %.2f us/write' % (elapsed * 100))


if __name__ == '__main__': unittest.main()
