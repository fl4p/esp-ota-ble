"""Native bridge IPC failures must reach OTA's disconnect and command waiters."""
import asyncio
import base64
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import esp_ota_ble_native as N


class NativeProtocol(unittest.IsolatedAsyncioTestCase):
    def client(self):
        event = asyncio.Event()
        with patch.object(N.sys, 'platform', 'darwin'):
            client = N.NativeClient(NS(address='00000000-0000-0000-0000-000000000001'),
                                    cmd_uuid='cmd', notify_uuid='notify', fw_uuid='fw',
                                    disconnected_callback=lambda _: event.set())
        client._process = NS(stdout=asyncio.StreamReader())
        return client, event

    async def test_notifications_and_ack_then_eof_fail_remaining_commands(self):
        client, event = self.client()
        payloads = []
        await client.start_notify('notify', lambda _, data: payloads.append(data))
        first = client._pending[1] = asyncio.get_running_loop().create_future()
        second = client._pending[2] = asyncio.get_running_loop().create_future()
        client._process.stdout.feed_data(b'{"id":1}\n{"event":"notify","data":"T1RBQiBSRUFEWQo="}\n')
        client._process.stdout.feed_eof()
        await client._read()
        self.assertEqual(await first, dict(id=1))
        with self.assertRaisesRegex(RuntimeError, 'exited'): await second
        self.assertEqual(payloads, [b'OTAB READY\n'])
        self.assertTrue(event.is_set())
        self.assertFalse(client.is_connected)

    async def test_invalid_ipc_and_explicit_errors_fail_closed(self):
        for frame in (b'not json\n', b'{"event":"error","message":"queue exceeded"}\n',
                      b'{"event":"notify","data":"!bad!"}\n'):
            client, event = self.client()
            client._notify = lambda *_: None
            pending = client._pending[1] = asyncio.get_running_loop().create_future()
            client._process.stdout.feed_data(frame)
            client._process.stdout.feed_eof()
            await client._read()
            with self.assertRaises(Exception): await pending
            self.assertTrue(event.is_set())


if __name__ == '__main__': unittest.main()
