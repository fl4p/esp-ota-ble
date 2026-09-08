#!/usr/bin/env python3
"""find_device: early exit must not cost the ambiguity guard.

No hardware and no bleak: a fake BleakScanner replays sightings on a schedule, so
the tests assert both the ANSWER and the TIME TAKEN.
"""
import asyncio, os, sys, time, types, unittest

HOST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "host")
sys.path.insert(0, os.path.abspath(HOST))


class Dev:
    def __init__(self, address, name=None):
        self.address, self.name = address, name


class Adv:
    def __init__(self, local_name=None, service_uuids=None):
        self.local_name = local_name
        self.service_uuids = service_uuids or []


class FakeScanner:
    """Replays (delay, dev, adv) and then goes quiet, like a real radio."""
    schedule = []

    def __init__(self, detection_callback=None, **kw):
        self._cb = detection_callback
        self._task = None

    async def start(self):
        async def feed():
            for delay, dev, adv in FakeScanner.schedule:
                await asyncio.sleep(delay)
                self._cb(dev, adv)
        self._task = asyncio.create_task(feed())

    async def stop(self):
        if self._task:
            self._task.cancel()

    @staticmethod
    async def find_device_by_address(address, timeout=0):
        for _, dev, _ in FakeScanner.schedule:
            if dev.address == address:
                return dev
        return None


fake = types.ModuleType("bleak")
fake.BleakScanner = FakeScanner
sys.modules["bleak"] = fake

import esp_ota_ble as O  # noqa: E402

UUID = "0000abcd-0000-1000-8000-00805f9b34fb"


def run(**kw):
    t0 = time.monotonic()
    got = asyncio.run(O.find_device(**kw))
    return got, time.monotonic() - t0


class TestFindDevice(unittest.TestCase):
    def tearDown(self):
        FakeScanner.schedule = []

    def test_exits_early_instead_of_sleeping_out_the_timeout(self):
        FakeScanner.schedule = [(0.05, Dev("AA", "fugu-flu"), Adv("fugu-flu"))]
        dev, secs = run(name_prefix="fugu-", timeout=8.0)
        self.assertEqual(dev.address, "AA")
        # The whole point: 8 s bound, but back in ~SCAN_SETTLE.
        self.assertLess(secs, 2.0, "did not exit early (%.2f s)" % secs)

    def test_two_matches_are_still_ambiguous(self):
        # The second board arrives INSIDE the settle window; the guard must see it.
        FakeScanner.schedule = [(0.05, Dev("AA", "fugu-flu"), Adv("fugu-flu")),
                                (0.05, Dev("BB", "fugu-fry"), Adv("fugu-fry"))]
        with self.assertRaises(O.OtaBleError) as cm:
            run(name_prefix="fugu-", timeout=8.0)
        self.assertIn("several devices match", str(cm.exception))

    def test_no_match_returns_none_and_uses_the_whole_bound(self):
        FakeScanner.schedule = [(0.05, Dev("AA", "other"), Adv("other"))]
        dev, secs = run(name_prefix="fugu-", timeout=0.4)
        self.assertIsNone(dev)
        self.assertGreaterEqual(secs, 0.4, "gave up before the bound")

    def test_service_uuid_must_also_match(self):
        FakeScanner.schedule = [(0.05, Dev("AA", "fugu-flu"), Adv("fugu-flu"))]
        dev, _ = run(name_prefix="fugu-", service_uuid=UUID, timeout=0.4)
        self.assertIsNone(dev, "matched on name despite a missing service UUID")
        FakeScanner.schedule = [(0.05, Dev("AA", "fugu-flu"),
                                 Adv("fugu-flu", [UUID.upper()]))]
        dev, _ = run(name_prefix="fugu-", service_uuid=UUID, timeout=8.0)
        self.assertEqual(dev.address, "AA", "UUID case-sensitivity regressed")

    def test_matches_on_either_name_field(self):
        # CoreBluetooth serves the cached d.name with no local_name, and the reverse.
        FakeScanner.schedule = [(0.05, Dev("AA", "fugu-flu"), Adv(None))]
        self.assertIsNotNone(run(name_prefix="fugu-", timeout=8.0)[0])
        FakeScanner.schedule = [(0.05, Dev("BB", None), Adv("fugu-flu"))]
        self.assertIsNotNone(run(name_prefix="fugu-", timeout=8.0)[0])
        # A stale cached name must not hide a current advertised one.
        FakeScanner.schedule = [(0.05, Dev("CC", "esp32s3-1234"), Adv("fugu-flu"))]
        self.assertIsNotNone(run(name_prefix="fugu-", timeout=8.0)[0])

    def test_address_still_bypasses_the_scan(self):
        FakeScanner.schedule = [(0.05, Dev("AA", "fugu-flu"), Adv("fugu-flu"))]
        dev, _ = run(address="AA", timeout=8.0)
        self.assertEqual(dev.address, "AA")
        with self.assertRaises(O.OtaBleError):
            run(address="ZZ", timeout=0.2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
