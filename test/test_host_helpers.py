#!/usr/bin/env python3
"""The helpers lifted out of the two consumer tools, plus the link contract.

These arrived in the shared module on 2026-09-10 from byte-identical copies in
fugu-mppt-firmware/etc/ota_ble.py and node-prototype/tools/ota_ble_push.py.
Copies drift; the point of lifting them is that there is now one behaviour to
pin, so pin it.
"""
import asyncio, io, os, struct, sys, tempfile, unittest
from contextlib import redirect_stdout

HOST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "host")
sys.path.insert(0, os.path.abspath(HOST))

import esp_ota_ble as O


def fake_image(version=b"1.2.3-abc", magic=O.APP_DESC_MAGIC, pad=0x20):
    """An esp_app_desc_t at a plausible offset, with the version where IDF puts it."""
    head = b"\xe9" + b"\x00" * (pad - 1)
    desc = struct.pack("<I", magic) + b"\x00" * 0x0C          # magic + secure_version/reserv
    desc += version + b"\x00" * (0x20 - len(version))          # version[32] at +0x10
    return head + desc + b"\x00" * 0x40


class AppDesc(unittest.TestCase):
    def test_version_from_bytes(self):
        self.assertEqual(O.read_local_app_desc(fake_image()), "1.2.3-abc")

    def test_version_from_a_path(self):
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
            f.write(fake_image(b"v9"))
            path = f.name
        try:
            self.assertEqual(O.read_local_app_desc(path), "v9")
        finally:
            os.unlink(path)

    def test_a_missing_file_is_none_not_an_exception(self):
        self.assertIsNone(O.read_local_app_desc("/nonexistent/firmware.bin"))

    def test_an_image_without_a_descriptor_is_none(self):
        self.assertIsNone(O.read_local_app_desc(b"\xe9" + b"\x00" * 400))

    def test_a_descriptor_past_the_first_512_bytes_is_not_found(self):
        # Recorded, not lamented: the search window is 512 bytes because the
        # descriptor always sits inside it in a real ESP app image. This asserts
        # what the function does rather than what a reader might assume.
        self.assertIsNone(O.read_local_app_desc(fake_image(pad=0x400)))


class BrickGuard(unittest.TestCase):
    IMG = b"....OTAB CRED....OTAB READY....BLE_HS...."

    def test_all_mode_requires_every_marker(self):
        self.assertTrue(O.image_keeps_the_push_path(self.IMG))
        self.assertFalse(O.image_keeps_the_push_path(b"....OTAB CRED...."))

    def test_any_mode_accepts_one(self):
        self.assertTrue(O.image_keeps_the_push_path(
            b"....BLE_HS....", (b"OTAB CRED", b"BLE_HS"), mode="any"))
        self.assertFalse(O.image_keeps_the_push_path(
            b"nothing here", (b"OTAB CRED", b"BLE_HS"), mode="any"))

    def test_an_empty_image_never_passes(self):
        self.assertFalse(O.image_keeps_the_push_path(b""))

    def test_no_markers_raises_rather_than_passing(self):
        # all(()) is True. A caller that passed an empty marker set would
        # otherwise get a guard that says "safe" about every image there is.
        with self.assertRaises(ValueError):
            O.image_keeps_the_push_path(self.IMG, ())

    def test_an_unknown_mode_raises_rather_than_defaulting(self):
        with self.assertRaises(ValueError):
            O.image_keeps_the_push_path(self.IMG, (b"OTAB CRED",), mode="most")


class ProgressBar(unittest.TestCase):
    def render(self, done, total):
        buf = io.StringIO()
        with redirect_stdout(buf):
            O.progress_bar(done, total, "push")
        return buf.getvalue()

    def test_it_reports_the_fraction(self):
        self.assertIn("50.0%", self.render(50, 100))
        self.assertIn("50/100", self.render(50, 100))

    def test_completion_ends_the_line(self):
        self.assertTrue(self.render(100, 100).endswith("\n"))
        self.assertFalse(self.render(50, 100).endswith("\n"))

    def test_a_zero_total_does_not_divide_by_zero(self):
        self.assertIn("0.0%", self.render(0, 0))


class ImageIsRunning(unittest.TestCase):
    """The one piece of post-push confirmation that belongs in the library: the criterion."""

    DIGEST = "ab" * 32

    def test_matching_digests_are_the_same_image(self):
        # image_id() is stubbed to the value under test: this asserts the COMPARISON,
        # and image_id()'s own parsing is covered where it is defined.
        real, O.image_id = O.image_id, lambda data: self.DIGEST
        try:
            self.assertTrue(O.image_is_running({"base": self.DIGEST}, b"whatever"))
            self.assertTrue(O.image_is_running({"base": self.DIGEST.upper()}, b"whatever"))
            self.assertFalse(O.image_is_running({"base": "cd" * 32}, b"whatever"))
        finally:
            O.image_id = real

    def test_unevaluable_is_none_and_not_false(self):
        # A caller must be able to tell "not that image" from "could not tell".
        # Reporting the second as the first would make a failed check look like a
        # verdict, which is the failure this whole module keeps warning about.
        self.assertIsNone(O.image_is_running(None, b"x"))
        self.assertIsNone(O.image_is_running({}, b"x"))
        self.assertIsNone(O.image_is_running({"base": None}, b"x"))   # OTAB BASE none
        real, O.image_id = O.image_id, lambda data: None
        try:
            self.assertIsNone(O.image_is_running({"base": self.DIGEST}, b"no appended hash"))
        finally:
            O.image_id = real


class Good:
    def __init__(self):
        self.chunk = 200
        self.disconnected = asyncio.Event()

    def set_line_handler(self, fn):
        pass

    async def write_cmd(self, text):
        pass

    async def write_fw(self, data):
        pass


class LinkContract(unittest.TestCase):
    def test_a_complete_link_passes(self):
        O.require_link(Good())

    def test_a_missing_member_is_named(self):
        link = Good()
        del link.disconnected
        with self.assertRaises(O.OtaBleError) as cm:
            O.require_link(link, "push_image")
        self.assertIn("disconnected", str(cm.exception))

    def test_a_disconnected_that_is_not_an_event_is_refused(self):
        # The one that would otherwise surface as an AttributeError inside a
        # failure path, a hundred seconds into a transfer.
        link = Good()
        link.disconnected = False
        with self.assertRaises(O.OtaBleError):
            O.require_link(link)

    def test_query_info_asks_for_less_than_push_image(self):
        # A query-only link has no firmware sink, and demanding one would reject
        # a link that can perfectly well answer `info`.
        class QueryOnly:
            def set_line_handler(self, fn):
                pass

            async def write_cmd(self, text):
                pass

        O.require_link(QueryOnly(), "query_info", O.OtaLink.REQUIRED_INFO)
        with self.assertRaises(O.OtaBleError):
            O.require_link(QueryOnly(), "push_image")


class WriteCapacity(unittest.TestCase):
    """resolve_write_capacity(): every branch narrows, and none of them refuse.

    The blocker this replaced: on bleak 3.0.2 / BlueZ 5.82 the reported capacity
    is `char MTU - 3`, so a 517-MTU ESP32 link reports 514 and the old range
    check (1..512) rejected it outright -- a firmware push that could not start
    at all. Refusing is the wrong answer for BOTH tails: what is needed is to
    write no MORE than something known safe.
    """

    def caps(self, reported, **kw):
        kw.setdefault("backend_is_bluez", True)
        return O.resolve_write_capacity(reported, kw.pop("mtu", 23), **kw)

    def test_unreportable_falls_back_to_the_safe_minimum(self):
        for reported in (None, 0, -1, -10**9, True, False, "244", 244.0, object()):
            with self.subTest(reported=reported):
                capacity, reason = self.caps(reported)
                self.assertEqual(capacity, O.SAFE_MIN_FW_WRITE)
                self.assertIn("FALLING BACK", reason)

    def test_over_the_att_maximum_is_clamped_down_not_refused(self):
        # 514 is bleak-3-on-BlueZ at ATT MTU 517, and is also the exact size
        # that silently corrupted an image -- so it is clamped, never trusted.
        self.assertEqual(self.caps(514)[0], O.BLUEZ_MAX_FW_WRITE)
        self.assertEqual(self.caps(513)[0], O.BLUEZ_MAX_FW_WRITE)
        self.assertEqual(self.caps(514, backend_is_bluez=False)[0], 512)
        self.assertIn("CLAMPED", self.caps(514)[1])

    def test_a_worse_report_never_buys_a_bigger_write(self):
        # Monotonicity, including the far tail: nothing above the platform limit
        # gets more than the limit, and nothing unusable gets more than 20.
        limit = O.BLUEZ_MAX_FW_WRITE
        for reported in (401, 512, 513, 514, 10**3, 10**9, 2**64):
            self.assertEqual(self.caps(reported)[0], limit, reported)
        for reported in (None, 0, -1, "big", float("inf")):
            self.assertEqual(self.caps(reported)[0], O.SAFE_MIN_FW_WRITE, reported)
        for reported in (1, 19, 20, 185, 244, 400):
            capacity, _ = self.caps(reported)
            self.assertLessEqual(capacity, reported)

    def test_a_plain_report_is_used_as_given(self):
        for reported in (1, 19, 185, 244, 400):
            capacity, reason = self.caps(reported)
            self.assertEqual(capacity, reported)
            self.assertIn("backend reported %d" % reported, reason)
        # CoreBluetooth's real answers are below mtu - 3 on purpose; the BlueZ
        # cap must not touch them.
        self.assertEqual(self.caps(185, backend_is_bluez=False)[0], 185)

    def test_an_acquired_mtu_rescues_a_stale_or_absent_report(self):
        for reported in (20, None, 0):
            with self.subTest(reported=reported):
                capacity, reason = self.caps(reported, mtu=517, mtu_acquired=True)
                self.assertEqual(capacity, O.BLUEZ_MAX_FW_WRITE)
                self.assertIn("acquired MTU 517", reason)
        self.assertEqual(self.caps(20, mtu=247, mtu_acquired=True)[0], 244)
        # Not acquired, or acquired and still the default: 20 stands as reported.
        self.assertEqual(self.caps(20)[0], 20)
        self.assertEqual(self.caps(20, mtu=23, mtu_acquired=True)[0], 20)
        # And a real characteristic limit below mtu - 3 is never inflated.
        self.assertEqual(self.caps(185, mtu=517, mtu_acquired=True)[0], 185)


class AcquireMtuStatus(unittest.TestCase):
    """The four outcomes an operator has to be able to tell apart in one line."""

    class Client:
        def __init__(self, mtu=23, after=None, raises=None, backend=True):
            self.mtu_size, self._after, self._raises = mtu, after, raises
            if backend:
                self._backend = self

        async def _acquire_mtu(self):
            if self._raises:
                raise self._raises
            if self._after:
                self.mtu_size = self._after

    def run_status(self, client):
        return asyncio.run(O.acquire_bluez_mtu_status(client))

    def test_each_outcome_names_itself(self):
        cases = [
            (self.Client(backend=False), False, "no _acquire_mtu"),
            (self.Client(mtu=247), False, "already reports MTU 247"),
            (self.Client(raises=RuntimeError("no AcquireWrite")), False, "FAILED"),
            (self.Client(), False, "attempted but MTU still 23"),
            (self.Client(after=517), True, "acquired (MTU 517)"),
        ]
        for client, expected, marker in cases:
            with self.subTest(marker=marker):
                acquired, reason = self.run_status(client)
                self.assertIs(acquired, expected)
                self.assertIn(marker, reason)

    def test_the_bool_wrapper_still_agrees(self):
        self.assertIs(asyncio.run(O.acquire_bluez_mtu(self.Client(after=517))), True)
        self.assertIs(asyncio.run(O.acquire_bluez_mtu(self.Client())), False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
