#!/usr/bin/env python3
"""Host half of resume: the negotiation, and the fallback that makes it safe to try.

No hardware and no bleak. A fake link plays a receiver whose OTAB behaviour the
test chooses -- including a receiver too old to have heard of `resume`, which is
the case the whole capability negotiation exists for and the one no amount of
device-side testing can reach.

The device-side half (what actually lands in flash, and what the digest catches)
is in test/host-stub/ota_ble-test.cpp; run ./test/run.sh for it.
"""
import asyncio, hashlib, os, sys, unittest

HOST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "host")
sys.path.insert(0, os.path.abspath(HOST))

import esp_ota_ble as O


class FakeReceiver:
    """A link that answers OTAB lines the way a receiver would.

    `resume_at` None means this receiver refuses every resume; `old` makes it
    answer `resume` the way a build that predates the command does, with the
    generic bad-command rejection.
    """

    def __init__(self, total, *, resume_at=None, old=False, window=8192):
        self.total, self.resume_at, self.old, self.window = total, resume_at, old, window
        self.mtu = 247
        self.chunk = 200
        self.disconnected = asyncio.Event()
        self._on_line = None
        self.cmds = []          # every control line the host sent
        self.fw = bytearray()   # every firmware byte the host sent
        self.started_at = None  # where the receiver believes the transfer began
        self.written = 0

    def set_line_handler(self, fn):
        self._on_line = fn

    def _say(self, line):
        if self._on_line:
            self._on_line(line)

    async def write_cmd(self, text):
        self.cmds.append(text)
        parts = text.split()
        if parts[0] == "begin":
            self.started_at = 0
            self.written = 0
            self._say("OTAB READY part=app1 size=%d xform=raw out=%d" % (self.total, self.total))
            self._say("OTAB CRED %d" % min(self.window, self.total))
        elif parts[0] == "resume":
            if self.old:
                self._say("OTAB FAIL bad-command")
                return
            off = int(parts[1])
            if self.resume_at is None or off != self.resume_at or int(parts[2]) != self.total:
                self._say("OTAB FAIL resume-mismatch")
                return
            self.started_at = off
            self.written = off
            self._say("OTAB READY part=app1 size=%d xform=raw out=%d from=%d"
                      % (self.total, self.total, off))
            self._say("OTAB CRED %d" % min(off + self.window, self.total))
        elif parts[0] == "end":
            self._say("OTAB OK rebooting")

    async def write_fw(self, data):
        self.fw += data
        self.written += len(data)
        self._say("OTAB PROG %d/%d" % (self.written, self.total))
        self._say("OTAB CRED %d" % min(self.written + self.window, self.total))


class InfoLink:
    """Replays a canned `info` reply, so query_info's parsing is tested directly."""

    def __init__(self, lines):
        self.lines, self.mtu, self.chunk = lines, 247, 200
        self.disconnected = asyncio.Event()
        self._on_line = None

    def set_line_handler(self, fn):
        self._on_line = fn

    async def write_cmd(self, text):
        for line in self.lines:
            self._on_line(line)


INFO_HEAD = ["OTAB INFO run=app0 slot=1769472", "OTAB BASE " + "ab" * 32]


class QueryInfo(unittest.TestCase):
    def info(self, lines):
        return asyncio.run(O.query_info(InfoLink(lines)))

    def test_resume_point_is_parsed(self):
        got = self.info(INFO_HEAD + ["OTAB RESUME 175104 751744", "OTAB XFORM raw,delta"])
        self.assertTrue(got["resume_capable"])
        self.assertEqual(got["resume"], {"off": 175104, "size": 751744})

    def test_none_is_capable_but_empty(self):
        got = self.info(INFO_HEAD + ["OTAB RESUME none", "OTAB XFORM raw"])
        self.assertTrue(got["resume_capable"])
        self.assertIsNone(got["resume"])

    def test_an_old_receiver_is_not_capable(self):
        # No RESUME line at all. This must not read as "capable, nothing to resume":
        # the two lead to the same push today, but only one of them means the NEXT
        # dropped link is recoverable, which is what a caller reports to a user.
        got = self.info(INFO_HEAD + ["OTAB XFORM raw"])
        self.assertFalse(got["resume_capable"])
        self.assertIsNone(got["resume"])

    def test_xform_still_ends_the_reply(self):
        # RESUME is emitted BEFORE XFORM precisely so this stays true: a host that
        # stops listening at XFORM (every host that predates resume) sees a complete
        # reply, and one that does not stop early sees the extra line.
        got = self.info(INFO_HEAD + ["OTAB RESUME 4096 20000", "OTAB XFORM raw"])
        self.assertEqual(got["xforms"], ("raw",))


class ResumeOffset(unittest.TestCase):
    PAYLOAD = b"x" * 20000

    def off(self, info, payload=None, xform="raw", enabled=True):
        return O.resume_offset(info, payload if payload is not None else self.PAYLOAD,
                               xform, enabled=enabled)

    def test_a_matching_point_is_taken(self):
        self.assertEqual(self.off({"resume_capable": True,
                                   "resume": {"off": 8192, "size": 20000}}), 8192)

    def test_an_old_receiver_starts_fresh(self):
        self.assertEqual(self.off({"resume_capable": False, "resume": None}), 0)

    def test_no_info_at_all_starts_fresh(self):
        self.assertEqual(self.off(None), 0)

    def test_a_different_size_starts_fresh(self):
        # The cheap half of "is this a prefix of THIS image". The receiver also
        # requires the digest, and re-hashes the flashed prefix before `end`; this
        # is only the place it costs nothing to notice.
        self.assertEqual(self.off({"resume_capable": True,
                                   "resume": {"off": 8192, "size": 19999}}), 0)

    def test_a_transform_starts_fresh(self):
        # A wire offset is not an image offset under tamp or delta, and the device's
        # transform state died with the session.
        for x in ("tamp", "delta"):
            self.assertEqual(self.off({"resume_capable": True,
                                       "resume": {"off": 8192, "size": 20000}}, xform=x), 0)

    def test_disabled_starts_fresh(self):
        self.assertEqual(self.off({"resume_capable": True,
                                   "resume": {"off": 8192, "size": 20000}}, enabled=False), 0)

    def test_an_offset_outside_the_payload_starts_fresh(self):
        for off in (0, 20000, 20001, -1):
            self.assertEqual(self.off({"resume_capable": True,
                                       "resume": {"off": off, "size": 20000}}), 0)

    def test_a_garbled_point_starts_fresh(self):
        # An unevaluable answer is never permission.
        self.assertEqual(self.off({"resume_capable": True,
                                   "resume": {"off": None, "size": 20000}}), 0)


class PushImage(unittest.TestCase):
    IMG = bytes((i * 31 + (i >> 8) * 7) & 0xFF for i in range(20000))

    def push(self, link, **kw):
        return asyncio.run(O.push_image(link, self.IMG, **kw))

    def test_a_resumed_push_sends_only_the_tail(self):
        link = FakeReceiver(len(self.IMG), resume_at=8192)
        self.assertTrue(self.push(link, resume_from=8192))
        self.assertEqual(bytes(link.fw), self.IMG[8192:])
        self.assertTrue(link.cmds[0].startswith("resume 8192 20000 "))
        self.assertIn(hashlib.sha256(self.IMG).hexdigest(), link.cmds[0])
        self.assertNotIn("begin", " ".join(link.cmds))

    def test_an_old_receiver_falls_back_to_a_whole_push(self):
        # THE compatibility case. A new host must not break against a receiver that
        # has never heard of `resume`: the refusal costs one round trip and the
        # image goes over in full.
        link = FakeReceiver(len(self.IMG), old=True)
        self.assertTrue(self.push(link, resume_from=8192, ready_timeout=5.0))
        self.assertTrue(link.cmds[0].startswith("resume "))
        self.assertTrue(link.cmds[1].startswith("begin 20000 "))
        self.assertEqual(bytes(link.fw), self.IMG)

    def test_a_refused_resume_falls_back_to_a_whole_push(self):
        # Same fallback, different reason: a receiver that knows the command but
        # will not stand behind the prefix in its slot.
        link = FakeReceiver(len(self.IMG), resume_at=None)
        self.assertTrue(self.push(link, resume_from=8192, ready_timeout=5.0))
        self.assertTrue(link.cmds[1].startswith("begin 20000 "))
        self.assertEqual(bytes(link.fw), self.IMG)

    def test_a_fresh_push_still_sends_the_original_command(self):
        # Byte-for-byte the two-argument form, so a receiver that predates all of
        # this sees nothing new.
        link = FakeReceiver(len(self.IMG))
        self.assertTrue(self.push(link))
        self.assertEqual(link.cmds[0],
                         "begin 20000 " + hashlib.sha256(self.IMG).hexdigest())
        self.assertEqual(bytes(link.fw), self.IMG)

    def test_resume_under_a_transform_is_refused_locally(self):
        link = FakeReceiver(len(self.IMG), resume_at=8192)
        with self.assertRaises(O.OtaBleError):
            self.push(link, resume_from=8192, xform="delta", out_size=40000)
        self.assertEqual(link.cmds, [])

    def test_an_offset_outside_the_payload_is_refused_locally(self):
        for off in (len(IMG := self.IMG), len(IMG) + 1):
            link = FakeReceiver(len(self.IMG), resume_at=off)
            with self.assertRaises(O.OtaBleError):
                self.push(link, resume_from=off)
            self.assertEqual(link.cmds, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
