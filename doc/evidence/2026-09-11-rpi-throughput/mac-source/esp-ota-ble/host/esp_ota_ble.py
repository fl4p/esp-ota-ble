"""Host side of esp-ota-ble: find the device, open the link, push the image.

Three tools grew their own copy of this (fugu-mppt-firmware/etc/ota_ble.py,
pwr-metering/smart-shunt-ota-ble.py, ha/farming/tech/node-prototype/tools/
ota_ble_push.py) — about 1500 lines between them, all speaking the same OTAB
protocol to the same receiver. This is that transport and protocol, once.

What differs between consumers is parameterised rather than forked:

  * the GATT layout. fugu and smart-shunt reach the receiver through a Nordic
    UART Service console plus a separate firmware characteristic, so commands
    and notifications land on two different characteristics. The farm node has
    its own service where one control characteristic both takes writes and
    notifies. `cmd_uuid`, `notify_uuid` and `fw_uuid` cover both; point the
    first two at the same UUID for the second shape.
  * the command prefix. A console-hosted receiver needs `ota-ble begin ...`;
    a dedicated characteristic takes `begin ...` bare. It is an argument to
    push_image() rather than a property of the link, because a console link
    also carries commands that must NOT be prefixed - fugu asks the same
    characteristic for `ping` and `uptime` before starting a push.

Only `bleak` is required.

PORTABILITY IS THE POINT, and specifically macOS *and* Linux. Every existing
copy of this code has only ever been run from macOS, and the first BlueZ run
corrupted the image silently — see `usable_chunk()`, which is the one piece of
knowledge in this module that cannot be recovered by reading a spec.
"""

import asyncio
import hashlib
import os
import platform
import re
import struct
import subprocess
import sys
import tempfile
import time

__all__ = [
    "OtaBleError", "find_device", "BleOtaLink", "adapt_link", "usable_chunk",
    "acquire_bluez_mtu",
    "push_image", "query_info", "resume_offset", "image_is_running",
    "OtaLink", "require_link",
    "image_id", "build_tamp_payload", "build_delta_payload", "choose_payload",
    "image_cache_dir", "cache_image", "cached_image",
    "read_local_app_desc", "image_keeps_the_push_path", "progress_bar",
    "TAMP_WINDOW_BITS", "DELTA_MAGIC", "DELTA_HEADER_SIZE",
    "APP_DESC_MAGIC", "OTAB_RECEIVER_MARKERS",
]

# The receiver's own status vocabulary (src/ota_ble.cpp).
_RE_READY = re.compile(r"OTAB READY\b")
_RE_CRED = re.compile(r"OTAB CRED (\d+)")
_RE_PROG = re.compile(r"OTAB PROG (\d+)/(\d+)")
_RE_INFO = re.compile(r"OTAB INFO run=(\S+) slot=(\d+)")
_RE_BASE = re.compile(r"OTAB BASE ([0-9a-fA-F]{64}|none)")
_RE_XFORM = re.compile(r"OTAB XFORM (\S+)")
# "OTAB RESUME <wireOffset> <wireSize>" or "OTAB RESUME none". A receiver too old for resume emits
# neither, and that absence is the capability test -- see resume_offset().
_RE_RESUME = re.compile(r"OTAB RESUME (?:(\d+) (\d+)|none)")

_MACOS = platform.system() == "Darwin"

# Cap on un-terminated status bytes held per link. Status lines are tens of
# bytes; anything approaching this is a peer that will never send a newline.
_RX_MAX = 64 * 1024

# The largest firmware write this module will issue on BlueZ, whatever the MTU
# says. See usable_chunk() for why this number exists and how it was arrived at.
BLUEZ_MAX_FW_WRITE = 400

# The mandatory ATT default. bleak's BlueZ backend reports exactly this until the
# real MTU is acquired, so it doubles as the sentinel for "nobody has asked yet".
_ATT_DEFAULT_MTU = 23


class OtaBleError(Exception):
    pass


def usable_chunk(mtu, backend_is_bluez=None, max_write=0):
    """Bytes of firmware to put in one write-without-response.

    The obvious answer, `mtu - 3`, is correct on CoreBluetooth and WRONG on
    BlueZ, and wrong in the worst available way: the writes are accepted by
    every layer the host can see and never arrive.

    MEASURED 2026-09-07, Raspberry Pi (BlueZ 5.82) -> ESP32-S3 (NimBLE), ATT MTU
    negotiated at 517 and reported as 517 by both ends:

        chunk 514 (= mtu - 3)   the whole 8192-byte credit window was written as
                                16 packets and the device received exactly ONE.
                                Proof it was the last rather than a random
                                survivor: it arrived as offset 7710 length 482,
                                and image[7710:7714] == b"erat". esp_ota_write
                                then rejected the image on its magic byte,
                                because the first byte it ever saw was the
                                middle of a string table.
        chunk 482               arrived (this was that survivor).
        chunk 400               a complete 706 640-byte image transferred and
                                the device confirmed the new slot.

    A write-without-response is unacknowledged by definition, so nothing
    upstream notices: bleak awaits BlueZ's D-Bus reply and BlueZ accepted all
    sixteen. Whatever drops them is below that — most likely the peripheral's
    mbuf pool, since a 514-byte ATT write costs a chain of buffers and NimBLE's
    pool is nowhere near a credit window. The exact threshold between 400 and
    514 was not bisected; 400 is used because it is measured end-to-end and 482
    is known to be too close to a size that fails.

    Acquiring the MTU is NOT the hazard; writing `mtu - 3` afterwards is. On
    BlueZ bleak reports the 23-byte default until the MTU is acquired, which is
    why the older tools chunk at 20 bytes there and have never corrupted an
    image — and also why they are ~6x slower than CoreBluetooth on the same
    link (1.76 MB image: 6m33s at 20-byte writes from a Pi vs 64 s from a Mac,
    measured 2026-09-08). BleOtaLink now acquires the MTU so this function sees
    the real number, and the cap below is what keeps that safe: do not remove
    it, and do not raise it above a size that has actually carried an image end
    to end.
    """
    # PREFER THE TRANSPORT'S OWN ANSWER. `max_write` is the characteristic's
    # max_write_without_response_size, which every backend computes from the
    # negotiated ATT MTU it actually has - unlike an OS test, which cannot see
    # through an adapted transport at all (fugu tunnels GATT through an ESPHome
    # proxy: the data plane is an ESP32, whatever OS the script runs on).
    if max_write:
        chunk = int(max_write)
    else:
        chunk = max(int(mtu) - 3, 20)

    # The policy cap. 400 is an EMPIRICAL regression point from one
    # Pi/BlueZ/NimBLE combination, not a derived safety bound - see the
    # measurements above. The derived bound is smaller: an ATT value of 244
    # (251 LL payload - 4 L2CAP - 3 ATT) is the largest that fits one
    # maximum-length link-layer PDU and so needs no L2CAP fragmentation. We cap
    # at the empirical 400 rather than the theoretical 244 because 400 is what
    # actually carried a 706 kB image end to end; 244 is the value to fall back
    # to if a new controller misbehaves.
    if backend_is_bluez is None:
        backend_is_bluez = not _MACOS
    if backend_is_bluez:
        chunk = min(chunk, BLUEZ_MAX_FW_WRITE)
    return max(chunk, 20)


async def acquire_bluez_mtu(client):
    """Make BlueZ report the ATT MTU it actually negotiated. Returns True if it did.

    bleak's BlueZ backend reports the mandatory default (23) and warns until the
    MTU is acquired, and the characteristic's `max_write_without_response_size`
    is derived from that same 23 — so BOTH numbers `usable_chunk()` consults say
    "20 bytes" on a link that negotiated far more. Measured on a Pi (BlueZ 5.82)
    2026-09-08: `mtu_size` 23 before this call and 247 after, on a link whose
    1.76 MB push took 6m33s at 20-byte writes where the same image over
    CoreBluetooth took 64 s.

    This is safe ONLY because `usable_chunk()` caps BlueZ at BLUEZ_MAX_FW_WRITE.
    Acquiring the MTU and then writing `mtu - 3` is what corrupted an image;
    acquiring it and writing a capped chunk is a different thing.

    Best-effort and idempotent: a backend without the private method (macOS), an
    MTU that is already known, or a failure all leave the caller exactly where it
    was — on the slow but never-corrupting 20-byte path.
    """
    acquire = getattr(getattr(client, "_backend", None), "_acquire_mtu", None)
    if acquire is None or client.mtu_size > _ATT_DEFAULT_MTU:
        return False
    try:
        await acquire()
    except Exception:
        return False
    return client.mtu_size > _ATT_DEFAULT_MTU


# After a match appears, keep listening this long before committing, purely so two
# devices matching the same prefix are still reported as ambiguous rather than silently
# picking one. The ambiguity check is the reason this function exists rather than
# bleak's own find_device_by_name.
SCAN_SETTLE = 0.5


async def find_device(name_prefix=None, address=None, service_uuid=None,
                      timeout=8.0):
    """One matching BLEDevice, or None.

    Matching is by advertised name prefix and/or service UUID. A receiver whose
    firmware puts its name only in the GAP table and not in the advertisement
    or scan response is invisible to a name match — a central cannot read the
    GAP name without connecting first — so a device that "cannot be found"
    while it is demonstrably advertising is usually that, not a closed window.

    `timeout` is a BOUND on the scan, not its duration: the scan stops as soon as
    something matches (plus SCAN_SETTLE). `BleakScanner.discover()`, which this used
    to call, always sleeps out its whole timeout — measured 2026-09-08, a fugu board
    is first seen 0.31/0.36/0.94 s into a scan, so every push spent the rest of the
    window on nothing. The bound still governs the FAILURE case, so callers holding a
    time-limited resource across the scan (o2p_ble_push.py holds the node's RS-485 bus
    claim) keep their fail-fast behaviour; that is why the default stays at 8 s.
    """
    from bleak import BleakScanner

    if address:
        dev = await BleakScanner.find_device_by_address(address, timeout=timeout)
        if dev is None:
            raise OtaBleError("no BLE device at address %s" % address)
        return dev

    # Match against BOTH names. CoreBluetooth frequently omits the advertised
    # local_name and sets only the cached d.name, and can also serve a stale d.name
    # alongside a current local_name, so neither alone is reliable.
    def _names(dev, adv):
        return [n for n in ((dev.name or ""), (adv.local_name or "")) if n]

    def _matches(dev, adv):
        if not (name_prefix or service_uuid):
            return False
        if name_prefix and not any(n.startswith(name_prefix) for n in _names(dev, adv)):
            return False
        if service_uuid:
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            if service_uuid.lower() not in uuids:
                return False
        return True

    seen, found = {}, asyncio.Event()

    def on_seen(dev, adv):
        seen[dev.address] = (dev, adv)
        if _matches(dev, adv):
            found.set()

    scanner = BleakScanner(detection_callback=on_seen)
    await scanner.start()
    try:
        if name_prefix or service_uuid:
            await asyncio.wait_for(found.wait(), timeout=timeout)
            await asyncio.sleep(SCAN_SETTLE)
        else:
            # Nothing to match, so nothing can end this early.
            await asyncio.sleep(timeout)
    except asyncio.TimeoutError:
        pass  # nothing matched; fall through and report it below
    finally:
        await scanner.stop()

    hits = [dev for dev, adv in seen.values() if _matches(dev, adv)]
    if len(hits) > 1:
        raise OtaBleError(
            "several devices match (%s) — pass an address or a longer name"
            % ", ".join("%s [%s]" % (d.name, d.address) for d in hits))
    return hits[0] if hits else None


class OtaLink:
    """THE LINK CONTRACT: what push_image() and query_info() require of `link`.

    They duck-type it, and that stays true -- nothing here is enforced by
    inheritance, and a link need not subclass this. It is written down because
    the interface was being rediscovered by reading push_image()'s body: fugu
    implements it twice (`BleakLink` and its ESPHome-proxy transport) and the
    farm node once (`NodeLink`), all independently, and a member that only the
    failure path touches -- `disconnected` -- is easy to leave out and hard to
    notice missing.

    Required:

      set_line_handler(fn)   Install fn(line: str) for every complete status
                             line, newline stripped. Replaces any previous
                             handler; the module installs its own for the
                             duration of a call. A status line can straddle two
                             notifications, so reassembly belongs in the link.
      async write_cmd(text)  Send one control line. The link is responsible for
                             the trailing newline if its transport needs one.
      async write_fw(data)   Send firmware bytes, write-WITHOUT-response.
      chunk                  int: the largest `data` write_fw() will take. See
                             usable_chunk() -- getting this wrong corrupts the
                             image silently in one direction and costs 25x
                             throughput in the other.
      disconnected           asyncio.Event, set when the link drops. push_image()
                             waits on it alongside every timeout; a link without
                             one turns every drop into a full timeout and reports
                             the wrong cause.

    Optional, but expected by anything that composes links:

      mtu                    int. Not read by push_image() -- it uses `chunk` --
                             but adapt_link() derives `chunk` from it.
      _on_line               The currently installed handler. query_info()
                             RESTORES rather than clears the caller's handler,
                             and reads this to do it. A link without it still
                             works; the caller's handler is simply not put back,
                             which is silent until its own lines stop arriving.
                             Both links in this module expose it.
    """

    #: What push_image() needs, in the form require_link() checks it.
    REQUIRED = ("set_line_handler", "write_cmd", "write_fw", "chunk", "disconnected")
    #: What query_info() needs. Deliberately smaller: it sends one command and reads the reply, and
    #: demanding a firmware sink it never writes to would reject a perfectly good query-only link.
    REQUIRED_INFO = ("set_line_handler", "write_cmd")

    def set_line_handler(self, fn):
        raise NotImplementedError

    async def write_cmd(self, text):
        raise NotImplementedError

    async def write_fw(self, data):
        raise NotImplementedError

    @property
    def chunk(self):
        raise NotImplementedError

    @property
    def disconnected(self):
        raise NotImplementedError


def require_link(link, what="this call", members=OtaLink.REQUIRED):
    """Fail a link that cannot carry a push, before it carries half of one.

    Presence only -- this cannot check that write_fw() actually sends anything.
    What it does buy is that a missing `disconnected` surfaces as a named error
    at the start rather than as an AttributeError a hundred seconds into a
    transfer, on the failure path, where it is easily misread as the device
    misbehaving.
    """
    missing = [m for m in members if not hasattr(link, m)]
    if missing:
        raise OtaBleError("%s needs a link with %s (see OtaLink); %r provides none of those"
                          % (what, ", ".join(missing), type(link).__name__))
    if "disconnected" in members and not hasattr(link.disconnected, "is_set"):
        raise OtaBleError("%s needs link.disconnected to be an asyncio.Event (see OtaLink)" % what)


class BleOtaLink:
    """A connected receiver: one command channel, one notify, one firmware sink."""

    def __init__(self, cmd_uuid, notify_uuid, fw_uuid, *, chunk=0):
        self.cmd_uuid = cmd_uuid
        self.notify_uuid = notify_uuid
        self.fw_uuid = fw_uuid
        self._chunk_override = int(chunk or 0)
        # False = not looked up yet; None = unavailable on this backend.
        self._cb_periph = False
        # Writes that had to wait for CoreBluetooth's queue. Zero across a whole
        # push means the host never outran the link and the pacing cost nothing.
        self.paced_writes = 0
        self._cli = None
        self.mtu = 23
        # True only once we have made BlueZ report the negotiated MTU; gates the
        # staleness fallback in _max_write() so CoreBluetooth is never affected.
        self._mtu_acquired = False
        self.disconnected = asyncio.Event()
        self._on_line = None
        self._rx = b""

    def set_line_handler(self, fn):
        self._on_line = fn

    def _feed(self, payload):
        # Status is a byte stream, not one line per notification: the device
        # drains its queue at MTU-3 a packet and a line can straddle two.
        self._rx += bytes(payload)
        # BOUNDED. A peer that never sends a newline would otherwise grow this
        # without limit; a status line from this receiver is tens of bytes.
        if len(self._rx) > _RX_MAX:
            self._rx = self._rx[-_RX_MAX:]
        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            text = line.decode("utf-8", "replace").strip()
            if text and self._on_line:
                self._on_line(text)

    async def open(self, device, attempts=3, settle_s=2.0, *, adapter=None):
        from bleak import BleakClient

        last = None
        for attempt in range(1, attempts + 1):
            self._cli = BleakClient(
                device, disconnected_callback=lambda _: self.disconnected.set(),
                **({'adapter':adapter} if adapter else {}))
            try:
                await self._cli.connect()
                await self._cli.start_notify(self.notify_uuid,
                                             lambda _, p: self._feed(p))
                await self._acquire_mtu_if_default()
                self.mtu = self._cli.mtu_size
                self.disconnected.clear()
                return
            except Exception as exc:
                # CoreBluetooth raises CBATTError 17 ("insufficient resources")
                # when a previous link's CCCD was not released. Dropping the
                # link and retrying after a settle clears it.
                last = exc
                try:
                    await self._cli.disconnect()
                except Exception:
                    pass
                self._cli = None
                if attempt < attempts:
                    await asyncio.sleep(settle_s)
        raise OtaBleError("could not establish a BLE link after %d attempts: %s"
                          % (attempts, last))

    @property
    def chunk(self):
        if self._chunk_override:
            return self._chunk_override
        return usable_chunk(self.mtu, max_write=self._max_write())

    async def _acquire_mtu_if_default(self):
        """Acquire the MTU for this link. See module-level acquire_bluez_mtu()."""
        self._mtu_acquired = await acquire_bluez_mtu(self._cli)

    def _max_write(self):
        """The characteristic's maximum write-without-response size, if honest.

        Normally this is better than an OS test, which cannot see through an
        adapted transport at all (fugu tunnels GATT through an ESPHome proxy:
        the data plane is an ESP32 whatever OS the script runs on).

        BlueZ computes it ONCE, at service resolution, from the MTU it had then,
        and acquiring the MTU afterwards does NOT refresh it — measured
        2026-09-08: still 20 while `mtu_size` reported 247. So once we have
        acquired the MTU ourselves, a value below what that MTU allows is stale
        rather than authoritative, and we let `usable_chunk()` derive (and cap)
        from the MTU instead. This narrow condition keeps CoreBluetooth, where
        the characteristic IS authoritative and deliberately reports less than
        `mtu - 3`, on its existing path.
        """
        try:
            ch = self._cli.services.get_characteristic(self.fw_uuid)
            n = getattr(ch, "max_write_without_response_size", None)
            n = int(n) if n else 0
            if n and self._mtu_acquired and n < self.mtu - 3:
                return 0
            return n
        except Exception:
            return 0

    async def write_cmd(self, text):
        if isinstance(text, str):
            text = text.encode()
        line = text
        if not line.endswith(b"\n"):
            line += b"\n"
        # With response: a command that is silently dropped costs the whole
        # transfer, and one acked write per command is free.
        await self._cli.write_gatt_char(self.cmd_uuid, line, response=True)

    def _cb_peripheral(self):
        """CoreBluetooth's CBPeripheral for this link, or None off macOS.

        Reached through bleak internals on purpose: there is no public API for
        the flow-control question below, and the alternative to asking it is
        losing firmware bytes silently. Every step is guarded, so a bleak whose
        internals moved degrades to "no flow control" rather than to an
        exception - the same behaviour as every non-CoreBluetooth backend.
        """
        if self._cb_periph is not False:
            return self._cb_periph
        self._cb_periph = None
        try:
            self._cb_periph = self._cli._backend._delegate.peripheral
        except Exception:
            pass
        return self._cb_periph

    async def _await_writable(self, budget_s=2.0):
        """Block until CoreBluetooth will actually carry the next write.

        WHY THIS EXISTS. A write-without-response is unacknowledged, and bleak's
        CoreBluetooth backend hands it straight to
        `writeValue_forCharacteristic_type_` with NO flow control at all - it
        never consults `canSendWriteWithoutResponse` and never waits for
        `peripheralIsReadyToSendWriteWithoutResponse:` (backends/corebluetooth/
        PeripheralDelegate.py: the `else` branch is one unawaited call). Apple's
        contract is that writes issued while that property is false MAY BE
        DROPPED, and nothing upstream can see it happen: the coroutine returns,
        the transfer looks healthy, and the device is simply short.

        The credit window used to hide this. At 8 KB the host wrote ~16 packets
        and then blocked on credit, which gave the queue time to drain; the
        drops only appear once the window is large enough to let the host run.
        See usable_chunk() for the BlueZ form of the same failure - there the
        mitigation was a smaller chunk, here it is asking before writing.

        A budget rather than an unbounded wait: if the property never comes back
        true the link is dead or the API moved, and the credit/stall machinery
        upstream is the right place to decide that, not a spin here.
        """
        p = self._cb_peripheral()
        if p is None:
            return
        try:
            if p.canSendWriteWithoutResponse():
                return
        except Exception:
            self._cb_periph = None      # API moved; stop asking
            return
        self.paced_writes += 1          # how often the queue was actually full
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            await asyncio.sleep(0.001)
            try:
                if p.canSendWriteWithoutResponse():
                    return
            except Exception:
                return

    async def write_fw(self, data):
        await self._await_writable()
        await self._cli.write_gatt_char(self.fw_uuid, data, response=False)

    async def release(self):
        if self._cli is not None and self._cli.is_connected:
            try:
                await self._cli.disconnect()
            except Exception:
                pass


class adapt_link(object):
    """Wrap a consumer's own transport so push_image() can drive it.

    fugu carries two transports — direct bleak, and a GATT connection tunnelled
    through an ESPHome `bluetooth_proxy` — and both predate this module. They
    share one shape: `set_notify(cb)` handing over raw notification payloads,
    plus `mtu`, `write_cmd`, `write_fw` and a `disconnected` Event. Adapting
    that is a dozen lines; porting a working proxy transport would not be, and
    would put the least-testable path at the most risk.

    The line reassembly lives here rather than in each caller because status is
    a byte stream, not one line per notification: the device drains its queue at
    MTU-3 a packet and a status line can straddle two.
    """

    def __init__(self, link, chunk=0):
        self._link = link
        self._chunk_override = int(chunk or 0)
        self._on_line = None
        self._rx = b""
        link.set_notify(self._feed)

    def _feed(self, payload):
        self._rx += bytes(payload)
        # BOUNDED. A peer that never sends a newline would otherwise grow this
        # without limit; a status line from this receiver is tens of bytes.
        if len(self._rx) > _RX_MAX:
            self._rx = self._rx[-_RX_MAX:]
        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            text = line.decode("utf-8", "replace").strip()
            if text and self._on_line:
                self._on_line(text)

    def set_line_handler(self, fn):
        self._on_line = fn

    @property
    def mtu(self):
        return self._link.mtu

    @property
    def chunk(self):
        return self._chunk_override or usable_chunk(self._link.mtu)

    @property
    def disconnected(self):
        return self._link.disconnected

    async def write_cmd(self, text):
        if isinstance(text, str):
            text = text.encode()
        if not text.endswith(b"\n"):
            text += b"\n"
        await self._link.write_cmd(text)

    async def write_fw(self, data):
        await self._link.write_fw(data)


# ---------------------------------------------------------------- payloads
#
# The BLE link is saturated and cannot be made faster: measured on a fugu board
# 2026-09-08, 384 credit stalls totalling 28.7 s of a ~32 s push against 0.47 s
# spent inside the host's own write call, i.e. 4 KB per 75 ms == 55 kB/s, which
# is exactly the air time for one credit window. A bigger MTU, a bigger window
# and the 2M PHY were all measured and none of them moved it. So the only lever
# left is to send fewer bytes, which is what these build.
#
# Measured on a 1.76 MB ESP32-S3 image, six real build pairs (2026-09-08):
#
#   raw     100 %
#   tamp     69 %        self-contained; the device needs nothing it does not have
#   delta   4.4-10.3 %   needs the EXACT image the device is running as the base
#
# Delta is worth an order of magnitude and is therefore the default whenever the
# base can be found -- hence the image cache below, which exists solely so that
# "what is this device running?" has an answer on the next push.

# Must match TAMP_WINDOW_BITS in src/ota_xform.cpp: the device sizes its window
# buffer at build time and refuses a stream compressed with a larger one. 12 is
# also the measured optimum (69.0 %, against 70.3 % at w=10 and 71.5 % at w=15).
TAMP_WINDOW_BITS = 12

# Container the device expects in front of a detools patch. Identical to the one
# Espressif's esp_delta_ota_patch_gen.py writes, so patches from either tool are
# interchangeable: magic, the base image's SHA-256, reserved padding to 64 bytes.
DELTA_MAGIC = 0xFCCDDE10
DELTA_HEADER_SIZE = 64


# Magic of ESP-IDF's esp_app_desc_t (esp_app_format.h). The struct sits immediately after the
# 24-byte image header plus the 8-byte first segment header, so it is always inside the first
# 512 bytes of a .bin -- but it is FOUND rather than assumed at a fixed offset, because that layout
# is a property of the bootloader's image format and not of this protocol.
APP_DESC_MAGIC = 0xABCD5432


def read_local_app_desc(image):
    """The version string out of a local firmware image's esp_app_desc_t, or None.

    `image` is a path or the image bytes. None means "could not read it" for
    every reason -- missing file, not an ESP app image, no descriptor -- and
    callers use it only to print what they are about to push next to what the
    device says it is running. Do not build a decision on it: it is a label the
    build wrote about itself, not a hash of anything.

    Lifted 2026-09-10 from two byte-identical copies (fugu etc/ota_ble.py,
    node tools/ota_ble_push.py) that had already diverged in spelling.
    """
    if isinstance(image, (bytes, bytearray)):
        head = bytes(image[:0x200])
    else:
        try:
            with open(image, "rb") as f:
                head = f.read(0x200)
        except OSError:
            return None
    off = head.find(struct.pack("<I", APP_DESC_MAGIC))
    if off < 0:
        return None
    return head[off + 0x10:off + 0x30].split(b"\x00", 1)[0].decode("utf-8", "replace")


# The receiver's own status lines, which are present in any image that still contains this module.
# They are the default marker set because they are what a BLE push actually needs to exist on the
# far side; a consumer whose transport needs more (a console, a BLE stack) passes its own.
OTAB_RECEIVER_MARKERS = (b"OTAB CRED", b"OTAB READY")


def image_keeps_the_push_path(data, markers=OTAB_RECEIVER_MARKERS, *, mode="all"):
    """Would flashing this image over BLE destroy the path being used to flash it?

    THE BRICK GUARD, generalised from `image_has_ota()` (node) and
    `image_has_ble()` (fugu). Both asked the same question -- does the image I
    am about to push still contain the transport I would need to push the NEXT
    one? -- and differed only in the marker strings and the quantifier, so both
    are parameters here rather than one project's answer hard-coded.

    `mode` is "all" (every marker must be present; the strict default) or "any"
    (one is enough, for a marker set where the markers are alternatives rather
    than requirements). It is validated rather than defaulted: an unrecognised
    mode raises, because the failure this guard prevents is a device buried in a
    compost pile that can only be recovered with a cable.

    An empty marker set raises for the same reason -- `all(...)` over nothing is
    True, which would turn a mis-wired caller into a silent pass.
    """
    if mode not in ("all", "any"):
        raise ValueError("mode must be 'all' or 'any', not %r" % (mode,))
    markers = tuple(markers)
    if not markers:
        raise ValueError("image_keeps_the_push_path needs at least one marker")
    quantifier = all if mode == "all" else any
    return quantifier(sig in data for sig in markers)


def progress_bar(done, total, label, width=30):
    """One-line terminal progress bar, ending in a newline when it completes.

    Here because both host tools had grown their own byte-for-byte copy. It
    writes to stdout unconditionally; a caller that does not want that should
    pass its own `on_progress` to push_image() instead of using this.
    """
    frac = done / total if total else 0
    filled = int(frac * width)
    sys.stdout.write("\r  %s [%s%s] %5.1f%% %d/%d"
                     % (label, "#" * filled, "-" * (width - filled), frac * 100, done, total))
    sys.stdout.flush()
    if done >= total:
        sys.stdout.write("\n")
        sys.stdout.flush()


def image_id(data):
    """The identity of an ESP32 app image: its own appended SHA-256.

    This is what `esptool image_info` prints as "Validation Hash" and what
    `esp_partition_get_sha256()` returns on the device, so it is the one value
    both sides can compute for the SAME image without exchanging the image.

    Returns None when `data` does not carry an appended hash, rather than
    returning the file's plain sha256 -- a base identity that silently means
    something different on each side is worse than no base at all.
    """
    if len(data) < DELTA_HEADER_SIZE:
        return None
    body, appended = data[:-32], data[-32:]
    if hashlib.sha256(body).digest() != appended:
        return None
    return appended.hex()


def image_cache_dir():
    """Where pushed images are kept so a later delta has a base to patch from."""
    root = os.environ.get("ESP_OTA_BLE_CACHE")
    if not root:
        root = os.path.join(os.path.expanduser("~"), ".cache", "esp-ota-ble", "images")
    return root


def cache_image(data):
    """Store `data` under its image id. Returns the path, or None if unusable.

    Called after a push SUCCEEDS, never before: an image the device rejected is
    not what it is running, and caching it would hand the next delta a base that
    does not exist anywhere.
    """
    iid = image_id(data)
    if not iid:
        return None
    root = image_cache_dir()
    try:
        os.makedirs(root, exist_ok=True)
        path = os.path.join(root, iid + ".bin")
        if not os.path.exists(path):
            # Write-then-rename: a half-written base would produce a patch that
            # only fails on the device, minutes later, as a corrupt image.
            fd, tmp = tempfile.mkstemp(dir=root)
            with os.fdopen(fd, "wb") as f:
                f.write(data)
            os.replace(tmp, path)
        return path
    except OSError:
        return None


def cached_image(iid, extra_dirs=()):
    """Find the image with this id, in the cache or in any of `extra_dirs`.

    `extra_dirs` lets a caller offer its build directories directly, which is
    what makes the very first delta possible on a device the cache has never
    seen -- the running image is usually still sitting in a build tree.
    """
    if not iid:
        return None
    path = os.path.join(image_cache_dir(), iid + ".bin")
    if os.path.exists(path):
        # Verify the CONTENT, not the filename. Anything can end up at a given pathname -- an
        # interrupted write, a hand-copied file, a stale entry -- and a base that is not the image
        # the device is running is exactly what produces a patch that cannot be applied.
        try:
            with open(path, "rb") as f:
                if image_id(f.read()) == iid:
                    return path
        except OSError:
            pass
    for d in extra_dirs:
        try:
            names = os.listdir(d)
        except OSError:
            continue
        for name in names:
            if not name.endswith(".bin"):
                continue
            cand = os.path.join(d, name)
            try:
                with open(cand, "rb") as f:
                    if image_id(f.read()) == iid:
                        return cand
            except OSError:
                continue
    return None


def build_tamp_payload(data, window=TAMP_WINDOW_BITS):
    """Compress a whole image with tamp. Raises ImportError without the package."""
    import tamp
    # The receiver's tamp 1.x decoder rejects the extended-format bit which
    # tamp 2.x enables by default. Keep the original wire format explicitly.
    try:
        payload = tamp.compress(data, window=window, extended=False)
    except TypeError as exc:
        # Older encoders have no `extended` keyword and already emit 1.x.
        if "unexpected keyword argument 'extended'" not in str(exc):
            raise
        payload = tamp.compress(data, window=window)
    if not payload or payload[0] & 0x03:
        raise OtaBleError("tamp encoder did not produce the receiver's legacy wire format")
    return payload


def build_delta_payload(base_path, data, *, verify=True, expect_base=None):
    """A device-ready delta: the 64-byte container plus a heatshrink patch.

    `expect_base` is the image id the DEVICE reported. Pass it whenever you have
    it: without it this trusts whatever is at `base_path`, and a base that is not
    what the device is running is the whole failure mode.

    `verify` re-applies the patch to the base here on the host and checks it
    reproduces `data` byte for byte. It costs a second and it is not optional in
    spirit: a patch is applied against flash the host cannot see, and a bad one
    reconstructs a plausible image that fails only at boot -- on a board whose
    previous firmware has already been erased.
    """
    import detools

    # Read the base ONCE and work from those bytes throughout. Re-opening the path for patch
    # creation and again for verification would let the file change underneath: the container would
    # name the image we hashed while the patch was built against a different one, and host
    # verification would happily confirm the wrong pairing.
    with open(base_path, "rb") as f:
        base = f.read()
    base_id = image_id(base)
    if not base_id:
        raise OtaBleError("base %s carries no appended SHA-256; it is not an app image" % base_path)
    if expect_base and base_id != expect_base:
        raise OtaBleError("base %s is %s, but the device is running %s"
                          % (base_path, base_id[:12], expect_base[:12]))

    with tempfile.TemporaryDirectory() as td:
        base_snap = os.path.join(td, "base.bin")
        new_path = os.path.join(td, "new.bin")
        patch_path = os.path.join(td, "patch.bin")
        with open(base_snap, "wb") as f:
            f.write(base)
        with open(new_path, "wb") as f:
            f.write(data)
        with open(base_snap, "rb") as ffrom, open(new_path, "rb") as fto, \
                open(patch_path, "wb") as fpatch:
            # heatshrink, and its default window/lookahead: that is the one
            # configuration the device's bundled detools decoder is built for.
            detools.create_patch(ffrom, fto, fpatch, compression="heatshrink")
        with open(patch_path, "rb") as f:
            patch = f.read()

        if verify:
            out_path = os.path.join(td, "rebuilt.bin")
            with open(base_snap, "rb") as ffrom, open(patch_path, "rb") as fpatch, \
                    open(out_path, "wb") as fto:
                detools.apply_patch(ffrom, fpatch, fto)
            with open(out_path, "rb") as f:
                if f.read() != data:
                    raise OtaBleError("delta patch does not reproduce the image on the host")

    header = struct.pack("<I", DELTA_MAGIC) + bytes.fromhex(base_id)
    header += b"\x00" * (DELTA_HEADER_SIZE - len(header))
    return header + patch


def choose_payload(data, info, *, prefer="auto", base_dirs=(), on_note=None):
    """Pick the smallest payload this device can actually accept.

    Returns (xform, payload). `info` is what query_info() returned; passing None
    (the device never answered) degrades to raw rather than guessing, because
    every transform depends on something only the device can confirm.

    Falls back rather than failing, and says why through `on_note`: a push that
    refuses to run because a base image is missing is strictly worse than a
    slower push that works.
    """
    def note(msg):
        if on_note:
            on_note(msg)

    supported = set((info or {}).get("xforms") or ())
    if prefer == "raw":
        return "raw", data

    # What actually authenticates a transformed push is the image's OWN appended SHA-256: the wire
    # digest only proves the payload arrived intact, and reconstruction happens on the device where
    # this host cannot check it. esp_ota_end() verifies that appended hash before anything is marked
    # bootable -- but only if the image carries one. Without it the sole integrity left is the ESP
    # image's one-byte checksum, which is not a guard, so refuse to transform such an image at all.
    # image_id() returns None in exactly that case, and a raw push does not depend on it.
    if image_id(data) is None:
        note("target image carries no appended SHA-256; only a raw push can be verified")
        if prefer != "auto":
            raise OtaBleError("transform %r refused: the target image is not hash-verifiable" % prefer)
        return "raw", data

    want = ("delta", "tamp") if prefer == "auto" else (prefer,)

    for xform in want:
        if xform == "delta":
            if "delta" not in supported:
                note("delta: device does not offer it")
                continue
            base_id = (info or {}).get("base")
            if not base_id:
                note("delta: device did not report a base digest")
                continue
            base_path = cached_image(base_id, base_dirs)
            if not base_path:
                note("delta: no local copy of the running image %s..." % base_id[:12])
                continue
            try:
                payload = build_delta_payload(base_path, data, expect_base=base_id)
            except ImportError:
                note("delta: detools not installed")
                continue
            except Exception as exc:
                # Anything at all: a detools internal error, a full temp directory, an API change.
                # Narrower handling let a ValueError out of an `auto` push as a traceback, when the
                # whole point of `auto` is that it ends in a transfer rather than an exception.
                note("delta: %s: %s" % (type(exc).__name__, exc))
                continue
            note("delta against %s" % os.path.basename(base_path))
            return "delta", payload

        if xform == "tamp":
            if "tamp" not in supported:
                note("tamp: device does not offer it")
                continue
            try:
                payload = build_tamp_payload(data)
            except ImportError:
                note("tamp: the tamp package is not installed")
                continue
            except Exception as exc:
                note("tamp: %s: %s" % (type(exc).__name__, exc))
                continue
            return "tamp", payload

    if prefer not in ("auto", "raw"):
        raise OtaBleError("transform %r unavailable and no fallback was requested" % prefer)
    return "raw", data


async def query_info(link, *, cmd_prefix="", timeout=15.0):
    """Ask the device what it is running and what payloads it accepts.

    Returns {"run", "slot", "base", "xforms", "resume_capable", "resume"}, or
    None on an old receiver that does not know the command -- which is not an
    error, it just means raw.

    `resume_capable` is False when the receiver emitted no RESUME line at all,
    which is how a receiver that predates resume identifies itself. `resume` is
    then always None; on a receiver that does know the command it is None when
    there is nothing to continue, and {"off", "size"} when there is. Feed it to
    resume_offset() rather than reading it directly -- the decision needs the
    payload as well.
    """
    require_link(link, "query_info", OtaLink.REQUIRED_INFO)
    got = {}
    seen = asyncio.Event()

    def handle(line):
        if "OTAB FAIL" in line:
            got["refused"] = True
        m = _RE_INFO.search(line)
        if m:
            got["run"], got["slot"] = m.group(1), int(m.group(2))
        m = _RE_BASE.search(line)
        if m:
            got["base"] = None if m.group(1) == "none" else m.group(1).lower()
        m = _RE_RESUME.search(line)
        if m:
            got["resume_capable"] = True
            got["resume"] = (None if m.group(1) is None
                             else {"off": int(m.group(1)), "size": int(m.group(2))})
        m = _RE_XFORM.search(line)
        if m:
            got["xforms"] = tuple(m.group(1).split(","))
            seen.set()
        if "OTAB FAIL" in line:
            seen.set()

    # Restore, do not clear. Clearing would silently disown a handler the caller installed before
    # calling us, and the caller has no way to notice until its own lines stop arriving.
    prev = getattr(link, "_on_line", None)
    link.set_line_handler(handle)
    try:
        await link.write_cmd("%sinfo" % cmd_prefix)
        await asyncio.wait_for(seen.wait(), timeout=timeout)
    except asyncio.TimeoutError:
        # No answer at all. Distinguishable from an explicit refusal below, because "this receiver
        # is too old to know `info`" and "the reply was lost" are different facts about the device
        # even though both end in a raw push.
        return None
    finally:
        link.set_line_handler(prev)
    # XFORM is emitted last, so its absence means the reply never completed.
    if "xforms" in got:
        # Absence of a RESUME line is a FACT about the receiver, not a missing key. Defaulting it
        # here rather than at every read site means a caller cannot accidentally treat "too old to
        # resume" as "resume is available and empty".
        got.setdefault("resume_capable", False)
        got.setdefault("resume", None)
        return got
    return {"refused": True} if got.get("refused") else None


def image_is_running(info, image):
    """Is the device from that `info` reply running exactly `image`? True / False / None.

    push_image() returning True is not proof the new image runs -- its own
    docstring says so at length and then tells every caller to confirm out of
    band, at which point fugu wrote one confirmation and the farm node wrote
    another. This is the part of that job which belongs here: the CRITERION.

    `OTAB BASE` is the running partition's own appended SHA-256, and image_id()
    computes the same number from a local .bin without either side sending the
    image. Equal means the device is running THIS build -- an identity, not an
    inference. Compare that with the weaker checks the same code paths reach
    for: "it advertised again" only shows something rebooted, "the slot number
    changed" only shows something different booted, and an uptime comparison
    shows nothing at all, because a REJECTED image reboots immediately and the
    previous one comes up with uptime zero.

    None is a third answer and not a soft False: no `info` reply, a device that
    cannot hash its running partition (`OTAB BASE none`), or an image with no
    appended hash to compare against. A caller must not report success on it.

    What is deliberately NOT here: the waiting. fugu re-scans for an
    advertisement, and the node waits for a LoRaWAN invitation, reopens a
    session with a nonce and asks over the link the push just used. Those share
    no code and no timing model, and flattening either into the other would cost
    exactly the guarantee it was written for.
    """
    if not info:
        return None
    base = info.get("base")
    if not base:
        return None
    want = image_id(image)
    if want is None:
        return None
    return base.lower() == want.lower()


def resume_offset(info, payload, xform="raw", *, enabled=True, on_note=None):
    """Decide whether this push may continue an interrupted one, and from where.

    Returns a wire offset to hand to push_image(resume_from=...), or 0 to start
    fresh. Either way it says why through `on_note`, because "it resumed" and
    "it started over" are both surprising if you expected the other one.

    Every condition below must hold, and anything this function cannot
    establish is answered "no". A resume is an optimisation; a wrong one costs
    a whole transfer to discover, and the only thing standing between it and a
    spliced image is a digest check at the far end of that transfer.

      * the receiver said it can resume (it emitted an OTAB RESUME line at all),
      * it has a transfer to continue (not "none"),
      * the payload is raw -- under tamp or delta a wire offset is not an image
        offset, and the device's transform state died with the session,
      * the size it recorded is exactly this payload's size. That is what stops
        a resume onto a prefix of a DIFFERENT build: the receiver additionally
        requires the digest to match, and re-hashes the flashed prefix before
        `end`, but the cheapest place to notice is here.

    Note that the offset is the RECEIVER'S number, echoed back untouched. The
    host does not get to choose where to restart -- it has no way to know what
    reached flash, and the receiver refuses any offset but its own.

    Callers that expose a `--no-resume` flag pass `enabled=False`; a forced
    fresh push is the first thing to try when a resume behaves oddly.
    """
    def note(msg):
        if on_note:
            on_note(msg)

    if not enabled:
        note("resume: disabled, starting from 0")
        return 0
    if xform and xform != "raw":
        note("resume: not defined for a %s payload, starting from 0" % xform)
        return 0
    if not info:
        note("resume: the device did not answer `info`, starting from 0")
        return 0
    if not info.get("resume_capable"):
        note("resume: this receiver predates resume, starting from 0")
        return 0
    r = info.get("resume")
    if not r:
        note("resume: the device holds no interrupted transfer")
        return 0
    off, size = r.get("off"), r.get("size")
    if not isinstance(off, int) or not isinstance(size, int):
        note("resume: the device's resume point did not parse, starting from 0")
        return 0
    if size != len(payload):
        note("resume: the device holds %d bytes of a %d-byte transfer, this payload is %d "
             "-- a different image, starting from 0" % (off, size, len(payload)))
        return 0
    if not 0 < off < size:
        note("resume: offset %d is not inside a %d-byte payload, starting from 0" % (off, size))
        return 0
    note("resume: continuing at %d of %d bytes (%.0f %% already in the slot)"
         % (off, size, 100.0 * off / size))
    return off


async def push_image(link, data, *, sha=None, cmd_prefix="", on_line=None,
                     on_progress=None, xform=None, out_size=None, resume_from=0,
                     ready_timeout=60.0, credit_timeout=20.0,
                     credit_retries=3, flush_timeout=60.0, pace_s=0.0):
    """Run `begin`/stream/`end` against an already-open link.

    `data` is always the WIRE payload -- the bytes that cross the link -- and
    every number in this function (size, digest, credit, progress) is about
    those bytes and nothing else. With `xform` set to "tamp" or "delta", that
    payload is no longer the image, so `out_size` must give the size of the
    image it reconstructs to; the device needs it to erase the right amount of
    flash up front and to check the result is the length it was promised. Use
    choose_payload() to build the pair.

    Returns True if the device accepted the image, WHICH IS NOT THE SAME as the
    new image running - see the note at the `end` write. Every caller needs its
    own out-of-band confirmation.

    The device grants credit and the host must never write past it; that is the
    receiver's only flow control and its staging ring is what it protects.

    `resume_from` continues an interrupted transfer at that wire offset instead
    of beginning at 0. It must be the offset the receiver itself reported --
    use resume_offset() to get it. A receiver that refuses the resume, for any
    reason at all including being too old to know the command, is not an error:
    this falls back to a fresh `begin` and the whole image goes over. That
    fallback is why resume never has to be negotiated perfectly.
    """
    require_link(link, "push_image")
    sha = sha or hashlib.sha256(data).hexdigest()
    total = len(data)
    state = {"granted": 0, "ok": False, "fail": None, "full": False}
    ready = asyncio.Event()
    credit = asyncio.Event()
    full = asyncio.Event()
    done = asyncio.Event()

    def handle(line):
        if on_line:
            on_line(line)
        if _RE_READY.search(line):
            ready.set()
        m = _RE_CRED.search(line)
        if m:
            state["granted"] = int(m.group(1))
            credit.set()
        m = _RE_PROG.search(line)
        if m:
            # BOTH fields, exactly. Accepting any numerator >= total and
            # ignoring the denominator made a synthetic "OTAB PROG 9/999" on an
            # 8-byte transfer count as a complete flush and send `end`.
            if int(m.group(1)) == total and int(m.group(2)) == total:
                state["full"] = True
                full.set()
        if "OTAB OK" in line:
            state["ok"] = True
            done.set()
        elif "OTAB FAIL" in line:
            state["fail"] = line
            done.set()

    link.set_line_handler(handle)

    async def arm(cmd):
        """Send one arming command and wait for READY. True if the device armed."""
        await link.write_cmd(cmd)
        # The wait is for a partition erase, not for a round trip.
        waits = [asyncio.create_task(ready.wait()),
                 asyncio.create_task(done.wait()),
                 asyncio.create_task(link.disconnected.wait())]
        await asyncio.wait(waits, timeout=ready_timeout,
                           return_when=asyncio.FIRST_COMPLETED)
        for t in waits:
            t.cancel()
        return ready.is_set()

    if resume_from:
        if xform and xform != "raw":
            raise OtaBleError("resume is only defined for a raw payload, not %r" % xform)
        if not 0 < resume_from < total:
            raise OtaBleError("resume offset %d is not inside a %d-byte payload"
                              % (resume_from, total))
        if not await arm("%sresume %d %d %s" % (cmd_prefix, resume_from, total, sha)):
            # Every refusal ends in the same place: a fresh push. `OTAB FAIL
            # bad-command` is an old receiver, `resume-none` / `resume-mismatch`
            # a new one that will not stand behind the prefix in its slot, and a
            # silence is a lost reply -- none of them is a reason to fail a push
            # that can simply send the whole image. The line is kept in the log
            # because "it silently did not resume" is the confusing outcome.
            if on_line:
                on_line("resume refused (%s); starting from 0"
                        % (state["fail"] or "no OTAB READY"))
            state["fail"] = None
            state["granted"] = 0
            done.clear()
            credit.clear()
            resume_from = 0

    if not resume_from:
        if xform and xform != "raw":
            if not out_size:
                raise OtaBleError("xform %r needs out_size (the reconstructed image size)" % xform)
            armed = await arm("%sbegin %d %s %s %d" % (cmd_prefix, total, sha, xform, out_size))
        else:
            # Byte-for-byte the original two-argument command, so a receiver that
            # predates transforms sees nothing new.
            armed = await arm("%sbegin %d %s" % (cmd_prefix, total, sha))
        if state["fail"]:
            raise OtaBleError("device refused begin: %s" % state["fail"])
        if not armed:
            raise OtaBleError("no OTAB READY within %.0f s" % ready_timeout)

    chunk = link.chunk
    # The already-flashed prefix is not re-sent, and is not re-hashed here either: the receiver
    # reads it back out of the slot to rebuild the digest, so `end` still checks a SHA-256 over the
    # whole payload -- see doc/resume.md.
    sent = resume_from
    while sent < total:
        if sent >= state["granted"]:
            stalled = 0
            while sent >= state["granted"]:
                if link.disconnected.is_set():
                    raise OtaBleError("link dropped at %d/%d" % (sent, total))
                if state["fail"]:
                    raise OtaBleError(state["fail"])
                credit.clear()
                try:
                    await asyncio.wait_for(credit.wait(), timeout=credit_timeout)
                except asyncio.TimeoutError:
                    stalled += 1
                    if stalled >= credit_retries:
                        raise OtaBleError("no credit at %d/%d" % (sent, total))
            continue
        n = min(chunk, state["granted"] - sent, total - sent)
        await link.write_fw(data[sent:sent + n])
        sent += n
        if on_progress:
            on_progress(sent, total)
        if pace_s:
            await asyncio.sleep(pace_s)

    # The image must be ON FLASH before `end`, not merely written to the link:
    # write-without-response packets can still be in flight and the device
    # would see a short image and reject it.
    if not state["full"]:
        # Wait on the failure paths too. Waiting only on `full` meant an
        # OTAB FAIL that had already arrived, or a link that had already
        # dropped, still cost the caller the whole timeout and then reported
        # the generic "never reported the full image flushed" instead of the
        # receiver's actual error.
        waits = [asyncio.create_task(full.wait()),
                 asyncio.create_task(done.wait()),
                 asyncio.create_task(link.disconnected.wait())]
        await asyncio.wait(waits, timeout=flush_timeout,
                           return_when=asyncio.FIRST_COMPLETED)
        for t in waits:
            t.cancel()
        if state["fail"]:
            raise OtaBleError(state["fail"])
        if link.disconnected.is_set():
            raise OtaBleError("link dropped before the image was flushed")
        if not state["full"]:
            raise OtaBleError("device never reported the full image flushed")

    await link.write_cmd("%send" % cmd_prefix)
    # A DROPPED LINK HERE IS TREATED AS SUCCESS, AND THAT IS A GUESS. On a real
    # success the device reboots immediately after queuing OTAB OK, so the
    # notification usually never drains and there is nothing else to wait for.
    #
    # But `PROG total/total` only proves the flash WRITES finished. The digest
    # check, the length check and esp_ota_set_boot_partition() all happen later,
    # inside otaBleEnd(). So a target that loses power, watchdog-resets, or
    # loses receiver state after the `end` write is acknowledged but before the
    # consumer tick executes it produces exactly this signature - and this
    # returns True while no boot slot was ever selected.
    #
    # The converse is possible too: if the receiver latches `end` but its ATT
    # response is lost, write_cmd() above raises on a transfer that then
    # completes.
    #
    # THE CALLER MUST NOT TREAT True AS PROOF THE NEW IMAGE IS RUNNING. Confirm
    # out of band: the farm node reads its running OTA slot over LoRaWAN and
    # requires it to have CHANGED; fugu and smart-shunt require the device to
    # advertise again, which is weaker but is at least evidence it rebooted.
    waits = [asyncio.create_task(done.wait()),
             asyncio.create_task(link.disconnected.wait())]
    await asyncio.wait(waits, timeout=35, return_when=asyncio.FIRST_COMPLETED)
    for t in waits:
        t.cancel()
    if state["fail"]:
        raise OtaBleError(state["fail"])
    return state["ok"] or link.disconnected.is_set()
