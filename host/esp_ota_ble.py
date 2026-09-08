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
import tempfile

__all__ = [
    "OtaBleError", "find_device", "BleOtaLink", "adapt_link", "usable_chunk",
    "acquire_bluez_mtu",
    "push_image", "query_info",
    "image_id", "build_tamp_payload", "build_delta_payload", "choose_payload",
    "image_cache_dir", "cache_image", "cached_image",
    "TAMP_WINDOW_BITS", "DELTA_MAGIC", "DELTA_HEADER_SIZE",
]

# The receiver's own status vocabulary (src/ota_ble.cpp).
_RE_READY = re.compile(r"OTAB READY\b")
_RE_CRED = re.compile(r"OTAB CRED (\d+)")
_RE_PROG = re.compile(r"OTAB PROG (\d+)/(\d+)")
_RE_INFO = re.compile(r"OTAB INFO run=(\S+) slot=(\d+)")
_RE_BASE = re.compile(r"OTAB BASE ([0-9a-fA-F]{64}|none)")
_RE_XFORM = re.compile(r"OTAB XFORM (\S+)")

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


async def find_device(name_prefix=None, address=None, service_uuid=None,
                      timeout=8.0):
    """One matching BLEDevice, or None.

    Matching is by advertised name prefix and/or service UUID. A receiver whose
    firmware puts its name only in the GAP table and not in the advertisement
    or scan response is invisible to a name match — a central cannot read the
    GAP name without connecting first — so a device that "cannot be found"
    while it is demonstrably advertising is usually that, not a closed window.
    """
    from bleak import BleakScanner

    if address:
        dev = await BleakScanner.find_device_by_address(address, timeout=timeout)
        if dev is None:
            raise OtaBleError("no BLE device at address %s" % address)
        return dev

    items = await BleakScanner.discover(timeout=timeout, return_adv=True)
    hits = []
    for dev, adv in items.values():
        name = dev.name or adv.local_name or ""
        if name_prefix and not name.startswith(name_prefix):
            continue
        if service_uuid:
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            if service_uuid.lower() not in uuids:
                continue
        if name_prefix or service_uuid:
            hits.append(dev)
    if len(hits) > 1:
        raise OtaBleError(
            "several devices match (%s) — pass an address or a longer name"
            % ", ".join("%s [%s]" % (d.name, d.address) for d in hits))
    return hits[0] if hits else None


class BleOtaLink:
    """A connected receiver: one command channel, one notify, one firmware sink."""

    def __init__(self, cmd_uuid, notify_uuid, fw_uuid, *, chunk=0):
        self.cmd_uuid = cmd_uuid
        self.notify_uuid = notify_uuid
        self.fw_uuid = fw_uuid
        self._chunk_override = int(chunk or 0)
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

    async def open(self, device, attempts=3, settle_s=2.0):
        from bleak import BleakClient

        last = None
        for attempt in range(1, attempts + 1):
            self._cli = BleakClient(
                device, disconnected_callback=lambda _: self.disconnected.set())
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

    async def write_fw(self, data):
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
        return path
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
    return tamp.compress(data, window=window)


def build_delta_payload(base_path, data, *, verify=True):
    """A device-ready delta: the 64-byte container plus a heatshrink patch.

    `verify` re-applies the patch to the base here on the host and checks it
    reproduces `data` byte for byte. It costs a second and it is not optional in
    spirit: a patch is applied against flash the host cannot see, and a bad one
    reconstructs a plausible image that fails only at boot -- on a board whose
    previous firmware has already been erased.
    """
    import detools

    with open(base_path, "rb") as f:
        base = f.read()
    base_id = image_id(base)
    if not base_id:
        raise OtaBleError("base %s carries no appended SHA-256; it is not an app image" % base_path)

    with tempfile.TemporaryDirectory() as td:
        new_path = os.path.join(td, "new.bin")
        patch_path = os.path.join(td, "patch.bin")
        with open(new_path, "wb") as f:
            f.write(data)
        with open(base_path, "rb") as ffrom, open(new_path, "rb") as fto, \
                open(patch_path, "wb") as fpatch:
            # heatshrink, and its default window/lookahead: that is the one
            # configuration the device's bundled detools decoder is built for.
            detools.create_patch(ffrom, fto, fpatch, compression="heatshrink")
        with open(patch_path, "rb") as f:
            patch = f.read()

        if verify:
            out_path = os.path.join(td, "rebuilt.bin")
            with open(base_path, "rb") as ffrom, open(patch_path, "rb") as fpatch, \
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
                payload = build_delta_payload(base_path, data)
            except ImportError:
                note("delta: detools not installed")
                continue
            except OtaBleError as exc:
                note("delta: %s" % exc)
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
            return "tamp", payload

    if prefer not in ("auto", "raw"):
        raise OtaBleError("transform %r unavailable and no fallback was requested" % prefer)
    return "raw", data


async def query_info(link, *, cmd_prefix="", timeout=15.0):
    """Ask the device what it is running and what payloads it accepts.

    Returns {"run", "slot", "base", "xforms"}, or None on an old receiver that
    does not know the command -- which is not an error, it just means raw.
    """
    got = {}
    seen = asyncio.Event()

    def handle(line):
        m = _RE_INFO.search(line)
        if m:
            got["run"], got["slot"] = m.group(1), int(m.group(2))
        m = _RE_BASE.search(line)
        if m:
            got["base"] = None if m.group(1) == "none" else m.group(1).lower()
        m = _RE_XFORM.search(line)
        if m:
            got["xforms"] = tuple(m.group(1).split(","))
            seen.set()
        if "OTAB FAIL" in line:
            seen.set()

    link.set_line_handler(handle)
    try:
        await link.write_cmd("%sinfo" % cmd_prefix)
        await asyncio.wait_for(seen.wait(), timeout=timeout)
    except asyncio.TimeoutError:
        return None
    finally:
        # Leaving it installed would let this closure keep eating the lines the
        # caller's own handler is about to be waiting for.
        link.set_line_handler(None)
    # XFORM is emitted last, so its absence means the reply never completed.
    return got if "xforms" in got else None


async def push_image(link, data, *, sha=None, cmd_prefix="", on_line=None,
                     on_progress=None, xform=None, out_size=None,
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
    """
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

    if xform and xform != "raw":
        if not out_size:
            raise OtaBleError("xform %r needs out_size (the reconstructed image size)" % xform)
        await link.write_cmd("%sbegin %d %s %s %d" % (cmd_prefix, total, sha, xform, out_size))
    else:
        # Byte-for-byte the original two-argument command, so a receiver that
        # predates transforms sees nothing new.
        await link.write_cmd("%sbegin %d %s" % (cmd_prefix, total, sha))
    # The wait is for a partition erase, not for a round trip.
    waits = [asyncio.create_task(ready.wait()),
             asyncio.create_task(done.wait()),
             asyncio.create_task(link.disconnected.wait())]
    await asyncio.wait(waits, timeout=ready_timeout,
                       return_when=asyncio.FIRST_COMPLETED)
    for t in waits:
        t.cancel()
    if state["fail"]:
        raise OtaBleError("device refused begin: %s" % state["fail"])
    if not ready.is_set():
        raise OtaBleError("no OTAB READY within %.0f s" % ready_timeout)

    chunk = link.chunk
    sent = 0
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
