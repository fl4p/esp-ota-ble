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
import platform
import re

__all__ = [
    "OtaBleError", "find_device", "BleOtaLink", "adapt_link", "usable_chunk",
    "push_image",
]

# The receiver's own status vocabulary (src/ota_ble.cpp).
_RE_READY = re.compile(r"OTAB READY\b")
_RE_CRED = re.compile(r"OTAB CRED (\d+)")
_RE_PROG = re.compile(r"OTAB PROG (\d+)/(\d+)")

_MACOS = platform.system() == "Darwin"

# Cap on un-terminated status bytes held per link. Status lines are tens of
# bytes; anything approaching this is a peer that will never send a newline.
_RX_MAX = 64 * 1024

# The largest firmware write this module will issue on BlueZ, whatever the MTU
# says. See usable_chunk() for why this number exists and how it was arrived at.
BLUEZ_MAX_FW_WRITE = 400


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

    Do NOT "fix" this by calling bleak's `_acquire_mtu()` and trusting the
    result. On BlueZ, bleak reports the 23-byte default until the MTU is
    acquired, which is exactly why the two older tools chunk at 20 bytes there
    and have never corrupted an image. Acquiring the MTU to go faster is what
    introduced the failure.
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

    def _max_write(self):
        """The characteristic's own maximum write-without-response size.

        This is the authoritative number and the OS is not: bleak documents
        BlueZ's `mtu_size` as always 23 until the MTU is acquired, so deriving
        the chunk from it silently yields 20 bytes. That is not a safety
        margin, it is a 25x throughput loss dressed as one - the measured
        1.4 kB/s on this path was 20-byte writes, not the 400 the cap implies.
        """
        try:
            ch = self._cli.services.get_characteristic(self.fw_uuid)
            n = getattr(ch, "max_write_without_response_size", None)
            return int(n) if n else 0
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


async def push_image(link, data, *, sha=None, cmd_prefix="", on_line=None,
                     on_progress=None,
                     ready_timeout=60.0, credit_timeout=20.0,
                     credit_retries=3, flush_timeout=60.0, pace_s=0.0):
    """Run `begin`/stream/`end` against an already-open link.

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
