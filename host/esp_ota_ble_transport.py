"""Direct OTA transports and explicit throughput controls. No global Bleak shadowing."""
import asyncio
from dataclasses import dataclass
import math
import sys
import time

import esp_ota_ble as O

TESTED_UB500 = "AC:A7:F1:83:27:AD"


@dataclass(frozen=True)
class Options:
    backend: str = "bleak"
    adapter: str = None
    chunk: int = 0
    interval_ms: float = None
    phy: int = None
    experimental_hci_packet_size: int = None

    def __post_init__(self):
        if self.backend not in ("bleak", "bumble", "native"):
            raise ValueError("unknown BLE backend")
        if type(self.chunk) is not int or not 0 <= self.chunk <= 512:
            raise ValueError("chunk must be 0 (automatic) or 1..512")
        if self.interval_ms is not None:
            n = self.interval_ms
            if not math.isfinite(n) or not 7.5 <= n <= 4000 or n / 1.25 != round(n / 1.25):
                raise ValueError("BLE interval must be 7.5..4000 ms in 1.25 ms steps")
        if self.phy is not None and (type(self.phy) is not int or self.phy not in (1, 2)):
            raise ValueError("PHY must be 1 or 2")
        if self.backend != "bumble" and (self.interval_ms is not None or self.phy is not None):
            raise ValueError("host interval/PHY controls require --ble-backend bumble")
        if self.experimental_hci_packet_size is not None:
            if self.experimental_hci_packet_size != 251 or self.backend != "bumble" or not self.adapter:
                raise ValueError("HCI experiment requires Bumble, explicit adapter and size 251")
        if self.backend == "native" and self.adapter:
            raise ValueError("native CoreBluetooth does not select a USB/HCI adapter")


def add_arguments(parser, *, chunk=True):
    parser.add_argument("--ble-backend", choices=("bleak", "bumble", "native"), default="bleak",
                        help="direct transport: OS Bleak, Linux Bumble, or native Mac sender")
    parser.add_argument("--adapter", help="Bluetooth controller (Bumble: prefer its MAC; BlueZ: hciN)")
    if chunk:
        parser.add_argument("--chunk", "--ble-chunk", type=int, default=0,
                            help="firmware write size, bounded by characteristic capacity")
    parser.add_argument("--ble-interval-ms", type=float,
                        help="Bumble connection interval in 1.25 ms steps; verified after READY")
    parser.add_argument("--ble-phy", type=int, choices=(1, 2), help="Bumble PHY request")
    parser.add_argument("--experimental-hci-packet-size", type=int, choices=(251,),
                        help="EXPERIMENTAL: exceed the tested UB500's reported HCI size; keeps credits")


def from_arguments(args):
    return Options(backend=getattr(args, "ble_backend", "bleak"),
                   adapter=getattr(args, "adapter", None), chunk=getattr(args, "chunk", 0),
                   interval_ms=getattr(args, "ble_interval_ms", None), phy=getattr(args, "ble_phy", None),
                   experimental_hci_packet_size=getattr(args, "experimental_hci_packet_size", None))


def default_pace_ms(options):
    return 15.0 if options.backend == "bleak" and sys.platform.startswith("linux") else 0.0


class DirectLink(O.BleOtaLink):
    def __init__(self, cmd_uuid, notify_uuid, fw_uuid, *, options=None, on_note=print):
        self.options = options or Options()
        super().__init__(cmd_uuid, notify_uuid, fw_uuid, chunk=self.options.chunk)
        self.note = on_note
        self._keeper = None
        self._queue = None
        self._original_packet_size = None
        self._flow = asyncio.Event()
        self._flow_callback = self._flow.set
        self._capacity = 0
        self._owns_client = True

    async def attach(self, client, *, disconnected):
        """Borrow an authenticated client; owner must forward CTRL bytes to feed_notification.

        Caller serializes OTA against other commands for the whole borrow,
        retains the backend, and supplies its actual disconnect event. This
        method neither connects nor replaces subscriptions. release() restores
        experimental queue settings but does not disconnect the owner's client.
        """
        if self._cli is not None or self.options.backend == "native":
            raise O.OtaBleError("attach needs an unused Bleak/Bumble link")
        if not isinstance(disconnected, asyncio.Event) or disconnected.is_set() or not client.is_connected:
            raise O.OtaBleError("attach needs a connected client and its live disconnect event")
        self._owns_client = False
        self._cli = client
        self.disconnected = disconnected
        try:
            await self._acquire_mtu_if_default()
            self.mtu = client.mtu_size
            await self._configure_capacity()
            if self.options.backend == "bumble":
                await self._configure_bumble()
        except BaseException:
            await self.release()
            raise

    def feed_notification(self, payload):
        """Feed CTRL notification bytes while borrowing an owner's subscription."""
        self._feed(payload)

    def scanner_class(self):
        if self.options.backend == "bumble":
            if not sys.platform.startswith("linux"):
                raise O.OtaBleError("Bumble transport requires Linux")
            from bumble_bleak import BleakScanner
        else:
            from bleak import BleakScanner
        return BleakScanner

    async def scan(self, finder, *args, **kwargs):
        if self.options.backend == "bumble" and self._keeper is None:
            from bumble_bleak._backend import get_backend
            self._keeper = await get_backend(self.options.adapter)
            try:
                await self._keeper.acquire()
            except BaseException:
                keeper, self._keeper = self._keeper, None
                try:
                    await keeper.release()
                except Exception as cleanup:
                    self.note("BLE backend cleanup failed: %s" % cleanup)
                raise
        try:
            device = await finder(*args, scanner_cls=self.scanner_class(),
                                  adapter=self.options.adapter, **kwargs)
            if device is None:
                await self.release()
            return device
        except BaseException:
            await self.release()
            raise

    async def find_device(self, **kwargs):
        return await self.scan(O.find_device, **kwargs)

    async def open(self, device, attempts=3, settle_s=2.0):
        if not self._owns_client:
            raise O.OtaBleError("a borrowed link cannot open a second connection")
        if device is None:
            raise O.OtaBleError("BLE device was not discovered")
        if self.options.backend == "bumble":
            from bumble_bleak import BleakClient
        elif self.options.backend == "native":
            from esp_ota_ble_native import NativeClient
            BleakClient = NativeClient
        else:
            from bleak import BleakClient
        last = None
        for attempt in range(attempts):
            try:
                self._rx = b""
                self._mtu_acquired = False
                kwargs = dict(disconnected_callback=lambda _: self.disconnected.set())
                if self.options.adapter:
                    kwargs["adapter"] = self.options.adapter
                if self.options.backend == "native":
                    kwargs.update(cmd_uuid=self.cmd_uuid, notify_uuid=self.notify_uuid, fw_uuid=self.fw_uuid)
                self._cli = BleakClient(device, **kwargs)
                await self._cli.connect()
                await self._cli.start_notify(self.notify_uuid, lambda _, p: self._feed(p))
                await self._acquire_mtu_if_default()
                self.mtu = self._cli.mtu_size
                self.disconnected.clear()
                await self._configure_capacity()
                if self.options.backend == "bumble":
                    await self._configure_bumble()
                self.note("BLE backend=%s adapter=%s mtu=%s max_write=%s chunk=%s" %
                          (self.options.backend, self.options.adapter or "default", self.mtu,
                           self._capacity, self.chunk))
                return
            except BaseException as exc:
                last = exc
                try:
                    await self.release()
                except Exception as cleanup:
                    self.note("BLE cleanup failed: %s" % cleanup)
                if not isinstance(exc, Exception) or isinstance(exc, (ValueError, O.OtaBleError)):
                    raise
                if attempt + 1 < attempts:
                    await asyncio.sleep(settle_s)
        raise O.OtaBleError("BLE connection failed: %s" % last)

    async def _configure_capacity(self):
        char = self._cli.services.get_characteristic(self.fw_uuid)
        if char is None or "write-without-response" not in char.properties:
            raise O.OtaBleError("firmware characteristic lacks write-without-response")
        if self.options.backend == "bumble":
            # ATT attribute values have a 512-byte maximum. These receiver
            # characteristics accept that capacity; the facade omits its getter.
            mtu = self._cli._connection.att_mtu
            if self.mtu != mtu or not 23 <= mtu <= 517:
                raise O.OtaBleError("Bumble ATT MTU is unverified")
            self._capacity = min(512, mtu - 3)
        else:
            capacity = getattr(char, "max_write_without_response_size", 0)
            if self._mtu_acquired and capacity == 20 and self.mtu > 23:
                capacity = min(400, self.mtu - 3)
            if type(capacity) is not int or not 1 <= capacity <= 512:
                raise O.OtaBleError("firmware characteristic write capacity is unverified")
            self._capacity = capacity
        if self.options.chunk > self._capacity:
            raise O.OtaBleError("requested chunk exceeds characteristic capacity")

    @property
    def chunk(self):
        if not self._capacity:
            raise O.OtaBleError("write capacity is not established")
        return self.options.chunk or min(244, self._capacity)

    async def _configure_bumble(self):
        from bumble.hci import HCI_LE_Read_Buffer_Size_Command
        conn = self._cli._connection
        self._queue = conn.device.host.get_data_packet_queue(conn.handle)
        self._queue.on("flow", self._flow_callback)
        buffers = await conn.device.host.send_sync_command(HCI_LE_Read_Buffer_Size_Command())
        reported = buffers.le_acl_data_packet_length
        credits = buffers.total_num_le_acl_data_packets
        self.note("HCI reported_packet_bytes=%s reported_credits=%s host_packet_bytes=%s host_credits=%s" %
                  (reported, credits, self._queue.max_packet_size, self._queue.max_in_flight))
        if self.options.experimental_hci_packet_size is not None:
            address = conn.device.public_address.to_string(False).upper()
            if address != TESTED_UB500 or reported != 27 or credits != 8:
                raise O.OtaBleError("HCI experiment only validated for UB500 %s reporting 27/8" % TESTED_UB500)
            if len(conn.device.connections) != 1 or self._queue.max_in_flight != credits:
                raise O.OtaBleError("HCI experiment requires an exclusive connection and unchanged credits")
            await self._wait_queue(1)
            self._original_packet_size = self._queue.max_packet_size
            self._queue.max_packet_size = 251
            self.note("EXPERIMENTAL NONSTANDARD HCI packet override: reported=27 selected=251 credits=8")

    async def _wait_queue(self, limit=256):
        if self._queue is None:
            return
        deadline = time.monotonic() + 5
        while True:
            self._flow.clear()
            if self.disconnected.is_set():
                raise O.OtaBleError("disconnected while waiting for HCI completion")
            if self._queue.pending < limit:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise O.OtaBleError("HCI completion timeout")
            await asyncio.wait_for(self._flow.wait(), remaining)

    async def prepare_transfer(self):
        # Called after READY: the receiver may itself switch connection
        # parameters when OTA begins. No firmware data has been sent yet.
        if self.options.backend != "bumble":
            return
        from bumble.hci import Phy
        conn = self._cli._connection
        if self.options.phy is not None:
            await conn.set_data_length(251, 2120)
            await conn.set_phy([Phy(self.options.phy)], [Phy(self.options.phy)])
        interval = self.options.interval_ms
        if interval is not None:
            await conn.update_parameters(interval, interval, 0, max(2000, interval * 3))
        deadline = time.monotonic() + 3
        while True:
            phy = await conn.get_phy() if self.options.phy is not None else None
            interval_ok = interval is None or (conn.parameters.connection_interval == interval
                                               and conn.parameters.peripheral_latency == 0)
            phy_ok = phy is None or (int(phy.tx_phy) == self.options.phy and int(phy.rx_phy) == self.options.phy)
            if interval_ok and phy_ok:
                break
            if time.monotonic() >= deadline:
                raise O.OtaBleError("requested BLE interval/PHY was not negotiated")
            await asyncio.sleep(.05)
        self.note("BLE transfer link: %s PHY=%s" % (conn.parameters, phy or "unchanged"))

    async def write_fw(self, data):
        if not self._cli or self.disconnected.is_set() or not 0 < len(data) <= self.chunk:
            raise O.OtaBleError("disconnected or invalid firmware write length")
        await self._wait_queue()
        if self.options.backend == "bleak":
            peripheral = getattr(getattr(getattr(self._cli, "_backend", None), "_delegate", None), "peripheral", None)
            if peripheral is not None:
                deadline = time.monotonic() + 2
                while not peripheral.canSendWriteWithoutResponse():
                    if self.disconnected.is_set() or time.monotonic() >= deadline:
                        raise O.OtaBleError("CoreBluetooth remained unwritable")
                    await asyncio.sleep(.001)
        await self._cli.write_gatt_char(self.fw_uuid, data, response=False)

    async def write_cmd(self, text):
        if text == "ramend" or text == b"ramend":
            await self._wait_queue(1)
        await super().write_cmd(text)

    async def release(self):
        try:
            if self._cli is not None and self._owns_client:
                await self._cli.disconnect()
        finally:
            self._cli = None
            self._capacity = 0
            if self._owns_client:
                self.disconnected.set()
            if self._queue is not None:
                self._queue.remove_listener("flow", self._flow_callback)
                if self._original_packet_size is not None:
                    self._queue.max_packet_size = self._original_packet_size
                self._queue = None
                self._original_packet_size = None
            if self._keeper is not None:
                keeper, self._keeper = self._keeper, None
                await keeper.release()
