"""Bleak-shaped client for the native CoreBluetooth write queue helper."""
import asyncio
import base64
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import uuid


class NativeClient:
    def __init__(self, device, *, cmd_uuid, notify_uuid, fw_uuid, disconnected_callback):
        if sys.platform != "darwin":
            raise RuntimeError("native CoreBluetooth requires macOS and Xcode command line tools")
        self.address = str(uuid.UUID(device.address))
        self.cmd_uuid, self.notify_uuid, self.fw_uuid = cmd_uuid, notify_uuid, fw_uuid
        self._disconnected = disconnected_callback
        self._notify = None
        self._pending = {}
        self._sequence = 0
        self._process = None
        self._reader = None
        self._temp = None
        self.is_connected = False
        self.mtu_size = 23
        self.services = SimpleNamespace(get_characteristic=lambda _: None)

    async def connect(self):
        self._temp = tempfile.TemporaryDirectory(prefix="esp-ota-ble-native-")
        binary = Path(self._temp.name) / "sender"
        source = Path(__file__).with_name("esp_ota_ble_native.swift")
        compiler = await asyncio.create_subprocess_exec(
            "xcrun", "swiftc", "-O", str(source), "-o", str(binary),
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        try:
            output, _ = await asyncio.wait_for(compiler.communicate(), 60)
        except BaseException:
            compiler.kill()
            await compiler.wait()
            raise
        if compiler.returncode:
            raise RuntimeError("native sender compilation failed: " + output.decode(errors="replace"))
        config = dict(address=self.address, cmd=self.cmd_uuid, notify=self.notify_uuid, fw=self.fw_uuid)
        self._pending[0] = asyncio.get_running_loop().create_future()
        self._process = await asyncio.create_subprocess_exec(
            str(binary), json.dumps(config), stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
        self._reader = asyncio.create_task(self._read())
        ready = await asyncio.wait_for(self._pending[0], 30)
        capacity = ready["max_write"]
        if type(capacity) is not int or not 1 <= capacity <= 512:
            raise RuntimeError("invalid native write capacity")
        self.mtu_size = capacity + 3  # CoreBluetooth exposes value capacity, not an ATT MTU getter.
        char = SimpleNamespace(properties=["write-without-response"], max_write_without_response_size=capacity)
        self.services = SimpleNamespace(get_characteristic=lambda key: char if key == self.fw_uuid else None)
        self.is_connected = True

    async def _read(self):
        error = RuntimeError("native BLE sender exited")
        try:
            while line := await self._process.stdout.readline():
                item = json.loads(line)
                if item.get("event") == "notify" and self._notify:
                    self._notify(None, base64.b64decode(item["data"], validate=True))
                elif item.get("event") == "error":
                    raise RuntimeError(item["message"])
                elif item.get("event") == "disconnected":
                    print("Native BLE:", item, flush=True)
                    break
                elif "id" in item:
                    future = self._pending.get(item["id"])
                    if future is not None and not future.done():
                        future.set_result(item)
        except Exception as exc:
            error = exc
        finally:
            self.is_connected = False
            for future in self._pending.values():
                if not future.done():
                    future.set_exception(error)
            self._disconnected(self)

    async def start_notify(self, characteristic, callback):
        if characteristic != self.notify_uuid:
            raise ValueError("native notification UUID mismatch")
        self._notify = callback

    async def write_gatt_char(self, characteristic, data, response=False):
        if not self.is_connected:
            raise RuntimeError("native BLE disconnected")
        expected = self.cmd_uuid if response else self.fw_uuid
        if characteristic != expected:
            raise ValueError("native characteristic mismatch")
        self._sequence += 1
        token = self._sequence
        if response:
            self._pending[token] = asyncio.get_running_loop().create_future()
        frame = dict(op="command" if response else "data", id=token,
                     data=base64.b64encode(data).decode())
        self._process.stdin.write(json.dumps(frame).encode() + b"\n")
        await self._process.stdin.drain()
        if response:
            try:
                await asyncio.wait_for(self._pending[token], 15)
            finally:
                self._pending.pop(token, None)

    async def disconnect(self):
        try:
            if self._process is not None and self._process.returncode is None:
                try:
                    self._process.stdin.write(b'{"op":"close"}\n')
                    await self._process.stdin.drain()
                    await asyncio.wait_for(self._process.wait(), 2)
                except (BrokenPipeError, ConnectionResetError, asyncio.TimeoutError):
                    if self._process.returncode is None:
                        self._process.kill()
                        await self._process.wait()
            if self._reader is not None:
                await self._reader
        finally:
            self.is_connected = False
            if self._temp is not None:
                self._temp.cleanup()
                self._temp = None
