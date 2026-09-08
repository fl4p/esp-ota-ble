# esp-ota-ble

Push a firmware image to an ESP32 over BLE, with no WiFi involved.

This module is the **receiver only**, and it knows nothing about BLE: it is a staging ring, a
credit-window flow-control scheme, a streaming SHA-256 and the `esp_ota_*` calls. The consumer owns
the GATT layer and feeds bytes in. That split is why the same code serves an ESP-IDF firmware using
the Arduino BLE wrapper and a PlatformIO firmware using `h2zero/NimBLE-Arduino`, whose callback
signatures have nothing in common.

Extracted from `fugu-mppt-firmware`, where the design was validated on hardware (1.78 MB pushed in
~57 s, ~32 KB/s, SHA verified, a truncated image correctly rejected without moving the boot
partition).

## Wire protocol

Control lines in (from the host), status lines out (to the host):

```
host → device : begin <size> <sha256hex>   arm: pick the passive slot, quiesce, open the OTA handle
                end                        finalize: verify length + digest, set boot slot, reboot
                abort                      tear down, leave the boot slot alone

device → host : OTAB READY part=<label> size=<n>
                OTAB CRED <cumulative byte offset the host may stream up to>
                OTAB PROG <written>/<size>
                OTAB OK rebooting
                OTAB FAIL <reason>
```

Firmware bytes go on a separate write-without-response channel, chunked at the negotiated ATT MTU
minus 3, and the host must never stream past the most recent `CRED` offset. Violating that window
aborts immediately with `OTAB FAIL credit-overrun`; the producer callback only latches the failure,
and the consumer tick reports and tears it down without doing flash work on the BLE host task.

There is **no per-chunk ack**. Flow control is a single cumulative high-water mark
(`bytes flushed + 8 KB`), and integrity is the final length check plus SHA-256. A dropped byte costs
a full retry and nothing less — that trade is what makes the transfer fast enough to be usable. The
receiver re-announces the current `CRED` every 5 seconds while active, so a lost credit notification
does not deadlock the stream.

An active transfer that accepts no firmware bytes for 30 seconds aborts with `OTAB FAIL stalled`.
This bounds both an open OTA handle and the consumer's quiesced state if a disconnect callback is
missed. `otaBleRequestAbort()` is still the prompt disconnect path: called while `begin` is latched,
it cancels that begin before it can quiesce the device. It is harmless when no OTA is in flight.

Sequence: `begin` → wait for `READY` → stream up to each `CRED` → **wait for `PROG <size>/<size>`**
→ `end`. Waiting for the final `PROG` matters: write-without-response packets can still be in flight,
and the device would otherwise see a short image. After `end` the device reboots immediately, so the
`OTAB OK` notification usually never drains — a host should treat `PROG` complete plus a disconnect
plus a successful re-advertise as success.

## Integrating

```cpp
#include <ota_ble.h>

static void status(OtaBleLevel lvl, const char *line) { /* notify the client, and/or log */ }
static void quiesce(bool halt) { sampler.halted = halt; }   // stop sampling while flash is busy
static void restart() { myOrderlyRestart(); }

OtaBleHooks h; h.status = status; h.quiesce = quiesce; h.restart = restart;
otaBleInit(h);

// BLE control-characteristic write callback, any task:
if (otaBleSubmitCommand(line) == OtaBleSubmit::Rejected) { /* tell the client it was malformed */ }

// BLE data-characteristic write callback, any task:
otaBleStageBytes(data, len);

// A slow periodic loop on an ordinary task:
otaBleTick(millis());

// BLE disconnect (unconditional -- this also cancels a begin waiting for the consumer tick):
otaBleRequestAbort();
```

Three rules the design depends on:

1. **`otaBleTick()` must not run on the BLE host task.** `esp_ota_begin/write/end` block on flash for
   milliseconds; stalling the host task there trips the connection supervision timeout and drops the
   link mid-update. The producer entry points (`otaBleSubmitCommand`, `otaBleStageBytes`) only copy
   and latch, and are safe anywhere.
2. **Preserve status severity when logging.** Every `OTAB FAIL …` is `Warn` or `Error`. Collapsing
   them to `Info` means raising the tag's log level silently swallows exactly the lines that let a
   host tell failure from a dead link.
3. **Quiesce is not optional under load.** Flash erase/write disables the CPU cache and stalls the
   other core. On the fugu converter, OTAing at full power reliably reset the device mid-download
   until the real-time loop was stopped first.

### PlatformIO

```ini
lib_deps = https://github.com/fl4p/esp-ota-ble.git
```

Or, to develop the module alongside its consumer — a git dependency pins a fetched copy and ignores
local edits, which is the wrong shape while the module is still changing:

```ini
lib_deps = symlink://../esp-ota-ble     ; relative to the project dir; clone it beside your repo
```

### ESP-IDF

```cmake
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../esp-ota-ble")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
```

`CMakeLists.txt` registers it as a component. Prefer failing loudly if the directory is absent — a
missing OTA receiver should stop the build, not silently produce firmware that cannot be updated.

## Partitioning

Requires a partition table with two app slots and an `otadata` partition.

**The size passed to `esp_ota_begin` chooses an erase strategy, and the constants read alike while
behaving oppositely.** `OTA_SIZE_UNKNOWN` erases the *whole* partition synchronously; a byte size
erases `ALIGN_UP(size, erase_size)` synchronously; only `OTA_WITH_SEQUENTIAL_WRITES` defers erasing
to per-sector calls inside `esp_ota_write`. On a ~1.7 MB slot a synchronous full erase blocks long
enough to starve the idle task and trip a panic-on-timeout task watchdog.

Three strategies, in the order the receiver prefers them:

1. **Skip identical sectors** (default; `OTA_BLE_SECTOR_SKIP`). `esp_ota_begin` is passed exactly
   one erase sector — the smallest request that still leaves `need_erase == false`, which is what
   hands every later erase to this module. Each sector of the reconstructed image is then compared
   against what the slot already holds, and erased and programmed only if it differs.

   This is worth more than it sounds, because flash work, not link time, is where a push goes: 32.4
   s of a measured 43.4 s raw push was inside `esp_ota_write`. A rebuild of the same firmware
   changes only part of the image — with a stable layout, a one-line edit leaves about two thirds of
   the sectors byte-identical (see `fugu-mppt-firmware/doc/2026-09-08-image-layout-stability-for-
   ota.md`) — and every identical sector costs one 4 KB read instead of an erase plus a program.

   It costs one erase-sector buffer of RAM and gives up 64 KB block erases, since a dirty sector is
   erased on its own. **Whether that trade holds depends on the 4 KB sector-erase time, which is not
   measured on this hardware.** The `OTAB SKIP` line reports `kept`, `wrote`, `erases` and
   `erase_ms`, so the first real push measures it: `erase_ms / erases` is the number.

   The receiver withdraws the strategy by itself on an encrypted flash (`esp_ota_write_with_offset`
   refuses unaligned sizes there, and none of it is testable on the boards in hand) or when the
   sector buffer will not allocate. Both fall back rather than fail.
2. **Erase ahead of the writer**, when skipping is off and the payload is under a quarter of the
   image. Erases a head block inside `begin` and keeps the rest erased just ahead of the write
   pointer, hiding the erase under link time. It cannot be combined with skipping: it erases flash
   before the bytes destined for it have arrived, which destroys the content the comparison reads.
3. **`OTA_WITH_SEQUENTIAL_WRITES`**, the always-safe fallback.

If the bootloader has `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`, the freshly flashed image boots as
`PENDING_VERIFY` and **must** call `esp_ota_mark_app_valid_cancel_rollback()` once it has proven
itself, or the next reset reverts it.

> **Arduino consumers: the core cancels rollback for you, before `setup()` runs.**
> `initArduino()` (esp32-hal-misc.c) calls `esp_ota_mark_app_valid_cancel_rollback()` gated on a weak
> `verifyRollbackLater()` that returns `false` — so an image is confirmed on nothing but having
> reached init, and any health check you write afterwards is dead code that never sees
> `PENDING_VERIFY`. Override it:
>
> ```cpp
> extern "C" bool verifyRollbackLater() { return true; }
> ```
>
> This was found on hardware, where a fresh OTA reached `VALID` at 43 s against a 60 s confirm gate
> that had never run. If a fresh OTA reaches `VALID` sooner than your gate, you have lost the
> override. Confirm on evidence that the application actually works — an
uptime threshold plus a liveness signal from the real work — not merely that `setup()` returned. Pair
it with a watchdog covering early boot: an image that hangs before the confirm point never resets, so
the bootloader never gets its chance to roll back, and the device is bricked until someone reflashes
it over a wire.

## Tests

```
./test/run.sh
```

Host-native (clang++, ASan/UBSan), no hardware, and run three times — once per erase strategy, since
the two the default displaces are still reachable fallbacks and compiling only the default would
leave them unbuilt as well as untested.

ESP-IDF is shimmed in `test/host-stub/`. The fake partition models *when* erases happen — begin-time
versus per-write — because that is the axis the `OTA_WITH_SEQUENTIAL_WRITES` choice turns on, and a
fake that only modelled the end state would pass on either constant. It also models the update
slot's actual bytes, that erasing destroys them, and that `esp_ota_end` refuses a handle nothing was
written through: skip-identical decisions are asserted against what the slot ends up holding, which
is the only thing the bootloader ever reads. SHA-256 in the stub is a real implementation: a digest
that always matched would make the mismatch test pass unconditionally.

Every failure case asserts that the boot partition did **not** move, not merely that a call returned
false. The suite is calibrated — swapping in `OTA_SIZE_UNKNOWN`, or removing the stale-abort
guards, makes it fail. So does making an unreadable sector compare as a match, or dropping the
final partial sector's flush.

One calibration that does **not** bite, recorded so it is not mistaken for coverage: removing the
"sector 0 is never skipped" rule leaves every test passing, because `esp_ota_begin` erases sector 0
before the first comparison can read it. The rule is a local restatement of an invariant enforced
somewhere else, not the thing enforcing it.

Run it before flashing any consumer.

## Host side: `host/esp_ota_ble.py`

The device half of this repo had three separate host tools speaking to it
(`fugu-mppt-firmware/etc/ota_ble.py`, `pwr-metering/smart-shunt-ota-ble.py`,
`ha/farming/tech/node-prototype/tools/ota_ble_push.py`) — about 1500 lines
between them, all reimplementing the same discovery, link setup and OTAB
protocol. `host/esp_ota_ble.py` is that, once. It needs only `bleak`.

```python
import esp_ota_ble as O

link = O.BleOtaLink(CTRL_UUID, CTRL_UUID, DATA_UUID)   # one char: write + notify
dev  = await O.find_device(name_prefix="farmnode-", service_uuid=SVC_UUID)
await link.open(dev)
ok = await O.push_image(link, image_bytes,
                        on_line=print,
                        on_progress=lambda s, t: bar(s, t))
```

A console-hosted receiver (NUS, `ota-ble begin ...`) passes
`cmd_prefix="ota-ble "` to `push_image` and points `cmd_uuid`/`notify_uuid` at
its RX/TX characteristics. A transport this module does not know about — fugu
tunnels GATT through an ESPHome `bluetooth_proxy` — is wrapped with
`O.adapt_link(link)`.

### The reason it exists: `usable_chunk()`

**Sizing a firmware write is the one thing every copy of this code got wrong**,
and it fails silently in both directions. A `write-without-response` is
unacknowledged *by definition*, so nothing upstream ever reports a bad choice.

Ask the characteristic for **`max_write_without_response_size`**. That is what
`usable_chunk()` uses when the transport can supply it.

#### Too large: the image is corrupted and nobody says so

Measured 2026-09-07, Raspberry Pi (BlueZ 5.82) to an ESP32-S3, ATT MTU
negotiated at 517 and reported as 517 by both ends, writing `mtu - 3` = 514:

| chunk | result |
|---|---|
| 514 (`mtu - 3`) | the 8192-byte credit window went out as 16 packets and the device received **one** |
| 400 | a full 706 640-byte image transferred and the device confirmed the new slot |

The survivor was identifiable rather than lucky: it landed at offset 7710,
length 482, and `image[7710:7714] == b"erat"`. `esp_ota_write` then rejected the
image on its **magic byte**, because the first byte it ever saw was the middle
of a string table — an error that points nowhere near the cause.

Nothing upstream noticed. bleak awaited BlueZ's D-Bus reply and BlueZ accepted
all sixteen writes. **This is why the protocol carries a streaming SHA-256 and a
length check**: over write-without-response the transport cannot be trusted to
report loss, so the receiver has to catch it.

#### Too small: a 25x throughput loss that looks like a slow link

Do not "fix" the above by acquiring the MTU and trusting `mtu - 3` — and do not
derive the chunk from `mtu_size` either. bleak reports BlueZ's **23-byte
default** until the MTU is acquired, so `max(mtu - 3, 20)` silently yields
20-byte writes. Measured: **1.4 kB/s** for a 680 kB image, where the same image
over CoreBluetooth takes one to two minutes.

#### An OS test is not a backend test

`platform.system() != "Darwin"` misclassifies WinRT and any custom backend, and
it is blind to an adapted transport: fugu tunnels GATT through an ESPHome
`bluetooth_proxy`, whose data plane is an ESP32 whatever OS runs the script.
`usable_chunk()` falls back to the OS test only when the characteristic cannot
answer.

#### Know which constant you are using

- **244 bytes** is *derived*: 251-byte maximum LL payload − 4 L2CAP − 3 ATT, the
  largest ATT value fitting one maximum-length link-layer PDU, so no L2CAP
  fragmentation. The negotiated Data Length may be smaller, so this is not
  universally safe either.
- **400 bytes** (`BLUEZ_MAX_FW_WRITE`) is *empirical*: one Pi/BlueZ/NimBLE
  combination, one successful image. It says nothing about other controllers or
  a proxied transport. It is a regression point, not a safety bound.

#### `push_image()` returning True is not proof the new image runs

A link drop after `end` is treated as success, because on a real success the
device reboots before the notification drains. But `PROG total/total` only
proves the flash *writes* finished — the digest check, the length check and
`esp_ota_set_boot_partition()` all happen later, inside `otaBleEnd()`. A target
that dies between the acknowledged `end` write and the consumer tick returns
`True` with no boot slot selected.

**Confirm out of band.** The strongest check is to read the device's running OTA
slot afterwards and require it to have *changed*; requiring the device to
advertise again is weaker but at least shows it rebooted. An uptime comparison
is not a check at all: a rejected image reboots immediately and the *previous*
image comes up with uptime zero, satisfying it.
