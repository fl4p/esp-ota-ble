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

Requires a partition table with two app slots and an `otadata` partition. `esp_ota_begin` is called
with **`OTA_WITH_SEQUENTIAL_WRITES`**, not `OTA_SIZE_UNKNOWN` — the two constants read alike and
behave oppositely. A byte size or `OTA_SIZE_UNKNOWN` erases synchronously inside `esp_ota_begin`
(and `OTA_SIZE_UNKNOWN` erases the *whole* partition, worse than passing the real size), which on a
~1.7 MB slot blocks long enough to starve the idle task and trip a panic-on-timeout task watchdog.
Only `OTA_WITH_SEQUENTIAL_WRITES` defers erasing to per-sector calls inside `esp_ota_write`.

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

Host-native (clang++, ASan/UBSan), no hardware. ESP-IDF is shimmed in `test/host-stub/`, with a fake
partition that models *when* erases happen — begin-time versus per-write — because that is the axis
the `OTA_WITH_SEQUENTIAL_WRITES` choice turns on, and a fake that only modelled the end state would
pass on either constant. SHA-256 in the stub is a real implementation: a digest that always matched
would make the mismatch test pass unconditionally.

Every failure case asserts that the boot partition did **not** move, not merely that a call returned
false. The suite is calibrated — swapping in `OTA_SIZE_UNKNOWN`, or removing the stale-abort
guards, makes it fail.

Run it before flashing any consumer.
