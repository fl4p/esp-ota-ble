# Pushing node BLE OTA throughput beyond parity

Measured 2026-09-10–11. The final three full raw rewrites reached
**87.168 kB/s mean**, versus the [verified 43.075 kB/s baseline](2026-09-10-node-throughput.md):
**2.024× throughput**. Those repeats were 86.766 / 86.797 / 87.943 kB/s,
using Graceful, a native CoreBluetooth delegate queue with verified
user-interactive QoS, 244-byte values, 10 ms and 2M PHY. The ordinary native
queue independently averaged **86.403 kB/s** over three rewrites. Selection
pilots are excluded from both means. The small difference between the two
means is within the observed variation, not an isolated scheduling benefit.
This agent's compiler was idle; unrelated applications remained active on
the shared Mac.

Every image byte was programmed, erase coverage was exact, and the exact
target image and changed slot were verified after boot. The native sender
checks `canSendWriteWithoutResponse` before every write and feeds the queue
directly from its readiness callback. **Raw throughput above 100 kB/s has
not been demonstrated.** RAM-only and transformed-image rates are different
measurements and do not establish that result.

Pushing practical full rewrites further: the native tamp transfer of the
757,104-byte Y image verified **122.955 image kB/s** in 6.158 s, or 85.972
wire kB/s. A closely related-image delta installation reached 146.872 image
kB/s with every image byte programmed. Those are explicitly transformed
updates, not raw BLE throughput. The additional `rpi.local` and `farmgw`
centrals were tested and were slower; their adapter and protocol results
are below. No hard radio or hardware ceiling was established.

Experiments run on the same PSRAM ESP32-S3 and macOS host. The starting
receiver image is `19ee4942881a9fcd451dcd23fed1a348e08d0676758d5552d8fcc31d60677cdd`.
Working source, binaries and logs are isolated under
`/Users/fab/o2p-push/ota-throughput-next/`. The earlier node worktree remains
untouched. The host tamp compatibility correction is committed separately
as `d819450`; experimental flash strategies remain private bench variants.

## Link sweep

All measured with 495-byte values, negotiated 2M PHY, zero skipped sectors,
and exact post-boot image plus running-slot verification. X and Y payloads
occupy the same number of sectors and differ in all of them; the receiver
image is restored between pushes. Decimal kB/s includes reboot disconnect.

| Exact accepted interval | Writes per pacing check | kB/s |
|---|---:|---:|
| 15 ms | 1 | 40.52 |
| 15 ms | 2 | 44.05 |
| 15 ms | 4 | 43.59 |
| 15 ms | 8 | 39.19 |
| 15 ms | 16 | 42.32 |
| 30 ms | 1 | 27.25 |
| 45 ms | 1 | 19.00 |
| 60 ms | 1 | 12.78 |
| 90 ms | 1 | 9.10 |
| 130 ms | 1 | 6.70 |

These single runs do not establish a gain from queue batching. Longer
intervals tested so far reduced throughput. An initial burst-2 attempt
failed discovery before transferring anything; its later retry is the
measurement in the table. Discovery failures are retained in the logs.

## Flash and transform trials

Initial full-write results at 15 ms / 2M / 495 bytes:

| Strategy | Raw kB/s | Relevant accounting |
|---|---:|---|
| Instrumented original sector path | 42.72 | 9.930 s erase; 4.743 s program; all 176 sectors written |
| 64 KiB erase-ahead, no skipping | 49.67 | 2.145 s subsequent erase; 2.726 s program; all 754,624 bytes programmed |
| Sequential writes | 42.44 | 13.205 s inside writes, including IDF-owned erases |
| Up-front erase | 22.45 | READY at 2.340 s; 2.687 s program; complete erase/write verified |

The first 10 ms erase-ahead run reached **67.503 kB/s**, with exact full
programming and post-boot SHA verification. Requests for 7.5 and 8.75 ms were
not accepted: the connection remained at 15 ms. **10 ms was actually accepted**
(`interval=8`). Do not infer a 15 ms floor from the earlier 15 ms result.
A repeated unbuffered run at 10 ms measured 63.84 kB/s; burst two measured
63.72 kB/s. The 67.50 kB/s peak is not a repeated mean.

On erase-ahead at 15 ms, 244/400/512-byte values measured 42.11/45.37/45.45
kB/s respectively; 495 remains the best measured packing in that comparison.

| Additional implementation trial | Accepted interval | Raw kB/s |
|---|---:|---:|
| Erase-ahead, 4 KiB default-heap write buffer | 15 ms | 53.23 |
| Same, burst four | 15 ms | 47.69 |
| Same buffer | 10 ms | 65.37 |
| Same, burst two | 10 ms | 58.80 |
| 4 KiB internal-RAM buffer | 10 ms | 32.18 |
| Same, burst two | 10 ms | 64.94 |
| Unbuffered consumer moved from core 0 to core 1 | 10 ms | 66.20 |
| Same, burst two | 10 ms | 56.78 |
| MSYS1 enlarged to 50 blocks, unbuffered flash | 10 ms | 62.97 |
| Same, burst two | 10 ms | 64.37 |

These are single trials per cell, including the low 32.18 kB/s result; do
not discard it or infer a reliable placement benefit from the faster sample.
The internal-buffer run's flash program time was 2.314 s, so its 23.449 s
total is not explained by program duration alone.

Built: instrumented original sector strategy, sequential writes, 64 KiB
erase-ahead for raw transfers, and up-front erase. These are private bench
builds, not shipping defaults. The forced block strategy overrides the
original restriction to small transformed payloads; its head erase is
bounded to one block and subsequent erases remain bounded by the image.

Native tests use the actual receiver with the existing flash model. They
check exact programmed bytes and erase coverage across block boundaries,
including an identical destination that must still be fully programmed,
plus truncation, digest mismatch, overflow and flash failures. Removing the
actual block erase is a failing negative control. Instrumentation counts
successful writes; reconstructed-byte counts alone cannot prove programming.

The small-edit pair has equal-sized 722,016-byte images and changes **3 of
177 sectors**. It changes a four-character diagnostic label without changing
its length; this is one controlled layout-preserving edit, not a claim about
arbitrary application changes.

The first tamp attempt aborted with `OTAB FAIL xform ESP_ERR_INVALID_CRC`.
The host has tamp **2.3.0**, whose default extended stream starts with `0x9a`;
the node vendors **1.11.1**, which rejects header bit `0x02`. Explicit
`extended=False` produces the legacy `0x98` header. A native test feeds the
actual host output through `ota_xform.cpp` and the node-vendored decoder:
the original payload fails, while legacy output reconstructs the complete
image at input fragments of 1, 17, 495 and 2048 bytes. Five host tests cover
new/legacy APIs, unrelated compressor errors, missing dependencies and bad
headers. Compression took 0.212 s with the old
default and 0.205 s with explicit legacy encoding in this sample; these
host-side times are outside the transfer timer.

The corrected tamp trial subsequently passed on hardware: 8.990 s for the
757,104-byte image, **84.216 image kB/s** and **58.885 wire kB/s**, with every
image byte programmed and the exact image booted. Delta took 8.807 s,
**85.962 image kB/s** and **27.699 wire kB/s**, also a full flash rewrite.
These transformed rates are not raw BLE throughput. See
[the compatibility fix](tamp-compatibility.md).

For the controlled 3-sector edit, both destination slots were first seeded
with the baseline image. Raw/tamp/delta then took **13.964 / 9.897 / 2.055 s**
at the original 15 ms setting. Each kept 174 sectors and programmed all three
changed sectors (9,312 image bytes). The delta was 11,440 bytes. This is the
largest measured update-time improvement, but it depends on a known base
and stable image layout.

## Follow-up from the ESP32-S3 research note

The farming note `docs/2026-09-10-esp32s3-ble-throughput.md` was read after the
initial matrix. It adds useful tests of reception without flash and actual
NimBLE buffer configuration. Its S3-to-S3 benchmark is not a measured limit
for this Mac-to-S3 link. Its inspection describes another source snapshot;
it does not override the measured negotiated state of these bench builds.

A private RAM-only command allocates bounded PSRAM, copies incoming values
on the same data characteristic, and reports received length, SHA-256 and
receiver elapsed time. It never opens an OTA handle or changes the boot slot.
A 1 MiB trial at 10 ms / 2M / 495 bytes verified **69.343 receiver kB/s**
(69.070 kB/s using the host completion timer). It is a short pilot, not a
60-second stability run or an OTA result.

The 50-block variant's 1 MiB RAM pilot verified **101.765 receiver kB/s**
(101.221 host kB/s). Both pilots passed the same length/SHA tests. One sample
per build is insufficient to attribute the full difference to buffer count;
it does establish that the 69 kB/s pilot is not a measured link ceiling.

Three 6 MiB RAM runs on the combined 4 KiB/internal-buffer, 50-MSYS1 build
verified 95.224 / 92.947 / 43.948 receiver kB/s over 66.07 / 67.69 / 143.16 s.
The running image and 10 ms / 2M link were checked again after each. The third
overlapped the full SDK rebuild. Earlier exploratory firmware builds also
overlapped some trials: those cells are observations, not an isolated causal
comparison. Final comparisons must run with this agent's compiler idle.

The combined build's three full raw OTA repeats were 65.088 / 65.510 / 67.206
kB/s, mean **65.935 kB/s** (+53.1% from 43.075), with automatic 10 ms tuning.
These establish the intermediate receiver's result; later variants are below.

## Deeper flash and host experiments

With wide pools, 4 KiB internal batches and the ordinary SDK, credit
deduplication measured 66.670 kB/s (one write per readiness check) and
69.560 kB/s (two). The PSRAM execution SDK without deduplication measured
70.924 and 74.309 kB/s respectively. Adding deduplication to that SDK then
measured **78.243 kB/s** with a readiness check before every write. These
are exploratory individual full-rewrite measurements, not repeated means.

The native CoreBluetooth readiness callback completed one full raw transfer
at 70.556 kB/s with 1,525 callbacks. It did not improve the corresponding
70.924 kB/s polling observation. The following restore lost BLE contact
during post-boot verification and remains recorded as unverified in its
original result. Opening USB triggered `USB_UART_CHIP_RESET`; a subsequent
BLE query confirmed the exact intended Xip image on app1. The reset and late
recovery are recorded separately, not silently promoted into the original
benchmark result.

The first Xip→XipQuiet installation reconstructed and programmed all 724,640
bytes from a 30,215-byte delta in 5.701 s: **127.103 image kB/s**, or 5.300
wire kB/s. This demonstrates practical update speed above 100 image kB/s;
it is neither raw throughput nor a controlled comparison against the older
Y target. The following measurements extend that pilot.

XipQuiet's burst-two raw result was 82.359 kB/s. Requests for supervision
timeouts of 200 and 100 ms left the actual timeout at 720 ms; the resulting
81.748 and 79.848 kB/s runs do not establish a timeout improvement.

The compiler-idle XipQuiet RAM sweep at accepted 10 / 12.5 / 15 / 20 / 30 /
60 / 125 / 130 ms measured 74.968 / 74.785 / 65.324 / 52.305 / 35.371 /
16.092 / 7.294 / 7.552 receiver kB/s, using 512 KiB payloads. The 60 ms
cell first failed discovery before transfer; the successful retry is shown.
At 10 ms, 1 MiB queue-burst tests of 2 / 4 / 8 verified 83.994 / 87.175 /
95.303 kB/s. **Burst 16 failed received-length/integrity verification**;
its 94.968 host submission kB/s is not receiver goodput. No unchecked burst
setting is promoted to a safe host default.

- MSYS2 and incoming ACL pools enlarged from 24 to 96; runtime values checked.
- Internal flash-write batches of 1, 2, 4, 8 and 16 KiB, independently of the
  64 KiB erase blocks. Native full-write/error tests pass for all five sizes.
  The first 16 KiB test exposed a fixture precondition: its 20,000-byte image
  did not reach the injected second write before `end`. The revised failure
  fixture uses 65,536 bytes and observes the second-write error and cleanup.
- Final-credit deduplication. The three intermediate OTA runs emitted 379,
  387 and 407 duplicate final grants. The regression's original implementation
  emits 33 identical grants over 32 drains; the candidate emits one and still
  repeats after five seconds to recover a lost notification. All three native
  erase-strategy suites pass with the candidate.
- PSRAM execution: rebuild with `CONFIG_SPIRAM_XIP_FROM_PSRAM`,
  `CONFIG_SPIRAM_FETCH_INSTRUCTIONS` and `CONFIG_SPIRAM_RODATA` enabled. Both
  80 MHz OPI PSRAM configuration and the enabled flags are checked in the
  rebuilt SDK. ESP-IDF 5.5.5's `spi_flash_os_func_app.c` selects
  `SPI_FLASH_CACHE_NO_DISABLE` when both fetch/rodata flags are set. This is
  the SDK used by the Xip and XipQuiet measurements above.
- CoreBluetooth's native readiness callback in place of host polling. The
  installed Bleak callback was unused; a process-local experiment forwards it
  to asyncio, then checks `canSendWriteWithoutResponse` before each write.
  A native callback smoke test and the hardware trial above pass.

The SDK rebuild uses an isolated, copy-on-write clone of PlatformIO's packages
under `ota-throughput-next/pio-xip`; source and destination inodes differ.
The shared toolchain is untouched. The bench reports flash ID `852018`, generic
driver, 16,777,216 bytes and capability mask `4`. Its application slots still
use the same 4 MB partition layout. Do not confuse that layout with a physical
4 MB flash measurement, or assume flash auto-suspend support from the chip ID.

Runtime buffer reporting established **12 × 256-byte MSYS1 blocks and 24
MSYS2 blocks** in the baseline. A simple command-line definition was overwritten
by the SDK's compatibility alias; the 50-block build therefore includes the
SDK configuration before explicitly overriding both MSYS1 spellings. Runtime
confirmation is required before counting that trial. This grows MSYS1 payload
storage by 9,728 bytes, plus allocator/mbuf overhead; it does not change MSYS2.

The initial DLE event listener was installed before NimBLE initialization,
which resets its listener registry. Its all-zero diagnostics are **unverified**,
not a zero-length negotiated connection. The final build registers afterwards;
only later nonzero observations can establish the negotiated octets/time.

## Native sender, controller scheduling and CoC

The native Swift sender feeds CoreBluetooth directly from its readiness
delegate and checks readiness before every GATT write. Its timer starts at
`begin` and ends at the reboot disconnect; connection, authentication and
channel setup occur outside that timer. A separate Python reconnection checks
the exact booted image and changed slot. Successful native rows also require
the transmitted file hash, every programmed byte and exact erase coverage.

The controller's `ce_len_type` field was built and reported as 0, 1 and 2,
using the SDK field/Kconfig meanings (original, HCI CE length, and Espressif
method). All three use the same XIP SDK, wide pools, internal 4 KiB flash
buffer, block erase and credit deduplication. Each sends the same X image.
The compiler was idle throughout these measurements.

| Controller CE mode | Raw OTA kB/s | RAM at 10 ms | RAM at 30 ms | RAM at 125 ms |
|---|---:|---:|---:|---:|
| 0 | 81.304 | 93.986 | 34.137 | 7.973 |
| 1 | 69.577 | 83.508 | 29.223 | 6.595 |
| 2 | 86.988 | 93.652 | 32.853 | 8.356 |

RAM cells use 262,144 bytes and verified actual negotiated intervals. These
are single observations per cell; mode 2 is the leading candidate, not a
statistically established universal winner. Its final restore first failed
discovery before starting a transfer; the retained retry verified the image.

An experimental LE credit-based L2CAP channel uses PSM 150 for data while
retaining the existing authenticated GATT control, OTA credits, flash path
and verification. It accepts only the authenticated connection and stages
bytes into the same ring. Disconnect requests an abort. This is a private
benchmark protocol, not a released interoperability promise.

| Node CoC receive MTU | Configured MPS | Raw OTA kB/s | Seconds |
|---|---:|---:|---:|
| 2,048 | 248 | 42.288 | 17.845 |
| 2,048 | 247 | 43.823 | 17.220 |
| 5,000 | 247 | 73.603 | 10.253 |
| 16,384 | 247 | 67.346 | 11.205 |
| 5,000 | 251 | 76.720 | 9.836 |
| 10,000 | 1,000 | 77.037 | 9.796 |

Every row programmed all 754,624 bytes, erased the exact aligned range and
booted X in the other slot. Larger receive windows helped over the initial
2 KiB trial, but none beat the native GATT observation. Changing MPS by one
did not produce a major gain. Do not turn the initial packet-packing
hypothesis into an established explanation. The sniffer observed the Mac
advertising its own MTU/MPS as 1,251; the node advertises its configured
receive values. The library callback's minimum-MTU value is not evidence
that both directions share a 1,251-byte receive limit.

The first `CocInitial` binary was never installed: inspection caught that
the library owns and deletes its callback object. The installed variants
allocate that callback accordingly. Build identities retain both versions;
an unflashed binary is not a hardware test result.

The runtime `esp_bt_sleep_disable()` probe returned `262`
(`ESP_ERR_NOT_SUPPORTED` in the installed SDK). Its accompanying raw run
verified 79.829 kB/s, but the requested sleep change did not succeed. Native
requests for intervals 6 and 7 both retained actual interval 8 (10 ms);
their 76.113 and 78.224 kB/s results do not measure 7.5 or 8.75 ms links.

Native unchecked batching passed at two writes per readiness check
(78.388 kB/s), but four writes stalled and aborted. The subsequent query
confirmed the Ce2 image and slot were unchanged. Burst eight was not attempted
with this frontend after that failure. The earlier Python burst-eight RAM
success does not make the native batching path safe. A native wrong-base
negative control then failed before `begin`, again leaving the receiver
unchanged. The strict 244-byte native GATT run verified 80.836 kB/s.
The corresponding 512-byte run measured 68.739 kB/s.

With Ce2 and the same 757,104-byte Y target used in the older transform
comparison, Python tamp verified **106.811 image kB/s** (74.684 wire kB/s,
7.088 s), and delta verified **110.810 image kB/s** (37.158 wire kB/s,
6.832 s). Both programmed all 757,104 bytes and erased 757,760 aligned bytes;
neither used sector skipping. The receiver was restored between trials.
These are update-time gains over the older 84.216 / 85.962 image kB/s
observations, with several settings changed together.

The Mac controller reports BCM_4388, firmware 22.5.542.2785, PCIe transport.
The host is an M3 Pro running macOS 15.7.3, with Swift 6.2.4. Firmware uses
NimBLE-Arduino 2.5.1 and the rebuilt ESP-IDF 5.5.5 SDK. Python package and
compiler details are retained in `dependency-versions.json` in the archive.
One connected device, the bench node, appeared in the metadata snapshot.
This is a shared development Mac with unrelated applications active; keeping
this agent's compiler idle does not mean the whole host was quiescent.

## Completion-time and core follow-up

The graceful-reboot variant flushes final status, waits 50 ms, requests a
BLE disconnect, waits at most another 100 ms, and then follows the existing
restart path. The hook runs only after the receiver has validated the image
and selected its boot partition. A disconnected link still does not establish
OTA success: the independent post-boot image and slot checks remain required.

In the 495-byte Graceful trial, the delay from `OTAB OK rebooting` to the
central's disconnect callback was **52 ms**, compared with approximately
820 ms in the preceding non-graceful trials. This removes completion-detection
delay; it does not make the radio transfer each payload byte faster. The
timer is still begin-to-disconnect, with the same separate boot verification.

| XIP receiver / completion path | Native GATT value bytes | Raw OTA kB/s |
|---|---:|---:|
| Core 1, ordinary restart | 495 | 81.521 |
| Core 0, graceful disconnect | 495 | 86.960 |
| Core 0, graceful disconnect | 244 | 88.282 |
| Core 1, graceful disconnect | 495 | 87.303 |
| Core 1, graceful disconnect | 244 | 86.558 |

All rows use the same X target and complete flash/boot checks. Core 1 did
not produce a large improvement. The 244-byte Graceful observation was
selected for independent repeat trials; the selection sample is excluded
from their mean.

The independent Graceful/244 repeats were **84.806 / 86.301 / 88.101 kB/s**,
mean **86.403 kB/s**, a 100.6% increase over the original three-run baseline.
Native GATT with 495-byte values then completed Y via tamp in **6.158 s**
(122.955 image / 85.972 wire kB/s) and via delta in **6.141 s**
(123.288 image / 41.347 wire kB/s). Both fully programmed the target and
verified its boot identity and changed slot.

A Graceful→Core1Grace installation also reconstructed and programmed all
725,136 image bytes from a 14,072-byte delta in **4.937 s**, or **146.872
image kB/s** (2.850 wire kB/s). This is a closely related-image installation,
not the controlled Y comparison or a raw-throughput measurement. Its log
retains exact program and erase counts and the post-boot identity check.

The first process-priority trial requested user-interactive thread QoS and
a latency-critical activity, but the callback's Foundation QoS getter did
not confirm that priority. Its 87.114 / 88.181 kB/s observations remain
unconfirmed as a QoS experiment. The corrected frontend gives CoreBluetooth
and all its state/timers a dedicated serial delegate queue. Its pthread QoS
query returned success and user-interactive QoS on both pilots and all three
repeats. Pilots at 244 / 495 bytes measured 88.413 / 88.199 kB/s; independent
244-byte repeats measured 86.766 / 86.797 / 87.943 kB/s, mean **87.168**.
No unchecked batching is used. These results do not demonstrate a large
scheduling gain over the ordinary native queue.

The final query verified Graceful on app1, image identity
`9f8abf18b2ad59476c71f4c3ab757b6e9bc3fb8b7f5f5a95cb64263f015ba587`,
10 ms, 2M in both directions, the intended buffer counts and CE mode 2.
The receiver reported RSSI −44 dBm. This is the bench firmware left installed.

## Measurement checks

The existing sector strategy requires final `kept=0` and exact sector counts.
The non-skipping experiments emit `OTAB FLASH` with the selected strategy,
successful programmed-byte count, erase-byte coverage where the receiver
owns erasing, and timing. Full-write acceptance requires a recognized mode,
exact image-byte programming, exact aligned erase coverage for the block and
up-front modes, and exact post-boot identity. Sequential mode leaves erase
accounting to IDF; its successful write calls still count every programmed
byte. Missing/malformed values are unverified, not a pass. Near/far wrong
counts, unknown modes and missing values are calibrated. The additional
host verdict costs **0.32 microseconds/check** over 100,000 evaluations.

The instrumented original path's 42.72 kB/s
falls in the earlier run range; one run cannot resolve a small overhead.
The first timing diagnostic exceeded the receiver's 96-byte line buffer:
required mode/program/erase counts arrived, but its trailing begin-time
field did not. V2 separates `OTAB FLASH` counts from `OTAB TIME` timings;
missing/truncated fields from the earlier logs are not treated as timings.
Timed write work overlaps radio work, so
`write_ms` cannot simply be subtracted from elapsed time. Final success
still depends on booting the exact image, never on timing or counters alone.

The RAM verifier also fails on missing fields, corrupted content and near/far
wrong lengths. Hardware negative controls reject zero/oversized allocation,
truncated data and overflow; a complete 1,024-byte control passes. A failed
test leaves no cached pass and the receive buffer is freed on completion or
disconnect. Verification hashes the received bytes, and source plus inputs
are archived; it does not substitute sender enqueue counts. The host check
took 1.60 microseconds per call for the 1,024-byte control over 10,000 calls;
large-stream hashing is included in the host timer, after the receiver's
first-to-last-byte timer. The test explicitly reports RAM reception, not OTA.

## Additional centrals: rpi.local and farmgw

The user authorized both hosts on 2026-09-11. Each uses an isolated
`~/ota-throughput-20260911` directory and venv, with Bleak 3.0.2, detools
0.53.0 and tamp 2.3.0. Source and payload hashes were checked after copying;
existing gateway tooling was not replaced. Only one central connected to
the bench node at a time.

| Central / adapter | Actual PHY | Raw full OTA kB/s | Receiver RAM kB/s |
|---|---|---:|---:|
| rpi, TP-Link UB500 USB / hci0 | 2M | 26.236 | 27.587 at 7.5 ms |
| rpi, built-in / hci1 | 1M | 48.307 | 52.985 at 7.5 ms |
| farmgw, built-in / hci0 | 1M | 44.386 | 49.531 at 7.5 ms |

All raw rows used the same XipQuiet receiver and 754,624-byte X target, with
complete program/erase accounting, exact post-boot image and changed slot.
The USB row used 495-byte values; the 1M rows used 400. These are individual
observations, not host-independent limits. RSSI at Pi discovery was about
−30 dBm. The built-in adapters report no LE 2M support.

BlueZ's `AcquireWrite` socket path did not rescue the USB adapter: at 10 ms,
RAM measured 20.805 kB/s versus 20.772 through D-Bus. Its socket sweep at
7.5 / 15 / 30 ms measured 27.587 / 13.793 / 6.924 kB/s. The 125 ms attempt
hit the 180-second deadline before a verified result and was interrupted
with SIGINT, allowing Python's disconnect cleanup. The following connection
list was empty. The timeout is not a throughput measurement.

HCI capture shows outgoing 27-byte fragments despite 251-byte negotiated
BLE data length. A subsequent LE Read Buffer Size query returned 27-byte
buffers, count 8, for USB hci0; the built-in hci1 returned 251, count 7.
Do not confuse HCI buffer size with negotiated link-layer data length. These
observations narrow the USB bottleneck; they do not establish a universal
rate ceiling or justify sending HCI packets larger than the controller allows.

An initial non-default-adapter raw attempt failed before transfer. Explicitly
passing the adapter to both discovery and connection fixed it; the rerun
logged the actual `/org/bluez/hci1/...` path. See
[the adapter correction](bluez-adapter-selection.md). Older RAM rows predate
that path diagnostic; the final raw rows include it.

## Reproduction and retained artifacts

The [evidence directory](evidence/2026-09-10-throughput-headroom/README.md)
contains result-to-log links, timestamped receiver replies, build identities,
source snapshots, failed attempts, native tests and sanitized air metadata.
The source snapshots are reviewable benchmark implementations, including the
flash strategies and host alternatives; they are not shipping configuration
changes. The compatible tamp encoder and final-credit fix are committed
separately as `d819450` and `5663ec1`.

The retained local bench workspace is
`/Users/fab/o2p-push/ota-throughput-next`. Its `build-identities.json` names
the exact receiver and payload binaries. `save_build.py` retains each binary,
ELF and source fingerprint before another PlatformIO environment can replace
its build directory. Rebuilt images can have different identities: never
reuse the recorded expected SHA for a newly built binary.

To repeat a strict 244-byte native trial using those retained files, first
install the measured receiver, run the trial with a new evidence name, and
restore the receiver afterwards:

```sh
cd /Users/fab/o2p-push/ota-throughput-next
/tmp/otavenv/bin/python -c 'from bench_driver import install; install("recheck-install", "Graceful")'
/tmp/otavenv/bin/python run_native_tuned.py Graceful recheck-raw --chunk 244
/tmp/otavenv/bin/python -c 'from bench_driver import install; install("recheck-restore", "Graceful")'
```

The tools refuse to replace an existing log. The raw trial changes the
running image to X; installations are recorded separately from measurements.
`run_native_xform.py Graceful NEW_NAME tamp` or `delta` uses Y and requires
the same restore sequence. Keep all other bench clients disconnected.

The SDK rebuild uses `esp32s3_throughput_xip` and its derived environments
with the private `PLATFORMIO_CORE_DIR`, not the global precompiled SDK.
`source/build-sources/Graceful/` records the final receiver configuration.
Its important settings are the XIP fetch/rodata flags, 80 MHz OPI PSRAM,
50 MSYS1 / 96 MSYS2 / 96 incoming ACL buffers, CE mode 2, exact 10 ms
request, 2M PHY, 4 KiB internal write batches, 64 KiB bounded erase-ahead,
credit deduplication and graceful completion. Both endpoints must still
report the negotiated link and exact running image.

To reconstruct a source tree, start from the node and library revisions
listed in the evidence README, apply their starting patches, then overlay
the selected snapshots. Use `--exclude='host/__pycache__/*'` when applying the
library starting patch; its bytecode difference is not source. Supply a
local bench `secrets.h` from the example, pin NimBLE-Arduino 2.5.1, and update
the private library symlink for the new checkout. Original binaries, ELFs,
credentials and unsanitized air payloads remain outside this archive.
