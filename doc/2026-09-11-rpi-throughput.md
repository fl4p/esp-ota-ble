# Raspberry Pi BLE OTA throughput follow-up

Measured 2026-09-11: **126.877 kB/s mean full raw OTA on `rpi.local`**, with
three verified runs above the requested 120 decimal kB/s target. This uses
the TP-Link UB500 USB adapter, Bumble, 2M PHY, 12.5 ms connection interval,
495-byte GATT values, and an **experimental, nonstandard 251-byte host HCI
packet-size override**. The controller still reports 27 bytes. This is a
bench result, not a supported controller setting or a production default.

| Full raw repeat | Begin-to-disconnect kB/s |
|---|---:|
| 1 | 128.059 |
| 2 | 126.694 |
| 3 | 125.879 |
| Mean | **126.877** |

Each run programmed all **754,624 image bytes**, erased **757,760 aligned
bytes**, kept zero sectors, and booted the exact X image on app0 after
Graceful ran on app1. Between runs Graceful was restored by delta over the
same Pi adapter; restoration rates are excluded. Scanning, connecting,
setup and post-boot verification are outside the OTA timer. Begin/erase,
payload transfer, finalize and graceful disconnect are inside it.

A preceding **6 MiB RAM integrity test** received the correct length and
SHA-256 over 43.373068 seconds at **145.054 receiver kB/s** (144.220 host
kB/s including command/response overhead). This supports sustained transfer
integrity; it is separate from the full-flash result.

The ESP32-S3 receiver is the same `Graceful` image as the
[Mac configuration](2026-09-11-mac-throughput-recipe.md). No firmware rebuild
was needed for the Pi gain. The earlier USB raw result was 26.236 kB/s;
the new mean is 4.84 times that historical observation, with several host
settings changed. The matched RAM comparisons below identify the two
useful levers more directly.

## USB dongle tests

The TP-Link UB500 (`2357:0604`, MAC `AC:A7:F1:83:27:AD`) exposes eight
27-byte LE HCI buffers. The initial trials below use those advertised limits.
Later trials explicitly override the HCI packet size; the credit count
remains eight. The override is experimental and is not a production default.
The BLE connection itself negotiated 2M PHY and 251-byte data length.

| Bumble transport/settings | Receiver RAM kB/s |
|---|---:|
| HCI user channel, 7.5 ms, 495-byte values, 262,144 bytes | 27.762 |
| Direct libusb, same interval/value/payload size | 24.545 |
| HCI user channel, 7.5 ms, requested CE length 7.5 ms, 495-byte values | 27.828 |
| HCI user channel, 30 ms, requested CE length 30 ms, 495-byte values | 6.795 |
| HCI user channel, 125 ms, requested CE length 125 ms, 495-byte values | 1.675 |
| HCI user channel, 7.5 ms, 209-byte values | 27.265 |
| HCI user channel, 7.5 ms, 244-byte values | 25.814 |
| HCI user channel, 7.5 ms, 512-byte values | 27.179 |

The last six rows use 131,072-byte payloads. Every row passed the received
length and SHA check. Longer intervals were confirmed in receiver
diagnostics; the CE length is a host request, not a measurement of actual
radio-event airtime. These are individual exploratory samples.

Direct USB and bypassing BlueZ did not rescue this dongle's throughput.
The earlier BlueZ full raw result was 26.236 kB/s; its RAM result at 7.5 ms
was 27.587 kB/s. These findings do not establish a universal controller
limit. See the [earlier host comparison](2026-09-10-throughput-headroom.md#additional-centrals-rpilocal-and-farmgw).

### Explicit HCI packet-size experiment

After the initial sweep, an explicitly labelled 251-byte HCI packet-size
override passed a small 8,192-byte RAM length/SHA test. The controller still
advertises only 27 bytes. With the override, 524,288-byte RAM tests verified
65.056 kB/s at 7.5 ms/244-byte values and, using 495-byte values,
65.953 / 128.008 / 65.648 / 15.824 kB/s at 7.5 / 15 / 30 / 125 ms.
Shorter intervals are not monotonically better on this controller.

A full raw rewrite at 15 ms then verified **117.359 kB/s** in 6.430057 s,
with all 754,624 bytes programmed, exact 757,760-byte erase coverage, and
the exact X image booted on the other slot. The receiver was restored over
the same Pi USB adapter. These are pilots, not repeat means.

The wrapper's original `hci_advertised_packet_bytes` field described the
host queue's initial size, not a fresh controller query. A 251 value on
reconnection prompted a direct `LE Read Buffer Size` check after link setup:
that returned **27 bytes, eight packets**. Therefore no standard-compliant
buffer refresh fix has been established; do not mislabel the override as
following a freshly advertised 251-byte capacity.

Refreshing the host queue from that direct response stayed at 27 bytes and
verified only **13.870 RAM kB/s at 15 ms**. Merely re-reading the buffer
capacity does not produce the gain.

With the explicit 251-byte override, the finer 524,288-byte RAM sweep was:

| Actual interval, ms | Receiver RAM kB/s, 495-byte values |
|---|---:|
| 10 | 73.703 |
| 11.25 | 77.087 |
| 12.5 | **143.667** |
| 13.75 | 136.885 |
| 15 | 128.008 |
| 16.25 | 120.779 |
| 17.5 | 113.104 |
| 18.75 | 103.962 |
| 20 | 98.512 |

The 15 ms row is from the preceding sweep. All are individual exploratory
samples; the chosen 12.5 ms setting then passed the long RAM test and three
full raw rewrites above. HCI packet size describes host/controller framing;
251-byte radio data length and 495-byte GATT values are different quantities.
These measurements suggest controller event scheduling matters, but no
packet-per-event airtime trace was collected in this follow-up.

A **502-byte HCI override failed** even the 8,192-byte RAM pilot: the
receiver reported the correct byte count but the wrong SHA-256. It was
rejected, larger tests were cancelled, and it was never used for flash.
The final wrapper permits only the owned UB500 and a 251-byte override.
The failed log is retained as a useful integrity-check calibration.

## Built-in adapter discovery

The built-in adapter is `2C:CF:67:AA:43:02`, with seven 251-byte LE HCI
buffers. Initial Bumble discovery attempts failed before transfer because
the name filter did not match. An unfiltered scan actually received the
bench node's advertisements: an interim interpretation that the node had
stopped advertising was incorrect. A USB reset was performed during that
investigation, but it did not fix the name-based discovery failure.

Selecting the known bench MAC `7C:4F:AD:20:2A:29` found an advertisement
whose name and local-name fields were both `None`. The working trial used
legacy active scanning and the adapter's public address, then verified
the exact receiver image before sending data. These settings were changed
together; the result does not isolate a benefit from legacy scanning or
public addressing. The subsequent link negotiated 1M and 7.5 ms.

This adapter verified one **96.915 RAM kB/s** pilot (262,144 bytes) and one
**88.272 full raw kB/s** pilot (8.548858 seconds, exact full programming,
erase coverage and X/app0 boot). The earlier BlueZ built-in raw observation
was 48.307 kB/s with different receiver and chunk settings; this is not an
isolated Bumble-versus-BlueZ comparison. Repeated connection establishment
failures, reason 62, prevented reliable repeats. Keeping the backend alive
across scan/connect and trying a 100 ms initial interval did not resolve
those failures. A native Mac delta restored Graceful after this pilot.

One early diagnostic ran `hciconfig` while Bumble owned the adapter and
produced unexpected HCI responses; that attempt is confounded. HCI tools
can issue commands even when used for diagnostics. Later experiments used
only the owning Bumble connection for HCI queries.

## Host buffering and verification

Bumble 0.0.233 and the copied bumble-bleak 0.1.0 facade run in the isolated
test environment. A private wrapper reuses the existing OTA/RAM verifiers.
It bounds the host's pending HCI queue to 256 packets before another GATT
write and waits on HCI completion events with a timeout. Controller credit
accounting remains Bumble's own. RAM end waits for the pending HCI queue;
OTA end additionally requires the receiver's final flash progress.

Missing results, timeouts and discovery errors remain failed/unverified
attempts. Sender enqueue rate is never substituted for received goodput.
The facade omits Bleak's maximum-write property: an early full-flash attempt
therefore correctly refused a zero capacity before sending. The wrapper
explicitly supplies the known bench characteristic's 512-byte capacity,
checks its write-without-response property, and bounds values by the actual
exchanged ATT MTU (517 here). It does not claim to discover a missing
property. The host's `paced_writes=0` counter belongs to another backend;
it does not describe this wrapper's completion-event pacing.

The X target lacks the bench-only `link`, `dleinfo` and `rssi` commands.
Their rejected post-boot diagnostic replies are retained. The subsequent
`info` reply supplies the exact boot image and slot used for verification.

### Verification review

This follow-up reuses the existing RAM and full-rewrite predicates. The
archived replay check exercises the captured good and bad observations
without making another BLE connection.

1. **Unevaluable input:** missing fields, failed probes, timeouts and
   exceptions cannot qualify a run. Full-rewrite accounting returns
   `None` for unavailable instrumentation; callers require literal `True`.
2. **Monotonicity:** missing/short/long byte counts, zero/negative elapsed
   time, and wrong digests fail. Replay covers near misses and values out
   to ±10¹²; worse input does not recover a pass.
3. **Preconditions:** the actual characteristic property and exchanged
   MTU are checked before the wrapper supplies the bench-specific value
   capacity. Exact running receiver identity is checked before payload.
   The initial missing-capacity refusal demonstrates this boundary.
4. **Source of truth:** final Pi stdout fingerprints the executed wrapper,
   bench and host modules. The archive also fingerprints dependencies,
   sources and image identities. Initial host queue size and the direct
   controller response have separate diagnostic fields.
5. **Persistence:** failed rows and logs remain failed; no success cache
   is used. The wrapper snapshots preserve the diagnostic-field correction
   and the rejected 502-byte experiment. `bumble_entry_v10.py` is the exact
   final measured wrapper; `bumble_entry.py` corrects only a stale comment
   about the packet override after measurement, with executable code unchanged.
6. **Provenance:** rates require received integrity or exact post-boot image
   identity plus full-program/erase accounting. RAM, delta restoration,
   rejected diagnostics and pre-transfer failures are labelled separately.
7. **Known-bad calibration:** the actual 502-byte HCI test received 8,192
   bytes with the wrong hash and failed. It never became a throughput win.
8. **Fix versus mute:** the successful 251-byte experiment changed HCI
   fragmentation and measured received goodput. It did not weaken the
   controller-query report, OTA credit checks or image verification.

On the Pi, replay timing was **4.124 ms per 6 MiB SHA/length check** (100
iterations) and **0.742 µs per flash-accounting check** (10,000 iterations).
These host checks run after the timed receive/OTA operation. The wrapper's
queue wait is inside the measured transfer, so its cost is included in
the reported rates; its isolated cost was not inferred from a no-wait
microbenchmark. See `rpi-verifier-check.log` and `check_rpi_verifiers.py`.

## Porting the settings back to native Mac Bluetooth

Fourteen full raw rewrites tested the applicable interval/write-size
settings with the existing native CoreBluetooth sender and the same
Graceful receiver. Every row below passed full programming, erase coverage,
exact X boot identity and changed-slot checks. All requested intervals in
this sweep were actually accepted, with 2M PHY; these are not labels for
unaccepted requests.

| Actual interval, ms | 244-byte GATT, raw kB/s | 495-byte GATT, raw kB/s | 512-byte GATT, raw kB/s |
|---|---:|---:|---:|
| 10 | **87.476 mean of three** | 89.085 | 74.607 |
| 12.5 | 71.740 | 70.255 | 61.749 |
| 13.75 | — | 65.009 | — |
| 15 | 61.240 | 61.275 | — |
| 16.25 | — | 57.801 | — |
| 20 | — | 49.328 | — |
| 30 | — | 34.057 | — |

Other cells are single exploratory trials. The fresh 10 ms/244-byte
controls were **87.495 / 86.196 / 88.736 kB/s**, consistent with the earlier
87.168 mean. The 89.085 single observation uses an already-tested Mac
configuration; it is not evidence that a new Pi optimization improved the
Mac. The 12.5 ms Pi winner was slower here, and 512-byte writes also lost.

The Mac keeps **10 ms, 244-byte writes and the native readiness callback**.
Every write in these trials used `burst=1`, checked CoreBluetooth readiness
and receiver credit, and ran on the verified user-interactive delegate
queue. There was no unchecked write batching. The sender calls Apple's
GATT API and has no Bumble HCI fragmentation queue to override; the Pi's
251-byte host-queue mutation was not ported to this native implementation.
The shared Mac and RF environment were not isolated, so small differences
between the two sets of baseline repeats do not establish a gain.

## Reproduction and scope

Hardware: Raspberry Pi running Linux 6.12.47+rpt-rpi-2712, Python 3.11.2,
BlueZ 5.66; TP-Link UB500 `2357:0604`, RTL8761BU, HCI revision `dfc6`,
LMP subversion `d922`. Isolated Python environment: Bumble 0.0.233,
bumble-bleak facade 0.1.0, Bleak 3.0.2. The existing Realtek firmware files
were used. CPU governor was `ondemand`, throttling flags were zero in the
snapshot, and USB runtime power control was already `on`.

The private work directory is `/home/fab/ota-throughput-20260911` on the Pi.
Only bench MAC `7C:4F:AD:20:2A:29` is selected, with exact receiver identity
checked before data. Running another central or HCI diagnostic against the
same adapter invalidates the experiment. The user explicitly authorized
stopping `smart-shunt-ble.service` and borrowing this adapter; service
restoration is recorded with the final evidence.

The verified experiment's invocation, from that prepared directory with
the bench receiver installed and the adapter exclusively available, is:

```sh
sudo -n env BENCH_PHY=2 BENCH_MAC_DISCOVERY=1 \
  BENCH_HCI_PACKET_OVERRIDE=251 BENCH_INTERVAL_MS=12.5 \
  timeout --signal=INT 150s venv/bin/python bumble_entry.py \
  node/tools/bench_ble_ota.py node_X.bin \
  --adapter AC:A7:F1:83:27:AD --chunk 495 \
  --setup 'interval 10' --setup link \
  --expect-base 9f8abf18b2ad59476c71f4c3ab757b6e9bc3fb8b7f5f5a95cb64263f015ba587 \
  --results rpi-bumble-flash.jsonl
```

`interval 10` uses 1.25 ms units; `BENCH_INTERVAL_MS` uses milliseconds.
The wrapper is a private experiment, not an installed host-library feature.
It deliberately exceeds the reported HCI packet length. Three raw repeats
and a 6 MiB hash check do not establish portability to other firmware,
controllers, RF conditions or unattended deployments.

## Final device and service state

The final Mac delta restoration verified Graceful's exact image identity
on app1. All test senders exited. At **07:07:05 CEST on 2026-09-11**,
`smart-shunt-ble.service` was restarted with its original configuration:
active/running, enabled, PID 3541320, zero restarts at the check. Fresh
journal entries show shunt ADC and temperature telemetry arriving again.
`fugu-ble-bridge.service` remained active. This verifies resumed reception;
it does not certify every field sensor or telemetry channel.

See `rpi-service-restoration.log` and `rpi-service-after-restart.log` in the
archive. The isolated benchmark environment and evidence were retained;
the smart-shunt environment and service configuration were not edited.

## External-source check

Local controller responses and captured verification results support the
performance claims. Two public primary sources were inspected in a separate
research check; neither certifies this override:

- [Bumble bench documentation](https://google.github.io/bumble/apps_and_tools/bench.html),
  current unversioned page inspected 2026-09-11: usage and central options
  describe PHY, interval, ATT MTU and extended data length. Its inspected
  option list does not document this override; no project-wide absence is
  inferred.
- [Realtek-authored RTL8822C firmware changelog](https://chromium.googlesource.com/chromiumos/third_party/linux-firmware/+/7d39ebcfb14f18101c94ff1b7153ee6d42a41a8e),
  authored 2025-05-15, mirror commit 2025-06-16: describes a correction to
  LE buffer-size reporting with length extension on **a different chip**.
  This neither establishes the same defect in RTL8761BU nor supports
  installing that chip's firmware here. No firmware was downloaded or
  installed.

Access log: both pages returned matching titles and native rendered DOM on
their first target fetch using an owned isolated Playwright headless shell
with plain Chrome UA; 6,628 and 8,843 UTF-8 bytes respectively. Before these
fetches, a missing default browser binary and a timed-out scratch CDP
connection were resolved by using the installed headless shell. The owned
browser was closed; no live user profile was accessed. No source fetch
remained blocked. Searches used `serper-search` with RTL8761/Realtek,
27/251-byte buffers, Bumble and throughput variants; search results were
discovery only and do not establish an absence or a hardware ceiling.

## Evidence

The [evidence directory](evidence/2026-09-11-rpi-throughput/) contains commands,
successful and failed logs, full timestamped OTA records, source snapshots,
dependency/source fingerprints, image identities and `SHA256SUMS`.
`rpi-bumble-runs.jsonl` records the later trial names, command arguments,
environment and outcomes; two initial transport pilots have separate logs.
The remote RAM matrix also includes earlier Pi measurements and must be
filtered by the named trial logs. Firmware binaries, credentials and SDKs
are excluded. The existing receiver/build provenance remains in the
[earlier evidence](evidence/2026-09-10-throughput-headroom/).
