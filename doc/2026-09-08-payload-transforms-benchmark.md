# Delta and compressed OTA payloads, measured on hardware

**Board:** fugu `flu` (ESP32-S3, no PSRAM), 1.77 MB image into a 1828 KB slot.
**Host:** macOS, direct CoreBluetooth, `mtu=247 chunk=244`.
**Code:** esp-ota-ble `3f3349b`, fugu `3e0b6b4`. **Date:** 2026-09-08.

## Why this exists

The BLE link is saturated and cannot be made faster. Profiling a raw push from the
Mac gave `WALL 57.1s writes=7306 in_write=0.47s` — the host spends essentially no
time writing — and `GAPS >5ms: n=384 sum=28.7s`. Those 384 stalls line up with the
~430 credit grants (`CRED_STEP` = 4 KB), at ~75 ms each, and **4096 B / 75 ms =
55 kB/s**: the stall *is* the air time for one credit window.

Everything else had already been ruled out by measurement, not argument:

| lever | result |
|---|---|
| bigger ATT MTU | 247 already fills one LL PDU |
| bigger credit window | 8 KB against a ~1 KB bandwidth-delay product |
| LE 2M PHY | engages (`tx=2 rx=2`), **no change**: 36.5/38.4/38.2 s vs 38.3/35.9/35.6/39.2 s |
| shorter connection interval | Apple's Accessory Design Guidelines pin an accessory to ≥15 ms |

So the only remaining lever is sending fewer bytes.

## Results

Transfer is the device's own `OTAB READY` → final `OTAB PROG`; erase is `begin` →
`READY` from device log timestamps; `write_ms` is the device's `OTAB STAT`, the time
inside `otaXformFeed` (reconstruction **plus** `esp_ota_write`).

| transform | wire bytes | % of image | erase | transfer | wire rate | write_ms | total wall |
|---|---|---|---|---|---|---|---|
| raw | 1 769 984 | 100 % | 5.8 s | **36.55 s** | 47.3 kB/s | 9015 | 61.0 s |
| raw | 1 772 512 | 100 % | 5.9 s | **38.15 s** | 45.4 kB/s | 9065 | 62.5 s |
| tamp | 1 223 703 | 69.0 % | 5.7 s | **25.49 s** | 46.9 kB/s | 8731 | 49.9 s |
| tamp | 1 223 703 | 69.0 % | 5.8 s | **25.07 s** | 47.7 kB/s | 8708 | 49.5 s |
| delta | 69 294 | 3.9 % | 5.8 s | **11.27 s** | 6.0 kB/s | 8503 | 35.9 s |
| delta | 95 202 | 5.4 % | 6.1 s | **11.51 s** | 8.1 kB/s | 8675 | 36.2 s |

**Transfer 3.3× faster, wall clock 1.7×.**

## What the numbers actually say

**The wire rate is the same ~47 kB/s for raw and tamp.** That is the independent
confirmation that the link was the constraint and that compression does nothing
clever — it just has fewer bytes to push through the same pipe.

**Delta moves the bottleneck onto the device.** Of an 11.3 s delta transfer, 8.5 s is
inside `otaXformFeed`; only ~2.8 s is link time. That is why a 95 KB patch (11.51 s)
costs the same as a 69 KB one (11.27 s): **patch size has almost stopped mattering.**

**`write_ms` is ~8.5–9.1 s in every row**, because all three write the same 1.77 MB to
flash. For raw and tamp that cost hides underneath the link; for delta there is no
link time left to hide behind, so it becomes the floor. Delta's ceiling is therefore
about `5.8 s erase + 8.6 s apply ≈ 14.3 s` of device time, however small the patch —
overlapping erase with reception is the only thing left after that.

**The remaining ~24 s of wall clock was not the push at all** — and has since been
removed; see the next section.

## Patch sizes, host side

Six real fugu build pairs, `detools create_patch -c heatshrink`:

| base → new | patch | % |
|---|---|---|
| build-cleanup → build-lf65 | 162 653 | 9.2 |
| build-lf65 → build-bleota | 78 139 | 4.4 |
| build-mcpwm → build-ledc | 173 941 | 10.3 |
| build-splitdt → build-cleanup | 149 243 | 8.5 |
| build-flu → build-mmio | 156 834 | 9.0 |
| build-lf65 → build-lf65 (identical) | 27 548 | 1.6 |

93 % of the bytes move between two real builds, yet the patch is ~9 % — the LZ delta
finds the shifted content. The identical-file row is the floor heatshrink's small
window imposes; lzma would do far better but the device's bundled detools decoder is
built for heatshrink only.

tamp is flat at 69.0 % regardless of window (70.3 % at w=10, 69.0 % at w=12, 69.3 %
at w=13, 70.1 % at w=14, 71.5 % at w=15), so w=12 is both the optimum and the cheap
option at 4 KB of RAM. For reference: gzip -9 62.6 %, zlib -9 62.6 %, xz -9 55.8 %.

## Safety checks that ran on hardware

- **Image identity agrees across the two sides.** The host computes an image's id as
  its appended SHA-256 (`sha256(bin[:-32]) == bin[-32:]`); the device answered
  `OTAB BASE cfd7c0a7…`, byte-identical. This is the value the whole delta path hangs
  on and it is now confirmed, not assumed.
- **A patch against the wrong base is refused.** Pushed a patch built against
  `build-bleota` to a device running `build-xform`: refused at the first 2048-byte
  slice with `OTAB FAIL xform ESP_ERR_INVALID_VERSION`, nothing reached flash, boot
  partition untouched. This is the failure that matters — a wrongly-applied patch
  reconstructs a plausible image that fails only at boot, on a board whose previous
  firmware has already been erased.
- **Old device, new host.** Before the receiver was updated, the device reported
  neither transform and the host fell back to raw without incident — the first row in
  the table is that push.
- **Explicit `--xform delta` does not silently fall back.** With `detools` missing it
  failed with `rc=1` and said why, rather than quietly sending 1.77 MB.

## Gotcha found by measuring

The first build reported `OTAB XFORM raw,delta` — tamp had been compiled out
silently. `__has_include(<tamp/decompressor.h>)` fails when a project vendors the
upstream C sources under `components/`, because that layout exports the `tamp/`
directory itself and the header is `<decompressor.h>`. Fixed in `3f3349b`; the
availability list over the wire is what caught it, which is the argument for having
the device report its capabilities rather than the host assuming them.


## Follow-up: the host overhead, removed

The ~24 s that was neither transfer nor erase turned out to be almost entirely one
line. `BleakScanner.discover()` always sleeps out its whole timeout, so every push
paid a flat 15 s scan. The board is actually first seen **0.31 / 0.36 / 0.94 s** into
a scan (three runs, 2026-09-08).

Scanning live with a detection callback and stopping on the first exact name match
makes the timeout a *bound* instead of a duration — which also allowed raising it from
15 s to 30 s, the reliability fix the 2026-09-03 miss asked for (a 15 s `discover()`
found nothing where a 30 s one saw the board at RSSI −69, same room). Slow to fail,
fast to succeed. Plus: the version probe now waits for its answer rather than sleeping
a flat 2 s, and is skipped under `--force`.

Same board, same 95 202-byte payload (fugu `76eac0b`):

| | before | after |
|---|---|---|
| delta, total wall | 36.22 s | **21.23 s** |
| ├ host overhead (connect → begin, minus erase) | ~18.9 s | **~2.1 s** |
| ├ device erase | 5.8 s | 6.2 s |
| └ transfer | 11.51 s | 9.01 s |
| raw, total wall | 62.53 s | **41.68 s** |

A second fast-path delta run gave transfer 9.02 s — the same to 10 ms.

### What the speedup uncovered

The slow scan had been hiding a real hazard by accident. A freshly booted image is
`PENDING_VERIFY` until the firmware confirms itself at 20 s uptime, and
`esp_ota_begin` refuses until then with `ESP_ERR_OTA_ROLLBACK_INVALID_STATE`.
Back-to-back pushes used to clear that window because the host spent ~19 s scanning
first. They no longer do, and the next push failed at 2.37 s. `push()` now waits and
retries for that specific failure — reproduced and confirmed working — while every
other refusal still surfaces immediately.

### Where the 21 s now is

    ~2 s   host: scan, connect, ping, info
     6.2 s device: up-front erase of the slot
     9.0 s device: apply patch + write 1.77 MB (8.6 s of it is esp_ota_write)
    ~4 s   host: waiting for the board to advertise again after reboot

The link is now ~0.4 s of the whole thing. The next real lever is the **6.2 s erase**,
which could overlap with reception if the receiver erased ahead of the write pointer
in the background instead of all at once up front; that would put the floor near 15 s.
Below that sits the 8.6 s of flash writing, which nothing avoids while the full image
has to be written — and sector-level skipping does not help, because 403 of 433 4 KB
sectors differ between two real builds (93 %), code having shifted underneath.

> **Superseded 2026-09-08 (see Follow-up 3).** True as measured, but only of an
> *unstabilised* image layout. `fugu-mppt-firmware`'s `linker.lf` +
> `-fno-merge-constants` work took the same figure to **147/437 (33.6 %)**, which
> reopened this door and produced `2233857`. Note that 33.6 % is a **build-level**
> figure — that work is a prototype, byte-reproducible but never flashed — whereas
> 403/433 was measured. Do not read them as two hardware measurements.

## Follow-up 2: erase-ahead, and the limit of this design

Two changes, aimed at the two non-transfer items in the 21 s breakdown.

**Erase ahead of the writer.** `esp_ota_begin` erased the whole 1828 KB slot before
the first byte could arrive — 6.2 s of dead time. Erasing only a head block and
keeping the rest erased just ahead of the write pointer brings **READY down from 6.2 s
to 0.40 s**. It works because `esp_ota_begin` sets `need_erase` only for
`OTA_WITH_SEQUENTIAL_WRITES` and `esp_ota_write` never bounds anything against the size
passed to `begin`, so a small size buys a small erase *and* leaves the erasing to us.

It is not free, and the reason is worth keeping: **erase and write serialise on one
flash die**, so overlapping them cannot save write time — the only thing erase-ahead
can recover is *link* time. Worse, a 64 KB block erase (~215 ms) blocks the same
consumer task that drains the staging ring; at ~47 kB/s the host delivers ~10 KB in
that window, more than the 8 KB ring holds, so the link stalls.

| | before | after |
|---|---|---|
| delta, total wall | 21.23 s | **19.80 s**, repeated 19.37 s |
| delta, begin → READY | 6.2 s | **0.40 s** |
| raw, transfer | 32.8 s | 35.1–36.8 s |

So it is enabled only when the payload is under a quarter of the image — a wide margin
between the two regimes seen (5 % against 69 %/100 %). **Honest limit:** raw transfers
ranged 30.8–37.4 s across the day, so single runs cannot resolve a few seconds there;
the raw column is not a conclusion. The delta result is outside that noise and repeated.

**Faster return.** `verify()` slept 2 s before each of 20 scans, costing at least 2 s
even when the board was already back. It now settles 0.3 s and scans continuously.

### A scheduling bug the speedups exposed

The PENDING_VERIFY retry was 3 attempts 8 s apart. Starting ~3 s after a reboot, those
land at 3 s, 11 s and 19 s of uptime — **missing the 20 s gate by 0.6 s** and failing
the push. Confirmed in a log showing all three attempts. It is now bounded by time
rather than attempt count, because the window is a fixed property of the device.

### Where this design stops

    ~2 s   host: scan, connect, info
     6.2 s device: erase   ]  serialised on one die: 14.7 s of flash work
     8.5 s device: write   ]  that no codec and no overlap can remove
    ~1 s   host: reboot + re-advertise

A delta push is now ~19.5 s against a ~15 s floor of pure flash work. Getting below it
means writing fewer bytes to flash, and that door is closed: **403 of 433 4 KB sectors
differ** between two real builds (93 %), because code shifts. Flash write throughput is
190–205 kB/s regardless of chunk size (raw writes 244 B at a time, delta writes ~47 KB
at a time, and they land within 8 % of each other), so batching does not help either.

> **Superseded 2026-09-08.** The door was not closed — it was closed *for that image
> layout*. Stabilising the layout took the differing-sector rate to 33.6 % **at build
> level** (prototype, never flashed), and `2233857` now compares each sector and skips
> the matches. The flash-throughput sentence still stands. See Follow-up 3.

## Follow-up 3: sector-skip on hardware, and where the link actually binds

**Date:** 2026-09-10. **Receiver:** esp-ota-ble at `2233857` (the sector-skip commit,
which `4ea3c8f` above predates and contradicts). **Board:** bench ESP32-S3
`7c:4f:ad:20:2a:28`, env `esp32s3_psrambench`, bench tree `~/o2p-push/nodehead`.
**Image:** 717 904 B raw. **Host:** macOS / CoreBluetooth, plus one Raspberry Pi
comparison. All figures below are `--xform raw` — see the honest limits at the end.

### The confound this commit introduced

`2233857` makes **re-pushing an image the device already holds** a near-no-op: one
sector programmed instead of all of them, and roughly **10× the apparent throughput**.
The transfer completes, the digest verifies, the device reboots into the image. Only
the counters say it never wrote anything — and the natural way to benchmark an OTA is
to push the same image repeatedly, so this is easy to hit by accident. It caught this
session's measurements more than once.

| signature | meaning |
|---|---|
| `OTAB SKIP kept=175 wrote=1 erases=1 erase_ms=45` | no-op; nothing was programmed |
| `erase_ms` in the tens of ms (43–108 observed) | one sector — a skip |
| `erase_ms` ≈ sectors × 49 ms (6205–8687 observed) | genuine write |
| `erase_ms` ≈ 23 000 | full-slot erase, pre-`2233857` behaviour |

The arithmetic gives it away without knowing the skip path exists: 717 904 B in
`write_ms=272` is 2.6 MB/s, and this part programs at ~200 kB/s.

**A throughput figure quoted without one of these lines is not a measurement of an
OTA.** It measures the link plus the sector comparison — a legitimate number for a
different question, which must be labelled as such.

Forcing a real write needs two images differing almost everywhere, confirmed *per run*:
one Kconfig flag (`FUGU_WITH_BLE_ADV`) moved **4 of 437** sectors, while rebuilding the
same source at `-Os` instead of `-O2` moved **437 of 437**. Layout-stabilisation work
makes this harder, not easier — that work exists precisely to keep sectors identical.

Canonical: `~/dev/kb/esp32/ota-throughput-numbers-are-meaningless-without-the-sector-skip-line.md`.

### Where 33.6 % comes from: the one-log-line experiment

The figure that reopened sector-skipping is not a general "two builds" rate — it is a
controlled minimal edit. `fugu-mppt-firmware`'s layout work adds **one log line**, the
smallest realistic rebuild, and measures what that disturbs
(`doc/2026-09-08-image-layout-stability-for-ota.md`):

| build | image | +delta | sectors rewritten | differing bytes |
|---|---:|---:|---|---:|
| as shipped | 1 773 040 | +48 B | **420 / 433 (97.0 %)** | 89.4 % |
| `+ linker.lf` | 1 771 904 | +32 B | 230 / 433 (53.1 %) | 26.0 % |
| `+ linker.lf + -fno-merge-constants` | 1 786 256 | +32 B | **147 / 437 (33.6 %)** | 14.9 % |

Carry these caveats with the numbers, all from that source doc:

- **Build-level prototype — not flashed, not run on hardware.** Builds are
  byte-reproducible, but no board executed them. The 403/433 figure this supersedes
  *was* measured, so the two are not comparable in status.
- The A/B destination holds the image from **two** pushes ago; 147/437 is the
  consecutive-build rate, and the n-vs-n−2 rate can only be worse.
- The **first** push after adopting the layout change rewrites the whole image anyway,
  because every address moves.
- The denominators differ (433 → 437) because `-fno-merge-constants` grows the image by
  +14 352 B; the *rates* are comparable, the raw counts are not.
- "Differing bytes" is a **positional** comparison of a shifted image, not a measure of
  changed content: the same pair matches 10.6 % positionally, 61.7 % under one global
  realignment, and 97.6 % when each 4 KB block gets its own offset. The last is an upper
  bound that also rewards zero runs and `0xff` padding, not a content measurement.

### What the same experiment says about delta

This is the part that validates the delta rows in the Results table above, and it was
measured on the *same* one-line edit:

- `detools` (bsdiff + heatshrink — the on-device codec) encodes the entire difference in
  **56 406 B (3.18 %)**, falling to **35 434 B (1.98 %)** with the layout fix. It can say
  "the next 900 KB is the old bytes, moved by 24"; the flash controller cannot, which is
  the whole reason delta beats sector-skipping on the wire.
- The two benchmarked delta pushes above (69 294 B / 3.9 % and 95 202 B / 5.4 %) sit in
  that same range, so **those runs are representative of a one-line edit** rather than a
  favourable case.
- **Patch size barely moves the wall clock:** a 37 % larger patch (69 294 → 95 202 B) cost
  2 % more transfer time (11.27 → 11.51 s), because the receiver reconstructs and writes
  the full 1.77 MB either way.

The shift is a patchwork, not one offset — per-block best offsets are +24 (249 blocks),
+4 (53), +12 (45), +16 (36), +40 (31), 0 (18) — because the string pool, rodata and text
each grew independently. That is why no single realignment fixes it, and why the
**sector** column, not the byte column, is what predicts OTA cost.

### fugu baseline

**~40.7 kB/s, n=4**, A↔C alternation with 437/437 sectors differing, every run verified
a real write by `erase_ms ≈ 23 000`. Notably fugu achieves this with `OTAB RING 8192`
and `SPIRAM 0`.

### What moved the node from 2.22 to ~28 kB/s

**The staging ring was the entire early gap.** `RING_CAP_PSRAM` had been parked at 8 KB
on an earlier confounded measurement, and the benchmark was running `esp32s3_blebench`
(no PSRAM) rather than `esp32s3_psrambench`. Restoring 256 KB: **1.88–2.37 kB/s → 26–28
kB/s**, and the run-to-run spread collapsed from 3.35× to roughly ±3 % around a ~27
median (with occasional 2× slow outliers — one 13.84 kB/s run did not reproduce).

Verified real-write runs at 256 KB: 26.30 (`erase_ms=7438`), 27.96 (`erase_ms=7317`),
26.84 (`erase_ms=6205`).

**Flash is fully hidden at 256 KB.** A skip push measured 27.90 kB/s against real-write
runs of 26.30/27.96/26.84 — statistically identical. Two consequences: the link is the
sole limiter in this configuration, and skip-vs-skip comparisons become legitimate for
link work specifically.

### Levers tested and eliminated

**Connection interval is not available.** Measured on air with an nRF52840 sniffer: 948
events over ~28.7 s = **30.3 ms**, matching `interval=24`. Requesting a QA1931-compliant
`12..24` changed nothing — **macOS grants the range *maximum*, not the minimum** — and
since QA1931 forbids Max < Min+15 ms with Min ≥ 15 ms, 15 ms is unobtainable by asking.
An earlier claim in fugu's `bleota.cpp` that a grant at the 15 ms floor was "worth ~2×"
is refuted by this capture.

**Chunk size does nothing, and host pacing is required.** Full host matrix:

| chunk | `_await_writable` pacing | result |
|---|---:|---|
| 512 | on | 26.3–28.0 kB/s |
| 244 | on | 28.7 / 28.3 kB/s |
| 512 | off | `OTAB FAIL stalled` |
| 244 | off | `OTAB FAIL stalled` |

Matching fugu's 244-byte chunk changed nothing (`calls` rose 1400 → 2130 as expected).
A bytes-per-event argument predicted otherwise and was **wrong**: 512-byte writes do
fragment into 2.29 LL packets against 244's exactly one, and the ratio happened to match
the throughput ratio, but that was a consistency check, not a mechanism. Removing pacing
stalls at *both* chunk sizes, so the node needs `_await_writable()` where fugu does not —
which locates that difference on the **receiving** side, not the host.

### The model that fits

**Throughput ≈ bytes-per-connection-event ÷ interval.** With the interval pinned at
30 ms for both firmwares, the whole remaining gap is bytes per event. Sniffer, during a
real transfer: 3210 packets × 251 B ≈ 805 kB over 948 events = **~850 B/event / 30 ms =
28.3 kB/s**, against 27.97 measured. Events carry mean 11.76 packets and peak at 26, so
the link has **substantial headroom** — the transfer is not filling the events it has.

### Where it stands, and what is NOT established

**Node ~28 kB/s against fugu 40.7 — a 1.45× gap, down from 18× at the start.**

- **Leading candidate, UNTESTED: core pinning.** `CONFIG_ARDUINO_RUNNING_CORE` is **1**
  on the node and **0** on fugu, so fugu's OTA consumer shares a core with the NimBLE
  host while the node's crosses cores on every credit round-trip. That shape fits both
  the bytes-per-event cap and the stall asymmetry. It comes from the precompiled Arduino
  libs' sdkconfig, so the fix is a dedicated `xTaskCreatePinnedToCore(..., core 0)` task
  rather than a define. **Not measured** — the bench board's USB console died (fifth
  endpoint death that day) and flashing was blocked.
- **Every number here is `--xform raw`.** Delta was forced off on both sides for
  comparability, which was right for isolating a mechanism and wrong for the goal: the
  delta path that puts ~9 % of the image on the wire was never exercised in this
  comparison at all.
- The 13.84 kB/s outlier is unexplained; ±3 % overstates the stability of the 256 KB
  configuration.
