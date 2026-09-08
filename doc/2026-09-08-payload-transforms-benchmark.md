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
