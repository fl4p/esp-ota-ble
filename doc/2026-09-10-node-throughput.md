# Node BLE OTA reaches fugu throughput

Measured 2026-09-10: **43.075 decimal kB/s mean** over three verified full raw
rewrites, versus **42.198 kB/s** from the historical fugu baseline on the same
physical PSRAM ESP32-S3 and macOS host. Earlier node runs were around 28 kB/s.
This establishes parity for these workloads, not a universal BLE throughput limit.

The receiver library does not own GATT negotiation or USB logging. The fixes
described here are in the node's integration and host wrapper; adding this note
does not install those policies in `esp-ota-ble` or other consumers.

## The working combination

1. **Request exactly 15 ms**, not a range of 15–30 ms. On this Mac, NimBLE's
   `updateConnParams(handle, 12, 12, 0, 72)` produced `interval=12` (1.25 ms
   units). The earlier `12..24` request produced 30 ms, which did not establish
   that 15 ms was unavailable. Log the accepted parameters, not just the request.
2. **Request 2M after interval negotiation.** The node waits for its connection
   parameter callback, with a bounded 1 s fallback, then requests 2M once.
   It defers negotiation during active OTA. The host allows 1.5 s after auth
   for setup. Final receiver queries reported `phy_known=1 tx=2 rx=2`.
   Compiled 2M support by itself is not evidence of an active 2M link.
3. **Use 495-byte values where the transport allows them.** With 251-byte LL
   payloads, two PDUs hold `2 * 251 - 4 L2CAP - 3 ATT = 495` value bytes.
   The node wrapper caps automatic selection at 495, retaining smaller backend
   limits and explicit overrides. The previous 244-versus-512 comparison did
   not test this packing. Preserve CoreBluetooth write pacing and OTA credits.
4. **Keep status delivery out of blocking USB output.** Route status to the
   transport driving the OTA. Mirroring BLE credits/progress synchronously to
   USB CDC can stall the consumer when nobody drains USB. Queue BLE output,
   and drain final sector accounting and completion status before restart.
   The last accepted run below had no USB reader open.

The measured configuration also uses a 256 KiB PSRAM staging ring and normal
`-Os` optimization. A 720 ms supervision timeout replaced the experimental
5 s setting; the latter added about 5 s to detecting a reboot disconnect.
This is completion-detection overhead, not five seconds of additional radio
throughput. The integration also avoids calling LoRa sleep in a bench build
that never initialized that radio.

**This was not a factorial experiment.** The final combination works, but
the individual gains from interval, PHY, chunk packing and status routing
were not isolated. In particular, 495 bytes alone initially performed poorly
with the earlier configuration. The no-PSRAM field node was not benchmarked
or deployed as part of this work.

## Accepted full-write measurements

Timing starts at `push_image()` entry and ends at the BLE disconnect after
reboot, including READY and flash completion. An early `OTAB OK` does not stop
the timer. Authentication/link setup and post-boot identity verification are
outside it. Rates use bytes / seconds / 1000.

| Run | Raw bytes | Elapsed s | kB/s | Final kept / wrote / erases | Final erase_ms |
|---|---:|---:|---:|---|---:|
| final-1 | 754,624 | 17.648 | 42.759 | 0 / 185 / 185 | 9828 |
| final-2 | 718,848 | 16.471 | 43.644 | 0 / 176 / 176 | 9422 |
| final-4 | 718,848 | 16.787 | 42.823 | 0 / 176 / 176 | 9396 |

Mean **43.075**, range **42.759–43.644 kB/s**. Every run starts in the same
receiver image and verifies a changed running slot plus the exact target
image SHA after reboot. Delta installations restore the receiver between
runs and are excluded from these raw measurements.

The five historical fugu rows recompute to 42.198 kB/s, range 41.262–43.741.
One handoff row had an arithmetic error: 1,650,912 bytes / 40.01 s is
**41.262**, not 40.1 kB/s. Fugu was not rerun in this continuation. Its
1.65–1.79 MB images are larger than the node payloads, so this is throughput
parity, not an equal-image-size comparison.

### Avoid the false full-write result

The excluded final-3 run was faster, **44.214 kB/s**, but its final line was:

```text
OTAB SKIP kept=9 wrote=176 erases=176 erase_ms=9388
```

For the active sector-comparison strategy, require `kept=0` and
`wrote=erases=ceil(image_bytes / 4096)`. Missing/malformed final accounting
is unverified. A large `erase_ms` proves substantial erase work, not that
every sector changed. `OTAB STAT` precedes final-tail processing; retain
the final `OTAB SKIP` line as well as post-boot identity.

Two-image alternation can write each image back into the slot that already
holds it. **Three distinct images are insufficient too:** a shorter image
leaves an older, larger image's tail intact, and a later push can skip that
tail. Arrange destination contents and check the counters on every run.
Other erase strategies need strategy-specific evidence; their erase timing
is not necessarily accumulated in the same counters.

## Levers for further speed

These are experiments, not demonstrated improvements beyond 43 kB/s:

| Lever | What to measure next | Existing limit on the claim |
|---|---|---|
| GATT queue depth and connection-event utilization | Instrument queued/in-flight bytes, credit stalls and payload per captured event on the final configuration; sweep bounded queue depth. | Removing pacing stalled. An earlier eight-write burst at 244 bytes did not help; it was not a sweep of the final 495-byte configuration. |
| Accepted connection interval | Sweep exact intervals while holding PHY, chunk size, queue policy and image work fixed. Count received bytes, not just successful host submissions. | 15 ms worked here; this does not show it is globally optimal or that shorter is always faster. |
| Flash erase/write strategy and overlap | Profile erase, program, read/compare and consumer scheduling. Compare sector skipping with another erase strategy for full rewrites, retaining watchdog and quiesce behavior. | The final runs spent 14.0–14.6 s inside the reported write/reconstruction path, versus 16.5–17.6 s total. These times overlap link work and cannot simply be added or subtracted. Erasing ahead destroys the old bytes needed for skip comparison. |
| Fewer transmitted and rewritten bytes | Use delta/compression and stable image layout for ordinary updates; report wire bytes, reconstructed bytes, skipped sectors and complete update time separately. | Existing [transform measurements](2026-09-08-payload-transforms-benchmark.md) show this can shorten updates. A short delta restore with mostly unchanged destination sectors is not a full-rewrite speed measurement. |

For raw throughput, first locate whether the final configuration leaves the
controller queue empty or the flash consumer saturated. For practical small
firmware updates, sending and rewriting fewer bytes is a separate useful goal.
No next-speed figure is established by these observations.

## Evidence and implementation provenance

Committed alongside this note: [result records](evidence/2026-09-10-node-throughput/results.json)
and receiver logs for [run 1](evidence/2026-09-10-node-throughput/final-1-raw.log),
[run 2](evidence/2026-09-10-node-throughput/final-2-raw.log),
[excluded run 3](evidence/2026-09-10-node-throughput/final-3-raw.log), and
[run 4](evidence/2026-09-10-node-throughput/final-4-raw.log).

The measured node image is 721,424 bytes, built in `esp32s3_psrambench`, with
embedded image SHA
`19ee4942881a9fcd451dcd23fed1a348e08d0676758d5552d8fcc31d60677cdd`.
It is based on node commit `a0773bcd68656a0793571814056a7c3d6f282326` plus
the recorded handoff/continuation patches, and esp-ota-ble commit
`496ebdb2a74672b8e4e8bca654cd5639e99d5fb8` plus recorded local changes.
Neither commit alone identifies the tested binary.

Full local source patches, final fingerprints, selected compiler commands,
build/test logs and fugu's original receiver logs are archived in
`/Users/fab/dev/ha/farming/docs/evidence/2026-09-10-node-ble-ota/`.
The report is `docs/2026-09-10-node-ble-ota-throughput-continuation.md` in
that repository; tools live in `/Users/fab/o2p-push/nodehead/tools/`.
Exact test binaries and the final ELF are retained at
`/Users/fab/o2p-push/ota-throughput-20260910/`.
