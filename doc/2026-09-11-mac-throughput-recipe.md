# How the Mac reached 87.17 kB/s raw BLE OTA

The ESP32-S3 bench node reached **87.168 kB/s mean across three full raw
rewrites**, up from the verified **43.075 kB/s** baseline: **2.024×**.
This combined receiver and sender configuration was measured on September
10–11, 2026. It is an experimental configuration, not the library defaults
or a guarantee for another central.

The crux was keeping reception and flash work moving together, then feeding
CoreBluetooth whenever it reported room. Several changes contributed; the
experiment does not isolate an additive speedup for every setting.

## What changed

| Area | Tested configuration | Purpose and evidence |
|---|---|---|
| Flash erase | Bounded 64 KiB erase-ahead, with full rewrites forced | Replaced repeated sector erases. At 15 ms, the initial sector path measured 42.72 kB/s versus 49.67 for block erase. |
| Flash programming | 4 KiB internal-RAM batches; 256 KiB PSRAM receive ring | Buffers incoming traffic while flash work runs. Other batch sizes were tested; larger was not consistently faster. |
| Execution during flash work | Rebuilt ESP-IDF with PSRAM instruction and read-only-data execution | With both flags enabled, the inspected IDF flash implementation selects `SPI_FLASH_CACHE_NO_DISABLE`, avoiding the ordinary cache-disable path. |
| BLE link | Actual 10 ms interval, 2M PHY, 251-byte data length, ATT MTU 517 | The Mac accepted 10 ms; 7.5 and 8.75 ms requests did not establish those intervals. |
| Receive resources | 50 MSYS1 blocks, 96 MSYS2 blocks, 96 incoming ACL buffers; controller CE mode 2 | Verified by receiver diagnostics. Compiler defines alone had previously failed to override an SDK alias. |
| Credit notifications | Suppress repeated identical final grants; retain periodic recovery notification | Earlier runs emitted hundreds of duplicate final grants. The correction is committed as `5663ec1`; see [credit deduplication](credit-notification-deduplication.md). |
| Mac sender | Native Swift/CoreBluetooth, 244-byte GATT writes without response | Check `canSendWriteWithoutResponse` before **every** write, and resume directly from its readiness callback. Retain the receiver's OTA credit limit. |
| Completion | Explicit BLE disconnect after successful image validation and boot-partition selection, before restart | Reduced final-OK-to-disconnect from approximately 820 ms to 52 ms. This removes detection delay; it does not increase radio bitrate. |

The SDK flags are `CONFIG_SPIRAM_XIP_FROM_PSRAM`,
`CONFIG_SPIRAM_FETCH_INSTRUCTIONS`, and `CONFIG_SPIRAM_RODATA`.
The tested board has 8 MiB of 80 MHz OPI PSRAM. Its flash reports 16 MiB,
while its application partition layout remains 4 MiB with 1,572,864-byte
OTA slots. The SDK was rebuilt privately; changing application defines
against the ordinary precompiled SDK does not reproduce this setup.

## What the number proves

The target was the same **754,624-byte X image** for every raw repeat.
Every successful run required all 754,624 bytes programmed, exactly
757,760 bytes of aligned erase coverage, and an independent reconnection
confirming the exact target image in the other running slot. No sector
skipping, compression or delta reconstruction contributes to the raw rate.

The timer runs from OTA begin/push entry through reboot disconnect.
Discovery, authentication and later boot verification are outside that
timer. Decimal kB/s means bytes divided by elapsed seconds and 1,000.

The dedicated serial CoreBluetooth delegate queue produced
**86.766 / 86.797 / 87.943 kB/s**, mean **87.168**. Its requested
user-interactive QoS was confirmed through the pthread getter. Selection
pilots were excluded. An ordinary native queue independently produced
**84.806 / 86.301 / 88.101 kB/s**, mean **86.403**. The difference between
these means is within the observed variation: no separate QoS benefit was
established. This agent's compiler was idle during the final measurements;
unrelated applications remained active on the shared Mac.

Raw OTA above 100 kB/s remains undemonstrated. RAM-only reception and
transformed-image rates above 100 are separate measurements.

## Exact tested artifacts

- Host: Apple M3 Pro, macOS 15.7.3, Swift 6.2.4; BCM_4388 controller,
  firmware 22.5.542.2785, PCIe transport.
- Receiver: `Graceful`, NimBLE-Arduino 2.5.1 and rebuilt ESP-IDF 5.5.5;
  image identity `9f8abf18b2ad59476c71f4c3ab757b6e9bc3fb8b7f5f5a95cb64263f015ba587`.
- [Receiver source and SDK configuration](evidence/2026-09-10-throughput-headroom/source/build-sources/Graceful/).
- [Native sender](evidence/2026-09-10-throughput-headroom/source/native_push_priority_queue.swift)
  and [verification wrapper](evidence/2026-09-10-throughput-headroom/source/run_native_priority_queue.py).
- [Dedicated-queue repeat results](evidence/2026-09-10-throughput-headroom/priority-queue-summary.json)
  and [ordinary-queue repeat results](evidence/2026-09-10-throughput-headroom/final-repeat-summary.json).
- [Full experiment report and reproduction steps](2026-09-10-throughput-headroom.md),
  including unsuccessful settings, source fingerprints and retained logs.

These artifacts preserve the tested bench implementation. The receiver
variants and Swift frontend have not been promoted to production defaults.

The [Pi follow-up and native Mac port trials](2026-09-11-rpi-throughput.md)
subsequently reached 126.88 raw kB/s on the Pi with an experimental USB HCI
override. Fourteen additional native Mac raw rewrites found no gain from
the Pi's interval/packing settings; the Mac retained 10 ms and 244-byte
values, with a fresh three-run mean of 87.48 kB/s.
