# Throughput experiment evidence

See [the measured results and reproduction notes](../../2026-09-10-throughput-headroom.md).

- `results.json` links each recorded result to its complete log. `purpose`
  distinguishes installs, full-write measurements, sector updates and RAM-only
  reception. A failed run may have only a log, with no result row.
- `flash-matrix.jsonl` and `link-matrix.jsonl` retain the receiver's timestamped
  status lines; `ram-matrix.jsonl` retains received-byte/hash validation.
  `native-matrix.jsonl` retains native GATT/CoC lines and post-boot checks.
  Remote raw results are in `results.json` and their logs; those rows do not
  include a complete timestamped list of intermediate PROG/CRED notifications.
- `build-identities.json` distinguishes the running ESP image identity from
  the SHA-256 of the entire binary. Binaries and ELFs remain under
  `/Users/fab/o2p-push/ota-throughput-next/`, named `node_<label>.bin/.elf`.
- `logs/` includes unsuccessful attempts, compiler logs (gzip), native failure
  controls and passing tests. Discovery failures did not start a transfer.
  Other logs over 1 MiB are also gzip-compressed.
- `source/` contains the experiment drivers, explicit source snapshots and
  starting patches. It excludes `secrets.h`. The node base is commit
  `a0773bcd68656a0793571814056a7c3d6f282326`; apply its starting patch, then
  overlay the relevant node snapshot in a complete checkout. The library
  starting patch is against `4dcef1e`; source snapshots carry the experiment's
  selected implementation and shared starting work. These are benchmark
  artifacts, not changes to the library's production defaults.
- `source/versions/` preserves earlier source generations. `source/xip-sdk-evidence/`
  records the rebuilt IDF 5.5.5 configuration and flash OS implementation;
  XIP uses a private `PLATFORMIO_CORE_DIR`. Ordinary variants use the original
  prebuilt SDK. Do not rebuild XIP against unmodified precompiled libraries.
- `source/build-sources/<label>/` records per-build source and SDK snapshots
  from Ce0 onward. Their fingerprints are in `build-identities.json`.
  `CocInitial` was built but never installed; only the later heap-owned
  callback version was tested on hardware.
- `remote-sources.json` records the independently copied host tools on
  `rpi.local` and `farmgw`. Remote adapter metadata and the USB HCI capture
  are retained in logs. Existing gateway tools were not replaced.
- Air captures retain hardware timestamps. Their initial buffered packets
  include stale traffic from the previous sniffer session. Summaries describe
  observed segments only; they are not a complete packet-loss census or a
  mapping of every segment to a named OTA run. The fast capture includes
  settled event spacing near 10 ms; restore images can negotiate 30 ms.
  Archived CSVs remove payload bytes and retain metadata plus derived RAM
  boundary markers. `air-metadata-provenance.json` records each original
  capture's SHA-256 and row count; full captures remain in the private bench
  directory because they contain firmware payload bytes.
  `coc-air-signals.json` retains the decoded CoC connection MTU/MPS/credits
  without firmware payloads. `dependency-versions.json` fingerprints the
  sniffer parser and records the SN/NESN naming correction in the exporter.
- `final-repeat-summary.json` excludes the candidate-selection sample from
  the repeated raw mean. The priority summaries separately record the main
  thread's unconfirmed QoS request and the dedicated queue's actual QoS check.
- `SHA256SUMS.json` fingerprints archived inputs, source, tests and results.
  Regenerate it after the final archive update.

The built-in private `ram` command counts and hashes actual received bytes.
It does not implement OTA. Full-write results require successful program-byte
and erase-coverage accounting plus the exact post-boot image and changed slot.
No throughput estimate may replace those verdicts.

Exploratory builds sometimes overlapped measurements. The long RAM trial
that overlapped the complete SDK rebuild is retained, with its much lower
observed rate. Final comparisons must state whether the compiler was idle.
