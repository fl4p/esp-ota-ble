# Pi 120 kB/s follow-up evidence

Read the [report](../../2026-09-11-rpi-throughput.md) for qualified results,
measurement boundaries and the nonstandard HCI override limitation.

This directory retains bench artifacts, including failed experiments. It
does not install settings in the production receiver or host library.
The private original workspace is `/Users/fab/o2p-push/ota-throughput-next`;
the remote workspace is `/home/fab/ota-throughput-20260911` on `rpi.local`.
Scripts retain those bench paths and require prepared firmware/dependencies;
this is an evidence archive, not a standalone test package.

- `rpi-120-repeat-summary.json` selects the three final USB raw repeats.
  `rpi-usb-raw125-repeat-*.log` includes their command output, source hashes,
  actual link settings, full erase/program accounting and boot identity.
  Files ending `-restore.log` are delta installations and do not enter the
  raw mean.
- `rpi-usb-hci251-long-integrity.log` contains the 6 MiB length/SHA result.
  `rpi-usb-hci502-small.log` is an intentionally retained failed hash test;
  it is not a successful throughput result.
- `rpi-bumble-runs.jsonl` records 46 later Pi attempts and their arguments,
  environment, exit status and result where available. The two initial
  HCI-user-channel/libusb pilots have separate `rpi-bumble-pilot.log` and
  `rpi-bumble-usb-pilot.log` files. Some early wrapper versions did not print
  their source hash. No source identity is fabricated for those attempts.
- `rpi-bumble-flash.jsonl` preserves timestamped status lines for Pi OTA.
  `rpi-20260911-ram-matrix.jsonl` is the remote RAM matrix, including earlier
  Pi rows; select this follow-up's rows by matching the named trial logs.
- `mac-port-sweep.jsonl`, the `mac-port-*.log` files and
  `mac-port-native-matrix.jsonl` cover native CoreBluetooth follow-up trials.
  The native logs retain all timestamped status lines inside `NATIVE_RESULT`.
  Mac restore logs are excluded from native raw results.
- `bumble_entry_v10.py` is the exact wrapper used for the long integrity
  run and final raw repeats. `bumble_entry.py` changes only a stale comment;
  its Python AST was checked identical. Earlier `v1`–`v9` snapshots preserve
  the investigation, including the rejected 502-byte experiment. Do not use
  historical snapshots as recommended settings.
- `rpi-source/` and `mac-source/` hold the host/bench copies actually used.
  Native Swift source and runner are retained at the top level. Build
  identities and `binary-fingerprints.json` identify receiver/target/native
  sender binaries; binaries themselves are excluded. The receiver's build
  sources and configuration are in the [preceding archive](../2026-09-10-throughput-headroom/).
- `bumble-bleak-source-sha256.txt` fingerprints every copied facade Python
  module. `rpi-facade-package-origin.txt` identifies its source installation
  (0.1.0); this copied package does not appear in the isolated environment's
  `pip freeze`. `rpi-package-versions.txt` records that environment separately.
- `check_rpi_verifiers.py` replays actual good and hash-corrupted observations,
  checks malformed accounting, and times existing verification functions.
  `rpi-verifier-check.log` gives the Pi result and executed-source hashes.
- `rpi-service-restoration.log` and `rpi-service-after-restart.log` record
  restoration of the smart-shunt service and fresh received telemetry.

The full-file SHA-256 of X is
`902e774c2bd1811f2982ea318c38b64191d4b402a4d9a0c19c085e9604704d33`.
Its embedded image identity, checked after boot, is
`f408a9dc9eb8a54e57a26d1ae3792059e1bfdad8e5991003dc0ba0d7b5c51de0`.
These hashes cover different byte ranges and must not be interchanged.

`SHA256SUMS` covers every retained file except itself. To check it on the Mac:

Raw command/serial output preserves original trailing spaces, CRLF and
blank lines. Consequently the complete archive triggers Git whitespace
notices; edited Markdown and source files were checked separately.

```sh
shasum -a 256 -c SHA256SUMS
```
