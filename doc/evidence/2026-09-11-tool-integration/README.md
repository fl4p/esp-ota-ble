# Tool integration evidence

The harness ran from `/Users/fab/o2p-push/ota-tools-integration` on the Mac,
and `/home/fab/ota-tools-integration` on the Pi, with an `esp-ota-ble/host`
directory beside it. Reproduce that layout when using this archived copy.
The harness contains the bench nonce; it only selects `farmnode-202A29`.

`mac-native-final-probe.log` is the final source's connection/auth/info
probe. `rpi-bumble-usb-integrated-probe.log` is the successful USB negotiation
probe before adding the unused borrowed-client API and failed-start cleanup.
`rpi-bumble-usb-final-probe.log` records the final source's failed controller
startup. No probe writes firmware. The earlier attempted full-write run
(`mac-native-integrated-3.log`) rejected the unexpected running image before
any firmware write.

The success commands were:

```sh
PYTHONDONTWRITEBYTECODE=1 /tmp/otavenv/bin/python check_transport_bench.py \
  /tmp/node_X.bin --ble-backend native --chunk 244 --probe

sudo env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/fab/ota-throughput-20260911 \
  /home/fab/ota-throughput-20260911/venv/bin/python -u \
  /home/fab/ota-tools-integration/check_transport_bench.py \
  /home/fab/ota-throughput-20260911/node_X.bin --probe \
  --ble-backend bumble --adapter usb:2357:0604 --chunk 495 \
  --ble-interval-ms 12.5 --ble-phy 2 --experimental-hci-packet-size 251
```

`SHA256SUMS` paths are relative to the esp-ota-ble repository root. Source
entries identify the final committed candidate, not peer edits in the shared
working tree. Historical full-flash throughput remains in the separate
2026-09-11-rpi-throughput evidence archive.
