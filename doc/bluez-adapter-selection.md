# Keep BlueZ discovery and connection on the same adapter

When selecting a non-default Linux Bluetooth adapter, pass it to both the
scanner and `BleOtaLink.open(device, adapter="hci1")`. Selecting it only for
discovery does not keep subsequent connection retries on that adapter.
Omitting `adapter` retains the existing platform-default behavior.

Measured on `rpi.local`, BlueZ 5.66 and Bleak 3.0.2, 2026-09-11: discovery
on `hci1` found the bench node, but a later connection attempt failed with
`device 'dev_7C_4F_AD_20_2A_29' not found`. Passing the adapter explicitly
then recorded `/org/bluez/hci1/dev_7C_4F_AD_20_2A_29` and completed a raw
754,624-byte OTA in 15.621 s. Every image byte was programmed and the exact
image plus changed running slot was verified after boot.

This is adapter routing, not a throughput optimization. The same Pi's USB
`hci0` supports 2M PHY but was slower in this test. See the host comparison
in [the throughput experiment](2026-09-10-throughput-headroom.md).
