# Shared host transports

`host/esp_ota_ble_transport.py` supplies `Options`, `add_arguments`,
`from_arguments` and `DirectLink` for fugu, smart-shunt, node OTA and the
node's probe keeper. It is a sibling of `esp_ota_ble.py`; keep both from the
same checkout. The three OTA CLIs accept `ESP_OTA_BLE_HOST=/path/to/host` to
select that checkout before bundled copies.

These controls package the host mechanisms from the
[Pi measurements](2026-09-11-rpi-throughput.md) and
[Mac measurements](2026-09-11-mac-throughput-recipe.md). Receiver flash,
PSRAM/XIP configuration, radio conditions and negotiated parameters still
determine throughput. Those historical results are not measurements of this
new packaging on every consumer firmware.

## CLI options

| Option | Meaning |
| --- | --- |
| `--ble-backend bleak` | Default OS transport. Preserves normal adapter selection. |
| `--ble-backend bumble` | Linux controller ownership via bumble-bleak; explicit import, no global Bleak shadow. |
| `--ble-backend native` | Mac CoreBluetooth sender running on a native serial queue; requires Xcode command line tools. |
| `--adapter` | BlueZ `hciN`, or a bumble-bleak controller MAC/transport such as `usb:2357:0604`. Native Mac rejects adapter selection. |
| `--chunk` | Firmware value size, default up to 244, bounded by the characteristic capacity. Shared parser also accepts `--ble-chunk`. Node retains its existing `--chunk` spelling. |
| `--ble-interval-ms` | Bumble only: exact requested interval, 7.5–4000 ms in 1.25 ms steps, latency zero. |
| `--ble-phy 1` / `2` | Bumble only: request and verify PHY; request 251-byte link-layer data length. |
| `--experimental-hci-packet-size 251` | Explicit nonstandard experiment described below. Disabled by default. |

All three OTA tools use these flags on their direct connection:

```sh
# Add the tool's normal device and image arguments.
python3 etc/ota_ble.py --ble-backend native
python3 smart-shunt-ota-ble.py --ble-backend native
python3 tools/ota_ble_push.py --ble-backend native
```

The node keeps its LoRaWAN invitation/nonce flow and propagates the same
options when reconnecting for image verification. Its extra write delay is
now zero on native, Bumble and Mac Bleak; Linux OS Bleak retains 15 ms.
`--pace-ms` remains an explicit node override. Fugu's ESPHome proxy keeps its
existing transport and rejects direct-only tuning flags.

For the tested Pi USB adapter, append these options to a direct OTA command:

```sh
--ble-backend bumble --adapter usb:2357:0604 --chunk 495 \
--ble-interval-ms 12.5 --ble-phy 2 --experimental-hci-packet-size 251
```

Install Bleak in the host environment. Bumble additionally needs
`bumble-bleak` and its dependencies; the integration probe used Bumble
0.0.233 and bumble-bleak 0.1.0. HCI user channels need the appropriate Linux
privileges; direct USB needs device access. Stop the process owning that
controller before taking it and restore it afterwards. Controller MAC
selection resolves to an HCI user channel, whereas `usb:...` uses libusb.
Neither backend may share the selected controller with another central.

The transfer preparation hook runs **after OTAB READY, before firmware
bytes**, because a receiver may request different connection parameters at
begin. It verifies the actual interval, latency and TX/RX PHY, failing before
data if negotiation did not match. A later peripheral renegotiation can still
change the link. CoreBluetooth has no equivalent interval knob here: the
bench firmware's `interval` command in the Mac report is not a production
node command.

## Flow control and the experimental knob

The native helper checks `canSendWriteWithoutResponse` before every data
write and resumes on its readiness callback. Python still owns receiver
byte credits and final programmed-byte progress. IPC data is ordered with
commands, its native pending queue is capped at 1 MiB, and failures propagate
as disconnects. The helper is compiled from its adjacent Swift source for
each connection; there is no executable cache. Native `mtu_size` is a hint
derived from CoreBluetooth's maximum write value, not a measured ATT MTU.

Bumble retains its controller packet-credit accounting and bounds the host
pending queue at 256 packets before the next GATT submission. Timeout or
disconnect fails the write. The experimental flag changes **packet size,
not credit count**. It deliberately exceeds the controller's reported
27-byte limit. It requires the actual controller address
`AC:A7:F1:83:27:AD`, a report of 27 bytes/eight credits, exactly one connection,
and unchanged host credits. It drains the queue before changing size and
restores the original size on release, including failed setup.

This restriction is intentional: 251 worked in the archived bench trials;
502 corrupted a RAM payload. A successful USB link probe is not sufficient
to qualify another controller or firmware revision. No experimental setting
is inferred from an adapter name, a fast mode or an environment variable.

Fugu and smart-shunt direct OTA now require a valid target image identity
and pre-update receiver identity. Success after the push requires the exact
target digest on a different running slot. Missing replies, a rollback or
advertising alone do not qualify. Fugu proxy confirmation retains its existing
advertising-only limitation and labels it as such.

## Keeper and borrowed connections

The keeper's connection boundary is
`o2p_ble_link.BleBusSession(..., transport_options=options)`. It preserves its
authenticated client for CTRL and BUS notifications. Default `Options()`
uses the existing connection path. Nondefault Bleak/Bumble options use the
shared connector, negotiate after authentication and clean up through it.
The keeper's `--chunk` remains the **BUS write-with-response** chunk; it must
not populate the OTA `Options.chunk`. Native currently supports the direct
OTA layouts, so a keeper requesting native fails explicitly rather than
losing its BUS notifications.

The keeper's `/update` flashes the STM32 probe through the node's Modbus
relay. This is not a node ESP32 OTA endpoint, and its throughput includes
RS-485 and probe flash costs. Connection tuning applies to its BLE hop.

For a future node OTA endpoint, an existing owner can borrow the sender:

```python
ota = T.DirectLink(CTRL_UUID, CTRL_UUID, DATA_UUID, options=options)
await ota.attach(client, disconnected=owner_disconnect_event)
```

The owner must retain its backend, serialize the whole OTA against other
CTRL/BUS work, and fan out CTRL notification bytes to
`ota.feed_notification(payload)`. Attachment does not subscribe, authenticate
or reconnect. `O.query_info(ota)` and `O.push_image(ota, payload, ...)` use the
normal protocol, including the post-READY preparation hook. `ota.release()`
restores its temporary HCI setting and removes its listener without
disconnecting the owner or setting the owner's disconnect event. The owner
still handles reboot/reconnection and exact image verification. This adapter
does not itself create a keeper HTTP endpoint.

## Integration validation, 2026-09-11

Evidence is in [the integration directory](evidence/2026-09-11-tool-integration/).

* Mac native: compiled, connected to `farmnode-202A29`, subscribed,
  authenticated and queried its exact image identity. No firmware bytes sent.
* Pi USB Bumble: actual 12.5 ms, latency zero, 2M TX/RX; MTU 517, value
  capacity 512, selected chunk 495. Reported HCI 27/eight, explicitly selected
  251/eight. Identity unchanged before/after negotiation; no firmware bytes sent.
* The saved Graceful throughput image was no longer running. The exact-image
  precondition rejected the full-write test: current base was
  `96689e6acab4b8a9296d96813c553bea4162c8bf61d8650059036c9a796c3a38`
  in `app0`. Its provenance is unresolved, so the packaged sender has no new
  qualified full-write throughput figure yet.
* A nonprivileged HCI probe failed with EPERM. The privileged HCI user-channel
  probe failed during controller startup with an unexpected response shape.
  The subsequent direct USB probe passed; a later USB restart also showed
  mismatched reset/command completions during startup. Startup reliability
  remains a bumble-bleak/controller limitation to investigate. This does not
  establish the cause of those failures. The Pi collector was restored and
  fresh shunt telemetry observed.

Offline checks cover options, far-tail invalid values, HCI ownership/credits,
queue stalls, capacity, negotiation refusal, attachment ownership, IPC errors,
authentication and exact-image confirmation. The library's existing host
receiver suite and tamp compatibility tests also pass. Run:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 test/test_transport.py
PYTHONDONTWRITEBYTECODE=1 python3 test/test_native_transport.py
sh test/run.sh
```

## Guard review

1. Missing/bad capacity, identity, connection state or HCI reports fails;
   a failed probe cannot produce a throughput result.
2. Invalid near-boundary and far-tail sizes/intervals remain rejected. Larger
   queues do not become writable; wrong images never become verified.
3. Live USB negotiation exercises the actual adapter and dependencies. The
   Mac helper was compiled and exercised through real authentication/info.
4. Evidence fingerprints the Python/Swift sources and harness, as well as
   receiver identities. No proxy is substituted for image identity.
5. There is no verified-state or native-binary cache. Borrowed connections
   preserve their owner's state; failed setup releases backend resources.
6. Connection probes, mocked protocol tests and historical full-write results
   are distinguished above. The new full-write measurement remains open.
7. Tests construct wrong adapter/report/ownership, oversized writes, stuck
   credits, refused negotiation, missing image/slot and malformed IPC, and
   observe failures. The live unexpected-image gate sent no firmware.
8. Flow control changes actual send eligibility and HCI packetization. It does
   not suppress a warning. Guard plus AsyncMock cost measured 12.78 µs/write
   over 10,000 calls on this Mac (mock overhead included), below 0.7% of a
   495-byte write budget at 120 kB/s. This is not a throughput measurement.

## Final integration handoff, 2026-09-11

Implementation and review are complete in the local source trees:

| Repository | Commit | Change |
| --- | --- | --- |
| esp-ota-ble | `90fa09c` | Shared options, native Mac/Bumble transports, borrowed-client API, tests and evidence. |
| fugu-mppt-firmware | `1ff63e5` | Direct OTA uses shared transport settings and exact boot verification. |
| node-prototype | `40fcd70` | OTA options, pacing and verification-reconnect propagation. |
| pwr-metering | `862cadd` | Smart-shunt OTA options and exact boot verification. |
| farming | `822c43d` | Keeper session transport options, ownership, cleanup and tests. |
| farming | `22a6a05` | Independent-review fix: reuse the existing CTRL subscription. |

These integration commits were made locally; this session did not push them
or advance the farming node-prototype gitlink. Unrelated shared-tree changes
were preserved. The private test snapshots remain under
`/Users/fab/o2p-push/ota-tools-integration/`.

### Independent review and correction

The reviewer found one P2: `DirectLink.open()` subscribed to CTRL, then the
keeper session subscribed again. CoreBluetooth rejects that second request
before authentication. Commit `22a6a05` forwards the shared connector's
existing line handler into `BleBusLink._on_ctrl`; the legacy connection path
still subscribes directly, and BUS retains its separate subscription.

The revised test transport subscribes during open and its fake client rejects
duplicates. It failed against the previous code and passed with the fix.
The reviewer independently confirmed the correction, passed all 15 shared
transport tests, two native IPC tests and five keeper session tests, and
reported no remaining actionable findings. The combined keeper CLI/HTTP suite
also passed 102 checks; the actual `serve --help` exposed the shared options.

### Keeper coordination and remaining work

The keeper owner on channel `otab` (`codex-farming-hhkl`) implemented and tested
the CLI wiring in `farming/tech/chirpstack/o2p_ble_keeper.py` and
`test_o2p_ble_keeper_http.py`. It resolves the options once, passes nondefault
options into the session, and keeps the default legacy path. The owner last
reported that this CLI change remained uncommitted, with a patch at
`/tmp/keeper-wiring.patch`; its commit is separate from the session commits
above. This status was unchanged at this handoff.

The owner reported deploying the keeper CLI to farmgw with tuning dormant:
the shared transport module was not yet installed there and no new flags
were passed. Deploy the library, latest session including `22a6a05`, and
keeper together, with one coordinated restart through `keeper-loop.sh`.
No farmgw throughput improvement or active Bumble deployment is established
by the source integration tests.

Remaining work is explicit:

1. Finish the keeper owner's CLI commit and coordinated farmgw deployment.
2. Identify the current `96689e...` bench image before replacing it, then run
   repeated full-write/boot-verified tests of the packaged senders. Historical
   Pi **126.88 kB/s** and Mac **87.48 kB/s** results remain in the linked
   throughput reports; the new packaging currently has connection/negotiation
   probes, not a new full-write result.
3. Diagnose the intermittent Pi controller startup command-response mismatch.
   The successful USB negotiation probe does not qualify restart reliability.
4. Validate the keeper owner's separate node firmware interval request
   (15–30 ms, zero latency) on the live farm connection. It was reported built
   but uncommitted/unverified on hardware; actual negotiated timing must be
   captured rather than inferred from the request.

The Pi smart-shunt collector was restored active and fresh telemetry observed
at the end of the hardware work. The bench image was unchanged. Experimental
HCI packet sizing remains behind its explicit flag; neither a default backend
nor the keeper enables it automatically. Future node OTA over the keeper's
held connection can use `attach`, but a `/update-node` endpoint was not
implemented by this work.
