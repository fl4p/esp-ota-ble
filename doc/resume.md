# Resume: continuing an interrupted push

*Written 2026-09-10, against the measurement below. Design plus scope; the code is
`src/ota_ble.cpp` (`recordResumePoint`, `durablePrefix`, the `resumeFrom` branch of
`beginWithDigest`) and `host/esp_ota_ble.py` (`resume_offset`, `push_image(resume_from=…)`).*

## The problem, measured

Farm node, ESP32-S3, macOS host, 2026-09-10. A `raw` push of a 751 744-byte image died at
`175104/751744` — 23 % — and cost the whole transfer, because `push_image()` always started at
offset 0 and the receiver requires strictly sequential offsets. An identical push a few minutes
later completed. Roughly a coin flip on a ~108 s operation, and a failure costs the full 108 s
again rather than the 83 s still outstanding.

Nothing about that is a link that is too lossy to use. The bytes that arrived *arrived*, and they
were in flash. What was missing was anything that remembered so.

## What resume is, in one paragraph

On every teardown of an interrupted **raw** transfer the receiver records the sector-aligned prefix
it can prove is in the update slot, together with the size and SHA-256 of the transfer that put it
there. `info` advertises that as `OTAB RESUME <offset> <size>`. A host that recognises the line, and
whose payload has exactly that size, sends `resume <offset> <size> <sha256hex>`; the receiver
re-opens the slot, **re-hashes the recorded prefix out of flash**, and streams the rest as usual.
`end` is unchanged: it still checks length and a SHA-256 over the whole payload before anything is
marked bootable.

## The digest decision

This is the load-bearing choice, so it gets its own section.

The protocol's digest is over the **whole** wire payload and is verified at `end`. Resuming from a
byte offset means the SHA-256 state covering `[0, offset)` has to come from somewhere. There were
two candidates:

1. **Carry the hash state across the drop.** Keep the `mbedtls_sha256_context` alive between
   sessions and pick it back up.
2. **Recompute it by reading the flashed prefix back out of the update slot.**

**This implements (2), and (2) is not merely the easier one — it is the stronger one.** A carried-over
hash state re-proves what the *link* delivered into *RAM* during the previous session. It says
nothing about what actually survived into flash, which is the only thing the bootloader will ever
read. Recomputing from the slot means the digest checked at `end` covers the bytes that will boot.

Concretely, after this change a resumed transfer's final check covers:

* `[0, offset)` — read back **out of the update partition**, hashed on the spot;
* `[offset, size)` — hashed as it streams in, exactly as before.

Everything that could make a resume wrong therefore ends at the same place: `OTAB FAIL
sha-mismatch`, an abort, and a boot partition that never moved. A resume onto a prefix of a
*different* build, a slot clobbered between sessions, a wrong offset, a write that silently did not
land — all of them. **Resume cannot splice two images together. The worst it can do is waste a
transfer**, which is exactly what happens today anyway.

Cost: one full read of the prefix plus a SHA-256 over it, on the consumer task, inside the resumed
`begin`. For the 175 kB case above that is well under the 6.2 s full-slot erase this module already
does. The module already reads and hashes 1.7 MB of flash for a delta's base digest.

`OtaXform::Raw` is what makes this possible at all: the wire prefix and the slot prefix are the same
bytes. See **Scope** below.

## Wire protocol

Added, host → device:

```
resume <wireOffset> <wireSize> <wireSha256hex>
```

Added, device → host, as part of the `info` reply:

```
OTAB RESUME <wireOffset> <wireSize>     an interrupted transfer is waiting at that offset
OTAB RESUME none                        this receiver can resume, but holds nothing to resume
```

`OTAB READY` gains a trailing ` from=<offset>` **on a resumed transfer only**; the fresh-transfer
line is unchanged, and every host matches `OTAB READY` as a prefix.

Nothing else changed. In particular `begin` is untouched in both its two-argument and its
four-argument form — `resume` is a separate verb precisely so that the command every existing
receiver and host already agree on stays byte-for-byte what it was.

## The negotiation

The library is shared: `fugu-mppt-firmware` and `smart-shunt-fw` also carry this receiver, and their
hosts and devices update independently. Both directions are handled by *absence*:

* **New host, old receiver.** An old receiver's `info` contains no `RESUME` line. `query_info()`
  reports `resume_capable: False` and `resume_offset()` returns 0, so the host does what it has
  always done. If a host sends `resume` anyway (an explicit `resume_from=`, say), the old receiver
  answers `OTAB FAIL bad-command` and `push_image()` **falls back to a fresh `begin`** and pushes the
  whole image. The cost of guessing wrong is one round trip.
* **Old host, new receiver.** The `RESUME` line is emitted **before** `XFORM`, which every host
  treats as the last line of the `info` reply, and unknown lines are ignored by every host's line
  handler. An old host never sends `resume`, and a new receiver never requires it.

`none` is deliberately not silence. "I can resume and have nothing" and "I have never heard of
resume" lead to the same push *today*, but only the first says the **next** dropped link is
recoverable — and that is what a tool tells its user.

## Scope

**In scope: a raw transfer, resumed within a session or across a reconnect.**

Explicitly **not** covered:

* **`tamp` and `delta`.** A byte offset in the wire payload is not a byte offset in the reconstructed
  image, and the transform's own state — the compression window, the patch decoder — died with the
  session. There is nothing a wire offset could mean. The receiver never records a resume point for a
  transformed transfer and `resume_offset()` refuses one host-side, in both cases explicitly rather
  than by omission, and both are tested.
* **Across a reboot or power loss.** The `esp_ota_handle_t` does not survive a restart, and neither
  do the statics holding the record. `otaBleInit()` clears them, so a rebooted device answers
  `OTAB RESUME none`. Note what this design would need to go further, because it is not much: the
  offset, size and digest in NVS. Everything else — the prefix, and the proof that it is the right
  prefix — is already recovered from flash rather than from memory. That is a deliberate property,
  not an accident, but persisting the record is a separate change with its own failure modes and it
  is not in this one.
* **Flash encryption.** `esp_ota_write_with_offset` refuses any size that is not a multiple of 16 on
  an encrypted slot, and a resumed session writes through nothing else. `recordResumePoint()` checks
  `esp_flash_encryption_enabled()` and records nothing, so an encrypted device is never offered a
  restart it could not honour. This is the same reason the skip-identical strategy withdraws itself
  there, and it is equally untested on hardware — there is no encrypted board in hand.

## Mechanism, and the four things that are easy to get wrong

**The offset is the receiver's number, echoed back.** The host does not choose where to restart; it
has no way to know what reached flash. `resume` is refused unless the offset, the size *and* the
digest all match what the receiver recorded. The digest check at `end` would catch a bad resume
anyway — but only after spending the whole transfer to find out, so the cheap check comes first.

**What counts as "in flash" is deliberately pessimistic.** `durablePrefix()` reports whole erase
sectors only. With the skip-identical strategy that is what `commitSector()` actually committed —
tracked on the success paths only, because `secOff` advances *before* the erase and so names a
sector that has not been written yet. Without it, `esp_ota_write` owns the slot and the count is
`imageWritten` floored to a sector, because `esp_ota_write` buffers a partial trailing chunk of its
own under encryption. Bytes above the reported offset are re-sent, which costs link time; a byte
wrongly claimed below it would be spliced in unread. Only one of those is recoverable.

**A resumed session drives the slot through the sector path whatever the build's default strategy
is.** That is not a preference. `esp_ota_write_with_offset` is the only writer here that can start
anywhere but zero, and it never erases for you, so the per-sector erase-and-program in
`commitSector()` is the only thing that can put the tail down. The sector buffer is therefore
mandatory for a resume, not an optimisation, and if it will not allocate the resume is refused
(`OTAB FAIL resume-no-mem`) rather than the push failed — the host still has a fresh `begin`.
The practical consequence: on an `OTA_BLE_SECTOR_SKIP=0` build the resume path is *different code*
from the one the interrupted transfer took on the way in, which is why the test suite runs the
resume cases under all three erase strategies.

**Sector 0 has to be rescued around `esp_ota_begin`.** One erase sector is the smallest size that
still leaves `need_erase == false` — which is what hands this module every later erase — and that
sector is the head of the prefix being resumed onto. A resumed begin reads sector 0 out *before*
calling `esp_ota_begin`, lets it be erased, and writes it straight back. The rewrite is not only
about the bytes: `esp_ota_end()` refuses a handle nothing was ever written through, and a resume
near the end of an image may otherwise write very little.

## When the record is dropped

A record is a claim about what is in the update slot *right now*. It is dropped:

* by `otaBleInit()` — a boot, or a consumer re-initialising, is not a state in which an old claim
  means anything;
* at the top of `beginWithDigest()`, before anything erases. This is not redundant with the abort
  path: `esp_ota_begin` erases *before* the later checks in that function run, so a begin that fails
  after it — a delta whose base cannot be hashed, say — destroys slot content and returns without
  ever reaching an abort. Tested (`resume: a begin that erases and then fails drops the record`), and
  the test fails if the clear is moved to the exit path;
* by a resume, which consumes its own record;
* by everything past the drain in `otaBleEnd()`. Those exits — too short, wrong digest,
  unreconstructable, refused by `esp_ota_end` — are verdicts on the whole payload rather than link
  problems, and a host that retried onto one would spend a full transfer rediscovering the same
  verdict.

It is *created* in exactly one place: `otaBleAbort()`, which is where a dropped link, a stall, a
credit overrun and a flash write error all end up.

## Tests

Device side, `test/host-stub/ota_ble-test.cpp`, run under all three erase strategies:

| case | what it constructs |
|---|---|
| `resume: an interrupted transfer finishes` | 9 of 20 sectors, link dropped, resumed, whole image in the slot, prefix not re-sent |
| `resume: a corrupted prefix fails the whole-image digest` | the slot is corrupted *under* the receiver between the two sessions; `end` must fail and the boot partition must not move |
| `resume: a mismatched digest is refused` | a different build of the same length; refused before any flash work, and the fresh-`begin` fallback still installs |
| `resume: an offset the receiver did not name is refused` | too high, too low, zero |
| `resume: nothing to continue reports none` | fresh receiver, and a completed push |
| `resume: a transformed transfer offers nothing` | interrupted delta |
| `resume: a fresh begin drops the recorded point` | and `…a begin that erases and then fails…` |
| `resume: malformed forms are rejected` | arity, trailing garbage, short digest, `resumes` |

Host side, `test/test_resume_host.py`, pure stdlib: `info` parsing including the old-receiver case,
every `resume_offset()` refusal, and the two fallbacks — an old receiver answering `bad-command` and
a new receiver answering `resume-mismatch` — both of which must end in a complete image.

**Calibration.** Each guard was removed in turn and the suite re-run; the failing cases are listed
so a future reader knows what each test is actually holding down:

| break | fails |
|---|---|
| resume trusts the prefix instead of re-hashing it from flash | `an interrupted transfer finishes` (4 checks) |
| `end` stops comparing the digest | `digest mismatch`, `a corrupted prefix…` (6 checks) |
| the record is cleared on the exit path instead of on the way in | `a begin that erases and then fails…` (+2 collateral) |
| `resume` accepts any offset the host names | `an offset the receiver did not name…` |
| sector 0 is not put back after `esp_ota_begin` erases it | `an interrupted transfer finishes` |
| the transform guard is removed **from both** `recordResumePoint()` and `durablePrefix()` | `a transformed transfer offers nothing` |

One calibration that does **not** bite alone, recorded so it is not mistaken for coverage: removing
the `xform != Raw` guard from *either* `recordResumePoint()` or `durablePrefix()` on its own leaves
every test passing, because the other one still refuses. That is defence in depth, not two
independently tested rules.

## Not fixed here, found on the way

**`tamp` is broken on the farm node.** A `tamp` push failed with `OTAB FAIL xform
ESP_ERR_INVALID_CRC` after exactly 8192 bytes — the first credit window. Not investigated and not
touched; resume is deliberately not built on top of it.
