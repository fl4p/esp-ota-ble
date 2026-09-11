#pragma once

#include <cstddef>
#include <cstdint>

// OTA firmware update pushed over BLE, with no WiFi involved. The host streams the image to a
// write-without-response characteristic; bytes are staged in a small ring and flushed to the passive
// OTA partition from the consumer task. Integrity is a streaming SHA-256 plus a length check at the
// end -- there is no per-chunk ack, so a lost byte costs a full retry and nothing less.
//
// This module knows nothing about BLE. The consumer owns the GATT layer and calls in:
//
//   producer task (the BLE write callback):  otaBleSubmitCommand(), otaBleStageBytes()
//   consumer task (a slow periodic loop):    otaBleTick()
//
// That split is load-bearing, not stylistic. esp_ota_begin/write/end block for milliseconds on
// flash; running them on the BLE host task stalls it long enough to trip the connection supervision
// timeout and drop the link mid-update.

/// Severity of a status line. Consumers that log must preserve it: every `OTAB FAIL ...` is Warn or
/// Error, and collapsing them into Info means a routine log-level raise silently swallows exactly
/// the lines a host tool needs to distinguish failure from a dead link.
enum class OtaBleLevel { Info, Warn, Error };

/// Compare each erase sector against the update slot before erasing it, and program only the
/// sectors that actually differ. On a rebuild of the same firmware most of the image is unchanged,
/// so most of the erasing and programming -- which is where a push spends its time -- is skipped.
/// Set to 0 to fall back to erasing the whole slot (the strategy chosen automatically by build
/// config); the receiver also withdraws it by itself on an encrypted flash or with no RAM to spare.
#ifndef OTA_BLE_SECTOR_SKIP
#define OTA_BLE_SECTOR_SKIP 1
#endif

struct OtaBleHooks {
    /// Required. Emits one protocol line (no trailing newline) to the transport.
    void (*status)(OtaBleLevel level, const char *line) = nullptr;
    /// Optional. Called with true before the first erase and false on every exit path. The consumer
    /// should stop sampling / de-energise: flash writes disable the CPU cache and stall the other
    /// core, which is enough to trip a real-time watchdog under load.
    void (*quiesce)(bool halt) = nullptr;
    /// Optional. Defaults to esp_restart().
    void (*restart)() = nullptr;
};

enum class OtaBleSubmit {
    Accepted, ///< latched; otaBleTick() will execute it
    Rejected, ///< malformed, or another command is still latched -- nothing was changed
};

/// Install the host hooks. Call once before any other entry point.
void otaBleInit(const OtaBleHooks &hooks);

/// Parse and latch one control line:
///   "begin <wireSize> <wireSha256hex>"                    -- a raw image, the original protocol
///   "begin <wireSize> <wireSha256hex> <xform> <imgSize>"  -- xform is "raw" | "tamp" | "delta"
///   "resume <wireOffset> <wireSize> <wireSha256hex>"      -- continue an interrupted raw transfer
///   "info" | "end" | "abort"
/// The size and digest always describe the bytes that go over the WIRE, so credit, progress and
/// transfer integrity mean the same thing under every transform; <imgSize> is the image those bytes
/// reconstruct to. "info" answers with OTAB INFO / BASE / RESUME / XFORM: the running slot, the
/// SHA-256 a delta must patch from, where an interrupted transfer can be picked up (or "none"),
/// and the transforms this build can accept. XFORM stays last, so a host that treats it as the end
/// of the reply keeps working; a receiver too old for resume emits no RESUME line at all, which is
/// the whole capability negotiation. "resume" is refused unless all three arguments match what the
/// receiver recorded when the previous transfer was torn down -- see doc/resume.md for why the
/// offset is the receiver's number rather than the host's, and why the already-flashed prefix is
/// re-hashed out of the slot instead of carried across the drop.
/// Safe from any task. Everything checkable without touching flash is checked here and reported
/// synchronously, so a consumer's command layer can fail the command instead of reporting success
/// and then failing asynchronously. Execution happens on the next otaBleTick().
OtaBleSubmit otaBleSubmitCommand(const char *line);

/// Copy firmware bytes into the staging ring. Producer task; never touches flash.
void otaBleStageBytes(const uint8_t *data, size_t len);

/// Consumer task: execute a latched command, drain the ring to flash, re-announce credit, and abort
/// a transfer that accepts no bytes for 30 seconds. nowMs must be a wrapping monotonic millisecond
/// counter (for example millis()).
///
/// HOW OFTEN YOU CALL THIS IS THE TRANSFER'S THROUGHPUT. This is not a housekeeping tick.
/// Credit is a single high-water mark of `bytes drained to flash + 8 KB`, and only this call drains
/// and re-announces it -- so the host can never be more than 8 KB ahead of your last tick, and any
/// delay between ticks is dead air on the link. Anything else in the same task that blocks for a
/// second or more throttles the push by exactly that much, for the whole transfer.
///
/// Measured 2026-09-10 on a farm node: a 751 744-byte raw push took 108.58 s (6.76 kB/s) whose
/// middle half ran at 25.59 kB/s, with only 13.1 s of flash work in it. The cause was the caller's
/// own loop, which also ran an RS-485 Modbus poll blocking up to 1500 ms per exchange plus a 1500 ms
/// quarantine after a silent one. Nothing in this module can see that, and nothing in it can
/// compensate: if a transfer is inexplicably slow, measure the tick interval first.
///
/// While otaBleActive(), either give this its own task or make sure everything else sharing the
/// task is non-blocking. The `quiesce` hook is the right place to stop the work that would block --
/// it already exists to stop real-time work during flash writes, and stopping it also removes the
/// thing most likely to be sitting between two ticks.
void otaBleTick(uint32_t nowMs);

/// True between a successful begin and the matching end/abort.
bool otaBleActive();

/// Ask the consumer task to abort. Safe from any task; also cancels a latched begin that has not run
/// yet, and is a no-op when neither a transfer nor a begin is in flight. Consumers should call this
/// unconditionally on BLE disconnect, which also fires for clients that never started an OTA.
void otaBleRequestAbort();

// Direct entry points. otaBleSubmitCommand() is the normal way in; these are exposed for a consumer
// that is already on the consumer task and wants to skip the latch. All three must run there.
/// `xformName` nullptr or "raw" keeps the classic behaviour; `outSize` is ignored for raw.
bool otaBleBegin(uint32_t size, const char *sha256hex, const char *xformName = nullptr,
                 uint32_t outSize = 0);
/// Continue a raw transfer torn down earlier in this session. `offset` must be exactly the one the
/// receiver reported (OTAB RESUME), and `size`/`sha256hex` must describe the same payload; anything
/// else is refused with OTAB FAIL resume-mismatch and the caller should fall back to otaBleBegin().
/// Nothing survives a reboot -- after a restart this always refuses.
bool otaBleResume(uint32_t offset, uint32_t size, const char *sha256hex);
bool otaBleEnd(); ///< on success this reboots and does not return
void otaBleAbort();
