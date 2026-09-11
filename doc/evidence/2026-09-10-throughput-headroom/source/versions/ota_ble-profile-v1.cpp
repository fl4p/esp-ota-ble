#include "ota_ble.h"
#include "ota_xform.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>

#include <esp_heap_caps.h>
#include <esp_flash_encrypt.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <mbedtls/sha256.h>

// Private bench experiments; zero preserves the existing strategy selection.
#ifndef OTA_BLE_BENCH_STRATEGY
#define OTA_BLE_BENCH_STRATEGY 0
#endif

// Staging ring: otaBleStageBytes (producer/BLE host task) only copies bytes in here; otaBleTick
// (consumer) drains to flash. Decoupling keeps the slow esp_ota_write off the host task -- a stall
// there trips the BLE supervision timeout. Capacity doubles as the host's credit window.
// Kept small (8 KB): on a no-PSRAM board internal heap is tight and fragmented.
//
// This used to add "so the credit round-trip never bottlenecks". IT DOES BOTTLENECK, and the
// original claim was wrong. Measured 2026-09-10, farm node (ESP32-S3), one complete raw push from
// macOS: 751 744 B in 108.58 s = 6.76 kB/s end to end, while the MIDDLE HALF of that same transfer
// moved 375 808 B in 14.34 s = 25.59 kB/s. The device reported write_ms=7587 erase_ms=5531, so 13.1
// s of the 108 s was flash. Roughly three quarters of the wall clock was neither link nor flash.
//
// The mechanism is this window, and it is working as designed: grantCredit() advances the host to
// written + ringCap, where `written` counts bytes the CONSUMER has drained, so the host can never
// run more than one window ahead of the flush. That makes the window a pacing mechanism whose throughput
// is set by how promptly otaBleTick() is called -- not a bug, but not "never a bottleneck" either.
//
// On that node the cause was outside this module: the caller's loop also ran an RS-485 poll that
// blocks for up to 1.5 s, and the push simply waited for it. The contract that follows from that is
// documented on otaBleTick() in ota_ble.h, because a consumer has no other way to learn it.
//
// THE WINDOW IS SIZED AT RUNTIME, but the size that wins is still the small one - see the
// measurements on RING_CAP_PSRAM below, which tried 256 KB on a board with 8 MB of PSRAM and came
// out SLOWER than 8 KB once the host stopped losing writes. The runtime sizing is kept because it
// is the mechanism a future measurement would need; the value is not an aspiration.
//
// RING_CAP_MIN is what a no-PSRAM board gets, and on today's evidence it is also the fast path.
static constexpr size_t RING_CAP_MIN   = 8 * 1024;
// PSRAM SIZING: THE MEASUREMENT THAT MATTERS IS A PUSH THAT ACTUALLY WRITES FLASH.
//
// This constant was set to 256 KB, then parked back at 8 KB on a comparison that was
// CONFOUNDED, and the retraction is worth more than the number. Measured 2026-09-10, bench
// ESP32-S3 with 8 MB PSRAM, 717 904-byte raw image, all runs complete and digest-verified.
//
// Re-pushing an image the slot ALREADY holds costs almost no flash: the sector-skip path
// (see commitSector) keeps 175 of 176 sectors and erases one, `OTAB SKIP kept=175 wrote=1
// erase_ms=45`. Those pushes measure the LINK and the comparison, not an OTA:
//
//     ring   8 KB   24.48 / 23.42 / 26.18 kB/s   (Mac)      kept=175 wrote=1
//     ring 256 KB   15.31 kB/s                              kept=175 wrote=1
//
// A push that writes the whole slot - `erase_ms` near 8700, i.e. ~176 sector erases - is a
// completely different machine, and it is the only one that says what an OTA costs:
//
//     ring   8 KB, no host flow control    1.88 kB/s
//     ring   8 KB, host flow control       2.37 kB/s
//     ring 256 KB, host flow control      12.88 kB/s   <-- 5.4x
//
// So the ring DOES decouple the link from flash, exactly as the argument above says, and the
// earlier "8 KB wins" reading came from putting a real 256 KB push next to a no-op 8 KB one.
// Flash is the bottleneck it removes, and flash only exists on a push that writes.
//
// STILL n=1 ON THE ROW THAT DECIDES IT. Before raising this, alternate two DIFFERENT images so
// every push writes every sector, and read `OTAB SKIP` on each run to prove it did: a
// throughput number from this bench is meaningless without the kept/wrote line beside it.
// UN-PARKED 2026-09-10, and the row above is why: on a push that actually writes flash the
// 256 KB ring is 5.4x the 8 KB one (12.88 vs 2.37 kB/s). Re-confirmed the same day on a
// no-PSRAM build of the node, which reports `OTAB RING 8192` and measured 2.22 and 7.44 kB/s
// on two identical real-write pushes - i.e. the 8 KB path is both slow AND wildly variable.
// A board without PSRAM still gets RING_CAP_MIN; allocRing() falls back on its own.
static constexpr size_t RING_CAP_PSRAM = 256 * 1024;
static size_t ringCap = RING_CAP_MIN;             // set by allocRing(), never read before it runs
static constexpr size_t FLUSH_SLICE = 2048;       // flash-page-friendly esp_ota_write granularity
// HOW OFTEN CREDIT IS RE-ANNOUNCED, AND WHY IT IS CAPPED.
//
// This was ringCap/2, which is correct at 8 KB (announce every 4 KB) and a DEADLOCK at 256 KB.
// MEASURED 2026-09-10, first bench run with the PSRAM-sized ring: the host was granted the whole
// 256 KB up front, wrote exactly 262144 bytes into it in about six seconds, and then had to wait
// for the next announcement - which needed the consumer to drain 128 KB before it would fire. The
// device meanwhile saw NO INBOUND BYTES, because the host was correctly obeying its credit, and
// killed the transfer with `OTAB FAIL stalled` on the 30 s no-bytes watchdog. Host waiting on
// credit, device waiting on bytes: a deadlock that a small ring hid, because with a 4 KB step
// credit always advanced long before the watchdog could fire.
//
// So the step is capped in ABSOLUTE terms rather than scaled to the window. 16 KB is eight
// FLUSH_SLICE drains, so credit advances every few consumer ticks no matter how large the ring is,
// while the notification rate stays far below the 8 KB ring's old one at 4 KB per step.
static constexpr size_t CRED_STEP_MAX = 16 * 1024;
static inline size_t credStep() {
    size_t h = ringCap / 2;
    return h < CRED_STEP_MAX ? h : CRED_STEP_MAX;
}
// Four chances inside the host tools' 20 s credit wait, at only 12 idle notifications per minute.
static constexpr uint32_t CRED_REPEAT_MS = 5000;
// Matches the 30 s watchdog already used by the node consumer, now a backstop for every transport.
static constexpr uint32_t STALL_MS = 30000;

// Erase-ahead. Erasing the whole 1828 KB slot inside esp_ota_begin costs 6.2 s (measured on flu)
// during which nothing else happens -- it is 30 % of a delta push and it is pure dead time, because
// the flash is idle again for the 8.5 s of writing that follows. Erase only a head block up front
// and keep the rest erased just ahead of the write pointer, so the erase hides underneath the
// writes instead of preceding them.
//
// The mechanism is a quirk worth stating plainly: esp_ota_begin sets need_erase ONLY for
// OTA_WITH_SEQUENTIAL_WRITES, and esp_ota_write never bounds anything against the size passed to
// begin (esp_ota_ops.c:170-199, :175-186). So passing a small size buys a small erase AND
// need_erase == false, after which this module owns every subsequent erase.
#if OTA_BLE_BENCH_STRATEGY == 2 || (defined(CONFIG_SPI_FLASH_YIELD_DURING_ERASE) && !defined(CONFIG_ESP_TASK_WDT_PANIC))
static constexpr uint32_t ERASE_HEAD = 64 * 1024;  // erased inside begin; one block erase
#endif
static constexpr uint32_t ERASE_CHUNK = 64 * 1024; // 64 KB at a time -> block erases, not 16 sectors
// Stay this far ahead of the write pointer. One chunk is enough: a chunk erase (~215 ms) buys
// 64 KB of writing (~310 ms at the measured 206 kB/s), so the writer never catches up.
static constexpr uint32_t ERASE_LEAD = ERASE_CHUNK;

// Skip-identical-sectors. The dominant cost of a push is flash work, not link time: 32.4 s of a
// 43.4 s raw push was inside esp_ota_write. But a rebuild of the same firmware does not change most
// of the image -- with a stable layout (fugu-mppt-firmware doc/2026-09-08-image-layout-stability-
// for-ota.md) a one-line edit leaves ~2/3 of the 4 KB sectors byte-identical to what the slot
// already holds. Erasing and reprogramming those is pure waste.
//
// So compare before erasing: buffer one sector of the reconstructed image, read the slot back, and
// erase+program only on a mismatch. A read of 4 KB costs well under a millisecond against ~20 ms to
// program it and tens of ms to erase it, and the compare bails at the first differing byte, so a
// dirty sector barely pays for the check.
//
// This is exclusive with erase-ahead, and not by preference: erase-ahead erases flash BEFORE the
// bytes that go there have arrived, which destroys the very content the comparison reads. Only one
// of the two strategies can own the slot, and skipping is worth more than hiding an erase --
// erase-ahead only ever recovers link time, while this removes flash work outright.
//
// What it costs: one erase-sector buffer of RAM, and the loss of 64 KB block erases (a dirty sector
// is erased on its own). Whether that trade holds depends on the 4 KB sector-erase time, which is
// NOT measured on this hardware -- the OTAB STAT line reports erase_ms and erases so the first real
// push measures it.
// Slot read-back granularity for the comparison; a mismatch stops the reads. OTA_BLE_SECTOR_SKIP
// (ota_ble.h) turns the whole strategy off at build time.
static constexpr size_t CMP_WINDOW = 256;

static uint8_t *ring = nullptr;
static size_t rHead = 0, rTail = 0, rCount = 0;   // byte ring indices + fill level
// Also serialises active/failed with freeRing(): the producer's locked recheck must finish before
// the consumer can make the storage unreachable.
static std::mutex ringMutex;

static esp_ota_handle_t otaHandle = 0;
static const esp_partition_t *otaPart = nullptr;
static mbedtls_sha256_context shaCtx;
static uint8_t expectedSha[32];

// The wire payload is not necessarily the image. `expectedSize`/`written`/`expectedSha` stay in WIRE
// bytes -- they drive credit, progress and transfer integrity, all of which are properties of the
// link -- while `expectedOut`/`imageWritten` track the reconstructed IMAGE the transform emits.
// For OtaXform::Raw the two are the same thing and every check below collapses to what it was.
static OtaXform xform = OtaXform::Raw;
static uint32_t expectedOut = 0;
static uint32_t imageWritten = 0;
static uint8_t baseSha[32]; // running image's own validation hash; the base a delta patches from

static bool active = false;
static bool failed = false;
static volatile bool abortReq = false; // set from any task; consumed by the consumer tick
static uint32_t expectedSize = 0;
static uint32_t written = 0;    // wire bytes consumed by the transform (consumer)
static uint32_t erasedTo = 0;   // bytes of the slot erased so far (consumer only)
static uint32_t eraseTarget = 0; // how much of the slot this image needs erased; 0 = erase-ahead off
// Consumer-only timing of the flash drain. esp_ota_write() with OTA_WITH_SEQUENTIAL_WRITES erases
// each 4 KB sector as it first crosses it, so its cost is bimodal: a plain page program, or a
// program plus a sector erase. Reported once at OTAB OK so a host can attribute a slow transfer to
// erase rather than to the link. Never used for control flow.
static uint64_t wrUs = 0;       // total time inside esp_ota_write
static uint32_t wrCalls = 0;
static uint32_t wrSlowCalls = 0; // calls >5 ms -- the erase signature
static uint32_t wrMaxUs = 0;
static uint32_t erUs = 0;       // time spent erasing ahead of the write pointer
static uint32_t erCalls = 0;
// Skip-identical state. `secSize` non-zero IS the "skip strategy is active" flag: it is set only
// after the sector buffer is allocated and esp_ota_begin has left this module owning every erase.
static uint8_t *secBuf = nullptr; // one erase sector of reconstructed image, awaiting comparison
static uint32_t secSize = 0;      // == otaPart->erase_size while skipping; 0 = strategy off
static uint32_t secOff = 0;       // slot offset the buffer's first byte belongs at
static uint32_t secFill = 0;
static uint32_t skSkipped = 0;    // sectors the slot already held
static uint32_t skWritten = 0;    // sectors erased and reprogrammed
static uint64_t benchProgramUs = 0, benchCompareUs = 0, benchBeginUs = 0;
static uint32_t benchProgramBytes = 0, benchEraseBytes = 0;
static const char *benchStrategy = "unknown";
// Resume. A dropped link costs the whole transfer only because nothing remembers what already
// reached flash: measured on a farm node 2026-09-10, a 751 744-byte raw push died at 175 104 bytes
// and the retry started again at zero, on an operation that takes ~108 s and completes about half
// the time.
//
// So remember. On every teardown of a raw session the receiver records the sector-aligned prefix it
// can PROVE is in the update slot, together with the size and digest of the transfer that put it
// there; `info` advertises it, and a `resume` command carrying the same three numbers picks the
// transfer back up at that offset.
//
// Three things make that safe, and none of them is the record:
//   * The record is only a hint about WHERE to restart. What authenticates the result is that a
//     resumed session re-hashes the prefix OUT OF THE SLOT (see beginWithDigest), so `end` still
//     checks a SHA-256 over the whole payload -- and over the bytes that will actually boot.
//   * Only OtaXform::Raw is ever recorded. Under tamp or delta a wire offset is not an image
//     offset and the transform's own state (window, patch decoder) died with the session, so there
//     is nothing a byte offset could mean.
//   * A fresh `begin` clears the record before it touches the slot, so a record never outlives the
//     flash content it describes within a session.
// It does NOT survive a reboot: the OTA handle does not, and neither do these.
static const esp_partition_t *resPart = nullptr;
static uint32_t resOff = 0;        // sector-aligned wire bytes provably in the slot
static uint32_t resSize = 0;       // wire size of the transfer that put them there
static uint8_t resSha[32];         // its digest -- a resume must name the same one
static bool resValid = false;
static bool resRecordable = false; // this session may leave a resume point behind
static uint32_t flushed = 0;       // sector-aligned bytes committed to the slot this session
static uint32_t staged = 0;     // accepted from the producer; sampled by the stall watchdog
static uint32_t lastGranted = 0;
static uint32_t lastProg = 0;
static uint32_t creditRepeatAt = 0;
static bool creditRepeatArmed = false;
static uint32_t stallMark = 0;
static uint32_t stallAt = 0;
static bool stallArmed = false;

static OtaBleHooks hooks;

// Single-slot command latch. The protocol is strictly one command at a time -- the host always waits
// for a status line before sending the next -- so a second command arriving before the consumer has
// drained the first is a host bug, and is reported as one rather than silently overwriting.
enum class PendingCmd { None, Begin, Resume, End, Abort, Info };
static PendingCmd pending = PendingCmd::None;
static uint32_t pendSize = 0;
static uint32_t pendOut = 0;
static uint32_t pendOff = 0; // Resume only: where the host wants to carry on from
static OtaXform pendXform = OtaXform::Raw;
static uint8_t pendSha[32];
static std::mutex cmdMutex;

static esp_err_t eraseAhead(uint32_t needTo); // defined below, beside the drain it paces

/// Transform sink: reconstructed image bytes on their way to the passive partition.
/// A failure here is flash refusing the write; a failure anywhere else under otaXformFeed is the
/// payload failing to reconstruct. The two send a host to completely different places -- a bad slot
/// versus a bad patch -- so the drain reports them as different lines rather than one "write" error.
static bool sinkFailed = false;

/// Consumer only. True only if the slot ALREADY holds exactly these bytes.
///
/// Every outcome that is not a proven byte-for-byte match returns false, including a read that
/// fails: this decides whether to erase, so "I could not tell" has to cost an erase and a program.
/// The inverse -- treating an unreadable sector as already correct -- writes nothing and leaves
/// stale bytes inside an image that then passes its own SHA, because the digest is over the wire
/// payload and never re-reads the slot.
static bool sectorMatches(uint32_t off, const uint8_t *want, uint32_t n) {
    const int64_t started = esp_timer_get_time();
    uint8_t cmp[CMP_WINDOW];
    for (uint32_t i = 0; i < n;) {
        const uint32_t m = (n - i) < CMP_WINDOW ? (n - i) : (uint32_t) CMP_WINDOW;
        if (esp_partition_read(otaPart, off + i, cmp, m) != ESP_OK ||
            memcmp(cmp, want + i, m) != 0) {
            benchCompareUs += esp_timer_get_time() - started;
            return false;
        }
        i += m;
    }
    benchCompareUs += esp_timer_get_time() - started;
    return true;
}

/// Consumer only. Commit the buffered sector: skip it, or erase and program it. Empties the buffer
/// and advances secOff either way, so the caller cannot accidentally commit the same bytes twice.
static esp_err_t commitSector() {
    const uint32_t off = secOff, n = secFill;
    secOff += n;
    secFill = 0;
    // Sector 0 is never skipped, because esp_ota_end() rejects a handle nothing was ever written
    // through (esp_ota_ops.c:490) -- so re-pushing an image the slot ALREADY holds would otherwise
    // fail at the last step with ESP_ERR_INVALID_ARG and no boot switch, for the one reason that
    // ought to be the fastest push there is.
    //
    // Measured, not assumed: this rule is currently redundant. esp_ota_begin erases sector 0 before
    // the first comparison can read it, so sector 0 never matches anyway -- removing the `off != 0`
    // below changes no test outcome. It stays because the invariant it states is local, and the
    // thing that actually enforces it today is the erase size of a call made a hundred lines away.
    // A whole sector that reaches the slot -- by matching or by being programmed -- is the unit the
    // resume point is measured in. Recorded only on the success paths, and only for a FULL sector:
    // secOff above advances before the erase, so it names a sector this call has not committed yet,
    // and the final partial sector is not a boundary a resumed transfer could restart on.
    if (off != 0 && sectorMatches(off, secBuf, n)) {
        ++skSkipped;
        if (n == secSize) flushed = off + n;
        return ESP_OK;
    }
    // A partial tail still erases its whole sector: the bytes past the image are not part of it,
    // and leaving them would make the sector's content depend on what happened to be there before.
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_partition_erase_range(otaPart, off, secSize);
    erUs += (uint32_t) (esp_timer_get_time() - t0);
    ++erCalls;
    if (err != ESP_OK) return err;
    benchEraseBytes += secSize;
    const int64_t programStarted = esp_timer_get_time();
    err = esp_ota_write_with_offset(otaHandle, secBuf, n, off);
    benchProgramUs += esp_timer_get_time() - programStarted;
    if (err == ESP_OK) {
        benchProgramBytes += n;
        ++skWritten;
        if (n == secSize) flushed = off + n;
    }
    return err;
}

/// Consumer only. Cut the reconstructed stream into erase sectors and commit each as it completes.
/// The final partial sector stays buffered until otaBleEnd flushes it -- there is nothing to
/// compare a half-filled sector against, and no reason to program it twice.
static esp_err_t sinkBySector(const uint8_t *data, size_t len) {
    while (len) {
        const uint32_t room = secSize - secFill;
        const uint32_t n = (uint32_t) (len < room ? len : room);
        memcpy(secBuf + secFill, data, n);
        secFill += n;
        data += n;
        len -= n;
        imageWritten += n;
        if (secFill == secSize) {
            const esp_err_t err = commitSector();
            if (err != ESP_OK) return err;
        }
    }
    return ESP_OK;
}

static esp_err_t xformSink(const uint8_t *data, size_t len, void *) {
    if (secSize) { // skip-identical strategy owns the slot; nothing is erased ahead of the data
        const esp_err_t err = sinkBySector(data, len);
        if (err != ESP_OK) sinkFailed = true;
        return err;
    }
    // Erase must lead the write pointer. This is where the image write offset is actually known --
    // a transform decides how many image bytes one wire slice becomes, and for delta that is tens
    // of KB, so bounding the wire slice instead would not bound this.
    const esp_err_t eerr = eraseAhead(imageWritten + (uint32_t) len + ERASE_LEAD);
    if (eerr != ESP_OK) { sinkFailed = true; return eerr; }
    const int64_t programStarted = esp_timer_get_time();
    const esp_err_t err = esp_ota_write(otaHandle, data, len);
    benchProgramUs += esp_timer_get_time() - programStarted;
    if (err != ESP_OK) { sinkFailed = true; return err; }
    benchProgramBytes += (uint32_t) len;
    imageWritten += (uint32_t) len;
    return ESP_OK;
}

/// Transform source, for Delta only: the base image, which is the one we are running from. Reading
/// the active partition while writing the passive one is safe -- esp_partition_read goes through the
/// flash driver's cache guard, not through the instruction cache we are executing out of.
static esp_err_t xformSrc(uint8_t *buf, size_t len, uint32_t offset, void *) {
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (!run) return ESP_ERR_INVALID_STATE;
    return esp_partition_read(run, offset, buf, len);
}

static void emit(OtaBleLevel level, const char *fmt, ...) {
    if (!hooks.status) return;
    char line[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    hooks.status(level, line);
}

static void quiesce(bool halt) {
    if (hooks.quiesce) hooks.quiesce(halt);
}

static int parseHex32(const char *hex, uint8_t out[32]) {
    auto nib = [](char ch) -> int {
        if (ch >= '0' && ch <= '9') return ch - '0';
        if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
        if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
        return -1;
    };
    for (int i = 0; i < 32; ++i) {
        int hi = nib(hex[i * 2]), lo = nib(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t) ((hi << 4) | lo);
    }
    return 0;
}

static void freeRing() {
    if (ring) { heap_caps_free(ring); ring = nullptr; }
    // secSize is the strategy flag as well as a size, so clearing it here disarms the sector path
    // on every teardown -- including the ones that free the ring and then keep running.
    if (secBuf) { heap_caps_free(secBuf); secBuf = nullptr; }
    secSize = secOff = secFill = 0;
    rHead = rTail = rCount = 0;
}

static void grantCredit() {
    // High-water mark: the host may stream up to (written + ringCap) cumulative bytes. Advance it as
    // flash drains; announce advances only in credStep() jumps, while repeatCredit recovers a lost one.
    uint32_t g = written + ringCap;
    if (g > expectedSize) g = expectedSize;
    if (g >= lastGranted + credStep() || g == expectedSize) {
        lastGranted = g;
        emit(OtaBleLevel::Info, "OTAB CRED %u", (unsigned) g);
        creditRepeatArmed = false;
    }
}

static void repeatCredit(uint32_t nowMs) {
    if (!creditRepeatArmed) {
        creditRepeatAt = nowMs;
        creditRepeatArmed = true;
        return;
    }
    if (nowMs - creditRepeatAt < CRED_REPEAT_MS) return;
    creditRepeatAt = nowMs;
    emit(OtaBleLevel::Info, "OTAB CRED %u", (unsigned) lastGranted);
}

/// Consumer only. How many wire bytes of the current session this module is willing to swear are
/// in the update slot, rounded down to an erase sector.
///
/// Rounding down is not caution about flash: esp_ota_write buffers a partial trailing chunk of its
/// own under flash encryption, and a resumed transfer has to restart on a boundary it may erase.
/// Every byte above the returned offset is re-sent, which costs link time and nothing else; a byte
/// wrongly claimed below it would be spliced into the image unread. Only one of those is recoverable.
static uint32_t durablePrefix() {
    if (xform != OtaXform::Raw) return 0; // a wire offset is not an image offset under a transform
    if (!otaPart) return 0;
    const uint32_t esz = (uint32_t) otaPart->erase_size;
    if (!esz) return 0;
    // With the sector path this is exactly what commitSector reported; otherwise esp_ota_write owns
    // the slot and imageWritten is what it was handed, so floor it.
    const uint32_t off = secSize ? flushed : (imageWritten / esz) * esz;
    if (off < esz) return 0;
    // Leave the host at least one sector to send. A resume that has nothing left to stream never
    // drains the ring, so the receiver never emits the final PROG the host waits for before `end`.
    if (off >= expectedSize) return expectedSize > esz ? ((expectedSize - 1) / esz) * esz : 0;
    return off;
}

/// Consumer only. Remember where an interrupted transfer got to, so the next one can carry on.
/// Called on teardown; anything it cannot establish leaves no record at all, because a record is a
/// promise about flash and a wrong one costs a whole transfer to discover.
static void recordResumePoint() {
    resValid = false;
    if (!resRecordable) return;
    if (xform != OtaXform::Raw) return;
    // esp_ota_write_with_offset refuses any size that is not a multiple of 16 under encryption, and
    // the resumed session writes through nothing else. Refusing here rather than at resume time
    // means the host is never offered a restart the receiver could not honour.
    if (esp_flash_encryption_enabled()) return;
    const uint32_t off = durablePrefix();
    if (!off) return;
    resPart = otaPart;
    resOff = off;
    resSize = expectedSize;
    memcpy(resSha, expectedSha, 32);
    resValid = true;
}

/// Consumer task. Answer "what are you running, and what can you accept?" -- everything a host
/// needs to decide between a delta, a compressed push and a raw one, before it commits to any.
/// Three short lines rather than one: a host still on the default 23-byte ATT MTU can only carry
/// 20 bytes per notification, and a single combined line would be silently truncated there.
static void reportInfo() {
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (!run) { emit(OtaBleLevel::Warn, "OTAB FAIL no-partition"); return; }
    const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
    emit(OtaBleLevel::Info, "OTAB INFO run=%s slot=%u", run->label,
         (unsigned) (next ? next->size : 0));

    uint8_t sha[32];
    if (esp_partition_get_sha256(run, sha) == ESP_OK) {
        char hex[65];
        for (int i = 0; i < 32; ++i) snprintf(hex + i * 2, 3, "%02x", sha[i]);
        emit(OtaBleLevel::Info, "OTAB BASE %s", hex);
    } else {
        // Without this a host cannot tell "no delta possible" from "the reply was lost", and the
        // difference decides whether it falls back or retries.
        emit(OtaBleLevel::Warn, "OTAB BASE none");
    }

    // Before XFORM, which the host tools treat as the last line of the reply. It is also the whole
    // capability negotiation: a receiver that predates resume emits no RESUME line, and a host that
    // sees none does a fresh push -- exactly what it did before this existed. `none` is deliberately
    // NOT silence: "I can resume but have nothing" and "I have never heard of resume" are different
    // facts about the device, and only the first one is worth retrying against.
    if (resValid && resPart && resPart == next) {
        emit(OtaBleLevel::Info, "OTAB RESUME %u %u", (unsigned) resOff, (unsigned) resSize);
    } else {
        emit(OtaBleLevel::Info, "OTAB RESUME none");
    }

    char list[48];
    int n = snprintf(list, sizeof(list), "raw");
    if (otaXformAvailable(OtaXform::Tamp)) n += snprintf(list + n, sizeof(list) - n, ",tamp");
    if (otaXformAvailable(OtaXform::Delta)) snprintf(list + n, sizeof(list) - n, ",delta");
    emit(OtaBleLevel::Info, "OTAB XFORM %s", list);
}

void otaBleInit(const OtaBleHooks &h) {
    hooks = h;
    // A resume point is a claim about what is in the update slot right now. Init means the start of
    // the world -- either a boot, where the claim died with the session that made it, or a consumer
    // re-initialising, where this module has no idea what happened in between. Neither is a state
    // in which an old claim is worth keeping.
    resValid = false;
    resRecordable = false;
}

bool otaBleActive() { return active; }

/// `resumeFrom` is 0 for a fresh transfer, or the wire offset a previously interrupted transfer of
/// exactly this size and digest is to be continued from. Resume is raw-only and the caller has
/// already checked the offset against the recorded one; see recordResumePoint().
static bool beginWithDigest(uint32_t size, const uint8_t sha[32], OtaXform x, uint32_t outSize,
                            uint32_t resumeFrom = 0) {
    if (active) { emit(OtaBleLevel::Warn, "OTAB FAIL already-active"); return false; }
    // Whichever way this goes, the recorded prefix stops describing the slot the moment anything
    // below erases it -- and a resume consumes its own record. Drop it here, before any of that.
    resValid = false;
    resRecordable = false;
    otaPart = esp_ota_get_next_update_partition(nullptr);
    if (!otaPart) { emit(OtaBleLevel::Warn, "OTAB FAIL no-partition"); return false; }
    // The IMAGE has to fit the slot; that is the only size bound worth enforcing here. The wire
    // payload is deliberately NOT required to be smaller than the image: a transform that expands
    // is pointless but not unsafe, and tamp genuinely does expand incompressible input, so the
    // "obvious" wire <= image rule would reject a legitimate push. What actually protects the slot
    // is the length check at the end -- imageWritten must equal outSize before anything boots.
    if (outSize == 0 || outSize > otaPart->size) {
        emit(OtaBleLevel::Warn, "OTAB FAIL size %u > part %u", (unsigned) outSize, (unsigned) otaPart->size);
        return false;
    }
    if (size == 0) { emit(OtaBleLevel::Warn, "OTAB FAIL bad-size"); return false; }
    // PSRAM first and BIG; internal memory second and small. The fallback is the old behaviour
    // exactly, so a no-PSRAM board is unaffected. ringCap is set from what was actually obtained -
    // never from what was asked for, or a failed large allocation would leave the modulo arithmetic
    // below indexing past the buffer.
    ringCap = RING_CAP_PSRAM;
    ring = (uint8_t *) heap_caps_malloc(ringCap, MALLOC_CAP_SPIRAM);
    if (!ring) {
        ringCap = RING_CAP_MIN;
        ring = (uint8_t *) heap_caps_malloc(ringCap, MALLOC_CAP_SPIRAM);
    }
    if (!ring) {
        ringCap = RING_CAP_MIN;
        ring = (uint8_t *) heap_caps_malloc(ringCap, MALLOC_CAP_DEFAULT); // PSRAM-less fallback
    }
    if (!ring) { emit(OtaBleLevel::Warn, "OTAB FAIL no-mem"); return false; }
    emit(OtaBleLevel::Info, "OTAB RING %u", (unsigned) ringCap);

    memcpy(expectedSha, sha, 32);
    quiesce(true); // free the CPU/flash for the erase + writes before anything touches the partition

    // Where the transfer time goes. OTA_WITH_SEQUENTIAL_WRITES defers erasing to esp_ota_write,
    // which then erases every 4 KB sector as the write pointer crosses it -- UNCONDITIONALLY, so
    // pre-erasing cannot help (esp_ota_ops.c:313-327). Measured on flu 2026-09-08 via OTAB STAT:
    // 32.4 s of a 43.4 s push was inside esp_ota_write, ~430 sector erases for a 1.76 MB image.
    // Passing the real size instead erases once, up front, in a single esp_partition_erase_range
    // that uses 64 KB block erases -- far cheaper per byte than 430 sector erases.
    //
    // That was originally rejected because the erase blocks and could starve the idle task into a
    // panic-on-timeout watchdog. Both halves of that are config-dependent, so take the fast path
    // only where they are provably absent: YIELD_DURING_ERASE makes the erase release the CPU
    // periodically (keeping the BLE task alive and the link up), and without WDT_PANIC a late idle
    // task logs rather than reboots. A consumer built without those keeps the old, slower, always
    // safe behaviour. Both modes need strictly sequential offsets, which the ring's FIFO drain gives.
    // Cleared first so the failure path below can tell whether a handle was ever handed out.
    // esp_ota_begin assigns *out_handle and registers the operation BEFORE it erases, and returns
    // an erase error without unregistering it (esp_ota_ops.c:181 vs :196-199 in IDF 5.5.1) -- so an
    // erase I/O failure leaves a live operation that only esp_ota_abort releases. Its earlier
    // returns never touch *out_handle, which is what makes the zero a reliable discriminator.
    otaHandle = 0;
    erasedTo = eraseTarget = 0;
    skSkipped = skWritten = 0;
    benchProgramUs = benchCompareUs = benchBeginUs = 0;
    benchProgramBytes = benchEraseBytes = 0;
    benchStrategy = "unknown";
    const uint32_t esz = (uint32_t) otaPart->erase_size;

    if (resumeFrom) {
        // Resume owns the slot through the sector path whether or not skip-identical is this
        // build's default strategy. That is not a preference: esp_ota_write_with_offset is the only
        // writer here that can start anywhere but zero, and it never erases for you, so the
        // per-sector erase+program in commitSector is the only thing that can put the tail down.
        // The sector buffer is therefore mandatory, not an optimisation, and its absence refuses
        // the resume rather than failing the push -- the host still has a fresh `begin`.
        if (esz < CMP_WINDOW || esz > 8192) {
            emit(OtaBleLevel::Warn, "OTAB FAIL resume-sector");
            freeRing();
            quiesce(false);
            return false;
        }
        secBuf = (uint8_t *) heap_caps_malloc(esz, MALLOC_CAP_SPIRAM);
        if (!secBuf) secBuf = (uint8_t *) heap_caps_malloc(esz, MALLOC_CAP_DEFAULT);
        if (!secBuf) {
            emit(OtaBleLevel::Warn, "OTAB FAIL resume-no-mem");
            freeRing();
            quiesce(false);
            return false;
        }
        // Sector 0 has to be rescued BEFORE esp_ota_begin. One sector is the smallest erase that
        // still leaves need_erase == false -- which is what hands this module every later erase --
        // and that sector is the head of the prefix we are resuming on top of. Read it out, let
        // begin erase it, put it straight back. The rewrite is not only about the bytes: esp_ota_end
        // refuses a handle nothing was ever written through, and a resume near the end of an image
        // may otherwise write very little.
        if (esp_partition_read(otaPart, 0, secBuf, esz) != ESP_OK) {
            emit(OtaBleLevel::Warn, "OTAB FAIL resume-read");
            freeRing();
            quiesce(false);
            return false;
        }
        esp_err_t rerr = esp_ota_begin(otaPart, esz, &otaHandle);
        if (rerr != ESP_OK) {
            emit(OtaBleLevel::Warn, "OTAB FAIL esp_ota_begin %s", esp_err_to_name(rerr));
            if (otaHandle) { esp_ota_abort(otaHandle); otaHandle = 0; }
            freeRing();
            quiesce(false);
            return false;
        }
        rerr = esp_ota_write_with_offset(otaHandle, secBuf, esz, 0);
        if (rerr != ESP_OK) {
            emit(OtaBleLevel::Warn, "OTAB FAIL resume-write %s", esp_err_to_name(rerr));
            esp_ota_abort(otaHandle);
            otaHandle = 0;
            freeRing();
            quiesce(false);
            return false;
        }
    } else {

    // Strategy, first refusal to skip-identical-sectors. It subsumes the other two: it removes
    // flash work rather than rescheduling it, and it does so under every transform, because it
    // acts on the RECONSTRUCTED image and never looks at the wire.
    bool skipping = false;
#if OTA_BLE_SECTOR_SKIP
    // Two things can withdraw it, and both fall back rather than fail.
    //  - Flash encryption. The ciphertext is address-tweaked, so identical plaintext at the same
    //    offset does encrypt identically and the comparison would in principle hold -- but
    //    esp_ota_write_with_offset refuses any size that is not a multiple of 16 under encryption
    //    (esp_ota_ops.c:410-413), and none of this is testable on the boards in hand. An untested
    //    guard on an encrypted slot is worth less than the erase it saves.
    //  - No RAM for the sector buffer. A no-PSRAM board is exactly where this matters most, so it
    //    is worth trying internal heap, but not worth failing the push over.
    if (!esp_flash_encryption_enabled() && esz >= CMP_WINDOW && esz <= 8192) {
        secBuf = (uint8_t *) heap_caps_malloc(esz, MALLOC_CAP_SPIRAM);
        if (!secBuf) secBuf = (uint8_t *) heap_caps_malloc(esz, MALLOC_CAP_DEFAULT);
        skipping = secBuf != nullptr;
    }
#endif
    esp_err_t err;
    const int64_t benchBeginStarted = esp_timer_get_time();
    if (skipping) {
        benchStrategy = "sector";
        // Erase exactly ONE sector inside begin. That is the smallest request that still leaves
        // need_erase == false (esp_ota_ops.c:179), which is what hands every later erase to this
        // module -- and it has to be the smallest, because anything esp_ota_begin erases up front
        // is slot content destroyed before the comparison could read it.
        err = esp_ota_begin(otaPart, esz, &otaHandle);
        if (err == ESP_OK) {
            secSize = esz; // arms the sector path in xformSink
            secOff = secFill = 0;
        }
    } else
#if OTA_BLE_BENCH_STRATEGY == 1
    {
        benchStrategy = "sequential";
        err = esp_ota_begin(otaPart, OTA_WITH_SEQUENTIAL_WRITES, &otaHandle);
    }
#elif OTA_BLE_BENCH_STRATEGY == 2
    {
        benchStrategy = "blocks";
        const uint32_t head = outSize < ERASE_HEAD ? outSize : ERASE_HEAD;
        err = esp_ota_begin(otaPart, head, &otaHandle);
        if (err == ESP_OK) {
            erasedTo = ((head + esz - 1) / esz) * esz;
            eraseTarget = ((outSize + esz - 1) / esz) * esz;
            benchEraseBytes = erasedTo;
        }
    }
#elif OTA_BLE_BENCH_STRATEGY == 3
    {
        benchStrategy = "upfront";
        err = esp_ota_begin(otaPart, outSize, &otaHandle);
        if (err == ESP_OK) benchEraseBytes = ((outSize + esz - 1) / esz) * esz;
    }
#else
#if defined(CONFIG_SPI_FLASH_YIELD_DURING_ERASE) && !defined(CONFIG_ESP_TASK_WDT_PANIC)
    // Which erase strategy, decided by who the bottleneck is going to be.
    //
    // Erasing up front costs 6.2 s of dead time before the first byte can arrive. Erasing ahead of
    // the write pointer instead removes that wait -- READY in 0.40 s rather than 6.2 s, measured --
    // but it is NOT free: a 64 KB block erase takes ~215 ms and blocks the same consumer task that
    // drains the staging ring. At the link's ~47 kB/s the host delivers ~10 KB in that time, more
    // than the 8 KB ring holds, so the credit window empties and the LINK stalls.
    //
    // Measured on flu 2026-09-08, and the two cases point opposite ways:
    //   delta, wire 5 % of the image:  total   21.2 s -> 19.8 s   (win: host was never the limit)
    //   raw,   wire 100 %:             transfer 32.8 s -> 36.8 s  (loss: every pause stalls the link)
    // Erase and write serialise on one flash die, so overlapping them cannot save write time; the
    // only thing erase-ahead can recover is LINK time, and only a payload far smaller than the
    // image leaves the device with idle moments to spend erasing. A quarter is a wide margin
    // between the two regimes seen here (5 % and 69 %/100 %).
    {
    const bool eraseAheadPays = (uint64_t) size * 4 < (uint64_t) outSize;
    if (eraseAheadPays) {
        benchStrategy = "ahead";
        // ALIGN_UP by hand: esp_ota_begin erases ALIGN_UP(size, erase_size) and we must know
        // exactly how much that was, because everything past it is ours to erase.
        const uint32_t head = outSize < ERASE_HEAD ? outSize : ERASE_HEAD;
        err = esp_ota_begin(otaPart, head, &otaHandle);
        if (err == ESP_OK) {
            erasedTo = ((head + esz - 1) / esz) * esz;
            eraseTarget = ((outSize + esz - 1) / esz) * esz;
            if (eraseTarget > otaPart->size) eraseTarget = (uint32_t) otaPart->size;
            if (erasedTo > eraseTarget) erasedTo = eraseTarget;
        }
    } else {
        benchStrategy = "upfront";
        err = esp_ota_begin(otaPart, outSize, &otaHandle); // erase it all now; eraseTarget stays 0
    }
    }
#else
    { benchStrategy = "sequential"; err = esp_ota_begin(otaPart, OTA_WITH_SEQUENTIAL_WRITES, &otaHandle); }
#endif
#endif
    benchBeginUs = esp_timer_get_time() - benchBeginStarted;
    if (err != ESP_OK) {
        emit(OtaBleLevel::Warn, "OTAB FAIL esp_ota_begin %s", esp_err_to_name(err));
        if (otaHandle) { esp_ota_abort(otaHandle); otaHandle = 0; }
        freeRing();
        quiesce(false);
        return false;
    }
    } // end of the fresh-transfer strategy selection; a resume opened its own handle above
    // A delta patch names the base it was built against. Hash the running image now so the
    // transform can reject a patch aimed at a different one -- the digest is over 1.7 MB of flash,
    // but it is read once per session and only when a delta actually asked for it.
    if (x == OtaXform::Delta) {
        const esp_partition_t *run = esp_ota_get_running_partition();
        if (!run || esp_partition_get_sha256(run, baseSha) != ESP_OK) {
            emit(OtaBleLevel::Warn, "OTAB FAIL no-base");
            esp_ota_abort(otaHandle);
            otaHandle = 0;
            freeRing();
            quiesce(false);
            return false;
        }
    }

    OtaXformIo io;
    io.write = &xformSink;
    io.read = &xformSrc;
    io.baseDigest = baseSha;
    const char *xerr = "xform";
    if (!otaXformBegin(x, outSize, io, &xerr)) {
        emit(OtaBleLevel::Warn, "OTAB FAIL %s", xerr);
        esp_ota_abort(otaHandle);
        otaHandle = 0;
        freeRing();
        quiesce(false);
        return false;
    }

    mbedtls_sha256_init(&shaCtx);
    mbedtls_sha256_starts(&shaCtx, 0); // 0 = SHA-256
    if (resumeFrom) {
        // THE DIGEST DECISION, and the whole reason a resume is safe.
        //
        // `end` verifies a SHA-256 over the entire wire payload, and the prefix that already
        // reached flash was hashed by a session that no longer exists. There are two ways to keep
        // that check honest: carry the hash state across the drop, or recompute it. This recomputes
        // it, by READING THE PREFIX BACK OUT OF THE UPDATE SLOT -- which is strictly the stronger
        // of the two, because the bytes in the slot are the bytes that will boot. A carried-over
        // hash state would only re-prove what the link delivered into RAM last time and would say
        // nothing about what survived in flash.
        //
        // What this buys, concretely: a resume onto a slot holding a prefix of some OTHER build, a
        // slot clobbered in between, a wrong offset, or a truncated write all end at `end` with
        // OTAB FAIL sha-mismatch, an abort, and a boot partition that never moved. Resume cannot
        // splice two images together; the worst it can do is waste a transfer.
        //
        // Raw only, so the wire prefix and the slot prefix are the same bytes. Sector 0 comes from
        // the buffer because esp_ota_begin has already erased it in flash.
        mbedtls_sha256_update(&shaCtx, secBuf, esz);
        bool readErr = false;
        for (uint32_t o = esz; o < resumeFrom;) {
            const uint32_t n = std::min<uint32_t>(esz, resumeFrom - o);
            if (esp_partition_read(otaPart, o, secBuf, n) != ESP_OK) { readErr = true; break; }
            mbedtls_sha256_update(&shaCtx, secBuf, n);
            o += n;
        }
        if (readErr) {
            // Unreadable slot: there is no prefix to build on, so there is no resume. Not a
            // downgrade to "hash what we can" -- an unevaluable check is never permission.
            emit(OtaBleLevel::Warn, "OTAB FAIL resume-read");
            mbedtls_sha256_free(&shaCtx);
            otaXformEnd();
            esp_ota_abort(otaHandle);
            otaHandle = 0;
            freeRing();
            quiesce(false);
            return false;
        }
        secSize = esz; // arms the sector path in xformSink, whatever the build default is
        secOff = resumeFrom;
        secFill = 0;
    }
    rHead = rTail = rCount = 0;
    xform = x;
    expectedSize = size;
    expectedOut = outSize;
    imageWritten = resumeFrom;
    written = resumeFrom;
    flushed = resumeFrom;
    staged = lastGranted = lastProg = 0;
    sinkFailed = false;
    wrUs = 0; wrCalls = wrSlowCalls = wrMaxUs = 0;
    erUs = erCalls = 0;
    failed = false;
    abortReq = false; // an abort aimed at a previous session must not poison this one
    creditRepeatArmed = false;
    stallArmed = false;
    active = true;
    resRecordable = true;
    // READY stays byte-for-byte what it was for a fresh transfer; a resumed one appends where it
    // picked up. Every host matches on the prefix, so the extra field costs nothing and a log that
    // does not say "from=" is a transfer that really did start at zero.
    if (resumeFrom) {
        emit(OtaBleLevel::Info, "OTAB READY part=%s size=%u xform=%s out=%u from=%u",
             otaPart->label, (unsigned) size, otaXformName(x), (unsigned) outSize,
             (unsigned) resumeFrom);
    } else {
        emit(OtaBleLevel::Info, "OTAB READY part=%s size=%u xform=%s out=%u",
             otaPart->label, (unsigned) size, otaXformName(x), (unsigned) outSize);
    }
    grantCredit();
    return true;
}

bool otaBleResume(uint32_t offset, uint32_t size, const char *sha256hex) {
    uint8_t sha[32];
    if (!sha256hex || strlen(sha256hex) != 64 || parseHex32(sha256hex, sha) != 0) {
        emit(OtaBleLevel::Warn, "OTAB FAIL bad-sha");
        return false;
    }
    // The offset is the receiver's own number, echoed back. Accepting the host's instead would let
    // a stale tool restart at an offset nothing ever wrote.
    if (!resValid || offset != resOff || size != resSize || memcmp(sha, resSha, 32) != 0 ||
        resPart != esp_ota_get_next_update_partition(nullptr)) {
        emit(OtaBleLevel::Warn, "OTAB FAIL resume-mismatch");
        return false;
    }
    return beginWithDigest(size, sha, OtaXform::Raw, size, offset);
}

bool otaBleBegin(uint32_t size, const char *sha256hex, const char *xformName, uint32_t outSize) {
    uint8_t sha[32];
    if (!sha256hex || strlen(sha256hex) != 64 || parseHex32(sha256hex, sha) != 0) {
        emit(OtaBleLevel::Warn, "OTAB FAIL bad-sha");
        return false;
    }
    OtaXform x = OtaXform::Raw;
    if (xformName && !otaXformParse(xformName, &x)) {
        emit(OtaBleLevel::Warn, "OTAB FAIL bad-xform");
        return false;
    }
    return beginWithDigest(size, sha, x, x == OtaXform::Raw ? size : outSize);
}

void otaBleStageBytes(const uint8_t *data, size_t len) {
    if (!len) return;
    std::lock_guard<std::mutex> lk(ringMutex);
    if (!active || failed || !ring) return; // re-check under lock: consumer teardown frees ring here too
    if (len > ringCap - rCount) { // host overran its credit window -- consumer reports + aborts
        failed = true;
        return;
    }
    size_t first = std::min(len, ringCap - rHead);
    memcpy(ring + rHead, data, first);
    if (len > first) memcpy(ring, data + first, len - first);
    rHead = (rHead + len) % ringCap;
    rCount += len;
    staged += len;
}

static bool abortIfOverrun() {
    bool overrun;
    {
        std::lock_guard<std::mutex> lk(ringMutex);
        overrun = active && failed;
    }
    if (!overrun) return false;
    // Reporting from the producer would run the consumer's status transport while ringMutex is
    // held on the BLE host task. Defer both the diagnosis and teardown to this consumer-side path.
    emit(OtaBleLevel::Warn, "OTAB FAIL credit-overrun");
    otaBleAbort();
    return true;
}

static bool abortIfStalled(uint32_t nowMs) {
    uint32_t stagedNow;
    size_t   countNow;
    {
        std::lock_guard<std::mutex> lk(ringMutex);
        if (!active) return false;
        stagedNow = staged;
        countNow  = rCount;
    }
    /* PROGRESS IS EITHER SIDE MOVING, NOT JUST BYTES ARRIVING.
     *
     * This watched `staged` alone - bytes the producer has taken from the link.
     * That is the right measure for a peer that vanished mid-transfer, and the
     * wrong one at the END of a healthy transfer: once the host has sent the
     * last byte there is nothing left to arrive, `staged` stops by definition,
     * and the consumer still has a whole ring to drain to flash. The watchdog
     * then killed a transfer that was progressing normally.
     *
     * MEASURED 2026-09-10, bench board with the 256 KB PSRAM ring: all 711 904
     * bytes reached the device (elapsed 109 s, 6.38 kB/s, first quarter
     * 28.27 kB/s) and the push still ended `OTAB FAIL stalled`, because
     * draining the tail of a large ring takes longer than STALL_MS with no
     * inbound bytes to refresh the mark. An 8 KB ring hid this: its tail drains
     * in milliseconds.
     *
     * So the mark is staged AND written. Either advancing means the transfer is
     * alive; only both stopping for STALL_MS is a real stall - which is exactly
     * what a vanished peer looks like, so the original purpose is preserved. */
    const uint32_t progressNow = stagedNow + written;
    if (!stallArmed || progressNow != stallMark) {
        stallMark = progressNow;
        stallAt = nowMs;
        stallArmed = true;
        return false;
    }
    if (nowMs - stallAt < STALL_MS) return false;
    // A disappeared peer cannot finish an open handle. Bounding this state also bounds how long a
    // consumer's quiesce hook can leave its sampler or power stage halted after a missed disconnect.
    // The counts are the whole diagnosis of a stall, and guessing them from PROG is how the
    // 256 KB ring got mis-blamed on the drain path: `staged` short of the size the host says it
    // sent means the LINK dropped writes, while staged == size with rCount > 0 means the consumer
    // really is not draining. One line settles which, months later, from a log.
    emit(OtaBleLevel::Warn, "OTAB FAIL stalled staged=%u written=%u ring=%u/%u",
         (unsigned) stagedNow, (unsigned) written, (unsigned) countNow, (unsigned) ringCap);
    otaBleAbort();
    return true;
}

/// Consumer only. Erase forward until `needTo` bytes of the slot are ready, one chunk at a time so
/// each call stays short enough to keep the BLE host task alive. Returns false on a flash error.
/// A no-op unless the up-front-erase strategy handed us the job (eraseTarget != 0).
static esp_err_t eraseAhead(uint32_t needTo) {
    if (!eraseTarget) return ESP_OK; // sequential-write strategy: esp_ota_write owns the erasing
    if (needTo > eraseTarget) needTo = eraseTarget;
    while (erasedTo < needTo) {
        uint32_t n = eraseTarget - erasedTo;
        if (n > ERASE_CHUNK) n = ERASE_CHUNK;
        const int64_t t0 = esp_timer_get_time();
        const esp_err_t err = esp_partition_erase_range(otaPart, erasedTo, n);
        erUs += (uint32_t) (esp_timer_get_time() - t0);
        erCalls++;
        if (err != ESP_OK) return err; // the caller's write path reports and tears down
        benchEraseBytes += n;
        erasedTo += n;
    }
    return ESP_OK;
}

/// Consumer only. Move everything staged to flash. Returns false if a write failed (session aborted).
static bool drainRing() {
    static uint8_t slice[FLUSH_SLICE];
    bool drained = false;
    for (;;) {
        size_t n;
        {
            std::lock_guard<std::mutex> lk(ringMutex);
            n = std::min(rCount, (size_t) FLUSH_SLICE);
            if (n == 0) break;
            size_t first = std::min(n, ringCap - rTail);
            memcpy(slice, ring + rTail, first);
            if (n > first) memcpy(slice + first, ring, n - first);
            rTail = (rTail + n) % ringCap;
            rCount -= n;
        }
        // Transform + flash write happen outside the lock so the producer's stage call never blocks
        // on flash I/O. For Raw this is exactly the esp_ota_write it replaced; for Tamp and Delta the
        // timing below now covers reconstruction as well, which is the point -- it is what tells a
        // host whether a smaller payload actually bought anything.
        const int64_t t0 = esp_timer_get_time();
        sinkFailed = false;
        esp_err_t err = otaXformFeed(slice, n);
        const uint32_t dt = (uint32_t) (esp_timer_get_time() - t0);
        wrUs += dt;
        wrCalls++;
        if (dt > 5000) wrSlowCalls++;
        if (dt > wrMaxUs) wrMaxUs = dt;
        if (err != ESP_OK) {
            emit(OtaBleLevel::Error, sinkFailed ? "OTAB FAIL esp_ota_write %s" : "OTAB FAIL xform %s",
                 esp_err_to_name(err));
            { std::lock_guard<std::mutex> lk(ringMutex); failed = true; }
            otaBleAbort();
            return false;
        }
        mbedtls_sha256_update(&shaCtx, slice, n);
        written += n;
        drained = true;
    }
    if (drained) {
        if (written >= lastProg + 64 * 1024 || written == expectedSize) {
            lastProg = written;
            emit(OtaBleLevel::Info, "OTAB PROG %u/%u", (unsigned) written, (unsigned) expectedSize);
            if (written == expectedSize) {
                emit(OtaBleLevel::Info, "OTAB STAT write_ms=%u calls=%u erase_ms=%u out=%u",
                     (unsigned) (wrUs / 1000), (unsigned) wrCalls,
                     (unsigned) (erUs / 1000), (unsigned) imageWritten);
            }
        }
        grantCredit();
    }
    return true;
}

void otaBleTick(uint32_t nowMs) {
    // Latched commands run here, never on the producer task: begin and end block on flash for far
    // longer than a BLE host callback may.
    PendingCmd cmd;
    uint32_t size, outSize, off;
    OtaXform x;
    uint8_t sha[32];
    {
        std::lock_guard<std::mutex> lk(cmdMutex);
        cmd = pending;
        size = pendSize;
        outSize = pendOut;
        off = pendOff;
        x = pendXform;
        memcpy(sha, pendSha, 32);
    }
    if (cmd != PendingCmd::None) {
        switch (cmd) {
            case PendingCmd::Begin: beginWithDigest(size, sha, x, outSize); break;
            // Re-checked inside, not merely at submit: the record could have been dropped by
            // anything that ran between the two, and the offset decides where bytes land.
            case PendingCmd::Resume: {
                char hex[65];
                for (int i = 0; i < 32; ++i) snprintf(hex + i * 2, 3, "%02x", sha[i]);
                otaBleResume(off, size, hex);
                break;
            }
            case PendingCmd::End:   otaBleEnd(); break; // reboots and does not return, on success
            case PendingCmd::Abort: otaBleAbort(); break;
            case PendingCmd::Info:  reportInfo(); break;
            case PendingCmd::None:  break;
        }
        bool cancelStartedBegin;
        {
            // Cleared only after execution, so a command arriving mid-execution is rejected rather
            // than queued behind one whose outcome the host has not seen yet. A disconnect may,
            // however, replace Begin with Abort while the slow begin is executing; honour it after
            // begin returns rather than letting beginWithDigest's stale-abort reset erase it.
            std::lock_guard<std::mutex> lk(cmdMutex);
            cancelStartedBegin = (cmd == PendingCmd::Begin || cmd == PendingCmd::Resume) &&
                                 pending == PendingCmd::Abort;
            pending = PendingCmd::None;
        }
        if (cancelStartedBegin && active) { otaBleAbort(); return; }
    }

    if (!active) return;
    if (abortIfOverrun()) return;
    if (abortReq) { otaBleAbort(); return; } // disconnect/abort requested off the consumer task
    if (abortIfStalled(nowMs)) return;
    if (!drainRing()) return;
    // Idle tick: nothing staged, so the flash is free and the link is mid-flight. This is where
    // the erase actually gets hidden -- on a raw or tamp push there are ~15-24 s of link time to
    // bury all 6.2 s of it in. On a delta push there is almost none, which is why delta gains
    // ~0.2 s here and not more: erase and write serialise on one die.
    bool idle;
    { std::lock_guard<std::mutex> lk(ringMutex); idle = rCount == 0; }
    if (idle && eraseTarget && erasedTo < eraseTarget) {
        const esp_err_t err = eraseAhead(erasedTo + ERASE_CHUNK);
        if (err != ESP_OK) {
            emit(OtaBleLevel::Error, "OTAB FAIL erase %s", esp_err_to_name(err));
            otaBleAbort();
            return;
        }
    }
    repeatCredit(nowMs);
}

bool otaBleEnd() {
    if (!active) { emit(OtaBleLevel::Warn, "OTAB FAIL not-active"); return false; }
    if (abortIfOverrun()) return false;
    if (!drainRing()) return false; // drain whatever is still staged; false = aborted on a write error
    if (abortIfOverrun()) return false;

    // Past this point the transfer is complete as far as the host is concerned, and every remaining
    // exit is a verdict on the whole payload -- too short, wrong digest, unreconstructable, refused
    // by esp_ota_end. None of those is a link problem, so none of them leaves a resume point: a
    // host that retried onto one would spend a full transfer rediscovering the same verdict.
    resRecordable = false;
    if (written != expectedSize) {
        emit(OtaBleLevel::Warn, "OTAB FAIL incomplete %u/%u", (unsigned) written, (unsigned) expectedSize);
        otaBleAbort();
        return false;
    }
    uint8_t got[32];
    mbedtls_sha256_finish(&shaCtx, got);
    if (memcmp(got, expectedSha, 32) != 0) {
        // Checked before the transform is finalised: this says the LINK corrupted the payload, and
        // it is the one diagnosis a failed reconstruction would otherwise mask with its own error.
        emit(OtaBleLevel::Warn, "OTAB FAIL sha-mismatch");
        otaBleAbort();
        return false;
    }
    esp_err_t ferr = otaXformFinish();
    if (ferr != ESP_OK) {
        emit(OtaBleLevel::Warn, "OTAB FAIL xform-finish %s", esp_err_to_name(ferr));
        otaBleAbort();
        return false;
    }
    // The payload arrived intact and still did not rebuild the image we were promised. For Raw this
    // is unreachable (the transform is a memcpy of bytes already counted); for Tamp and Delta it is
    // the check that stops a truncated or wrong-base reconstruction from being flagged bootable.
    if (imageWritten != expectedOut) {
        emit(OtaBleLevel::Warn, "OTAB FAIL out %u/%u", (unsigned) imageWritten, (unsigned) expectedOut);
        otaBleAbort();
        return false;
    }
    // The last sector is still in the buffer: it could not be committed until the transform said
    // there were no more bytes coming, because a partial sector has nothing to compare against.
    if (secSize && secFill) {
        const esp_err_t terr = commitSector();
        if (terr != ESP_OK) {
            emit(OtaBleLevel::Warn, "OTAB FAIL esp_ota_write %s", esp_err_to_name(terr));
            otaBleAbort();
            return false;
        }
    }
    if (secSize) {
        // What the comparison actually bought, in the units the erase budget is argued in. Reported
        // before esp_ota_end so it survives a slot that fails validation -- that is precisely when
        // you want to know how much of the slot this push left alone.
        emit(OtaBleLevel::Info, "OTAB SKIP kept=%u wrote=%u erases=%u erase_ms=%u",
             (unsigned) skSkipped, (unsigned) skWritten, (unsigned) erCalls,
             (unsigned) (erUs / 1000));
    }
    emit(OtaBleLevel::Info, "OTAB FLASH mode=%s program_bytes=%u erase_bytes=%u program_ms=%u compare_us=%u begin_ms=%u",
         benchStrategy, (unsigned) benchProgramBytes, (unsigned) benchEraseBytes,
         (unsigned) (benchProgramUs / 1000), (unsigned) benchCompareUs, (unsigned) (benchBeginUs / 1000));
    esp_err_t err = esp_ota_end(otaHandle); // image validation (magic, esp_app_desc, signature)
    if (err != ESP_OK) {
        emit(OtaBleLevel::Warn, "OTAB FAIL esp_ota_end %s", esp_err_to_name(err));
        otaXformEnd();
        mbedtls_sha256_free(&shaCtx);
        { std::lock_guard<std::mutex> lk(ringMutex); active = false; freeRing(); }
        otaHandle = 0;
        quiesce(false);
        return false;
    }
    err = esp_ota_set_boot_partition(otaPart);
    otaXformEnd();
    mbedtls_sha256_free(&shaCtx);
    { std::lock_guard<std::mutex> lk(ringMutex); active = false; freeRing(); }
    otaHandle = 0;
    if (err != ESP_OK) {
        emit(OtaBleLevel::Warn, "OTAB FAIL set-boot %s", esp_err_to_name(err));
        quiesce(false);
        return false;
    }
    emit(OtaBleLevel::Info, "OTAB OK rebooting");
    if (hooks.restart) hooks.restart();
    else esp_restart();
    return true;
}

void otaBleRequestAbort() {
    // A disconnect may land after Begin was published but before the consumer claims it. Replace
    // that command in its own slot; if the consumer already copied it, the post-execution check in
    // otaBleTick catches the replacement without holding cmdMutex across flash work.
    {
        std::lock_guard<std::mutex> lk(cmdMutex);
        if (pending == PendingCmd::Begin || pending == PendingCmd::Resume) {
            pending = PendingCmd::Abort;
            return;
        }
    }
    // Otherwise ignore an idle disconnect. Latching abortReq with no session to consume it would
    // kill the next transfer the moment it armed.
    if (!active) return;
    abortReq = true;
}

void otaBleAbort() {
    if (!active) return;
    // Before freeRing(), which clears the sector bookkeeping this reads. An abort is the ONLY place
    // a resume point is born: a transfer that reached `end` either installed or was rejected, and
    // neither leaves a prefix worth continuing.
    recordResumePoint();
    { std::lock_guard<std::mutex> lk(ringMutex); active = false; freeRing(); }
    otaXformEnd();
    if (otaHandle) { esp_ota_abort(otaHandle); otaHandle = 0; }
    mbedtls_sha256_free(&shaCtx);
    quiesce(false);
    abortReq = false;
    creditRepeatArmed = false;
    stallArmed = false;
    emit(OtaBleLevel::Warn, "OTAB FAIL aborted");
}

OtaBleSubmit otaBleSubmitCommand(const char *line) {
    // Rejections are announced on the status channel as well as returned. The caller's own error
    // path may only reach a console, and a host that sent a malformed command would otherwise learn
    // nothing and sit out its READY timeout -- indistinguishable from a dead link.
    auto reject = [](const char *why) {
        emit(OtaBleLevel::Warn, "OTAB FAIL %s", why);
        return OtaBleSubmit::Rejected;
    };

    if (!line) return reject("bad-command");
    while (*line == ' ') ++line;

    PendingCmd cmd;
    uint32_t size = 0, outSize = 0, off = 0;
    OtaXform x = OtaXform::Raw;
    uint8_t sha[32];

    if (strncmp(line, "begin", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        // "begin <wireSize> <wireSha256hex> [<xform> <imageSize>]". The two-argument form is the
        // original protocol and stays exactly what it was: a raw image, wire size == image size.
        const char *p = line + 5;
        char shaHex[80], xformName[16], extra[8];
        unsigned long parsedSize = 0, parsedOut = 0;
        // The trailing %7s exists to be REJECTED. Without it sscanf stops at four conversions and
        // says nothing about the rest of the line, so "begin n sha delta out EXTRA" parsed as a
        // clean four-field command with EXTRA silently dropped -- a command the host did not mean
        // to send being executed as one it did.
        const int fields = sscanf(p, " %lu %79s %15s %lu %7s",
                                  &parsedSize, shaHex, xformName, &parsedOut, extra);
        if (fields != 2 && fields != 4) return reject("bad-command");
        if (parsedSize == 0 || parsedSize > UINT32_MAX) return reject("bad-size");
        if (strlen(shaHex) != 64 || parseHex32(shaHex, sha) != 0) return reject("bad-sha");
        if (fields == 4) {
            if (!otaXformParse(xformName, &x)) return reject("bad-xform");
            // Answer "can this build do that?" here rather than at execution time: the host has to
            // pick a fallback, and by then it would already have committed to the transfer.
            if (!otaXformAvailable(x)) return reject("xform-unsupported");
            if (parsedOut == 0 || parsedOut > UINT32_MAX) return reject("bad-size");
            outSize = (uint32_t) parsedOut;
        } else {
            outSize = (uint32_t) parsedSize;
        }
        // Reject an oversized image before anything is latched, so the host learns synchronously
        // rather than via an async failure after the consumer has already quiesced sampling.
        const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
        if (!next) return reject("no-partition");
        if (outSize > next->size) {
            emit(OtaBleLevel::Warn, "OTAB FAIL size %u > part %u",
                 (unsigned) outSize, (unsigned) next->size);
            return OtaBleSubmit::Rejected;
        }
        size = (uint32_t) parsedSize;
        cmd = PendingCmd::Begin;
    } else if (strncmp(line, "resume", 6) == 0 && (line[6] == ' ' || line[6] == '\0')) {
        // "resume <wireOffset> <wireSize> <wireSha256hex>" -- carry on an interrupted RAW transfer.
        // A separate verb rather than a fifth field on `begin`, so `begin` stays byte-for-byte the
        // command every existing receiver and host already agree on.
        //
        // All three numbers must match what this receiver recorded, and the offset is its own
        // number echoed back rather than a position the host chose. That is deliberately stricter
        // than it needs to be: the digest check at `end` would catch a wrong resume anyway, but
        // only after spending the entire transfer to find out.
        const char *p = line + 6;
        char shaHex[80], extra[8];
        unsigned long parsedOff = 0, parsedSize = 0;
        const int fields = sscanf(p, " %lu %lu %79s %7s", &parsedOff, &parsedSize, shaHex, extra);
        if (fields != 3) return reject("bad-command");
        if (parsedOff == 0 || parsedOff > UINT32_MAX) return reject("bad-size");
        if (parsedSize == 0 || parsedSize > UINT32_MAX) return reject("bad-size");
        if (strlen(shaHex) != 64 || parseHex32(shaHex, sha) != 0) return reject("bad-sha");
        // Answered synchronously, because the host's next move on a refusal is a fresh `begin` and
        // it should not have to wait out a READY timeout to learn that.
        if (!resValid) return reject("resume-none");
        if (resPart != esp_ota_get_next_update_partition(nullptr)) return reject("resume-none");
        if ((uint32_t) parsedOff != resOff || (uint32_t) parsedSize != resSize ||
            memcmp(sha, resSha, 32) != 0)
            return reject("resume-mismatch");
        size = (uint32_t) parsedSize;
        outSize = size;
        off = (uint32_t) parsedOff;
        cmd = PendingCmd::Resume;
    } else if (strncmp(line, "info", 4) == 0 && (line[4] == ' ' || line[4] == '\0' ||
                                                 line[4] == '\r' || line[4] == '\n')) {
        cmd = PendingCmd::Info;
    } else if (strncmp(line, "end", 3) == 0 && (line[3] == ' ' || line[3] == '\0' ||
                                                line[3] == '\r' || line[3] == '\n')) {
        cmd = PendingCmd::End;
    } else if (strncmp(line, "abort", 5) == 0 && (line[5] == ' ' || line[5] == '\0' ||
                                                  line[5] == '\r' || line[5] == '\n')) {
        cmd = PendingCmd::Abort;
    } else {
        return reject("bad-command");
    }

    std::lock_guard<std::mutex> lk(cmdMutex);
    if (pending != PendingCmd::None) return reject("busy");
    pending = cmd;
    pendSize = size;
    pendOut = outSize;
    pendOff = off;
    pendXform = x;
    if (cmd == PendingCmd::Begin || cmd == PendingCmd::Resume) memcpy(pendSha, sha, 32);
    return OtaBleSubmit::Accepted;
}
