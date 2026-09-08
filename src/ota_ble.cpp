#include "ota_ble.h"
#include "ota_xform.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>

#include <esp_heap_caps.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <mbedtls/sha256.h>

// Staging ring: otaBleStageBytes (producer/BLE host task) only copies bytes in here; otaBleTick
// (consumer) drains to flash. Decoupling keeps the slow esp_ota_write off the host task -- a stall
// there trips the BLE supervision timeout. Capacity doubles as the host's credit window.
// Kept small (8 KB): on a no-PSRAM board internal heap is tight and fragmented, and BLE throughput
// (~tens of KB/s) is far below what this window sustains, so the credit round-trip never bottlenecks.
static constexpr size_t RING_CAP = 8 * 1024;
static constexpr size_t FLUSH_SLICE = 2048;       // flash-page-friendly esp_ota_write granularity
static constexpr size_t CRED_STEP = RING_CAP / 2; // re-grant credit in half-window steps (limits notifies)
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
#if defined(CONFIG_SPI_FLASH_YIELD_DURING_ERASE) && !defined(CONFIG_ESP_TASK_WDT_PANIC)
static constexpr uint32_t ERASE_HEAD = 64 * 1024;  // erased inside begin; one block erase
#endif
static constexpr uint32_t ERASE_CHUNK = 64 * 1024; // 64 KB at a time -> block erases, not 16 sectors
// Stay this far ahead of the write pointer. One chunk is enough: a chunk erase (~215 ms) buys
// 64 KB of writing (~310 ms at the measured 206 kB/s), so the writer never catches up.
static constexpr uint32_t ERASE_LEAD = ERASE_CHUNK;

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
enum class PendingCmd { None, Begin, End, Abort, Info };
static PendingCmd pending = PendingCmd::None;
static uint32_t pendSize = 0;
static uint32_t pendOut = 0;
static OtaXform pendXform = OtaXform::Raw;
static uint8_t pendSha[32];
static std::mutex cmdMutex;

static esp_err_t eraseAhead(uint32_t needTo); // defined below, beside the drain it paces

/// Transform sink: reconstructed image bytes on their way to the passive partition.
/// A failure here is flash refusing the write; a failure anywhere else under otaXformFeed is the
/// payload failing to reconstruct. The two send a host to completely different places -- a bad slot
/// versus a bad patch -- so the drain reports them as different lines rather than one "write" error.
static bool sinkFailed = false;

static esp_err_t xformSink(const uint8_t *data, size_t len, void *) {
    // Erase must lead the write pointer. This is where the image write offset is actually known --
    // a transform decides how many image bytes one wire slice becomes, and for delta that is tens
    // of KB, so bounding the wire slice instead would not bound this.
    const esp_err_t eerr = eraseAhead(imageWritten + (uint32_t) len + ERASE_LEAD);
    if (eerr != ESP_OK) { sinkFailed = true; return eerr; }
    const esp_err_t err = esp_ota_write(otaHandle, data, len);
    if (err != ESP_OK) { sinkFailed = true; return err; }
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
    rHead = rTail = rCount = 0;
}

static void grantCredit() {
    // High-water mark: the host may stream up to (written + RING_CAP) cumulative bytes. Advance it as
    // flash drains; announce advances only in CRED_STEP jumps, while repeatCredit recovers a lost one.
    uint32_t g = written + RING_CAP;
    if (g > expectedSize) g = expectedSize;
    if (g >= lastGranted + CRED_STEP || g == expectedSize) {
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

    char list[48];
    int n = snprintf(list, sizeof(list), "raw");
    if (otaXformAvailable(OtaXform::Tamp)) n += snprintf(list + n, sizeof(list) - n, ",tamp");
    if (otaXformAvailable(OtaXform::Delta)) snprintf(list + n, sizeof(list) - n, ",delta");
    emit(OtaBleLevel::Info, "OTAB XFORM %s", list);
}

void otaBleInit(const OtaBleHooks &h) { hooks = h; }

bool otaBleActive() { return active; }

static bool beginWithDigest(uint32_t size, const uint8_t sha[32], OtaXform x, uint32_t outSize) {
    if (active) { emit(OtaBleLevel::Warn, "OTAB FAIL already-active"); return false; }
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
    ring = (uint8_t *) heap_caps_malloc(RING_CAP, MALLOC_CAP_SPIRAM);
    if (!ring) ring = (uint8_t *) heap_caps_malloc(RING_CAP, MALLOC_CAP_DEFAULT); // PSRAM-less fallback
    if (!ring) { emit(OtaBleLevel::Warn, "OTAB FAIL no-mem"); return false; }

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
    const bool eraseAheadPays = (uint64_t) size * 4 < (uint64_t) outSize;
    const uint32_t esz = (uint32_t) otaPart->erase_size;
    esp_err_t err;
    if (eraseAheadPays) {
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
        err = esp_ota_begin(otaPart, outSize, &otaHandle); // erase it all now; eraseTarget stays 0
    }
#else
    esp_err_t err = esp_ota_begin(otaPart, OTA_WITH_SEQUENTIAL_WRITES, &otaHandle);
#endif
    if (err != ESP_OK) {
        emit(OtaBleLevel::Warn, "OTAB FAIL esp_ota_begin %s", esp_err_to_name(err));
        if (otaHandle) { esp_ota_abort(otaHandle); otaHandle = 0; }
        freeRing();
        quiesce(false);
        return false;
    }
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
    rHead = rTail = rCount = 0;
    xform = x;
    expectedSize = size;
    expectedOut = outSize;
    imageWritten = 0;
    written = staged = lastGranted = lastProg = 0;
    sinkFailed = false;
    wrUs = 0; wrCalls = wrSlowCalls = wrMaxUs = 0;
    erUs = erCalls = 0;
    failed = false;
    abortReq = false; // an abort aimed at a previous session must not poison this one
    creditRepeatArmed = false;
    stallArmed = false;
    active = true;
    emit(OtaBleLevel::Info, "OTAB READY part=%s size=%u xform=%s out=%u",
         otaPart->label, (unsigned) size, otaXformName(x), (unsigned) outSize);
    grantCredit();
    return true;
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
    if (len > RING_CAP - rCount) { // host overran its credit window -- consumer reports + aborts
        failed = true;
        return;
    }
    size_t first = std::min(len, RING_CAP - rHead);
    memcpy(ring + rHead, data, first);
    if (len > first) memcpy(ring, data + first, len - first);
    rHead = (rHead + len) % RING_CAP;
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
    {
        std::lock_guard<std::mutex> lk(ringMutex);
        if (!active) return false;
        stagedNow = staged;
    }
    if (!stallArmed || stagedNow != stallMark) {
        stallMark = stagedNow;
        stallAt = nowMs;
        stallArmed = true;
        return false;
    }
    if (nowMs - stallAt < STALL_MS) return false;
    // A disappeared peer cannot finish an open handle. Bounding this state also bounds how long a
    // consumer's quiesce hook can leave its sampler or power stage halted after a missed disconnect.
    emit(OtaBleLevel::Warn, "OTAB FAIL stalled");
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
            size_t first = std::min(n, RING_CAP - rTail);
            memcpy(slice, ring + rTail, first);
            if (n > first) memcpy(slice + first, ring, n - first);
            rTail = (rTail + n) % RING_CAP;
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
    uint32_t size, outSize;
    OtaXform x;
    uint8_t sha[32];
    {
        std::lock_guard<std::mutex> lk(cmdMutex);
        cmd = pending;
        size = pendSize;
        outSize = pendOut;
        x = pendXform;
        memcpy(sha, pendSha, 32);
    }
    if (cmd != PendingCmd::None) {
        switch (cmd) {
            case PendingCmd::Begin: beginWithDigest(size, sha, x, outSize); break;
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
            cancelStartedBegin = cmd == PendingCmd::Begin && pending == PendingCmd::Abort;
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
        if (pending == PendingCmd::Begin) {
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
    uint32_t size = 0, outSize = 0;
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
    pendXform = x;
    if (cmd == PendingCmd::Begin) memcpy(pendSha, sha, 32);
    return OtaBleSubmit::Accepted;
}
