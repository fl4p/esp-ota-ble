#include "ota_ble.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>

#include <esp_heap_caps.h>
#include <esp_ota_ops.h>
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

static uint8_t *ring = nullptr;
static size_t rHead = 0, rTail = 0, rCount = 0;   // byte ring indices + fill level
static std::mutex ringMutex;                      // guards ring + rCount across producer / consumer

static esp_ota_handle_t otaHandle = 0;
static const esp_partition_t *otaPart = nullptr;
static mbedtls_sha256_context shaCtx;
static uint8_t expectedSha[32];

static bool active = false;
static bool failed = false;
static volatile bool abortReq = false; // set from any task; consumed by the consumer tick
static uint32_t expectedSize = 0;
static uint32_t written = 0;    // flushed to flash (consumer)
static uint32_t lastGranted = 0;
static uint32_t lastProg = 0;

static OtaBleHooks hooks;

// Single-slot command latch. The protocol is strictly one command at a time -- the host always waits
// for a status line before sending the next -- so a second command arriving before the consumer has
// drained the first is a host bug, and is reported as one rather than silently overwriting.
enum class PendingCmd { None, Begin, End, Abort };
static PendingCmd pending = PendingCmd::None;
static uint32_t pendSize = 0;
static uint8_t pendSha[32];
static std::mutex cmdMutex;

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
    // flash drains; re-announce only in CRED_STEP jumps to avoid notify spam.
    uint32_t g = written + RING_CAP;
    if (g > expectedSize) g = expectedSize;
    if (g >= lastGranted + CRED_STEP || g == expectedSize) {
        lastGranted = g;
        emit(OtaBleLevel::Info, "OTAB CRED %u", (unsigned) g);
    }
}

void otaBleInit(const OtaBleHooks &h) { hooks = h; }

bool otaBleActive() { return active; }

static bool beginWithDigest(uint32_t size, const uint8_t sha[32]) {
    if (active) { emit(OtaBleLevel::Warn, "OTAB FAIL already-active"); return false; }
    otaPart = esp_ota_get_next_update_partition(nullptr);
    if (!otaPart) { emit(OtaBleLevel::Warn, "OTAB FAIL no-partition"); return false; }
    if (size == 0 || size > otaPart->size) {
        emit(OtaBleLevel::Warn, "OTAB FAIL size %u > part %u", (unsigned) size, (unsigned) otaPart->size);
        return false;
    }
    ring = (uint8_t *) heap_caps_malloc(RING_CAP, MALLOC_CAP_SPIRAM);
    if (!ring) ring = (uint8_t *) heap_caps_malloc(RING_CAP, MALLOC_CAP_DEFAULT); // PSRAM-less fallback
    if (!ring) { emit(OtaBleLevel::Warn, "OTAB FAIL no-mem"); return false; }

    memcpy(expectedSha, sha, 32);
    quiesce(true); // free the CPU/flash for the erase + writes before anything touches the partition

    // OTA_WITH_SEQUENTIAL_WRITES, *not* OTA_SIZE_UNKNOWN. The two constants read alike and behave
    // oppositely: a byte size or OTA_SIZE_UNKNOWN makes esp_ota_begin erase synchronously right here
    // (and OTA_SIZE_UNKNOWN erases the WHOLE partition, worse than passing the real size), which on a
    // ~1.7 MB slot blocks long enough to starve the idle task and trip a panic-on-timeout task
    // watchdog. Only this constant defers erasing to per-sector calls inside esp_ota_write, a few tens
    // of ms each. It requires strictly sequential write offsets, which the ring's FIFO drain gives us.
    esp_err_t err = esp_ota_begin(otaPart, OTA_WITH_SEQUENTIAL_WRITES, &otaHandle);
    if (err != ESP_OK) {
        emit(OtaBleLevel::Warn, "OTAB FAIL esp_ota_begin %s", esp_err_to_name(err));
        freeRing();
        quiesce(false);
        return false;
    }
    mbedtls_sha256_init(&shaCtx);
    mbedtls_sha256_starts(&shaCtx, 0); // 0 = SHA-256
    rHead = rTail = rCount = 0;
    expectedSize = size;
    written = lastGranted = lastProg = 0;
    failed = false;
    abortReq = false; // an abort aimed at a previous session must not poison this one
    active = true;
    emit(OtaBleLevel::Info, "OTAB READY part=%s size=%u", otaPart->label, (unsigned) size);
    grantCredit();
    return true;
}

bool otaBleBegin(uint32_t size, const char *sha256hex) {
    uint8_t sha[32];
    if (!sha256hex || strlen(sha256hex) != 64 || parseHex32(sha256hex, sha) != 0) {
        emit(OtaBleLevel::Warn, "OTAB FAIL bad-sha");
        return false;
    }
    return beginWithDigest(size, sha);
}

void otaBleStageBytes(const uint8_t *data, size_t len) {
    if (!len) return;
    std::lock_guard<std::mutex> lk(ringMutex);
    if (!active || failed || !ring) return; // re-check under lock: consumer teardown frees ring here too
    if (len > RING_CAP - rCount) { // host overran its credit window -- fatal, caught later by sha/len
        failed = true;
        return;
    }
    size_t first = std::min(len, RING_CAP - rHead);
    memcpy(ring + rHead, data, first);
    if (len > first) memcpy(ring, data + first, len - first);
    rHead = (rHead + len) % RING_CAP;
    rCount += len;
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
        // Flash write happens outside the lock so the producer's stage call never blocks on flash I/O.
        esp_err_t err = esp_ota_write(otaHandle, slice, n);
        if (err != ESP_OK) {
            emit(OtaBleLevel::Error, "OTAB FAIL esp_ota_write %s", esp_err_to_name(err));
            failed = true;
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
        }
        grantCredit();
    }
    return true;
}

void otaBleTick(uint32_t nowMs) {
    (void) nowMs;

    // Latched commands run here, never on the producer task: begin and end block on flash for far
    // longer than a BLE host callback may.
    PendingCmd cmd;
    uint32_t size;
    uint8_t sha[32];
    {
        std::lock_guard<std::mutex> lk(cmdMutex);
        cmd = pending;
        size = pendSize;
        memcpy(sha, pendSha, 32);
    }
    if (cmd != PendingCmd::None) {
        switch (cmd) {
            case PendingCmd::Begin: beginWithDigest(size, sha); break;
            case PendingCmd::End:   otaBleEnd(); break; // reboots and does not return, on success
            case PendingCmd::Abort: otaBleAbort(); break;
            case PendingCmd::None:  break;
        }
        // Cleared only after execution, so a command arriving mid-execution is rejected rather than
        // queued behind one whose outcome the host has not seen yet.
        std::lock_guard<std::mutex> lk(cmdMutex);
        pending = PendingCmd::None;
    }

    if (!active) return;
    if (abortReq) { otaBleAbort(); return; } // disconnect/abort requested off the consumer task
    drainRing();
}

bool otaBleEnd() {
    if (!active) { emit(OtaBleLevel::Warn, "OTAB FAIL not-active"); return false; }
    if (!drainRing()) return false; // drain whatever is still staged; false = aborted on a write error

    if (failed || written != expectedSize) {
        emit(OtaBleLevel::Warn, "OTAB FAIL incomplete %u/%u", (unsigned) written, (unsigned) expectedSize);
        otaBleAbort();
        return false;
    }
    uint8_t got[32];
    mbedtls_sha256_finish(&shaCtx, got);
    if (memcmp(got, expectedSha, 32) != 0) {
        emit(OtaBleLevel::Warn, "OTAB FAIL sha-mismatch");
        otaBleAbort();
        return false;
    }
    esp_err_t err = esp_ota_end(otaHandle); // image validation (magic, esp_app_desc, signature)
    if (err != ESP_OK) {
        emit(OtaBleLevel::Warn, "OTAB FAIL esp_ota_end %s", esp_err_to_name(err));
        mbedtls_sha256_free(&shaCtx);
        { std::lock_guard<std::mutex> lk(ringMutex); active = false; freeRing(); }
        otaHandle = 0;
        quiesce(false);
        return false;
    }
    err = esp_ota_set_boot_partition(otaPart);
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
    // Ignored when nothing is in flight. Consumers wire this to BLE disconnect, which also fires for
    // clients that never started an OTA; latching the flag then would leave it set forever (nothing
    // clears it while inactive) and kill the *next* transfer the moment it armed.
    if (!active) return;
    abortReq = true;
}

void otaBleAbort() {
    if (!active) return;
    { std::lock_guard<std::mutex> lk(ringMutex); active = false; freeRing(); }
    if (otaHandle) { esp_ota_abort(otaHandle); otaHandle = 0; }
    mbedtls_sha256_free(&shaCtx);
    quiesce(false);
    abortReq = false;
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
    uint32_t size = 0;
    uint8_t sha[32];

    if (strncmp(line, "begin", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        const char *p = line + 5;
        char shaHex[80];
        unsigned long parsedSize = 0;
        if (sscanf(p, " %lu %79s", &parsedSize, shaHex) != 2) return reject("bad-command");
        if (parsedSize == 0 || parsedSize > UINT32_MAX) return reject("bad-size");
        if (strlen(shaHex) != 64 || parseHex32(shaHex, sha) != 0) return reject("bad-sha");
        // Reject an oversized image before anything is latched, so the host learns synchronously
        // rather than via an async failure after the consumer has already quiesced sampling.
        const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
        if (!next) return reject("no-partition");
        if (parsedSize > next->size) {
            emit(OtaBleLevel::Warn, "OTAB FAIL size %u > part %u",
                 (unsigned) parsedSize, (unsigned) next->size);
            return OtaBleSubmit::Rejected;
        }
        size = (uint32_t) parsedSize;
        cmd = PendingCmd::Begin;
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
    if (cmd == PendingCmd::Begin) memcpy(pendSha, sha, 32);
    return OtaBleSubmit::Accepted;
}
