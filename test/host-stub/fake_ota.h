#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <esp_ota_ops.h>

/// Observable state of the fake flash, so tests assert on what actually happened rather than on
/// return codes. In particular `bootPart` is the thing that must NOT move on any failure path.
struct FakeOta {
    // --- configuration ---
    size_t partSize = 0x1B0000;     ///< matches the smart shunt's slot
    size_t sectorSize = 4096;
    bool noPartition = false;       ///< esp_ota_get_next_update_partition() returns nullptr
    bool failBegin = false;
    /// Fail the way IDF actually fails an erase: the handle is already assigned and the operation
    /// registered, so only esp_ota_abort() releases it. `failBegin` fails earlier, before a handle
    /// exists -- the two are different bugs and only this one leaks.
    bool failBeginDuringErase = false;
    int failWriteAtCall = -1;       ///< 1-based esp_ota_write call number to fail, -1 = never
    bool failEnd = false;
    bool failSetBoot = false;
    bool spiramAvailable = false;   ///< the bench board has none; exercises the malloc fallback

    // --- observations ---
    int beginCalls = 0;
    int liveOtaOps = 0;             ///< esp_ota_begin minus end/abort
    // Erase bookkeeping. The receiver now owns most of the erasing, so the fake has to police the
    // invariant that used to be ESP-IDF's job: nothing may be written to flash that was not erased
    // first. Without this a bug in erase-ahead would produce a passing test and a corrupt image.
    std::vector<bool> erasedSectors;
    bool wroteUnerased = false;     ///< set if esp_ota_write touched an un-erased sector
    int eraseRangeCalls = 0;        ///< explicit esp_partition_erase_range calls (erase-ahead)
    size_t eraseRangeBytes = 0;
    size_t eraseBytesInBegin = 0;   ///< bytes erased synchronously inside esp_ota_begin
    int eraseCallsInWrite = 0;      ///< number of incremental erases performed during writes
    size_t eraseBytesInWrite = 0;
    int writeCalls = 0;
    std::vector<uint8_t> flashed;   ///< everything handed to esp_ota_write, in order
    bool ended = false;
    bool aborted = false;
    const esp_partition_t *bootPart = nullptr;
    int liveAllocs = 0;             ///< heap_caps_malloc minus heap_caps_free

    // --- the running partition, i.e. the base a delta patches from ---
    std::vector<uint8_t> base;      ///< contents esp_partition_read serves
    uint8_t baseSha[32] = {0};      ///< what esp_partition_get_sha256 reports for it
    bool failBaseSha = false;
    std::vector<uint8_t> baseReads; ///< offsets read, so a test can see the source was consulted
};

extern FakeOta g_fake;

void fakeOtaReset();
