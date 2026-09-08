// In-memory stand-in for ESP-IDF's app_update + heap_caps, faithful on the one axis the module's
// design turns on: WHEN the partition gets erased.
//
// Real behaviour, from components/app_update/esp_ota_ops.c:
//   :179       need_erase = (image_size == OTA_WITH_SEQUENTIAL_WRITES)
//   :189-197   otherwise erase synchronously inside esp_ota_begin -- the whole partition for
//              image_size 0 or OTA_SIZE_UNKNOWN, else ALIGN_UP(image_size, erase_size)
//   :314-323   with need_erase, erase per affected sector from inside esp_ota_write
//
// Modelling only the end state (bytes land in flash) would make both strategies look identical and
// the suite would happily pass on the constant that blocks for seconds inside begin.

#include "fake_ota.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <esp_heap_caps.h>
#include <esp_partition.h>
#include <esp_system.h>

FakeOta g_fake;

static esp_partition_t g_part = {"app1", 0x1B0000, 4096};
static bool g_needErase = false;
static bool g_handleOpen = false;
static size_t g_wroteSize = 0;

void fakeOtaReset() {
    g_fake = FakeOta();
    g_part.label = "app1";
    g_part.size = g_fake.partSize;
    g_part.erase_size = g_fake.sectorSize;
    g_needErase = false;
    g_handleOpen = false;
    g_wroteSize = 0;
}

const char *esp_err_to_name(esp_err_t err) {
    switch (err) {
        case ESP_OK: return "ESP_OK";
        case ESP_ERR_NO_MEM: return "ESP_ERR_NO_MEM";
        case ESP_ERR_INVALID_ARG: return "ESP_ERR_INVALID_ARG";
        case ESP_ERR_INVALID_SIZE: return "ESP_ERR_INVALID_SIZE";
        case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
        case ESP_ERR_NOT_SUPPORTED: return "ESP_ERR_NOT_SUPPORTED";
        case ESP_ERR_INVALID_CRC: return "ESP_ERR_INVALID_CRC";
        case ESP_ERR_INVALID_VERSION: return "ESP_ERR_INVALID_VERSION";
        case ESP_ERR_OTA_VALIDATE_FAILED: return "ESP_ERR_OTA_VALIDATE_FAILED";
        default: return "ESP_FAIL";
    }
}

// The running slot is deliberately a DIFFERENT partition object from the update slot: a delta reads
// one while writing the other, and a fake that conflated them would hide a mixed-up pointer.
static esp_partition_t g_runPart = {"app0", 0x1B0000, 4096};

const esp_partition_t *esp_ota_get_running_partition(void) {
    g_runPart.size = g_fake.partSize;
    g_runPart.erase_size = g_fake.sectorSize;
    return &g_runPart;
}

esp_err_t esp_partition_read(const esp_partition_t *partition, size_t src_offset, void *dst, size_t size) {
    if (partition != &g_runPart) return ESP_ERR_INVALID_ARG;
    if (src_offset + size > g_fake.base.size()) return ESP_ERR_INVALID_SIZE;
    memcpy(dst, g_fake.base.data() + src_offset, size);
    g_fake.baseReads.push_back(1);
    return ESP_OK;
}

static void markErased(size_t offset, size_t size) {
    const size_t sec = g_fake.sectorSize;
    if (g_fake.erasedSectors.size() < g_fake.partSize / sec) {
        g_fake.erasedSectors.assign(g_fake.partSize / sec, false);
    }
    for (size_t o = offset; o < offset + size; o += sec) {
        const size_t i = o / sec;
        if (i < g_fake.erasedSectors.size()) g_fake.erasedSectors[i] = true;
    }
}

esp_err_t esp_partition_erase_range(const esp_partition_t *partition, size_t offset, size_t size) {
    if (partition != &g_part) return ESP_ERR_INVALID_ARG;
    // Real flash refuses a misaligned range outright; a fake that accepted one would hide exactly
    // the kind of arithmetic slip erase-ahead can make.
    if (offset % g_fake.sectorSize || size % g_fake.sectorSize) return ESP_ERR_INVALID_ARG;
    if (offset + size > g_fake.partSize) return ESP_ERR_INVALID_SIZE;
    ++g_fake.eraseRangeCalls;
    g_fake.eraseRangeBytes += size;
    markErased(offset, size);
    return ESP_OK;
}

esp_err_t esp_partition_get_sha256(const esp_partition_t *partition, uint8_t *sha_256) {
    if (partition != &g_runPart) return ESP_ERR_INVALID_ARG;
    if (g_fake.failBaseSha) return ESP_FAIL;
    memcpy(sha_256, g_fake.baseSha, 32);
    return ESP_OK;
}

const esp_partition_t *esp_ota_get_next_update_partition(const esp_partition_t *) {
    if (g_fake.noPartition) return nullptr;
    g_part.size = g_fake.partSize;
    g_part.erase_size = g_fake.sectorSize;
    return &g_part;
}

esp_err_t esp_ota_begin(const esp_partition_t *partition, size_t image_size, esp_ota_handle_t *out) {
    ++g_fake.beginCalls;
    // Before a handle exists, exactly like IDF's argument/state checks. Nothing to release.
    if (g_fake.failBegin) return ESP_FAIL;

    // IDF registers the operation and publishes the handle BEFORE erasing (esp_ota_ops.c:181),
    // then returns an erase error without unregistering it (:196-199). Model that order, or a
    // test cannot tell a leaked operation from a clean refusal.
    g_handleOpen = true;
    ++g_fake.liveOtaOps;
    *out = 1;
    if (g_fake.failBeginDuringErase) return ESP_FAIL;

    g_needErase = (image_size == OTA_WITH_SEQUENTIAL_WRITES);
    if (!g_needErase) {
        size_t eraseSize;
        if (image_size == 0 || image_size == OTA_SIZE_UNKNOWN) {
            eraseSize = partition->size;
        } else {
            eraseSize = ((image_size + partition->erase_size - 1) / partition->erase_size) *
                        partition->erase_size;
        }
        g_fake.eraseBytesInBegin += eraseSize;
        markErased(0, eraseSize > g_fake.partSize ? g_fake.partSize : eraseSize);
    }
    g_wroteSize = 0;
    return ESP_OK;
}

esp_err_t esp_ota_write(esp_ota_handle_t, const void *data, size_t size) {
    ++g_fake.writeCalls;
    if (g_fake.failWriteAtCall == g_fake.writeCalls) return ESP_FAIL;

    if (g_needErase) {
        size_t sec = g_part.erase_size;
        size_t first = g_wroteSize / sec;
        size_t last = (g_wroteSize + size - 1) / sec;
        size_t bytes = (g_wroteSize % sec == 0) ? ((last - first) + 1) * sec : (last - first) * sec;
        if (bytes) {
            ++g_fake.eraseCallsInWrite;
            g_fake.eraseBytesInWrite += bytes;
            markErased(first * sec, ((last - first) + 1) * sec);
        }
    } else {
        // need_erase == false means SOMEBODY ELSE promised these sectors were erased. Check it.
        for (size_t o = g_wroteSize; o < g_wroteSize + size; o += g_fake.sectorSize) {
            const size_t i = o / g_fake.sectorSize;
            if (i >= g_fake.erasedSectors.size() || !g_fake.erasedSectors[i]) {
                g_fake.wroteUnerased = true;
                break;
            }
        }
    }
    const uint8_t *p = (const uint8_t *) data;
    g_fake.flashed.insert(g_fake.flashed.end(), p, p + size);
    g_wroteSize += size;
    return ESP_OK;
}

esp_err_t esp_ota_end(esp_ota_handle_t) {
    g_handleOpen = false;
    --g_fake.liveOtaOps;
    g_fake.ended = true;
    return g_fake.failEnd ? ESP_ERR_OTA_VALIDATE_FAILED : ESP_OK;
}

esp_err_t esp_ota_abort(esp_ota_handle_t) {
    g_handleOpen = false;
    --g_fake.liveOtaOps;
    g_fake.aborted = true;
    return ESP_OK;
}

esp_err_t esp_ota_set_boot_partition(const esp_partition_t *partition) {
    if (g_fake.failSetBoot) return ESP_FAIL;
    g_fake.bootPart = partition;
    return ESP_OK;
}

void *heap_caps_malloc(size_t size, unsigned caps) {
    if ((caps & MALLOC_CAP_SPIRAM) && !g_fake.spiramAvailable) return nullptr;
    void *p = malloc(size);
    if (p) ++g_fake.liveAllocs;
    return p;
}

void heap_caps_free(void *p) {
    if (p) --g_fake.liveAllocs;
    free(p);
}

void esp_restart() {
    // The module falls back to this only when no restart hook is installed; the tests always install
    // one, so getting here means the hook wiring broke.
    fprintf(stderr, "FATAL: esp_restart() reached -- restart hook was not installed\n");
    abort();
}
