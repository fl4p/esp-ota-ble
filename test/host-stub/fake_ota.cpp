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
        case ESP_ERR_OTA_VALIDATE_FAILED: return "ESP_ERR_OTA_VALIDATE_FAILED";
        default: return "ESP_FAIL";
    }
}

const esp_partition_t *esp_ota_get_next_update_partition(const esp_partition_t *) {
    if (g_fake.noPartition) return nullptr;
    g_part.size = g_fake.partSize;
    g_part.erase_size = g_fake.sectorSize;
    return &g_part;
}

esp_err_t esp_ota_begin(const esp_partition_t *partition, size_t image_size, esp_ota_handle_t *out) {
    ++g_fake.beginCalls;
    if (g_fake.failBegin) return ESP_FAIL;

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
    }
    g_handleOpen = true;
    g_wroteSize = 0;
    *out = 1;
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
        }
    }
    const uint8_t *p = (const uint8_t *) data;
    g_fake.flashed.insert(g_fake.flashed.end(), p, p + size);
    g_wroteSize += size;
    return ESP_OK;
}

esp_err_t esp_ota_end(esp_ota_handle_t) {
    g_handleOpen = false;
    g_fake.ended = true;
    return g_fake.failEnd ? ESP_ERR_OTA_VALIDATE_FAILED : ESP_OK;
}

esp_err_t esp_ota_abort(esp_ota_handle_t) {
    g_handleOpen = false;
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
