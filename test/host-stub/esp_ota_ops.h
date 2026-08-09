#pragma once

#include "esp_partition.h"

// Values and semantics mirror ESP-IDF's app_update component. The distinction between these two is
// the whole reason this shim exists -- see fake_ota.cpp.
#define OTA_SIZE_UNKNOWN 0xffffffff           /* erases the ENTIRE partition inside esp_ota_begin */
#define OTA_WITH_SEQUENTIAL_WRITES 0xfffffffe /* defers erasing to per-sector calls in esp_ota_write */

typedef uint32_t esp_ota_handle_t;

const esp_partition_t *esp_ota_get_next_update_partition(const esp_partition_t *start_from);
esp_err_t esp_ota_begin(const esp_partition_t *partition, size_t image_size, esp_ota_handle_t *out_handle);
esp_err_t esp_ota_write(esp_ota_handle_t handle, const void *data, size_t size);
esp_err_t esp_ota_end(esp_ota_handle_t handle);
esp_err_t esp_ota_abort(esp_ota_handle_t handle);
esp_err_t esp_ota_set_boot_partition(const esp_partition_t *partition);
