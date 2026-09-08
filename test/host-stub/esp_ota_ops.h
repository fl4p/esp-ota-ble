#pragma once

#include "esp_partition.h"

// Values and semantics mirror ESP-IDF's app_update component. The distinction between these two is
// the whole reason this shim exists -- see fake_ota.cpp.
#define OTA_SIZE_UNKNOWN 0xffffffff           /* erases the ENTIRE partition inside esp_ota_begin */
#define OTA_WITH_SEQUENTIAL_WRITES 0xfffffffe /* defers erasing to per-sector calls in esp_ota_write */

typedef uint32_t esp_ota_handle_t;

const esp_partition_t *esp_ota_get_next_update_partition(const esp_partition_t *start_from);
const esp_partition_t *esp_ota_get_running_partition(void);
esp_err_t esp_ota_begin(const esp_partition_t *partition, size_t image_size, esp_ota_handle_t *out_handle);
esp_err_t esp_ota_write(esp_ota_handle_t handle, const void *data, size_t size);
// Writes at an explicit slot offset instead of the handle's implicit cursor, and asserts the
// region was erased first. It is what makes skipping a sector possible: the writer can leave a
// hole in the sequence without the OTA layer losing track of where the next byte belongs.
esp_err_t esp_ota_write_with_offset(esp_ota_handle_t handle, const void *data, size_t size, uint32_t offset);
esp_err_t esp_ota_end(esp_ota_handle_t handle);
esp_err_t esp_ota_abort(esp_ota_handle_t handle);
esp_err_t esp_ota_set_boot_partition(const esp_partition_t *partition);
