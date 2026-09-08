#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    const char *label;
    size_t size;
    size_t erase_size;
} esp_partition_t;

// The base image a delta patches from. fake_ota.cpp backs these with g_fake.base.
esp_err_t esp_partition_read(const esp_partition_t *partition, size_t src_offset, void *dst, size_t size);
esp_err_t esp_partition_get_sha256(const esp_partition_t *partition, uint8_t *sha_256);
