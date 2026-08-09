#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    const char *label;
    size_t size;
    size_t erase_size;
} esp_partition_t;
