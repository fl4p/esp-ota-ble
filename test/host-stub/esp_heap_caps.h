#pragma once

#include <stddef.h>

#define MALLOC_CAP_DEFAULT (1 << 0)
#define MALLOC_CAP_SPIRAM (1 << 1)

void *heap_caps_malloc(size_t size, unsigned caps);
void heap_caps_free(void *p);
