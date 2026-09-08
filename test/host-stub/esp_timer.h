#pragma once
// Host stub: monotonic microseconds. The receiver uses this only to time the flash
// drain for the OTAB STAT line, never for control flow, so a plain clock is enough.
#include <stdint.h>
#include <time.h>

static inline int64_t esp_timer_get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t) ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}
