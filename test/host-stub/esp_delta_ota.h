#pragma once

// Host stand-in for espressif/esp_delta_ota. It does NOT apply a bsdiff patch: it concatenates the
// patch bytes it is fed straight into the write callback. That is enough to test everything
// ota_xform.cpp is responsible for -- stripping and validating the 64-byte container header,
// routing the remainder to the encoder, and propagating a sink failure -- without pulling detools
// into a host build. Real patch application is exercised on hardware.

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef void *esp_delta_ota_handle_t;

typedef esp_err_t (*src_read_cb_t)(uint8_t *buf_p, size_t size, int src_offset);
typedef esp_err_t (*merged_stream_write_cb_t)(const uint8_t *buf_p, size_t size);
typedef esp_err_t (*merged_stream_write_cb_with_user_ctx_t)(const uint8_t *buf_p, size_t size, void *user_data);

typedef struct esp_delta_ota_cfg {
    void *user_data;
    src_read_cb_t read_cb;
    union {
        merged_stream_write_cb_with_user_ctx_t write_cb_with_user_data;
        merged_stream_write_cb_t write_cb;
    };
} esp_delta_ota_cfg_t;

esp_delta_ota_handle_t esp_delta_ota_init(esp_delta_ota_cfg_t *cfg);
esp_err_t esp_delta_ota_feed_patch(esp_delta_ota_handle_t handle, const uint8_t *buf, int size);
esp_err_t esp_delta_ota_finalize(esp_delta_ota_handle_t handle);
esp_err_t esp_delta_ota_deinit(esp_delta_ota_handle_t handle);
