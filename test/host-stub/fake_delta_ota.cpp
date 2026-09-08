// Backs test/host-stub/esp_delta_ota.h. See that header for why the "patch" is a passthrough.

#include <esp_delta_ota.h>

#include <cstdlib>

namespace {
struct FakeDelta {
    esp_delta_ota_cfg_t cfg;
    bool finalized = false;
};
} // namespace

esp_delta_ota_handle_t esp_delta_ota_init(esp_delta_ota_cfg_t *cfg) {
    if (!cfg || !cfg->read_cb) return nullptr;
    auto *h = new FakeDelta();
    h->cfg = *cfg;
    return h;
}

esp_err_t esp_delta_ota_feed_patch(esp_delta_ota_handle_t handle, const uint8_t *buf, int size) {
    if (!handle || size <= 0) return ESP_ERR_INVALID_ARG;
    auto *h = static_cast<FakeDelta *>(handle);
    return h->cfg.write_cb_with_user_data(buf, (size_t) size, h->cfg.user_data);
}

esp_err_t esp_delta_ota_finalize(esp_delta_ota_handle_t handle) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    static_cast<FakeDelta *>(handle)->finalized = true;
    return ESP_OK;
}

esp_err_t esp_delta_ota_deinit(esp_delta_ota_handle_t handle) {
    delete static_cast<FakeDelta *>(handle);
    return ESP_OK;
}
