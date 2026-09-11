#include "ota_xform.h"

#include <algorithm>
#include <cstring>

#include <esp_heap_caps.h>

// tamp ships under two include layouts and a consumer may have either: the registry component
// (brianpugh/tamp) exports its root, so the header is <tamp/decompressor.h>, while a vendored copy
// of the upstream C sources exports the tamp/ directory itself and the header is <decompressor.h>.
// fugu has the second. Try both, then confirm from the header's OWN include guard rather than from
// which spelling resolved -- <decompressor.h> is a generic enough name to belong to something else,
// and "the include worked" is not the same claim as "this is tamp".
#if __has_include(<tamp/decompressor.h>)
#include <tamp/decompressor.h>
#elif __has_include(<decompressor.h>)
#include <decompressor.h>
#endif
#ifdef TAMP_DECOMPRESSOR_H
#define OTA_XFORM_HAVE_TAMP 1
#endif

#if __has_include(<esp_delta_ota.h>)
#include <esp_delta_ota.h>
#define OTA_XFORM_HAVE_DELTA 1
#endif

// Tamp window, in bits. The host MUST compress with the same value: the decompressor validates the
// stream header against the buffer it was given and refuses a larger window outright.
// 12 is not a compromise, it is the measured optimum for this image -- 1.76 MB of fugu firmware
// compresses to 70.3 % at w=10, 69.0 % at w=12, 69.3 % at w=13, 70.1 % at w=14 and 71.5 % at w=15.
// A bigger window costs RAM and makes the output WORSE, so there is nothing to tune here.
static constexpr uint8_t TAMP_WINDOW_BITS = 12;
static constexpr size_t TAMP_WINDOW_SIZE = 1u << TAMP_WINDOW_BITS;
// Reconstructed bytes are handed straight to the sink, so this only bounds how often we call it.
static constexpr size_t XFORM_OUT_SLICE = 512;

// Patch container written by tools/esp_delta_ota_patch.py: a 4-byte magic, the base image's
// SHA-256, then reserved padding to 64 bytes, followed by the raw detools/heatshrink patch.
// Magic is the first four bytes of sha256("esp_delta_ota"), matching Espressif's generator.
static constexpr uint32_t DELTA_MAGIC = 0xfccdde10;
static constexpr size_t DELTA_HDR_SIZE = 64;
static constexpr size_t DELTA_DIGEST_OFF = 4;

static OtaXform current = OtaXform::Raw;
static bool started = false;
static OtaXformIo xio;

#if OTA_XFORM_HAVE_TAMP
static TampDecompressor tampDec;
static uint8_t *tampWindow = nullptr;
#endif

#if OTA_XFORM_HAVE_DELTA
static esp_delta_ota_handle_t deltaHandle = nullptr;
static uint8_t deltaHdr[DELTA_HDR_SIZE];
static size_t deltaHdrRead = 0;
#endif

bool otaXformParse(const char *name, OtaXform *out) {
    if (!name || !out) return false;
    if (strcmp(name, "raw") == 0) { *out = OtaXform::Raw; return true; }
    if (strcmp(name, "tamp") == 0) { *out = OtaXform::Tamp; return true; }
    if (strcmp(name, "delta") == 0) { *out = OtaXform::Delta; return true; }
    return false;
}

const char *otaXformName(OtaXform x) {
    switch (x) {
        case OtaXform::Tamp:  return "tamp";
        case OtaXform::Delta: return "delta";
        case OtaXform::Raw:   break;
    }
    return "raw";
}

bool otaXformAvailable(OtaXform x) {
    switch (x) {
        case OtaXform::Raw: return true;
        case OtaXform::Tamp:
#if OTA_XFORM_HAVE_TAMP
            return true;
#else
            return false;
#endif
        case OtaXform::Delta:
#if OTA_XFORM_HAVE_DELTA
            return true;
#else
            return false;
#endif
    }
    return false;
}

#if OTA_XFORM_HAVE_DELTA
static esp_err_t deltaWriteCb(const uint8_t *buf, size_t size, void *user) {
    (void) user;
    return xio.write(buf, size, xio.ctx);
}

// FOUR arguments, and installed in read_cb_with_user_data. esp_delta_ota picks between the two
// union members on `user_data` being non-null (esp_delta_ota.c:64) -- and it does that for the READ
// callback as well as the write one. Installing the three-argument member while passing user_data
// meant the library called this through a four-argument pointer type: it happens to work on Xtensa,
// which ignores the extra argument, but it is undefined and would not survive LTO or another target.
static esp_err_t deltaReadCb(uint8_t *buf, size_t size, int srcOffset, void *user) {
    (void) user;
    if (srcOffset < 0) return ESP_ERR_INVALID_ARG;
    return xio.read(buf, size, (uint32_t) srcOffset, xio.ctx);
}
#endif

bool otaXformBegin(OtaXform x, uint32_t outSize, const OtaXformIo &io, const char **err) {
    auto fail = [&](const char *why) { if (err) *err = why; return false; };
    (void) outSize;

    otaXformEnd(); // a previous session that ended badly must not leak its buffers into this one
    if (!io.write) return fail("no-sink");
    if (!otaXformAvailable(x)) return fail("xform-unsupported");

    xio = io;

    switch (x) {
        case OtaXform::Raw:
            break;

        case OtaXform::Tamp: {
#if OTA_XFORM_HAVE_TAMP
            tampWindow = (uint8_t *) heap_caps_malloc(TAMP_WINDOW_SIZE, MALLOC_CAP_SPIRAM);
            if (!tampWindow) tampWindow = (uint8_t *) heap_caps_malloc(TAMP_WINDOW_SIZE, MALLOC_CAP_DEFAULT);
            if (!tampWindow) return fail("xform-no-mem");
            // conf = nullptr: read the window/literal configuration from the stream's own header,
            // validated against TAMP_WINDOW_BITS, so a host that used a different window is
            // rejected here rather than producing silent garbage.
            if (tamp_decompressor_init(&tampDec, nullptr, tampWindow, TAMP_WINDOW_BITS) != TAMP_OK) {
                heap_caps_free(tampWindow);
                tampWindow = nullptr;
                return fail("xform-init");
            }
#endif
            break;
        }

        case OtaXform::Delta: {
#if OTA_XFORM_HAVE_DELTA
            if (!io.read || !io.baseDigest) return fail("no-base");
            deltaHdrRead = 0;
            esp_delta_ota_cfg_t cfg = {};
            cfg.user_data = &xio; // non-null selects the with-user-data write callback union member
            cfg.read_cb_with_user_data = &deltaReadCb;
            cfg.write_cb_with_user_data = &deltaWriteCb;
            deltaHandle = esp_delta_ota_init(&cfg);
            if (!deltaHandle) return fail("xform-init");
#endif
            break;
        }
    }

    current = x;
    started = true;
    return true;
}

#if OTA_XFORM_HAVE_DELTA
/// Consume as much of `data` as the 64-byte patch header still needs, validating it once complete.
/// Returns bytes consumed, or -1 when the header is bad.
static int deltaTakeHeader(const uint8_t *data, size_t len) {
    if (deltaHdrRead >= DELTA_HDR_SIZE) return 0;
    const size_t want = std::min(len, DELTA_HDR_SIZE - deltaHdrRead);
    memcpy(deltaHdr + deltaHdrRead, data, want);
    deltaHdrRead += want;
    if (deltaHdrRead < DELTA_HDR_SIZE) return (int) want;

    uint32_t magic;
    memcpy(&magic, deltaHdr, sizeof(magic)); // deltaHdr is a uint8_t array; a cast would be unaligned
    if (magic != DELTA_MAGIC) return -1;
    if (memcmp(deltaHdr + DELTA_DIGEST_OFF, xio.baseDigest, 32) != 0) return -1;
    return (int) want;
}
#endif

esp_err_t otaXformFeed(const uint8_t *data, size_t len) {
    if (!started) return ESP_ERR_INVALID_STATE;
    if (!len) return ESP_OK;

    switch (current) {
        case OtaXform::Raw:
            return xio.write(data, len, xio.ctx);

        case OtaXform::Tamp: {
#if OTA_XFORM_HAVE_TAMP
            static uint8_t out[XFORM_OUT_SLICE];
            size_t consumed = 0;
            while (consumed < len) {
                size_t got = 0, took = 0;
                const tamp_res res = tamp_decompressor_decompress(
                        &tampDec, out, sizeof(out), &got, data + consumed, len - consumed, &took);
                if (res != TAMP_OK && res != TAMP_INPUT_EXHAUSTED && res != TAMP_OUTPUT_FULL) {
                    return ESP_ERR_INVALID_CRC;
                }
                if (got) {
                    const esp_err_t werr = xio.write(out, got, xio.ctx);
                    if (werr != ESP_OK) return werr;
                }
                consumed += took;
                // A cycle that neither consumed input nor produced output cannot make progress on
                // the next one either; without this the loop spins forever on a truncated stream.
                if (!took && !got) break;
            }
            return ESP_OK;
#else
            return ESP_ERR_NOT_SUPPORTED;
#endif
        }

        case OtaXform::Delta: {
#if OTA_XFORM_HAVE_DELTA
            const int used = deltaTakeHeader(data, len);
            if (used < 0) return ESP_ERR_INVALID_VERSION;
            if ((size_t) used == len) return ESP_OK; // still inside the header
            return esp_delta_ota_feed_patch(deltaHandle, data + used, (int) (len - used));
#else
            return ESP_ERR_NOT_SUPPORTED;
#endif
        }
    }
    return ESP_ERR_INVALID_STATE;
}

esp_err_t otaXformFinish() {
    if (!started) return ESP_ERR_INVALID_STATE;
    switch (current) {
        case OtaXform::Raw:
            return ESP_OK;
        case OtaXform::Tamp: {
#if OTA_XFORM_HAVE_TAMP
            // Drain what the decompressor is still holding. It can retain decoded output after
            // consuming ALL of its input -- a match that ran past the end of the output slice is
            // resumed on the next call (TampDecompressor::skip_bytes) -- so the feed loop, which
            // stops when its input is consumed, cannot be the last word. Mid-stream that costs
            // nothing because the next feed resumes it; on the FINAL feed the tail would simply be
            // lost, and the image then comes up short after the slot has already been erased.
            // Measured: 28 of 400 consecutive image sizes stranded a tail, e.g. 20205 bytes fed
            // 512 at a time lost exactly one byte.
            static uint8_t out[XFORM_OUT_SLICE];
            for (;;) {
                size_t got = 0;
                const tamp_res res = tamp_decompressor_decompress(
                        &tampDec, out, sizeof(out), &got, (const unsigned char *) "", 0, nullptr);
                if (res != TAMP_OK && res != TAMP_INPUT_EXHAUSTED && res != TAMP_OUTPUT_FULL) {
                    return ESP_ERR_INVALID_CRC;
                }
                if (!got) break; // nothing left to emit; more calls cannot make progress
                const esp_err_t werr = xio.write(out, got, xio.ctx);
                if (werr != ESP_OK) return werr;
            }
            return ESP_OK;
#else
            return ESP_ERR_NOT_SUPPORTED;
#endif
        }
        case OtaXform::Delta:
#if OTA_XFORM_HAVE_DELTA
            // A patch that never delivered its full header never delivered a patch either.
            if (deltaHdrRead < DELTA_HDR_SIZE) return ESP_ERR_INVALID_SIZE;
            return esp_delta_ota_finalize(deltaHandle);
#else
            return ESP_ERR_NOT_SUPPORTED;
#endif
    }
    return ESP_ERR_INVALID_STATE;
}

void otaXformEnd() {
#if OTA_XFORM_HAVE_TAMP
    if (tampWindow) { heap_caps_free(tampWindow); tampWindow = nullptr; }
#endif
#if OTA_XFORM_HAVE_DELTA
    if (deltaHandle) { esp_delta_ota_deinit(deltaHandle); deltaHandle = nullptr; }
    deltaHdrRead = 0;
#endif
    started = false;
    current = OtaXform::Raw;
}
