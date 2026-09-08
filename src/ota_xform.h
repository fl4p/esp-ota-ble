#pragma once

#include <cstddef>
#include <cstdint>

#include <esp_err.h>

// Wire-payload transforms for the OTAB protocol.
//
// The link is the bottleneck and it is saturated: measured on flu 2026-09-08, 384 of a push's
// stalls totalled 28.7 s against 0.47 s spent inside the host's write call, and 4096 B per 75 ms
// credit stall is 55 kB/s -- exactly the air time for one credit window. Neither a bigger window
// (8 KB against a ~1 KB bandwidth-delay product), a bigger MTU (247 already fills one LL PDU) nor
// the 2M PHY (measured, no change) moves it. The only remaining lever is sending fewer bytes.
//
// So the wire payload stops being the image. A transform reconstructs the image on the device from
// a smaller payload, and everything downstream of it -- the ring, credit accounting, the streaming
// SHA-256 -- keeps working on wire bytes exactly as before. Measured patch/payload sizes for a
// 1.76 MB fugu image (2026-09-08):
//
//   raw     100 %
//   tamp     69 %   window=12; self-contained, needs nothing from the device
//   delta   4.4-10.3 % over six real build pairs, median ~9 %   needs the device's exact base image
//
// Both are optional at compile time (__has_include): the IDF build declares them in
// idf_component.yml and always has them, while the PlatformIO/Arduino consumer builds from
// library.json and gets neither. otaXformAvailable() reports what this build can actually accept,
// so a host falls back to raw instead of failing mid-transfer.
enum class OtaXform : uint8_t { Raw, Tamp, Delta };

/// Byte sinks/sources the transform drives. `ctx` is passed back to both.
struct OtaXformIo {
    /// Required. Takes reconstructed IMAGE bytes (esp_ota_write in production).
    esp_err_t (*write)(const uint8_t *data, size_t len, void *ctx) = nullptr;
    /// Required for Delta only: read the base image (the running partition).
    esp_err_t (*read)(uint8_t *buf, size_t len, uint32_t offset, void *ctx) = nullptr;
    /// Required for Delta only: 32 bytes, the SHA-256 the base image carries as its own validation
    /// hash (esp_partition_get_sha256 of the running partition). The patch header names the base it
    /// was built against; a mismatch is rejected before a single byte reaches flash, because a patch
    /// applied to the wrong base produces a plausible-looking image that fails only at boot.
    const uint8_t *baseDigest = nullptr;
    void *ctx = nullptr;
};

/// Parse a wire transform name ("raw" | "tamp" | "delta"). False on an unknown name.
bool otaXformParse(const char *name, OtaXform *out);
const char *otaXformName(OtaXform x);

/// True when this build carries the dependency that transform needs. Raw is always available.
bool otaXformAvailable(OtaXform x);

/// Allocate and start a transform. `outSize` is the reconstructed image size (Raw ignores it).
/// On false, `*err` names the reason for the protocol line and nothing was allocated.
bool otaXformBegin(OtaXform x, uint32_t outSize, const OtaXformIo &io, const char **err);

/// Feed wire bytes. Calls io.write zero or more times with reconstructed image bytes.
esp_err_t otaXformFeed(const uint8_t *data, size_t len);

/// Flush any transform state at end of stream. Must be called before checking the written length.
esp_err_t otaXformFinish();

/// Release everything. Idempotent, and safe to call without a matching Begin.
void otaXformEnd();
