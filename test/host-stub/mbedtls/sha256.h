#pragma once

#include <stddef.h>
#include <stdint.h>

// A REAL SHA-256, deliberately. A stub that returned a constant digest would make the
// sha-mismatch test pass no matter what the module did -- a check that cannot evaluate its
// input reporting "fine" is precisely the failure this suite exists to catch.

typedef struct {
    uint32_t state[8];
    uint64_t bitlen;
    uint8_t buf[64];
    size_t buflen;
} mbedtls_sha256_context;

void mbedtls_sha256_init(mbedtls_sha256_context *ctx);
void mbedtls_sha256_free(mbedtls_sha256_context *ctx);
int mbedtls_sha256_starts(mbedtls_sha256_context *ctx, int is224);
int mbedtls_sha256_update(mbedtls_sha256_context *ctx, const unsigned char *input, size_t ilen);
int mbedtls_sha256_finish(mbedtls_sha256_context *ctx, unsigned char output[32]);
