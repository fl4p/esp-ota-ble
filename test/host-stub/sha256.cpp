// Compact public-domain-style SHA-256, used only by the host tests.

#include <cstring>

#include <mbedtls/sha256.h>

static const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

static inline uint32_t ror(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

static void transform(mbedtls_sha256_context *c, const uint8_t d[64]) {
    uint32_t m[64];
    for (int i = 0; i < 16; ++i)
        m[i] = (uint32_t) d[i * 4] << 24 | (uint32_t) d[i * 4 + 1] << 16 |
               (uint32_t) d[i * 4 + 2] << 8 | (uint32_t) d[i * 4 + 3];
    for (int i = 16; i < 64; ++i) {
        uint32_t s0 = ror(m[i - 15], 7) ^ ror(m[i - 15], 18) ^ (m[i - 15] >> 3);
        uint32_t s1 = ror(m[i - 2], 17) ^ ror(m[i - 2], 19) ^ (m[i - 2] >> 10);
        m[i] = m[i - 16] + s0 + m[i - 7] + s1;
    }
    uint32_t a = c->state[0], b = c->state[1], cc = c->state[2], dd = c->state[3];
    uint32_t e = c->state[4], f = c->state[5], g = c->state[6], h = c->state[7];
    for (int i = 0; i < 64; ++i) {
        uint32_t S1 = ror(e, 6) ^ ror(e, 11) ^ ror(e, 25);
        uint32_t ch = (e & f) ^ (~e & g);
        uint32_t t1 = h + S1 + ch + K[i] + m[i];
        uint32_t S0 = ror(a, 2) ^ ror(a, 13) ^ ror(a, 22);
        uint32_t mj = (a & b) ^ (a & cc) ^ (b & cc);
        uint32_t t2 = S0 + mj;
        h = g; g = f; f = e; e = dd + t1;
        dd = cc; cc = b; b = a; a = t1 + t2;
    }
    c->state[0] += a; c->state[1] += b; c->state[2] += cc; c->state[3] += dd;
    c->state[4] += e; c->state[5] += f; c->state[6] += g; c->state[7] += h;
}

void mbedtls_sha256_init(mbedtls_sha256_context *ctx) { memset(ctx, 0, sizeof(*ctx)); }
void mbedtls_sha256_free(mbedtls_sha256_context *ctx) { memset(ctx, 0, sizeof(*ctx)); }

int mbedtls_sha256_starts(mbedtls_sha256_context *ctx, int is224) {
    (void) is224;
    ctx->bitlen = 0;
    ctx->buflen = 0;
    ctx->state[0] = 0x6a09e667; ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372; ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f; ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab; ctx->state[7] = 0x5be0cd19;
    return 0;
}

int mbedtls_sha256_update(mbedtls_sha256_context *ctx, const unsigned char *input, size_t ilen) {
    for (size_t i = 0; i < ilen; ++i) {
        ctx->buf[ctx->buflen++] = input[i];
        if (ctx->buflen == 64) {
            transform(ctx, ctx->buf);
            ctx->bitlen += 512;
            ctx->buflen = 0;
        }
    }
    return 0;
}

int mbedtls_sha256_finish(mbedtls_sha256_context *ctx, unsigned char output[32]) {
    size_t i = ctx->buflen;
    ctx->buf[i++] = 0x80;
    if (i > 56) {
        while (i < 64) ctx->buf[i++] = 0;
        transform(ctx, ctx->buf);
        i = 0;
    }
    while (i < 56) ctx->buf[i++] = 0;
    ctx->bitlen += (uint64_t) ctx->buflen * 8;
    for (int j = 0; j < 8; ++j) ctx->buf[56 + j] = (uint8_t) (ctx->bitlen >> (56 - j * 8));
    transform(ctx, ctx->buf);
    for (int j = 0; j < 8; ++j)
        for (int k = 0; k < 4; ++k) output[j * 4 + k] = (uint8_t) (ctx->state[j] >> (24 - k * 8));
    return 0;
}
