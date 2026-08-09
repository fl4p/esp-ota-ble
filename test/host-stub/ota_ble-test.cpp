// Host-native tests for the OTA receiver. No hardware, no flash: clang++ against the shims in this
// directory. Every failure case asserts the OBSERVABLE outcome -- above all that the boot partition
// did not move -- rather than merely that a function returned false.

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <mbedtls/sha256.h>

#include "fake_ota.h"
#include "ota_ble.h"

// ---------------------------------------------------------------- harness

static int g_failures = 0;
static const char *g_case = "";

#define CHECK(cond, ...)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            ++g_failures;                                                    \
            printf("  FAIL [%s] %s:%d: ", g_case, __FILE__, __LINE__);       \
            printf(__VA_ARGS__);                                             \
            printf("\n");                                                    \
        }                                                                    \
    } while (0)

struct Status {
    OtaBleLevel level;
    std::string line;
};

static std::vector<Status> g_status;
static std::vector<bool> g_quiesce;
static bool g_restarted = false;

static void onStatus(OtaBleLevel level, const char *line) { g_status.push_back({level, line}); }
static void onQuiesce(bool halt) { g_quiesce.push_back(halt); }
static void onRestart() { g_restarted = true; }

static bool sawLine(const char *needle) {
    for (auto &s : g_status)
        if (s.line.find(needle) != std::string::npos) return true;
    return false;
}

static const Status *findLine(const char *needle) {
    for (auto &s : g_status)
        if (s.line.find(needle) != std::string::npos) return &s;
    return nullptr;
}

static void begin_case(const char *name) {
    g_case = name;
    fakeOtaReset();
    g_status.clear();
    g_quiesce.clear();
    g_restarted = false;
    OtaBleHooks h;
    h.status = onStatus;
    h.quiesce = onQuiesce;
    h.restart = onRestart;
    otaBleInit(h);
}

/// Invariants that must hold at the end of every case, whatever it was testing.
static void end_case() {
    CHECK(!otaBleActive(), "session still active at end of case");
    CHECK(g_fake.liveAllocs == 0, "staging ring leaked (%d live allocs)", g_fake.liveAllocs);
    // Any FAIL the module reports must be visible at a severity a log filter will not swallow.
    for (auto &s : g_status)
        if (s.line.find("OTAB FAIL") != std::string::npos)
            CHECK(s.level != OtaBleLevel::Info, "FAIL line emitted at Info: %s", s.line.c_str());
    // quiesce must always be released again if it was ever asserted.
    int halts = 0, releases = 0;
    for (bool q : g_quiesce) (q ? halts : releases)++;
    CHECK(halts == releases || g_restarted, "quiesce unbalanced (%d halt / %d release)", halts, releases);
}

// ---------------------------------------------------------------- helpers

static std::vector<uint8_t> makeImage(size_t n) {
    std::vector<uint8_t> v(n);
    for (size_t i = 0; i < n; ++i) v[i] = (uint8_t) (i * 31 + (i >> 8) * 7);
    return v;
}

static std::string sha256hex(const std::vector<uint8_t> &v) {
    mbedtls_sha256_context c;
    uint8_t d[32];
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, v.data(), v.size());
    mbedtls_sha256_finish(&c, d);
    mbedtls_sha256_free(&c);
    char out[65];
    for (int i = 0; i < 32; ++i) snprintf(out + i * 2, 3, "%02x", d[i]);
    return out;
}

static std::string beginCmd(const std::vector<uint8_t> &img) {
    return "begin " + std::to_string(img.size()) + " " + sha256hex(img);
}

/// Stage the image in `chunk`-sized writes, ticking every `drainEvery` chunks. Small drainEvery
/// mimics a fast consumer; large values force the ring to fill and wrap.
static void pushAll(const std::vector<uint8_t> &img, size_t chunk = 512, int drainEvery = 1) {
    size_t sent = 0;
    int since = 0;
    while (sent < img.size()) {
        size_t n = img.size() - sent < chunk ? img.size() - sent : chunk;
        otaBleStageBytes(img.data() + sent, n);
        sent += n;
        if (++since >= drainEvery) {
            otaBleTick(0);
            since = 0;
        }
    }
    otaBleTick(0);
}

// ---------------------------------------------------------------- cases

static void test_happy_path() {
    begin_case("happy path");
    auto img = makeImage(20000);
    CHECK(otaBleSubmitCommand(beginCmd(img).c_str()) == OtaBleSubmit::Accepted, "begin rejected");
    otaBleTick(0);
    CHECK(otaBleActive(), "not active after begin");
    CHECK(sawLine("OTAB READY"), "no READY");
    pushAll(img);
    CHECK(otaBleSubmitCommand("end") == OtaBleSubmit::Accepted, "end rejected");
    otaBleTick(0);
    CHECK(g_fake.flashed == img, "flashed bytes differ from source image");
    CHECK(g_fake.ended, "esp_ota_end not called");
    CHECK(g_fake.bootPart != nullptr, "boot partition not set");
    CHECK(g_restarted, "did not restart");
    CHECK(sawLine("OTAB OK"), "no OK");
    CHECK(!sawLine("OTAB FAIL"), "unexpected FAIL");
    end_case();
}

static void test_erase_granularity() {
    // The point of OTA_WITH_SEQUENTIAL_WRITES: nothing may be erased inside begin, or the call blocks
    // for seconds on a real 1.7 MB slot and trips a panic-on-timeout task watchdog.
    begin_case("erase granularity");
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    CHECK(g_fake.eraseBytesInBegin == 0,
          "esp_ota_begin erased %zu bytes synchronously; expected none", g_fake.eraseBytesInBegin);
    pushAll(img);
    CHECK(g_fake.eraseCallsInWrite > 1, "expected incremental erases during writes, got %d",
          g_fake.eraseCallsInWrite);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    end_case();
}

static void test_truncated() {
    begin_case("truncated image");
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    std::vector<uint8_t> half(img.begin(), img.begin() + 9000);
    pushAll(half);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL incomplete"), "no incomplete FAIL");
    CHECK(g_fake.bootPart == nullptr, "boot partition MOVED on a truncated image");
    CHECK(g_fake.aborted, "esp_ota_abort not called");
    CHECK(!g_restarted, "restarted despite failure");
    end_case();
}

static void test_sha_mismatch() {
    begin_case("digest mismatch");
    auto img = makeImage(20000);
    std::string cmd = beginCmd(img);
    otaBleSubmitCommand(cmd.c_str());
    otaBleTick(0);
    auto corrupted = img;
    corrupted[10000] ^= 0xff; // same length, different content
    pushAll(corrupted);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL sha-mismatch"), "no sha-mismatch FAIL");
    CHECK(g_fake.bootPart == nullptr, "boot partition MOVED on a corrupted image");
    CHECK(!g_restarted, "restarted despite failure");
    end_case();
}

static void test_oversized() {
    begin_case("oversized image");
    std::string cmd = "begin " + std::to_string(g_fake.partSize + 1) + " " + sha256hex(makeImage(4));
    CHECK(otaBleSubmitCommand(cmd.c_str()) == OtaBleSubmit::Rejected,
          "oversized image was not rejected synchronously");
    CHECK(g_fake.beginCalls == 0, "esp_ota_begin called for an oversized image");
    CHECK(g_fake.eraseBytesInBegin == 0 && g_fake.eraseCallsInWrite == 0, "erased for an oversized image");
    otaBleTick(0);
    CHECK(!otaBleActive(), "became active after a rejected begin");
    end_case();
}

static void test_malformed_commands() {
    begin_case("malformed commands");
    const char *bad[] = {
        "",                                       // empty
        "bogus",                                  // unknown verb
        "begin",                                  // no args
        "begin 100",                              // no digest
        ("begin 0 00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff"), // zero size
        "begin 100 deadbeef",                     // digest too short
        "begin 100 zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz", // non-hex
        "beginning 100 00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff", // prefix
    };
    for (const char *b : bad) {
        CHECK(otaBleSubmitCommand(b) == OtaBleSubmit::Rejected, "accepted malformed command: '%s'", b);
    }
    CHECK(otaBleSubmitCommand(nullptr) == OtaBleSubmit::Rejected, "accepted a null command");
    CHECK(g_fake.beginCalls == 0, "a malformed command reached esp_ota_begin");
    otaBleTick(0);
    CHECK(!otaBleActive(), "a malformed command started a session");
    end_case();
}

static void test_credit_overrun() {
    begin_case("credit overrun");
    auto img = makeImage(40000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    // Stream far past the credit window without ever letting the consumer drain.
    for (size_t sent = 0; sent < img.size(); sent += 1024)
        otaBleStageBytes(img.data() + sent, 1024);
    otaBleTick(0);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL"), "overrun did not fail");
    CHECK(g_fake.bootPart == nullptr, "boot partition MOVED after a credit overrun");
    end_case();
}

static void test_abort_mid_transfer() {
    begin_case("abort mid-transfer");
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    std::vector<uint8_t> part(img.begin(), img.begin() + 6000);
    pushAll(part);
    otaBleSubmitCommand("abort");
    otaBleTick(0);
    CHECK(!otaBleActive(), "still active after abort");
    CHECK(g_fake.aborted, "esp_ota_abort not called");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved on abort");
    CHECK(g_quiesce.size() >= 2 && g_quiesce.back() == false, "sampling not released on abort");

    // A fresh session must work immediately afterwards, with no reboot in between.
    g_status.clear();
    auto img2 = makeImage(5000);
    CHECK(otaBleSubmitCommand(beginCmd(img2).c_str()) == OtaBleSubmit::Accepted, "re-begin rejected");
    otaBleTick(0);
    CHECK(otaBleActive(), "re-begin did not take");
    pushAll(img2);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.bootPart != nullptr, "second session did not complete");
    end_case();
}

static void test_stale_abort_latch() {
    // Regression: consumers wire otaBleRequestAbort() to BLE disconnect, which fires for plain
    // telemetry clients too. If the request latched while inactive, nothing would ever clear it and
    // the NEXT transfer would die immediately after READY -- and stay dead until reboot.
    begin_case("abort requested while inactive");
    otaBleRequestAbort();
    otaBleRequestAbort();
    otaBleTick(0);

    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    CHECK(otaBleActive(), "begin was killed by a stale abort request");
    pushAll(img);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.bootPart != nullptr, "transfer aborted by a stale abort latch");
    CHECK(!sawLine("OTAB FAIL aborted"), "stale abort fired");
    end_case();
}

static void test_begin_while_active() {
    begin_case("begin while already active");
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    pushAll(std::vector<uint8_t>(img.begin(), img.begin() + 4000));
    size_t writtenBefore = g_fake.flashed.size();

    otaBleBegin(img.size(), sha256hex(img).c_str()); // direct, bypassing the latch
    CHECK(sawLine("OTAB FAIL already-active"), "no already-active FAIL");
    CHECK(otaBleActive(), "the in-flight session was torn down");
    CHECK(g_fake.flashed.size() == writtenBefore, "the in-flight session lost data");

    otaBleSubmitCommand("abort");
    otaBleTick(0);
    end_case();
}

static void test_latch_collision() {
    begin_case("second command before the tick");
    auto img = makeImage(5000);
    CHECK(otaBleSubmitCommand(beginCmd(img).c_str()) == OtaBleSubmit::Accepted, "first rejected");
    CHECK(otaBleSubmitCommand("abort") == OtaBleSubmit::Rejected,
          "a second command was accepted while one was still latched");
    otaBleTick(0);
    CHECK(otaBleActive(), "the first command did not execute");
    pushAll(img);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.bootPart != nullptr, "first command's session did not complete");
    end_case();
}

static void test_end_without_session() {
    begin_case("end with no session");
    CHECK(otaBleSubmitCommand("end") == OtaBleSubmit::Accepted, "end not latched");
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL not-active"), "no not-active FAIL");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved");
    end_case();
}

static void test_write_failure() {
    begin_case("esp_ota_write fails mid-stream");
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    g_fake.failWriteAtCall = 2;
    pushAll(img);
    CHECK(sawLine("OTAB FAIL esp_ota_write"), "no write FAIL");
    CHECK(!otaBleActive(), "still active after a write failure");
    CHECK(g_fake.aborted, "esp_ota_abort not called");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved after a write failure");
    CHECK(g_quiesce.back() == false, "sampling not released after a write failure");
    end_case();
}

static void test_end_failure() {
    begin_case("esp_ota_end fails");
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    pushAll(img);
    g_fake.failEnd = true;
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL esp_ota_end"), "no end FAIL");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved despite esp_ota_end failing");
    CHECK(!g_restarted, "restarted despite esp_ota_end failing");
    end_case();
}

static void test_set_boot_failure() {
    begin_case("esp_ota_set_boot_partition fails");
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    pushAll(img);
    g_fake.failSetBoot = true;
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL set-boot"), "no set-boot FAIL");
    CHECK(g_fake.bootPart == nullptr, "boot partition set despite the call failing");
    CHECK(!g_restarted, "restarted despite set-boot failing");
    end_case();
}

static void test_no_partition() {
    begin_case("no OTA partition");
    g_fake.noPartition = true;
    auto img = makeImage(5000);
    CHECK(otaBleSubmitCommand(beginCmd(img).c_str()) == OtaBleSubmit::Rejected,
          "begin accepted with no OTA partition");
    CHECK(!otaBleBegin(img.size(), sha256hex(img).c_str()), "direct begin succeeded with no partition");
    CHECK(sawLine("OTAB FAIL no-partition"), "no no-partition FAIL");
    otaBleTick(0);
    end_case();
}

static void test_ring_wrap() {
    // The ring's wrap arithmetic lives in both producer and consumer and never runs off-hardware
    // otherwise. Chunk and drain sizes are chosen so writes straddle the 8 KB boundary repeatedly.
    begin_case("ring wrap byte-exactness");
    auto img = makeImage(60000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    pushAll(img, 3000, 2); // 6000 B staged between drains, into an 8192 B ring
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.flashed == img, "bytes were reordered or lost across a ring wrap");
    CHECK(g_fake.bootPart != nullptr, "transfer did not complete");
    end_case();
}

static void test_progress_and_credit() {
    begin_case("progress and credit lines");
    auto img = makeImage(200000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    CHECK(sawLine("OTAB CRED"), "no initial credit grant");
    pushAll(img, 512, 4);
    const Status *prog = nullptr;
    std::string want = "OTAB PROG " + std::to_string(img.size()) + "/" + std::to_string(img.size());
    prog = findLine(want.c_str());
    CHECK(prog != nullptr, "no final PROG line (host waits for it before sending end)");
    otaBleSubmitCommand("end");
    otaBleTick(0);
    end_case();
}

int main() {
    printf("ota_ble host tests\n");
    test_happy_path();
    test_erase_granularity();
    test_truncated();
    test_sha_mismatch();
    test_oversized();
    test_malformed_commands();
    test_credit_overrun();
    test_abort_mid_transfer();
    test_stale_abort_latch();
    test_begin_while_active();
    test_latch_collision();
    test_end_without_session();
    test_write_failure();
    test_end_failure();
    test_set_boot_failure();
    test_no_partition();
    test_ring_wrap();
    test_progress_and_credit();

    if (g_failures) {
        printf("\n%d check(s) FAILED\n", g_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
