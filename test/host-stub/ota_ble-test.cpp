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
#include "ota_xform.h"

#if __has_include(<tamp/compressor.h>)
#include <tamp/compressor.h>
#define TEST_HAVE_TAMP 1
#endif

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

static size_t countLines(const char *needle) {
    size_t n = 0;
    for (auto &s : g_status)
        if (s.line.find(needle) != std::string::npos) ++n;
    return n;
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
    CHECK(g_fake.liveOtaOps == 0, "leaked %d IDF OTA operation(s)", g_fake.liveOtaOps);
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

/// "begin" for a transformed payload: wire size/digest describe `wire`, `outSize` the image it
/// rebuilds to. Deliberately built from the same sha256hex() as the raw form -- if the digest ever
/// stopped covering exactly the bytes that go over the link, every one of these cases would break.
static std::string beginCmdX(const std::vector<uint8_t> &wire, const char *xform, size_t outSize) {
    return "begin " + std::to_string(wire.size()) + " " + sha256hex(wire) + " " + xform + " " +
           std::to_string(outSize);
}

#if TEST_HAVE_TAMP
/// Compress with the real tamp encoder, so the decompress loop under test faces genuine streams
/// (partial token consumption, output-full cycles) rather than a mock of its own behaviour.
static std::vector<uint8_t> tampCompress(const std::vector<uint8_t> &in, uint8_t windowBits) {
    std::vector<unsigned char> window((size_t) 1 << windowBits);
    TampConf conf = {};
    conf.window = windowBits;
    conf.literal = 8;
    conf.use_custom_dictionary = false;
    TampCompressor c;
    if (tamp_compressor_init(&c, &conf, window.data()) != TAMP_OK) return {};

    std::vector<uint8_t> out(in.size() * 2 + 64);
    size_t written = 0, consumed = 0;
    const tamp_res res = tamp_compressor_compress_and_flush(
            &c, out.data(), out.size(), &written, in.data(), in.size(), &consumed, false);
    if (res != TAMP_OK || consumed != in.size()) return {};
    out.resize(written);
    return out;
}
#endif

/// A delta patch container: 4-byte magic, the base digest, reserved to 64 bytes, then the patch.
/// The host-stub esp_delta_ota passes the patch through unchanged, so `patch` is what must reach
/// flash -- which makes "the header was stripped, exactly once, at exactly 64 bytes" observable.
static std::vector<uint8_t> deltaWire(const uint8_t digest[32], const std::vector<uint8_t> &patch) {
    std::vector<uint8_t> w(64, 0);
    const uint32_t magic = 0xfccdde10;
    memcpy(w.data(), &magic, 4);
    memcpy(w.data() + 4, digest, 32);
    w.insert(w.end(), patch.begin(), patch.end());
    return w;
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
    CHECK(sawLine("OTAB FAIL size"), "oversized rejection was not announced");
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
        g_status.clear();
        CHECK(otaBleSubmitCommand(b) == OtaBleSubmit::Rejected, "accepted malformed command: '%s'", b);
        // Announced, not just returned: the caller's error path may only reach a console, and a host
        // that hears nothing waits out its READY timeout instead of learning it sent garbage.
        CHECK(sawLine("OTAB FAIL"), "rejection of '%s' was not announced on the status channel", b);
    }
    g_status.clear();
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
    for (size_t sent = 0; sent < 9 * 1024; sent += 1024)
        otaBleStageBytes(img.data() + sent, 1024);
    CHECK(!sawLine("OTAB FAIL credit-overrun"), "producer reported an overrun synchronously");
    CHECK(g_fake.writeCalls == 0, "producer touched flash while latching an overrun");
    int writesBefore = g_fake.writeCalls;
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL credit-overrun"), "overrun was not reported by name");
    CHECK(!otaBleActive(), "overrun remained active after the consumer observed it");
    CHECK(g_fake.writeCalls == writesBefore, "consumer flushed bytes after an overrun was latched");
    CHECK(g_fake.aborted, "esp_ota_abort not called after a credit overrun");
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

static void test_abort_latched_begin() {
    // Exact disconnect race: Begin is published by the host task, but the consumer has not run it
    // yet. The abort belongs to that pending session, not to some future transfer.
    begin_case("abort a latched begin before the tick");
    auto img = makeImage(20000);
    CHECK(otaBleSubmitCommand(beginCmd(img).c_str()) == OtaBleSubmit::Accepted, "begin rejected");
    CHECK(!otaBleActive(), "latched begin reported active before it executed");
    otaBleRequestAbort();
    otaBleTick(1);
    CHECK(!otaBleActive(), "latched begin became active after its disconnect abort");
    CHECK(g_fake.beginCalls == 0, "esp_ota_begin ran after the pending begin was cancelled");
    CHECK(g_quiesce.empty(), "quiesce ran for a begin cancelled before execution");
    if (otaBleActive()) otaBleAbort(); // keep later cases independent if this regression returns
    end_case();
}

static void test_stall_timeout() {
    begin_case("no-byte stall timeout");
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(100);
    CHECK(otaBleActive(), "begin did not arm");
    otaBleTick(30099);
    CHECK(otaBleActive(), "stall timeout fired early");
    otaBleTick(30100);
    CHECK(!otaBleActive(), "stalled transfer stayed active");
    CHECK(sawLine("OTAB FAIL stalled"), "stall timeout did not report its cause");
    CHECK(g_fake.aborted, "esp_ota_abort not called for a stalled transfer");
    CHECK(g_quiesce.size() >= 2 && !g_quiesce.back(), "sampling not released after a stall");
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
    CHECK(sawLine("OTAB FAIL busy"), "latch collision was not announced");
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
    CHECK(sawLine("OTAB FAIL no-partition"), "no-partition rejection was not announced");
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

static void test_credit_repeat() {
    begin_case("credit re-announcement");
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(100);
    otaBleStageBytes(img.data(), 4096);
    otaBleTick(200);
    CHECK(countLines("OTAB CRED 12288") == 1, "drained credit was not announced exactly once");
    otaBleTick(5199);
    CHECK(countLines("OTAB CRED 12288") == 1, "credit repeated before five seconds");
    otaBleTick(5200);
    CHECK(countLines("OTAB CRED 12288") == 2, "drained credit was not re-announced after five seconds");
    otaBleSubmitCommand("abort");
    otaBleTick(5201);
    end_case();
}

// ------------------------------------------------- payload transforms

static void test_raw_xform_is_the_old_protocol() {
    // The four-argument form naming "raw" must be indistinguishable from the two-argument form.
    begin_case("explicit raw xform");
    auto img = makeImage(20000);
    CHECK(otaBleSubmitCommand(beginCmdX(img, "raw", img.size()).c_str()) == OtaBleSubmit::Accepted,
          "explicit raw begin rejected");
    otaBleTick(0);
    pushAll(img);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.flashed == img, "explicit raw did not flash the image verbatim");
    CHECK(g_restarted, "did not restart");
    end_case();
}

static void test_xform_unknown_and_unavailable() {
    begin_case("unknown xform rejected");
    auto img = makeImage(1000);
    CHECK(otaBleSubmitCommand(beginCmdX(img, "brotli", img.size()).c_str()) == OtaBleSubmit::Rejected,
          "unknown transform accepted");
    CHECK(sawLine("bad-xform"), "no bad-xform diagnosis");
    // Rejected at submit, so nothing was latched and nothing quiesced the consumer's sampling.
    otaBleTick(0);
    CHECK(!otaBleActive(), "unknown transform started a session");
    CHECK(g_fake.beginCalls == 0, "unknown transform reached esp_ota_begin");
    CHECK(g_quiesce.empty(), "unknown transform halted sampling");
    end_case();
}

static void test_xform_wire_may_exceed_the_image() {
    // Deliberately allowed. A transform that expands is pointless but not unsafe, and tamp does
    // expand incompressible input -- rejecting wire > image here would refuse a legitimate push.
    // The guard that matters is the reconstructed length, which this case also exercises: the
    // payload is 64 bytes longer than the image it claims, so the session must still fail.
    begin_case("wire may exceed the image");
    auto img = makeImage(20000);
    auto wire = deltaWire(g_fake.baseSha, img); // 64 bytes of container on top of a 20000-byte body
    CHECK(otaBleSubmitCommand(beginCmdX(wire, "delta", img.size() - 64).c_str()) == OtaBleSubmit::Accepted,
          "an expanding payload was rejected up front");
    otaBleTick(0);
    pushAll(wire);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL out"), "over-long reconstruction was not caught by the length check");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved on a wrong-length image");
    end_case();
}

static void test_info_reports_base_and_transforms() {
    begin_case("info");
    for (int i = 0; i < 32; ++i) g_fake.baseSha[i] = (uint8_t) (0xA0 + i);
    CHECK(otaBleSubmitCommand("info") == OtaBleSubmit::Accepted, "info rejected");
    otaBleTick(0);
    const Status *base = findLine("OTAB BASE ");
    CHECK(base != nullptr, "no OTAB BASE line");
    if (base) CHECK(base->line.find("a0a1a2a3") != std::string::npos,
                    "BASE does not carry the running digest: %s", base->line.c_str());
    CHECK(sawLine("OTAB INFO run=app0"), "INFO did not name the RUNNING slot");
    const Status *x = findLine("OTAB XFORM ");
    CHECK(x != nullptr, "no OTAB XFORM line");
    if (x) {
        CHECK(x->line.find("raw") != std::string::npos, "XFORM omits raw");
        CHECK((x->line.find("delta") != std::string::npos) == otaXformAvailable(OtaXform::Delta),
              "XFORM disagrees with otaXformAvailable: %s", x->line.c_str());
    }
    // Every reply must be readable on a 20-byte-payload link, or a default-MTU host sees fragments.
    for (auto &st : g_status) CHECK(st.line.size() < 80, "status line too long: %s", st.line.c_str());
    end_case();
}

static void test_info_when_the_base_cannot_be_hashed() {
    // "no delta possible" and "the reply was lost" must not look the same to a host: one means fall
    // back to a full push, the other means retry.
    begin_case("info without a base digest");
    g_fake.failBaseSha = true;
    otaBleSubmitCommand("info");
    otaBleTick(0);
    CHECK(sawLine("OTAB BASE none"), "a failed base hash reported nothing");
    end_case();
}

#if TEST_HAVE_TAMP
static void test_tamp_round_trip() {
    begin_case("tamp round trip");
    auto img = makeImage(20000);
    auto wire = tampCompress(img, 12);
    CHECK(!wire.empty(), "tamp compression produced nothing");
    CHECK(wire.size() < img.size(), "tamp payload (%zu) not smaller than the image (%zu)",
          wire.size(), img.size());
    CHECK(otaBleSubmitCommand(beginCmdX(wire, "tamp", img.size()).c_str()) == OtaBleSubmit::Accepted,
          "tamp begin rejected");
    otaBleTick(0);
    CHECK(sawLine("xform=tamp"), "READY did not echo the transform");
    pushAll(wire);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.flashed == img, "tamp did not reconstruct the image (%zu of %zu bytes)",
          g_fake.flashed.size(), img.size());
    CHECK(g_fake.bootPart != nullptr, "boot partition not set");
    CHECK(!sawLine("OTAB FAIL"), "unexpected FAIL");
    end_case();
}

static void test_tamp_survives_tiny_chunks() {
    // The decompressor consumes input and produces output on independent schedules. Feeding it in
    // 3-byte slices forces the input-exhausted and output-full paths the 512-byte case never sees.
    begin_case("tamp in tiny chunks");
    auto img = makeImage(9000);
    auto wire = tampCompress(img, 12);
    otaBleSubmitCommand(beginCmdX(wire, "tamp", img.size()).c_str());
    otaBleTick(0);
    pushAll(wire, 3);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.flashed == img, "chunked tamp did not reconstruct the image");
    end_case();
}

static void test_tamp_chunk_size_sweep() {
    // The decompressor may keep decoded output buffered after consuming ALL of its input -- the
    // output slice fills partway through a match. A feed loop that stops when the input is
    // consumed loses that tail, and the image comes up short AFTER the slot has been erased.
    // Sweeping chunk sizes is how that lands on the exact boundary that triggers it.
    for (size_t chunk : {1u, 2u, 3u, 5u, 7u, 13u, 64u, 127u, 255u, 511u, 512u, 513u, 1024u, 2048u}) {
        begin_case("tamp chunk sweep");
        auto img = makeImage(30000);
        auto wire = tampCompress(img, 12);
        otaBleSubmitCommand(beginCmdX(wire, "tamp", img.size()).c_str());
        otaBleTick(0);
        pushAll(wire, chunk);
        otaBleSubmitCommand("end");
        otaBleTick(0);
        CHECK(g_fake.flashed == img, "chunk=%zu reconstructed %zu of %zu bytes",
              chunk, g_fake.flashed.size(), img.size());
        end_case();
    }
}

static void test_tamp_strands_no_tail() {
    // Known-bad calibration for a real defect: tamp can keep decoded output buffered after
    // consuming ALL of its input, because a match ran past the end of the output slice
    // (TampDecompressor::skip_bytes is exactly that resumption state). Only the LAST feed of a
    // stream can strand such a tail -- any earlier one is resumed by the next feed -- so it takes
    // a specific length to land on it. A 400-wide sweep of image sizes over the real encoder found
    // 28, of which this is one: 20205 bytes fed 512 at a time. Before otaXformFinish() drained the
    // decompressor this reconstructed short, and failed AFTER the slot had already been erased.
    begin_case("tamp strands no tail");
    std::vector<uint8_t> img(20205);
    for (size_t i = 0; i < img.size(); ++i) img[i] = (uint8_t) ((i / 311) % 5);
    auto wire = tampCompress(img, 12);
    otaBleSubmitCommand(beginCmdX(wire, "tamp", img.size()).c_str());
    otaBleTick(0);
    pushAll(wire, 512);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.flashed == img, "reconstructed %zu of %zu bytes -- a tail was stranded",
          g_fake.flashed.size(), img.size());
    CHECK(!sawLine("OTAB FAIL"), "a valid tamp stream was rejected");
    end_case();
}

static void test_tamp_window_too_large_is_refused() {
    // The device sizes its window buffer at build time; a host that compressed with a bigger one
    // must be refused rather than silently decoded against a too-small dictionary.
    begin_case("tamp window too large");
    auto img = makeImage(9000);
    auto wire = tampCompress(img, 14);
    CHECK(!wire.empty(), "w=14 compression produced nothing");
    otaBleSubmitCommand(beginCmdX(wire, "tamp", img.size()).c_str());
    otaBleTick(0);
    pushAll(wire);
    CHECK(sawLine("OTAB FAIL xform"), "an oversized tamp window was accepted");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved on a refused stream");
    otaBleTick(0);
    end_case();
}

static void test_tamp_truncated_fails_on_length() {
    begin_case("tamp truncated");
    auto img = makeImage(20000);
    auto wire = tampCompress(img, 12);
    wire.resize(wire.size() / 2);
    otaBleSubmitCommand(beginCmdX(wire, "tamp", img.size()).c_str());
    otaBleTick(0);
    pushAll(wire);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    // The wire arrived whole and hashed correctly; it simply does not rebuild a full image.
    CHECK(sawLine("OTAB FAIL out"), "a truncated tamp stream was not caught by the image length");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved on a short image");
    end_case();
}
#endif // TEST_HAVE_TAMP

static void test_delta_strips_its_header() {
    begin_case("delta header");
    for (int i = 0; i < 32; ++i) g_fake.baseSha[i] = (uint8_t) (i * 3 + 1);
    auto patch = makeImage(5000);
    auto wire = deltaWire(g_fake.baseSha, patch);
    CHECK(otaBleSubmitCommand(beginCmdX(wire, "delta", patch.size()).c_str()) == OtaBleSubmit::Accepted,
          "delta begin rejected");
    otaBleTick(0);
    pushAll(wire);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.flashed == patch, "delta wrote %zu bytes, expected the %zu-byte patch body",
          g_fake.flashed.size(), patch.size());
    CHECK(g_restarted, "did not restart");
    end_case();
}

static void test_delta_header_split_across_chunks() {
    // The 64-byte header arrives over many BLE writes. Feeding 7 bytes at a time puts a boundary
    // inside the magic, inside the digest, and on the header/body seam.
    begin_case("delta header split");
    for (int i = 0; i < 32; ++i) g_fake.baseSha[i] = (uint8_t) (i * 3 + 1);
    auto patch = makeImage(2000);
    auto wire = deltaWire(g_fake.baseSha, patch);
    otaBleSubmitCommand(beginCmdX(wire, "delta", patch.size()).c_str());
    otaBleTick(0);
    pushAll(wire, 7);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(g_fake.flashed == patch, "split header corrupted the patch body");
    end_case();
}

static void test_delta_wrong_base_is_refused() {
    // The whole risk of delta: a patch applied to the wrong base produces a plausible image that
    // fails only at boot. It has to be caught before anything is marked bootable.
    begin_case("delta wrong base");
    for (int i = 0; i < 32; ++i) g_fake.baseSha[i] = (uint8_t) (i * 3 + 1);
    uint8_t other[32];
    for (int i = 0; i < 32; ++i) other[i] = (uint8_t) (i * 3 + 2);
    auto wire = deltaWire(other, makeImage(5000));
    otaBleSubmitCommand(beginCmdX(wire, "delta", 5000).c_str());
    otaBleTick(0);
    pushAll(wire);
    CHECK(sawLine("OTAB FAIL xform"), "a patch against the wrong base was accepted");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved on a wrong-base patch");
    CHECK(g_fake.flashed.empty(), "wrong-base patch reached flash");
    otaBleTick(0);
    end_case();
}

static void test_delta_bad_magic_is_refused() {
    begin_case("delta bad magic");
    auto wire = deltaWire(g_fake.baseSha, makeImage(5000));
    wire[0] ^= 0xFF;
    otaBleSubmitCommand(beginCmdX(wire, "delta", 5000).c_str());
    otaBleTick(0);
    pushAll(wire);
    CHECK(sawLine("OTAB FAIL xform"), "a patch with a bad magic was accepted");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved");
    otaBleTick(0);
    end_case();
}

static void test_delta_header_only_is_refused() {
    // A payload that is all container and no patch must not look like a complete transfer.
    begin_case("delta header only");
    auto wire = deltaWire(g_fake.baseSha, {});
    otaBleSubmitCommand(beginCmdX(wire, "delta", 5000).c_str());
    otaBleTick(0);
    pushAll(wire);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL out"), "a header-only delta was not rejected");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved");
    end_case();
}

static void test_delta_truncated_header_is_refused() {
    begin_case("delta truncated header");
    auto wire = deltaWire(g_fake.baseSha, {});
    wire.resize(40);
    otaBleSubmitCommand(beginCmdX(wire, "delta", 5000).c_str());
    otaBleTick(0);
    pushAll(wire);
    otaBleSubmitCommand("end");
    otaBleTick(0);
    CHECK(sawLine("OTAB FAIL"), "a truncated container header was accepted");
    CHECK(g_fake.bootPart == nullptr, "boot partition moved");
    end_case();
}

static void test_begin_erase_failure_releases_the_operation() {
    // esp_ota_begin publishes the handle before it erases and does NOT unregister the operation
    // when the erase fails, so the caller owns the cleanup. Without the abort, repeated begins
    // accumulate live operations until IDF runs out of slots -- a slow leak that only shows up on
    // a board with a failing sector, which is exactly when an OTA needs to still work.
    begin_case("begin fails during erase");
    g_fake.failBeginDuringErase = true;
    auto img = makeImage(20000);
    otaBleSubmitCommand(beginCmd(img).c_str());
    otaBleTick(0);
    CHECK(!otaBleActive(), "session active after a failed begin");
    CHECK(sawLine("OTAB FAIL esp_ota_begin"), "no begin failure reported");
    CHECK(g_fake.liveOtaOps == 0, "leaked %d OTA operation(s) on a failed erase", g_fake.liveOtaOps);
    CHECK(g_fake.bootPart == nullptr, "boot partition moved");
    end_case();
}

static void test_begin_rejects_trailing_garbage() {
    begin_case("begin with trailing garbage");
    auto img = makeImage(20000);
    const std::string sha = sha256hex(img);
    const std::string n = std::to_string(img.size());
    // Accepted forms, unchanged.
    CHECK(otaBleSubmitCommand(("begin " + n + " " + sha).c_str()) == OtaBleSubmit::Accepted,
          "two-field begin rejected");
    otaBleTick(0);
    otaBleSubmitCommand("abort");
    otaBleTick(0);
    // A fifth token means the host sent something this receiver does not implement. Executing the
    // four fields it does understand and dropping the rest runs a command nobody asked for.
    CHECK(otaBleSubmitCommand(("begin " + n + " " + sha + " raw " + n + " EXTRA").c_str())
              == OtaBleSubmit::Rejected, "begin with a trailing token was accepted");
    CHECK(otaBleSubmitCommand(("begin " + n + " " + sha + " raw").c_str()) == OtaBleSubmit::Rejected,
          "begin with a transform but no image size was accepted");
    otaBleTick(0);
    CHECK(g_fake.beginCalls == 1, "a malformed begin reached esp_ota_begin");
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
    test_begin_rejects_trailing_garbage();
    test_credit_overrun();
    test_abort_mid_transfer();
    test_stale_abort_latch();
    test_abort_latched_begin();
    test_stall_timeout();
    test_begin_while_active();
    test_latch_collision();
    test_end_without_session();
    test_write_failure();
    test_end_failure();
    test_set_boot_failure();
    test_no_partition();
    test_begin_erase_failure_releases_the_operation();
    test_ring_wrap();
    test_progress_and_credit();
    test_credit_repeat();
    test_raw_xform_is_the_old_protocol();
    test_xform_unknown_and_unavailable();
    test_xform_wire_may_exceed_the_image();
    test_info_reports_base_and_transforms();
    test_info_when_the_base_cannot_be_hashed();
#if TEST_HAVE_TAMP
    test_tamp_round_trip();
    test_tamp_survives_tiny_chunks();
    test_tamp_chunk_size_sweep();
    test_tamp_strands_no_tail();
    test_tamp_window_too_large_is_refused();
    test_tamp_truncated_fails_on_length();
#endif
    test_delta_strips_its_header();
    test_delta_header_split_across_chunks();
    test_delta_wrong_base_is_refused();
    test_delta_bad_magic_is_refused();
    test_delta_header_only_is_refused();
    test_delta_truncated_header_is_refused();

    if (g_failures) {
        printf("\n%d check(s) FAILED\n", g_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
