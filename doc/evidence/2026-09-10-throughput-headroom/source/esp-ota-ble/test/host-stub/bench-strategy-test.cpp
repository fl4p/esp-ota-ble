// Exercise real receiver code with the existing flash model and failure cases.
#define main baseline_suite_main
#include "ota_ble-test.cpp"
#undef main

static void full_write(size_t size) {
    begin_case("bench forced full rewrite");
    auto img=makeImage(size);
    // Even an identical destination must be programmed in these strategies.
    g_fake.slot.assign(g_fake.partSize,0xff);
    std::copy(img.begin(),img.end(),g_fake.slot.begin());
    CHECK(otaBleSubmitCommand(beginCmd(img).c_str())==OtaBleSubmit::Accepted,"begin rejected");
    otaBleTick(0);
    pushAll(img);
    otaBleSubmitCommand("end"); otaBleTick(0);
    CHECK(g_restarted && g_fake.bootPart,"did not validate/select boot");
    CHECK(g_fake.flashed==img,"not every byte was programmed");
    CHECK(std::equal(img.begin(),img.end(),g_fake.slot.begin()),"slot differs");
    CHECK(sawLine(("program_bytes="+std::to_string(size)+" ").c_str()),"program count absent/wrong");
#if OTA_BLE_BENCH_STRATEGY == 2 || OTA_BLE_BENCH_STRATEGY == 4 || OTA_BLE_BENCH_STRATEGY == 5
    CHECK(g_fake.eraseBytesInBegin<=65536,"begin exceeded one block");
    CHECK(g_fake.eraseBytesInBegin+g_fake.eraseRangeBytes==((size+4095)/4096)*4096,"wrong erase coverage");
    CHECK(sawLine(OTA_BLE_BENCH_STRATEGY==5 ? "mode=internal" : OTA_BLE_BENCH_STRATEGY==4 ? "mode=buffered" : "mode=blocks"),"wrong strategy report");
#elif OTA_BLE_BENCH_STRATEGY == 3
    CHECK(g_fake.eraseBytesInBegin==((size+4095)/4096)*4096,"wrong upfront erase");
    CHECK(sawLine("mode=upfront"),"wrong strategy report");
#else
    CHECK(g_fake.eraseBytesInBegin==0,"sequential erased upfront");
    CHECK(sawLine("mode=sequential"),"wrong strategy report");
#endif
    end_case();
}
static void batch_write_failure() {
    begin_case("second batched write fails");
    // Two full writes even with the largest 16 KiB bench buffer. The old
    // 20,000-byte fixture never issued its second write before `end` there.
    auto img=makeImage(65536);
    otaBleSubmitCommand(beginCmd(img).c_str()); otaBleTick(0);
    g_fake.failWriteAtCall=2; pushAll(img);
    CHECK(sawLine("OTAB FAIL esp_ota_write"),"second write failure not reported");
    CHECK(!otaBleActive() && g_fake.aborted,"failed write did not abort");
    CHECK(!g_fake.bootPart && !g_restarted,"failed write selected a boot image");
    CHECK(g_quiesce.back()==false,"failed write retained quiesce");
    end_case();
}
int main() {
    full_write(4096);full_write(65536);full_write(65537);full_write(190001);
    test_truncated();test_sha_mismatch();test_oversized();test_credit_overrun();
    test_abort_mid_transfer();batch_write_failure();test_end_failure();test_set_boot_failure();
    if(g_failures)return 1;
    puts("PASS: exact full programming/erase coverage, boundaries, truncation, digest, overflow and flash failures");
}
