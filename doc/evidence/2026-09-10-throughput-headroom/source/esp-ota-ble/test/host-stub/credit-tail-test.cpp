// A fully granted payload needs one announcement, plus timed loss recovery.
#define main existing_suite_main
#include "ota_ble-test.cpp"
#undef main

int main() {
    begin_case("unchanged final credit is not emitted for every drain");
    auto img=makeImage(8192);
    otaBleSubmitCommand(beginCmd(img).c_str());otaBleTick(0);
    CHECK(countLines("OTAB CRED 8192")==1,"initial grant absent or duplicated");
    for(size_t off=0;off<img.size();off+=256) {
        otaBleStageBytes(img.data()+off,std::min(size_t(256),img.size()-off));
        otaBleTick(0);
    }
    printf("final-grant announcements after 32 drains: %zu\n",countLines("OTAB CRED 8192"));
    CHECK(countLines("OTAB CRED 8192")==1,"unchanged final grant flooded status");
    otaBleTick(4999);
    CHECK(countLines("OTAB CRED 8192")==1,"repeat arrived early");
    otaBleTick(5000);
    CHECK(countLines("OTAB CRED 8192")==2,"timed lost-credit recovery disappeared");
    otaBleSubmitCommand("end");otaBleTick(5001);
    CHECK(g_restarted && g_fake.bootPart,"credit suppression broke complete transfer");
    end_case();
    return g_failures ? 1 : 0;
}
