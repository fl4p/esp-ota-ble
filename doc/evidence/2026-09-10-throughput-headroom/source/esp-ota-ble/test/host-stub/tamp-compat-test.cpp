#include <fstream>
#include <iterator>
#include <vector>
#include <cstdio>
#include "ota_xform.h"
static std::vector<uint8_t> output;
static esp_err_t sink(const uint8_t *p,size_t n,void*) {
    output.insert(output.end(),p,p+n);return ESP_OK;
}
static std::vector<uint8_t> read(const char *name) {
    std::ifstream f(name,std::ios::binary);
    if(!f)throw "cannot read input";
    return {std::istreambuf_iterator<char>(f),{}};
}
int main(int argc,char **argv) {
    if(argc!=3)return 2;
    auto image=read(argv[1]), wire=read(argv[2]);
    for(size_t chunk:{size_t(1),size_t(17),size_t(495),size_t(2048)}) {
        output.clear(); OtaXformIo io;io.write=&sink;
        const char *why=nullptr;
        if(!otaXformBegin(OtaXform::Tamp,image.size(),io,&why))return 3;
        for(size_t off=0;off<wire.size();off+=chunk) {
            if(otaXformFeed(wire.data()+off,std::min(chunk,wire.size()-off))!=ESP_OK)return 4;
        }
        if(otaXformFinish()!=ESP_OK)return 5;
        otaXformEnd();
        if(output!=image)return 6;
    }
    puts("PASS: host payload through actual transform and node-vendored tamp decoder, four fragment sizes");
}
