#include <chrono>
#include <cstdint>
#include <cstdio>
__attribute__((noinline)) bool original(uint32_t g,uint32_t last,uint32_t end) {
    return g>=last+16384 || g==end;
}
__attribute__((noinline)) bool revised(uint32_t g,uint32_t last,uint32_t end) {
    return g>last && (g>=last+16384 || g==end);
}
int main() {
    for(auto f:{original,revised}) {
        volatile uint64_t total=0;
        auto a=std::chrono::steady_clock::now();
        for(uint32_t i=0;i<10000000;i++)total+=f(i%754625,500000,754624);
        double ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-a).count()/10000000;
        printf("%s %.3f ns/check, checksum %llu\n",f==original ? "original" : "revised",ns,(unsigned long long)total);
    }
}
