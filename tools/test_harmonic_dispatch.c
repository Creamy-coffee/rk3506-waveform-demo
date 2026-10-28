#define _DEFAULT_SOURCE
#include <assert.h>
#include "../src/v32cap3.c"
wave_source_t wave_source_v32;
static void put(uint8_t *p,uint32_t v){for(int i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static void packet(uint8_t *p,unsigned id){
    memset(p,0,216);memcpy(p,"V32HRM1!",8);put(p+8,2);put(p+12,216);put(p+16,id);put(p+20,1);
    put(p+24,65000000);put(p+28,4096);put(p+32,66076420);put(p+36,1);put(p+44,0x80000200);put(p+56,31);
    for(int i=0;i<5;i++){uint32_t b=i?1000:1000000;put(p+72+8*i,b);put(p+152+8*i,6*b);put(p+192+4*i,63*(i+1));}
    put(p+212,crc32_calc(p,212));
}
int main(void){
    crc_init();uint8_t p[216];v32_harmonic_view_t m;packet(p,1);
    /* Every possible split, same single mixed-stream parser. */
    for(int cut=1;cut<216;cut++){
        memset(&s_harmonic,0,sizeof s_harmonic);s_len=0;
        v32_feed(p,cut);assert(!v32_get_harmonic(&m));v32_feed(p+cut,216-cut);
        assert(v32_get_harmonic(&m)&&m.valid&&m.m.thd_valid);assert(m.m.thd_q24>1061000&&m.m.thd_q24<1061200);
    }
    uint32_t before=s_harmonic.seq;s_harmonic_ms=mono_ms()-2000;v32_feed(p,216);
    assert(v32_get_harmonic(&m)&&!m.valid&&m.seq==before); /* duplicate must stay stale */
    uint8_t mix[432];memcpy(mix,p,216);mix[100]^=1;packet(mix+216,2);v32_feed(mix,sizeof mix);
    assert(v32_get_harmonic(&m)&&m.valid&&m.m.result_id==2); /* CRC rejection + resync */
    packet(p,3);put(p+8,1);put(p+44,0x80000000);put(p+64,m.m.thd_q24);put(p+212,crc32_calc(p,212));
    v32_feed(p,216);assert(v32_get_harmonic(&m)&&m.m.version==1&&m.m.thd_valid);
    packet(p,4);put(p+40,130000000);put(p+212,crc32_calc(p,212));v32_feed(p,216);
    assert(v32_get_harmonic(&m)&&!m.valid&&m.age_ms>=2000);
    int fds[2];assert(pipe(fds)==0);s_fd=fds[1];s_live=true;
    assert(v32_cmd_harmonic(1,1,0x12345678)==0);uint8_t cmd[17];assert(read(fds[0],cmd,17)==17);
    const uint8_t expected[]={0xa5,0xf3,0,0xa5,0xf0,1,0xa5,0xf1,0x34,0x12,0xa5,0xf2,0x78,0x56,0xa5,0xf3,1};
    assert(!memcmp(cmd,expected,17));assert(v32_cmd_harmonic(1,2,1)<0);assert(v32_cmd_harmonic(1,0,0x80000000)<0);
    close(fds[0]);close(fds[1]);puts("HARMONIC_DISPATCH_PASS splits CRC resync duplicate version age command");return 0;
}
