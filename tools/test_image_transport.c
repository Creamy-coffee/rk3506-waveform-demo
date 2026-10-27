#include "image_transport.h"
#include "ook_decoder.h"
#include "bpsk_decoder.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned char symbols[200000],pixels[2048],packets[68*64];
static size_t ns,np,npack;
static size_t read_file(const char *dir,const char *name,unsigned char *out,size_t max){
    char path[1024];snprintf(path,sizeof path,"%s/%s",dir,name);FILE *f=fopen(path,"rb");assert(f);
    size_t n=fread(out,1,max,f);assert(!ferror(f) && feof(f));fclose(f);return n;
}
static void check(const image_receiver_t *r){
    assert(r->complete && r->have_image && r->width==32 && r->height==32 && r->received==4);
    assert(r->total_bytes==np && !memcmp(r->pixels,pixels,np));
    for(unsigned y=0;y<32;y++)for(unsigned x=0;x<32;x++)assert(image_receiver_pixel(r,x,y)==((pixels[4*y+x/8]>>(7-x%8))&1));
}
static void physical_symbols(unsigned mode,double phase,double offset){
    image_receiver_t image;image_receiver_clear(&image);
    ook_decoder_t binary={0};bpsk_decoder_t bpsk={0};
    binary.sink=bpsk.sink=image_receiver_symbol;binary.sink_context=bpsk.sink_context=&image;
    float filter=-1;double fs=65000000.0/4096;
    for(unsigned fid=1;fid<=850;fid++){
        v32_ana_view_t v={0};v.valid=1;v.frame_id=fid;v.cfg_id=3;v.fcw=66076420;
        v.mode=mode;v.kind=mode==6?0:mode==3?2:1;v.decim=4096;v.nvalid=256;
        for(int i=0;i<256;i++){
            double t=((fid-1)*256+i)/fs;size_t symbol=(size_t)floor(t*1000+.31)%ns;
            int bit=symbols[symbol];filter=.45f*filter+.55f*(bit?1:-1);
            double p=phase+6.283185307179586*offset*t;
            if(mode==6){v.y[i]=100*filter*cos(p);v.q[i]=100*filter*sin(p);}
            else v.y[i]=mode==3?1000*filter:50*(1+filter);
            v.yvalid[i]=1;
        }
        if(mode==6)bpsk_push(&bpsk,&v,1000,0,0);
        else binary_push(&binary,&v,1000,0,mode==3);
        if(image.complete)break;
    }
    printf("image mode=%u phase=%.2f cfo=%.2f complete=%u packets=%llu gaps=%llu crc_errors=%llu\n",mode,phase,offset,image.complete,(unsigned long long)image.packets,(unsigned long long)image.gaps,(unsigned long long)image.crc_errors);
    fflush(stdout);check(&image);
    bpsk_reset(&bpsk);ook_reset(&binary);
    check(&image); /* A link reset preserves the last CRC-verified image. */
}
int main(int argc,char **argv){
    assert(argc==2);ns=read_file(argv[1],"symbols.bin",symbols,sizeof symbols);
    np=read_file(argv[1],"image.raw",pixels,sizeof pixels);
    npack=read_file(argv[1],"packets.bin",packets,sizeof packets);assert(npack==272);
    for(int skip=0;skip<2;skip++)for(int invert=0;invert<2;invert++){
        image_receiver_t r;image_receiver_clear(&r);assert(image_receiver_pixel(&r,0,0)==-1);
        for(size_t i=skip;i<ns;i++)image_receiver_symbol(&r,symbols[i]^invert);
        check(&r);
    }
    image_receiver_t r;image_receiver_clear(&r);
    unsigned char bad[68];memcpy(bad,packets,68);bad[45]^=1;image_accept_packet(&r,bad);
    assert(!r.have_image && r.crc_errors==1);
    // Out-of-order delivery, duplicates, a gap inside an incomplete packet,
    // and re-synchronization on repeated packets must not paint unknown data.
    image_accept_packet(&r,packets+204);assert(r.received==1 && image_receiver_pixel(&r,0,0)==-1);
    image_accept_packet(&r,packets+204);assert(r.received==1 && r.duplicates==1);
    for(int i=0;i<200;i++)image_receiver_symbol(&r,symbols[512+i]);
    image_receiver_symbol(&r,-1);assert(r.received==1);
    for(size_t i=0;i<ns;i++)image_receiver_symbol(&r,symbols[i]);check(&r);
    // A structurally valid packet with a wrong whole-image digest cannot pass.
    image_receiver_clear(&r);
    for(unsigned i=0;i<4;i++){
        memcpy(bad,packets+68*i,68);bad[20]^=1;uint32_t crc=image_crc32(bad,64);
        for(unsigned b=0;b<4;b++)bad[64+b]=(uint8_t)(crc>>(8*b));image_accept_packet(&r,bad);
    }
    assert(!r.complete && !r.received && r.crc_errors==1);
    image_receiver_clear(&r);ook_decoder_t cut={0};cut.sink=image_receiver_symbol;cut.sink_context=&r;
    cut.locked=1;cut.have_edge=1;cut.bits=123;
    ook_timing_reset(&cut);assert(r.gaps==1 && !cut.locked && cut.bits==123 && cut.sink==image_receiver_symbol);
    ook_timing_reset(&cut);assert(r.gaps==1); /* Already unlocked: no duplicate discontinuity. */
    physical_symbols(5,0,0);physical_symbols(3,0,0);
    physical_symbols(6,.7,0);physical_symbols(6,2.3,80);
    puts("IMAGE_TRANSPORT_PASS polarity_pair_phase_CRC_duplicate_gap_OOK_FSK_BPSK");
}
