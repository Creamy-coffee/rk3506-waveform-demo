/* Offline replay of real ANA packets through the SAME OOK core as the screen. */
#include "ook_decoder.h"
#include <stdio.h>
static unsigned u32(const unsigned char *p){return (unsigned)p[0]|((unsigned)p[1]<<8)|((unsigned)p[2]<<16)|((unsigned)p[3]<<24);}
static unsigned crc32(const unsigned char *p,int n){unsigned c=~0u;while(n--){c^=*p++;for(int k=0;k<8;k++)c=(c>>1)^((c&1)?0xedb88320u:0);}return ~c;}
int main(int argc,char **argv)
{
    if(argc!=4){fprintf(stderr,"usage: %s capture.bin known_baud alternating_test_0_or_1\n",argv[0]);return 2;}
    FILE *f=fopen(argv[1],"rb");if(!f)return 2;
    fseek(f,0,SEEK_END);long size=ftell(f);rewind(f);if(size<0 || size>64*1024*1024){fclose(f);return 2;}
    unsigned char *data=malloc((size_t)size);if(!data){fclose(f);return 2;}
    if(fread(data,1,(size_t)size,f)!=(size_t)size){fclose(f);free(data);return 2;}fclose(f);
    ook_decoder_t r={0};unsigned good=0,bad=0,gaps=0;
    for(long off=0;off+1076<=size;off++){
        unsigned char *p=data+off;if(memcmp(p,"V32ANA1!",8))continue;
        if(crc32(p,1072)!=u32(p+1072)){bad++;continue;}
        unsigned info=u32(p+36);if(u32(p+8)!=1 || u32(p+24)!=256 || (info&255)!=1 || ((info>>16)&255)!=5 || u32(p+16)!=65000000)continue;
        v32_ana_view_t v={0};v.valid=1;v.mode=5;v.kind=1;v.frame_id=u32(p+12);v.decim=u32(p+20);v.flags=u32(p+28);v.cfg_id=u32(p+32);v.fcw=u32(p+40);v.channel=(info>>8)&255;
        for(int i=0;i<256;i++){unsigned y=u32(p+48+4*i);v.y[i]=(y&0xffffff)/256.f;v.yvalid[i]=!(y&0x80000000u);v.nvalid+=v.yvalid[i];}
        if(r.have_frame && (r.cfg!=v.cfg_id || r.fcw!=v.fcw || r.decim!=v.decim || r.channel!=v.channel || (unsigned)(v.frame_id-r.fid)!=1))gaps++;
        ook_push(&r,&v,atoi(argv[2]),atoi(argv[3]));good++;
    }
    printf("{\"CRC_good_OOK_frames\":%u,\"CRC_bad_ANA_candidates\":%u,\"frame_or_config_gaps\":%u,\"last_segment_bits\":%llu,\"erasures\":%llu,\"alternating_errors\":%llu,\"alternating_checks\":%llu,\"locked\":%d,\"levels\":[%.3f,%.3f],\"recent\":\"%s\"}\n",good,bad,gaps,r.bits,r.erasures,r.alternating_errors,r.alternating_checks,r.locked,r.low,r.high,r.recent);
    free(data);return 0;
}
