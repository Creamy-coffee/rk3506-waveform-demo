#include "bpsk_decoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
static int argument(const char *s,int low,int high,int *result)
{char *end;errno=0;long n=strtol(s,&end,10);if(errno || end==s || *end || n<low || n>high)return 0;*result=(int)n;return 1;}
static unsigned u32(const unsigned char *p){return (unsigned)p[0]|((unsigned)p[1]<<8)|((unsigned)p[2]<<16)|((unsigned)p[3]<<24);}
static unsigned crc32(const unsigned char *p,int n){unsigned c=~0u;while(n--){c^=*p++;for(int k=0;k<8;k++)c=(c>>1)^((c&1)?0xedb88320u:0);}return ~c;}
int main(int argc,char **argv)
{
    if(argc<5 || argc>6){fprintf(stderr,"usage: %s capture.bin baud pn7_test_0_or_1 invert_0_or_1 [bits.txt]\n",argv[0]);return 2;}
    int baud,test,invert;if(!argument(argv[2],1,65000000,&baud)||!argument(argv[3],0,1,&test)||!argument(argv[4],0,1,&invert)){fprintf(stderr,"Invalid baud or boolean argument\n");return 2;}
    FILE *f=fopen(argv[1],"rb");if(!f)return 2;fseek(f,0,SEEK_END);long size=ftell(f);rewind(f);
    if(size<0 || size>64000000){fclose(f);return 2;}unsigned char *data=malloc((size_t)size);if(!data){fclose(f);return 2;}
    if(fread(data,1,(size_t)size,f)!=(size_t)size){free(data);fclose(f);return 2;}fclose(f);
    FILE *bits=argc==6?fopen(argv[5],"w"):NULL;if(argc==6 && !bits){free(data);return 2;}
    bpsk_decoder_t r={0};unsigned good=0,bad=0,gaps=0;
    for(long off=0;off+1076<=size;off++){
        unsigned char *p=data+off;if(memcmp(p,"V32ANA1!",8))continue;
        if(crc32(p,1072)!=u32(p+1072)){bad++;continue;}
        unsigned info=u32(p+36);if(u32(p+8)!=1 || u32(p+16)!=65000000 || u32(p+24)!=256 || u32(p+44) || (info&255) || ((info>>16)&255)!=6 || (info>>24) || ((info>>8)&255)>1)continue;
        v32_ana_view_t v={0};v.valid=1;v.mode=6;v.kind=0;v.frame_id=u32(p+12);v.cfg_id=u32(p+32);v.flags=u32(p+28);v.decim=u32(p+20);v.fcw=u32(p+40);v.channel=(info>>8)&255;v.nvalid=256;
        if(v.decim!=2048 && v.decim!=4096 && v.decim!=8192)continue;
        for(int i=0;i<256;i++){unsigned w=u32(p+48+4*i);v.y[i]=(int16_t)w;v.q[i]=(int16_t)(w>>16);v.yvalid[i]=1;}
        int gap=r.have_frame && (r.cfg!=v.cfg_id || r.fcw!=v.fcw || r.decim!=v.decim || r.channel!=v.channel || (unsigned)(v.frame_id-r.fid)!=1);
        if(gap)gaps++;unsigned long long before=gap?0:r.bits+r.erasures;
        bpsk_push(&r,&v,baud,invert,test);good++;
        if(bits){unsigned long long total=r.bits+r.erasures,delta=total>=before?total-before:total;if(gap || total<before)fputs("\n# segment\n",bits);if(delta>(unsigned)r.recent_n)delta=r.recent_n;if(delta)fwrite(r.recent+r.recent_n-(int)delta,1,(size_t)delta,bits);}
    }
    printf("{\"CRC_good_BPSK_frames\":%u,\"CRC_bad_ANA_candidates\":%u,\"frame_or_config_gaps\":%u,\"last_segment_bits\":%llu,\"erasures\":%llu,\"locked\":%d,\"coarse_CFO_hz\":%.6f,\"evm_rms\":%.6f,\"pn7_locked\":%d,\"pn7_errors\":%llu,\"pn7_checks\":%llu,\"pn7_direction\":%d,\"pn7_invert\":%d,\"manual_invert\":%d,\"recent\":\"%s\"}\n",good,bad,gaps,r.bits,r.erasures,r.locked,r.cfo_hz,(double)r.evm,r.pn_locked,r.pn_errors,r.pn_checks,r.pn_dir,r.pn_invert,r.invert,r.recent);
    if(bits)fclose(bits);bpsk_reset(&r);free(data);return 0;
}
