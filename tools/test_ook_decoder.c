#include "ook_decoder.h"
#include <assert.h>
#include <stdio.h>
static int pn[127];
static v32_ana_view_t frame(unsigned fid,int pattern,double phase,float *filter)
{
    v32_ana_view_t v={0};v.valid=1;v.mode=5;v.kind=1;v.decim=4096;v.cfg_id=1;v.fcw=66076420;v.frame_id=fid;v.nvalid=256;
    double T=(65000000.0/4096)/1000;
    for(int i=0;i<256;i++){
        int n=(int)((fid-1)*256+i),symbol=(int)floor((n+phase)/T);
        int bit=pattern?pn[symbol%127]:symbol&1;
        float target=bit?100:0;*filter=.6f*(*filter)+.4f*target;
        v.y[i]=fmaxf(0,*filter+.25f*sinf(n*.73f));v.yvalid[i]=1;
    }
    return v;
}
int main(void)
{
    unsigned lfsr=127;for(int i=0;i<127;i++){pn[i]=lfsr&1;lfsr=(lfsr>>1)|(((lfsr^(lfsr>>1))&1)<<6);}
    for(int pattern=0;pattern<2;pattern++)for(int p=0;p<5;p++){
        ook_decoder_t r={0};float filter=0;double phase=p*2.1,T=(65000000.0/4096)/1000;
        for(int f=1;f<=80;f++){
            v32_ana_view_t v=frame(f,pattern,phase,&filter);ook_push(&r,&v,1000,!pattern);
            if(r.recent_n==64){int end=(int)floor((r.decision_center+phase)/T);
                for(int j=0;j<64;j++)assert(r.recent[j]=='0'+(pattern?pn[(end-63+j)%127]:(end-63+j)&1));}
        }
        assert(r.locked && r.bits>1000 && r.erasures==0);
        if(!pattern)assert(r.alternating_errors==0 && r.alternating_checks>1000);
        unsigned long long bits=r.bits;double t=r.t;v32_ana_view_t duplicate=frame(80,pattern,phase,&filter);ook_push(&r,&duplicate,1000,!pattern);assert(r.bits==bits && r.t==t);
        v32_ana_view_t gap=frame(1000,pattern,phase,&filter);ook_push(&r,&gap,1000,!pattern);assert(r.bits<20);
    }
    ook_decoder_t r={0};float filter=0;
    for(int f=1;f<=20;f++){v32_ana_view_t v=frame(f,0,0,&filter);for(int i=0;i<256;i++)v.y[i]=35+.2f*sinf(i);ook_push(&r,&v,1000,0);}assert(!r.locked && !r.bits);
    ook_reset(&r);filter=0;
    for(int f=1;f<=8;f++){v32_ana_view_t v=frame(f,0,0,&filter);ook_push(&r,&v,1650,0);}assert(!r.locked && !r.bits);
    ook_reset(&r);filter=0;for(int f=1;f<=8;f++){v32_ana_view_t v=frame(f,0,0,&filter);ook_push(&r,&v,1000,0);}
    v32_ana_view_t v=frame(9,0,0,&filter);v.yvalid[128]=0;v.nvalid--;ook_push(&r,&v,1000,0);assert(r.erasures>0);
    v.flags=1;v.frame_id++;ook_push(&r,&v,1000,0);assert(!r.bits && !r.locked);
    v.flags=0;ook_push(&r,&v,4000,0);assert(!r.bits && !r.locked);
    v.mode=1;ook_push(&r,&v,1000,0);assert(!r.bits);
    puts("OOK_DECODER_PASS filtered_0101_PN7_phases_duplicates_gaps_invalid_CW_wrong_rate_flags");
}
