#include "ook_decoder.h"
#include <assert.h>
#include <stdio.h>
static int pn[127];
static v32_ana_view_t frame(unsigned fid,int pattern,double phase,float offset,float *filter)
{
    v32_ana_view_t v={0};v.valid=1;v.mode=3;v.kind=2;v.decim=4096;v.cfg_id=2;v.fcw=66076420;v.frame_id=fid;v.nvalid=256;
    double T=(65000000.0/4096)/1000;
    for(int i=0;i<256;i++){
        int n=(int)((fid-1)*256+i),symbol=(int)floor((n+phase)/T);
        int bit=pattern?pn[symbol%127]:symbol&1;
        *filter=.55f*(*filter)+.45f*(bit?700:-700);
        v.y[i]=*filter+offset+2*sinf(n*.73f);v.yvalid[i]=1;
    }
    return v;
}
int main(void)
{
    unsigned lfsr=127;for(int i=0;i<127;i++){pn[i]=lfsr&1;lfsr=(lfsr>>1)|(((lfsr^(lfsr>>1))&1)<<6);}
    for(int pattern=0;pattern<2;pattern++)for(int p=0;p<5;p++)for(int dc=-1;dc<=1;dc++){
        ook_decoder_t r={0};float filter=-700;double phase=p*2.1,T=(65000000.0/4096)/1000;
        for(int f=1;f<=80;f++){
            v32_ana_view_t v=frame(f,pattern,phase,dc*2000,&filter);fsk_push(&r,&v,1000,!pattern);
            if(r.recent_n==64){int end=(int)floor((r.decision_center+phase)/T);for(int j=0;j<64;j++)assert(r.recent[j]=='0'+(pattern?pn[(end-63+j)%127]:(end-63+j)&1));}
        }
        assert(r.locked && r.bits>1000 && !r.erasures && !r.alternating_errors);
        unsigned long long bits=r.bits;v32_ana_view_t v=frame(80,pattern,phase,dc*2000,&filter);fsk_push(&r,&v,1000,!pattern);assert(r.bits==bits);
        v=frame(1000,pattern,phase,dc*2000,&filter);fsk_push(&r,&v,1000,!pattern);assert(r.bits<20);
    }
    for(int signal=0;signal<3;signal++){
        ook_decoder_t r={0};float filter=-700;
        for(int f=1;f<=10;f++){v32_ana_view_t v=frame(f,0,0,0,&filter);for(int i=0;i<256;i++)v.y[i]=signal==0?-400+2*sinf(i):signal==1?700*sinf(i*.1f):v.y[i]*.01f;fsk_push(&r,&v,1000,0);}
        assert(!r.locked && !r.bits); /* CW/noise, smooth FM, insufficient separation. */
    }
    ook_decoder_t r={0};float filter=-700;for(int f=1;f<=8;f++){v32_ana_view_t v=frame(f,0,0,0,&filter);fsk_push(&r,&v,1000,1);}
    v32_ana_view_t v=frame(9,0,0,0,&filter);v.yvalid[128]=0;v.nvalid--;fsk_push(&r,&v,1000,1);assert(r.erasures>0);
    v.flags=1;v.frame_id++;fsk_push(&r,&v,1000,1);assert(!r.bits);
    v.flags=0;fsk_push(&r,&v,4000,1);assert(!r.bits);
    v.mode=5;fsk_push(&r,&v,1000,1);assert(!r.bits);
    puts("FSK_DECODER_PASS signed_offsets_filtered_0101_PN7_CW_FM_smallspan_gaps_flags_invalid");
}
