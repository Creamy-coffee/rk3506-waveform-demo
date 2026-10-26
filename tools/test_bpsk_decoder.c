#include "bpsk_decoder.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define PI 3.14159265358979323846
static int pn[127];
static double symbol_rate=1000,timing_phase=.31;
static v32_ana_view_t frame(unsigned fid,double phase,double cfo,float *filter)
{
    v32_ana_view_t v={0};v.valid=1;v.mode=6;v.kind=0;v.decim=4096;v.cfg_id=3;v.fcw=66076420;v.frame_id=fid;v.nvalid=256;
    double fs=65000000.0/4096;
    for(int i=0;i<256;i++){
        double t=((fid-1)*256+i)/fs;int symbol=(int)floor(t*symbol_rate+timing_phase),bit=pn[symbol%127];
        *filter=.45f*(*filter)+.55f*(bit?1:-1);
        double a=2*PI*cfo*t+phase;
        v.y[i]=100*(*filter)*cos(a)+.15*sin(t*fs*.73);v.q[i]=100*(*filter)*sin(a)+.15*cos(t*fs*.41);v.yvalid[i]=1;
    }
    return v;
}
int main(void)
{
    unsigned s=127;for(int i=0;i<127;i++){pn[i]=s&1;s=(s>>1)|(((s^(s>>1))&1)<<6);}
    for(int phase=0;phase<4;phase++)for(int freq=-1;freq<=1;freq++){
        symbol_rate=1000*(1+(phase-1.5)*.0002);timing_phase=.08+phase*.22;
        bpsk_decoder_t r={0};float filter=-1;
        for(int fid=1;fid<=250;fid++){v32_ana_view_t v=frame(fid,phase*.9,freq*145.7,&filter);bpsk_push(&r,&v,1000,0,1);}
        printf("case phase=%d freq=%d bits=%llu evm=%f pn=%d errors=%llu checks=%llu\n",phase,freq,r.bits,r.evm,r.pn_locked,r.pn_errors,r.pn_checks);fflush(stdout);
        assert(r.locked && r.bits>3000 && !r.erasures && r.pn_locked && r.pn_errors==0 && r.pn_checks>2000);
        assert(fabs(r.cfo_hz-freq*145.7)<2);
        unsigned long long count=r.bits;v32_ana_view_t v=frame(250,phase*.9,freq*145.7,&filter);bpsk_push(&r,&v,1000,0,1);assert(r.bits==count);
        v.frame_id+=10;bpsk_push(&r,&v,1000,0,1);assert(!r.bits && !r.locked);
        v.flags=1;bpsk_push(&r,&v,1000,0,1);assert(!r.have_frame);
        v.flags=0;v.kind=1;bpsk_push(&r,&v,1000,0,1);assert(!r.have_frame);
        v.kind=0;v.yvalid[128]=0;bpsk_push(&r,&v,1000,0,1);assert(!r.have_frame);
        v.yvalid[128]=1;bpsk_push(&r,&v,4000,0,1);assert(!r.have_frame);
        bpsk_reset(&r);
    }
    bpsk_decoder_t r={0};float filter=-1;
    for(int fid=1;fid<=150;fid++){v32_ana_view_t v=frame(fid,0,40,&filter);for(int i=0;i<256;i++){v.y[i]=100;v.q[i]=0;}bpsk_push(&r,&v,1000,0,1);}assert(!r.locked && !r.bits);bpsk_reset(&r);
    puts("BPSK_DECODER_PASS PN7_CFO_phase_timing_300ppm_duplicates_gaps_flags_invalid_CW");
}
