#include "bpsk_decoder.h"
#include "liquid.h"
#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define BPSK_PI 3.14159265358979323846
#include "bpsk_coarse.h"

void bpsk_reset(bpsk_decoder_t *r)
{
    if(r->tracker)symtrack_cccf_destroy((symtrack_cccf)r->tracker);
    if(r->resampler)msresamp_crcf_destroy((msresamp_crcf)r->resampler);
    memset(r,0,sizeof *r);
}
static int pn7(int phase)
{
    unsigned s=127;phase=(phase%127+127)%127;
    for(int j=0;j<phase;j++)s=(s>>1)|(((s^(s>>1))&1)<<6);
    return s&1;
}
static void pn_bit(bpsk_decoder_t *r,int bit)
{
    if(bit<0){r->pn_locked=r->training_n=r->training_tick=0;return;}
    if(r->pn_locked){
        if(bit!=(pn7(r->pn_phase)^r->pn_invert))r->pn_errors++;
        r->pn_checks++;r->pn_phase=(r->pn_phase+r->pn_dir+127)%127;return;
    }
    if(r->training_n==256){memmove(r->training,r->training+1,255);r->training_n=255;}
    r->training[r->training_n++]=(unsigned char)bit;
    if(r->training_n<256 || (++r->training_tick%32))return;
    int best=257,bphase=0,bdir=1,binv=0,reference[127];
    for(int j=0;j<127;j++)reference[j]=pn7(j);
    for(int dir=-1;dir<=1;dir+=2)for(int phase=0;phase<127;phase++)for(int inv=0;inv<2;inv++){
        int errors=0,at=phase;
        for(int j=0;j<256;j++){errors+=(r->training[j]!=(reference[at]^inv));at=(at+dir+127)%127;}
        if(errors<best){best=errors;bphase=at;bdir=dir;binv=inv;}
    }
    /* Acquisition is excluded from held-out consistency counts. */
    if(best<=2){r->pn_locked=1;r->pn_phase=bphase;r->pn_dir=bdir;r->pn_invert=binv;}
}
static void emit(bpsk_decoder_t *r,int bit)
{
    if(r->recent_n==64){memmove(r->recent,r->recent+1,63);r->recent_n=63;}
    r->recent[r->recent_n++]=bit<0?'?':(char)('0'+bit);r->recent[r->recent_n]=0;
    if(bit<0)r->erasures++;else r->bits++;
    if(r->test)pn_bit(r,bit);
}
static void process(bpsk_decoder_t *r,float complex input)
{
    float complex out[16],symbols[2];unsigned n=0,ns=0;
    input*=cexpf(-_Complex_I*(float)r->phase);
    r->phase=remainder(r->phase+r->step,2*BPSK_PI);
    msresamp_crcf_execute((msresamp_crcf)r->resampler,&input,1,out,&n);
    for(unsigned j=0;j<n;j++){
        symtrack_cccf_execute((symtrack_cccf)r->tracker,out[j],symbols,&ns);
        for(unsigned k=0;k<ns;k++){
            float a=crealf(symbols[k]),b=cimagf(symbols[k]);
            if(!isfinite(a)||!isfinite(b)){r->locked=0;continue;}
            r->recovered++;r->wi[r->window_at]=a;r->wq[r->window_at]=b;r->window_at=(r->window_at+1)%64;if(r->window_n<64)r->window_n++;
            float error=0;int positive=0,transitions=0,last=0;
            for(int u=0;u<r->window_n;u++){
                float x=r->wi[u],y=r->wq[u];int bit=x>=0;positive+=bit;if(u && bit!=last)transitions++;last=bit;
                error+=(fabsf(x)-1)*(fabsf(x)-1)+y*y;
            }
            r->evm=sqrtf(error/r->window_n);
            int good=r->recovered>=256 && r->window_n==64 && r->evm<.25f && positive>=8 && positive<=56 && transitions>=4;
            if(!good){if(r->locked)emit(r,-1);r->locked=0;r->pn_locked=r->training_n=r->training_tick=0;continue;}
            r->locked=1;
            int bit=(fabsf(a)<.35f || fabsf(b)>.6f)?-1:((a>=0)^r->invert);
            emit(r,bit);
            r->sym_i[r->sym_at]=a;r->sym_q[r->sym_at]=b;r->sym_at=(r->sym_at+1)%128;if(r->sym_n<128)r->sym_n++;
        }
    }
}
void bpsk_push(bpsk_decoder_t *r,const v32_ana_view_t *v,int baud,int invert,int test)
{
    invert=!!invert;test=!!test;
    if(!v->valid || v->mode!=6 || v->kind!=0 || v->flags || v->nvalid!=256 || !v->decim || baud<=0){bpsk_reset(r);return;}
    double fs=65000000.0/v->decim;
    if(fs/baud<6 || fs/baud>256){bpsk_reset(r);return;}
    int same=r->have_frame && r->cfg==v->cfg_id && r->fcw==v->fcw && r->decim==v->decim && r->channel==v->channel && r->baud==baud && r->invert==invert && r->test==test;
    if(same && r->fid==v->frame_id)return;
    if(!same || (uint32_t)(v->frame_id-r->fid)!=1)bpsk_reset(r);
    r->have_frame=1;r->cfg=v->cfg_id;r->fid=v->frame_id;r->fcw=v->fcw;r->decim=v->decim;r->channel=v->channel;r->baud=baud;r->invert=invert;r->test=test;r->fs=fs;
    float energy=0;
    for(int j=0;j<256;j++){
        if(!v->yvalid[j] || !isfinite(v->y[j]) || !isfinite(v->q[j])){bpsk_reset(r);return;}
        energy+=v->y[j]*v->y[j]+v->q[j]*v->q[j];
    }
    if(energy/256<16){bpsk_reset(r);return;}
    if(!r->tracker){
        r->tracker=symtrack_cccf_create(LIQUID_FIRFILT_RRC,4,7,.35f,LIQUID_MODEM_BPSK);
        r->resampler=msresamp_crcf_create((float)(4*baud/fs),60.f);
        if(!r->tracker || !r->resampler){bpsk_reset(r);return;}
        symtrack_cccf_set_bandwidth((symtrack_cccf)r->tracker,.9f);
    }
    for(int j=0;j<256;j++){
        if(!r->coarse_ready){
            r->ci[r->used]=(int16_t)v->y[j];r->cq[r->used]=(int16_t)v->q[j];
            if(++r->used<1024)continue;
            if(bpsk_coarse(r->ci,r->cq,1024,fs,baud,2,&r->step)){bpsk_reset(r);return;}
            double complex sum=0;
            for(int k=0;k<1024;k++){
                double complex z=(r->ci[k]+_Complex_I*r->cq[k])*cexp(-_Complex_I*r->step*k);sum+=z*z;
            }
            r->phase=carg(sum)/2;r->cfo_hz=r->step*fs/(2*BPSK_PI);r->coarse_ready=1;
            for(int k=0;k<1024;k++)process(r,(r->ci[k]+_Complex_I*r->cq[k])/2048.f);
        }else process(r,(v->y[j]+_Complex_I*v->q[j])/2048.f);
    }
}
