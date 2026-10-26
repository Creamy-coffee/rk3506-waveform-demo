#ifndef V32_OOK_DECODER_H
#define V32_OOK_DECODER_H
#include "v32cap3.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
/* Known-rate OOK observation decoder. No protocol framing, FEC or BER claims. */
typedef struct {
    uint32_t cfg,fid,fcw,decim,channel,mode;
    int have_frame,baud,levels,state,have_state,have_edge,good_edges,locked;
    double fs,t,step,last_edge,next_center,sum,decision_center;
    int count,last_bit,have_bit;
    float low,high,threshold_low,threshold_high,previous;
    unsigned long long bits,erasures,alternating_checks,alternating_errors;
    char recent[65];int recent_n;
} ook_decoder_t;
static inline void ook_reset(ook_decoder_t *r){memset(r,0,sizeof *r);}
static inline int ook_float_compare(const void *a,const void *b){float x=*(const float*)a,y=*(const float*)b;return (x>y)-(x<y);}
static inline void ook_timing_reset(ook_decoder_t *r){r->have_edge=r->have_state=r->good_edges=r->locked=r->count=r->have_bit=0;r->sum=0;}
static inline void ook_emit(ook_decoder_t *r,int bit,int alternating){
    if(r->recent_n==64){memmove(r->recent,r->recent+1,63);r->recent_n=63;}
    r->recent[r->recent_n++]=bit<0?'?':(char)('0'+bit);r->recent[r->recent_n]=0;
    if(bit<0){r->erasures++;r->have_bit=0;return;}
    r->bits++;
    if(alternating && r->have_bit){r->alternating_checks++;if(bit==r->last_bit)r->alternating_errors++;}
    r->last_bit=bit;r->have_bit=1;
}
/* Shared binary timing; FSK input remains signed frequency deviation in Hz. */
static inline void binary_push(ook_decoder_t *r,const v32_ana_view_t *v,int baud,int alternating,int fsk)
{
    if(!v->valid || v->mode!=(fsk?3u:5u) || v->kind!=(fsk?2u:1u) || v->flags || !v->decim || v->nvalid<=0){ook_reset(r);return;}
    int same=r->have_frame && r->mode==v->mode && r->cfg==v->cfg_id && r->fcw==v->fcw && r->decim==v->decim && r->channel==v->channel && r->baud==baud;
    if(same && v->frame_id==r->fid)return; /* Duplicate delivery never emits duplicate bits. */
    if(!same || (uint32_t)(v->frame_id-r->fid)!=1u)ook_reset(r);
    r->have_frame=1;r->mode=v->mode;r->cfg=v->cfg_id;r->fid=v->frame_id;r->fcw=v->fcw;r->decim=v->decim;r->channel=v->channel;r->baud=baud;
    r->fs=(double)V32_ADC_FS_HZ/v->decim;r->step=baud>0?r->fs/baud:0;
    if(r->step<6 || !isfinite(r->step)){ook_timing_reset(r);return;}
    float sorted[V32_ANA_POINTS];int n=0;
    for(int i=0;i<V32_ANA_POINTS;i++)if(v->yvalid[i] && isfinite(v->y[i]) && (fsk || v->y[i]>=0))sorted[n++]=v->y[i];
    if(n>=32){
        qsort(sorted,(size_t)n,sizeof(float),ook_float_compare);
        float lo=sorted[n/10],hi=sorted[(n-1)*9/10];
        /* Require a clearly separated off/on population; constant CW cannot acquire. */
        float span=hi-lo;
        int separated=fsk ? span>=40 && sorted[3*n/8]-sorted[n/8]<=.15f*span && sorted[7*n/8]-sorted[5*n/8]<=.15f*span
                          : hi>=4 && span>=3 && lo<=.35f*hi;
        if(separated){
            if(!r->levels){r->low=lo;r->high=hi;r->levels=1;}
            else {r->low=.9f*r->low+.1f*lo;r->high=.9f*r->high+.1f*hi;}
            r->threshold_low=r->low+.4f*(r->high-r->low);r->threshold_high=r->low+.6f*(r->high-r->low);
        }
    }
    for(int i=0;i<V32_ANA_POINTS;i++,r->t++){
        float y=v->y[i];double t=r->t;
        if(!v->yvalid[i] || !isfinite(y) || (!fsk && y<0)){
            if(r->locked)ook_emit(r,-1,alternating);
            ook_timing_reset(r);continue;
        }
        if(!r->levels)continue;
        int state=r->have_state?r->state:(y>(r->threshold_low+r->threshold_high)*.5f);
        if(y>=r->threshold_high)state=1;else if(y<=r->threshold_low)state=0;
        if(r->have_state && state!=r->state){
            float mid=(r->threshold_low+r->threshold_high)*.5f;
            double edge=t;
            if(fabsf(y-r->previous)>1e-6f){double f=(mid-r->previous)/(y-r->previous);if(f>=0 && f<=1)edge=t-1+f;}
            if(r->have_edge){
                double dt=edge-r->last_edge,k=floor(dt/r->step+.5);
                if(k>=1 && k<=32 && fabs(dt-k*r->step)<=.20*r->step)r->good_edges++;
                else {r->good_edges=0;r->locked=0;r->have_bit=0;}
            }
            r->have_edge=1;r->last_edge=edge;r->next_center=edge+.5*r->step;r->sum=0;r->count=0;
            if(r->good_edges>=3)r->locked=1;
        }
        r->state=state;r->have_state=1;r->previous=y;
        if(r->have_edge && t-r->last_edge>32*r->step){ook_timing_reset(r);continue;}
        if(!r->have_edge)continue;
        if(fabs(t-r->next_center)<=.20*r->step){r->sum+=y;r->count++;}
        if(t>r->next_center+.20*r->step){
            if(r->locked){
                int bit=-1,minimum=(int)floor(.2*r->step);if(minimum<1)minimum=1;
                if(r->count>=minimum){double a=r->sum/r->count;if(a>=r->threshold_high)bit=1;else if(a<=r->threshold_low)bit=0;}
                r->decision_center=r->next_center;ook_emit(r,bit,alternating);
            }
            r->next_center+=r->step;r->sum=0;r->count=0;
        }
    }
}
static inline void ook_push(ook_decoder_t *r,const v32_ana_view_t *v,int baud,int alternating)
{binary_push(r,v,baud,alternating,0);}
static inline void fsk_push(ook_decoder_t *r,const v32_ana_view_t *v,int baud,int alternating)
{binary_push(r,v,baud,alternating,1);}
#endif
