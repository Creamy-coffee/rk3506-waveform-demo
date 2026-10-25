#ifndef V32_ANALYSIS_HISTORY_H
#define V32_ANALYSIS_HISTORY_H
#include "v32cap3.h"
#include <math.h>
#include <string.h>
#define ANA_HISTORY_CAPACITY 16384
typedef struct {
    float y[ANA_HISTORY_CAPACITY], q[ANA_HISTORY_CAPACITY];
    int length, have_previous;
    uint32_t cfg, kind, mode, channel, decim, fcw, fid;
} ana_history_t;
static inline void ana_history_clear(ana_history_t *h)
{ h->length=0;h->have_previous=0; }
static inline int ana_history_push(ana_history_t *h,const v32_ana_view_t *v)
{
    if(!v->valid || v->flags || v->nvalid<=0) { ana_history_clear(h);return 0; }
    int same=h->have_previous && h->cfg==v->cfg_id && h->kind==v->kind &&
        h->mode==v->mode && h->channel==v->channel && h->decim==v->decim && h->fcw==v->fcw;
    if(same && h->fid==v->frame_id) { ana_history_clear(h);return 0; }
    if(!same || (uint32_t)(v->frame_id-h->fid)!=1u) h->length=0;
    if(h->length+V32_ANA_POINTS>ANA_HISTORY_CAPACITY) {
        const int drop=ANA_HISTORY_CAPACITY/2;
        memmove(h->y,h->y+drop,(size_t)(h->length-drop)*sizeof(float));
        memmove(h->q,h->q+drop,(size_t)(h->length-drop)*sizeof(float));h->length-=drop;
    }
    for(int i=0;i<V32_ANA_POINTS;i++) {
        h->y[h->length]=v->yvalid[i]?v->y[i]:NAN;
        h->q[h->length]=v->yvalid[i]?v->q[i]:NAN;h->length++;
    }
    h->have_previous=1;h->cfg=v->cfg_id;h->kind=v->kind;h->mode=v->mode;
    h->channel=v->channel;h->decim=v->decim;h->fcw=v->fcw;h->fid=v->frame_id;
    return 1;
}
static inline int ana_history_take(const ana_history_t *h,int maximum)
{ return h->length<maximum?h->length:maximum; }
static inline int ana_echo_matches(const v32_ana_view_t *v,int mode,int channel,int rate,uint32_t fcw)
{
    static const uint32_t r[3]={8192,4096,2048};
    return rate>=0 && rate<3 && v->mode==(uint32_t)mode && v->channel==(uint32_t)channel &&
        v->decim==r[rate] && v->fcw==fcw;
}
#endif
