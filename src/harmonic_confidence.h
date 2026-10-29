#ifndef HARMONIC_CONFIDENCE_H
#define HARMONIC_CONFIDENCE_H
#include "harmonic_receiver.h"
#include <string.h>
typedef struct {
    uint32_t cfg,fcw,channel,rate,window,id;
    unsigned active,count[5],confirmed;
} harmonic_confidence;
/* Display qualification only. Never modifies wire powers or THD arithmetic. */
static unsigned harmonic_confirm(harmonic_confidence *s,const v32_harmonic_measurement *m,int ready)
{
    if(!ready){memset(s,0,sizeof *s);return 0;}
    if(!s->active || s->cfg!=m->cfg_id || s->fcw!=m->reference_fcw ||
       s->channel!=m->channel || s->rate!=m->decimation || s->window!=m->window){
        memset(s,0,sizeof *s);s->cfg=m->cfg_id;s->fcw=m->reference_fcw;
        s->channel=m->channel;s->rate=m->decimation;s->window=m->window;
    }else if(s->id==m->result_id)return s->confirmed;
    s->active=1;s->id=m->result_id;s->confirmed=0;
    for(unsigned i=0;i<5;i++){
        double dbc;
        if(v32_harmonic_dbc(m,i+1,&dbc)){
            if(s->count[i]<3)s->count[i]++;
            if(s->count[i]>=3)s->confirmed|=1u<<i;
        }else s->count[i]=0;
    }
    return s->confirmed;
}
#endif
