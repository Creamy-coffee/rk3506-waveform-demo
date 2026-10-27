#ifndef V32_BPSK_DECODER_H
#define V32_BPSK_DECODER_H
#include "v32cap3.h"
#include "decoded_bit_sink.h"
typedef struct {
    decoded_bit_sink_t sink;void *sink_context;
    void *tracker,*resampler;
    uint32_t cfg,fid,fcw,decim,channel;
    int have_frame,baud,invert,test,coarse_ready,locked,used,window_n,window_at;
    double fs,step,phase,cfo_hz;
    int16_t ci[1024],cq[1024];
    float wi[64],wq[64],sym_i[128],sym_q[128];int sym_n,sym_at;
    unsigned long long recovered,bits,erasures,pn_checks,pn_errors;
    char recent[65];int recent_n;
    unsigned char training[256];int training_n,training_tick,pn_locked,pn_phase,pn_dir,pn_invert;
    float evm;
} bpsk_decoder_t;
/* State must be zero initialized. Resets destroy allocated DSP state. */
void bpsk_reset(bpsk_decoder_t *r);
void bpsk_push(bpsk_decoder_t *r,const v32_ana_view_t *v,int baud,int invert,int pn7_test);
#endif
