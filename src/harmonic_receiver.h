#ifndef V32_HARMONIC_RECEIVER_H
#define V32_HARMONIC_RECEIVER_H
#include <stddef.h>
#include <stdint.h>
#define V32_HARMONIC_BYTES 216u
/* Feed complete packets from the existing SINGLE UART owner. No serial I/O. */
typedef struct {
 uint32_t version,result_id,cfg_id,adc_fs,nfft,reference_fcw,decimation,age_ticks;
 uint32_t flags,wire_flags,channel,window,valid_mask,noise_mask;
 uint32_t thd_q24,fpga_thd_q24,dropped_frames;
 uint64_t band[5],noise[5],signal_x6[5];
 uint32_t peak_bin[5];
 int thd_valid,computed_on_host,thd_below_noise;
} v32_harmonic_measurement;
/* 1=valid packet (THD may be invalid), 0=reject; out untouched on rejection. */
int v32_harmonic_decode(const uint8_t *packet,size_t size,v32_harmonic_measurement *out);
/* 1=displayable ratio, 0=unavailable/below noise. Requires libm at link time. */
int v32_harmonic_dbc(const v32_harmonic_measurement *m,unsigned order,double *dbc);
#endif
