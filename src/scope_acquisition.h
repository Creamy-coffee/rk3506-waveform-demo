#ifndef SCOPE_ACQUISITION_H
#define SCOPE_ACQUISITION_H
#include <math.h>

/* Prefer independently measured carrier frequency to a decimated alias.
   Manual timebase remains manual; only undo an unsafe sampling rate. */
static inline int scope_decimation_target(double adc_fs,int points,int current,
    double carrier,int trusted,float margin,int manual,int low_ac)
{
    if(adc_fs<=0 || points<2 || current<1 || !isfinite(carrier) || carrier<=0)return current;
    const double fs=adc_fs/current;
    if(manual){
        if(!trusted)return current;
        int want=current;
        while(want>1 && carrier>=.4*adc_fs/want)want>>=1;
        return want;
    }
    if(!trusted && low_ac)return current;
    int want=current;double periods=carrier*points/fs;
    if(!trusted && margin<15.f){if(want<256)want<<=1;return want;}
    while(periods<2 && want<256){want<<=1;periods*=2;}
    while(periods>30 && want>1){want>>=1;periods*=.5;}
    /* Two periods alone cannot guarantee that the carrier is below Nyquist. */
    while(want>1 && carrier>=.4*adc_fs/want)want>>=1;
    return want;
}

static inline void scope_auto_range(float minimum,float maximum,float *lo,float *scale)
{
    /* A constant ADC input is not a full-scale AC signal. Keep DC offsets,
       but never magnify less than 32 ADC counts to fill the entire plot. */
    const float center=(minimum+maximum)*.5f;
    float span=(maximum-minimum)*1.1f;if(span<32.f)span=32.f;
    *lo=center-span*.5f;*scale=1.f/span;
}
#endif
