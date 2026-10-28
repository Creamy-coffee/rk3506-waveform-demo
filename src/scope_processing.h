#ifndef SCOPE_PROCESSING_H
#define SCOPE_PROCESSING_H
#include <math.h>
/* Values stay in ADC counts; multiply/divide have independent display units. */
static inline float scope_math_sample(float a,float b,int mode)
{
    switch(mode){case 1:return a+b;case 2:return a-b;case 3:return a*b;
    case 4:return fabsf(b)<2.f?NAN:a/b;default:return NAN;}
}
/* A genuine crossing with complete pre/post samples; never wrap a frame. */
static inline int scope_trigger_start(const float *a,int n,int span,float level,int falling)
{
    if(!a || n<8 || span<8 || span>n || !isfinite(level))return -1;
    float lo=a[0],hi=a[0];
    for(int i=1;i<n;i++){if(a[i]<lo)lo=a[i];if(a[i]>hi)hi=a[i];}
    if(hi-lo<=1.f)return -1;
    float hysteresis=fmaxf(1.f,(hi-lo)*.01f);int armed=0,pre=span/5;
    for(int i=1;i<n;i++){
        if(!isfinite(a[i-1]) || !isfinite(a[i])){armed=0;continue;}
        if(falling?a[i-1]>=level+hysteresis:a[i-1]<=level-hysteresis)armed=1;
        int crossing=falling?(a[i-1]>level && a[i]<=level):(a[i-1]<level && a[i]>=level);
        if(armed && crossing){
            armed=0;
            if(i>=pre && i-pre+span<=n)return i-pre;
        }
    }
    return -1;
}
#endif
