#ifndef V32_MODULATION_VIEW_H
#define V32_MODULATION_VIEW_H
#include <math.h>
#include <stdio.h>
#include <stddef.h>
/* Presentation only: never estimates modulation, fills missing data, or sends commands. */
static inline int mod_view_count(int available,double fs,double milliseconds)
{
    if(available<2 || !isfinite(fs) || fs<=0 || milliseconds<=0)return 0;
    double want=ceil(fs*milliseconds/1000.0);
    int n=want>available?available:(int)want;
    return n<2?2:n;
}
static inline void mod_view_time(char *out,size_t len,double seconds)
{
    if(!isfinite(seconds) || seconds<0)snprintf(out,len,"--");
    else if(seconds<.001)snprintf(out,len,"%.2f us",seconds*1e6);
    else if(seconds<1)snprintf(out,len,"%.2f ms",seconds*1e3);
    else snprintf(out,len,"%.3f s",seconds);
}
static inline int mod_view_range(const float *y,int n,int nonnegative,float *lo,float *hi)
{
    float a=INFINITY,b=-INFINITY;
    for(int i=0;i<n;i++)if(isfinite(y[i])){if(y[i]<a)a=y[i];if(y[i]>b)b=y[i];}
    if(!isfinite(a)||!isfinite(b))return 0;
    if(nonnegative){*lo=0;*hi=b>1e-6f?b*1.12f:1.f;}
    else {float d=b-a;if(d<1e-6f)d=fmaxf(fabsf(a)*.1f,1.f);*lo=a-d*.08f;*hi=b+d*.08f;}
    return 1;
}
static inline void mod_view_resample(const float *src,int count,float *out,int n,float lo,float hi)
{
    if(count<2 || n<2 || !(hi>lo)){for(int i=0;i<n;i++)out[i]=NAN;return;}
    if(count>n){
        const int buckets=n/2;
        for(int k=0;k<buckets;k++){
            int a=(int)((long long)k*count/buckets),b=(int)((long long)(k+1)*count/buckets);
            int imin=a,imax=a,hole=0;
            for(int j=a;j<b;j++){if(!isfinite(src[j]))hole=1;else {if(src[j]<src[imin])imin=j;if(src[j]>src[imax])imax=j;}}
            int first=imin<imax?imin:imax,last=imin<imax?imax:imin;
            out[2*k]=hole?NAN:(src[first]-lo)/(hi-lo);out[2*k+1]=hole?NAN:(src[last]-lo)/(hi-lo);
        }
        if(n&1)out[n-1]=isfinite(src[count-1])?(src[count-1]-lo)/(hi-lo):NAN;
    }else{
        for(int i=0;i<n;i++){
            double t=(double)i*(count-1)/(n-1);int a=(int)t,b=a+1<count?a+1:a;float f=(float)(t-a);
            if(f<1e-6f)out[i]=isfinite(src[a])?(src[a]-lo)/(hi-lo):NAN;
            else out[i]=isfinite(src[a])&&isfinite(src[b])?(src[a]+(src[b]-src[a])*f-lo)/(hi-lo):NAN;
        }
    }
}
#endif
