/* Actual FFT gain and sidelobe regression, independent of UI labels. */
#include "fft.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
static float x[1024],m[4096];
int main(void){
    const int n=1024;
    for(int w=0;w<3;w++){
        for(int i=0;i<n;i++)x[i]=123.f*cosf(2.f*3.14159265358979323846f*73*i/n);
        fft_magnitude_spectrum_zero_win(x,n,8192,(fft_win_t)w,m);
        assert(fabsf(m[73*8]-123.f)<.02f);
        /* Distant sidelobes of a coherent tone should be negligible. */
        assert(m[73*8+56]<.02f);
        for(int i=0;i<n;i++)x[i]=37.f;
        fft_magnitude_spectrum_zero_win(x,n,8192,(fft_win_t)w,m);
        assert(fabsf(m[0]-37.f)<.01f);
        printf("window=%d coherent gain / DC / distant sidelobe PASS\n",w);
    }
    for(int i=0;i<n;i++)x[i]=123.f*cosf(2.f*3.14159265358979323846f*73.37f*i/n);
    fft_magnitude_spectrum_zero_win(x,n,8192,FFT_WIN_FLATTOP,m);
    float peak=0;for(int i=560;i<610;i++)if(m[i]>peak)peak=m[i];
    assert(fabsf(peak/123.f-1.f)<.003f);
    puts("FlatTop off-bin amplitude PASS");
    return 0;
}
