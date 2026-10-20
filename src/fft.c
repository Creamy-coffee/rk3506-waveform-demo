/*
 * fft.c —— radix-2 原地 FFT + 实数序列幅度谱
 */
#include "fft.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 复用的工作缓冲，避免每帧 malloc。512 点 FFT 用 1024 容量足够。 */
static float s_re[FFT_MAX_N];
static float s_im[FFT_MAX_N];

/* 位反转重排 */
static void bit_reverse(float *re, float *im, int n)
{
    int j = 0;
    for (int i = 1; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;

        if (i < j) {
            float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
}

void fft_radix2(float *re, float *im, int n)
{
    if (n <= 1 || n > FFT_MAX_N) return;
    /* 判断是不是 2 的幂 */
    if (n & (n - 1)) return;

    bit_reverse(re, im, n);

    for (int len = 2; len <= n; len <<= 1) {
        const double ang = -2.0 * M_PI / (double)len;
        const float  wr  = (float)cos(ang);
        const float  wi  = (float)sin(ang);
        const int    half = len >> 1;

        for (int i = 0; i < n; i += len) {
            float cr = 1.0f, ci = 0.0f;
            for (int k = 0; k < half; k++) {
                const int   a = i + k;
                const int   b = a + half;

                const float ur = re[a], ui = im[a];
                const float xr = re[b], xi = im[b];

                const float vr = xr * cr - xi * ci;
                const float vi = xr * ci + xi * cr;

                re[a] = ur + vr;  im[a] = ui + vi;
                re[b] = ur - vr;  im[b] = ui - vi;

                /* 旋转因子递推 */
                const float ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = ncr;
            }
        }
    }
}

/* 窗系数（与 FPGA 交付包 linux/spectrum.c 一致） */
static float win_coef(fft_win_t win, int i, int n)
{
    const float z = 2.0f * (float)M_PI * (float)i / (float)n;
    switch (win) {
    case FFT_WIN_BLACKMAN:                       /* SciPy blackman: a=[0.42,0.50,0.08] */
        return 0.42f - 0.5f * cosf(z) + 0.08f * cosf(2.0f * z);
    case FFT_WIN_FLATTOP: {                      /* SciPy flattop 五项 */
        static const float a[5] = { 0.21557895f, 0.41663158f, 0.277263158f,
                                    0.083578947f, 0.006947368f };
        float w = 0.0f;
        for (int k = 0; k < 5; k++) w += a[k] * cosf((float)k * z);
        return w;
    }
    case FFT_WIN_HANN:
    default:
        return 0.5f - 0.5f * cosf(z);            /* SciPy hann: a=[0.5,0.5] */
    }
}

void fft_magnitude_spectrum_zero_win(const float *x, int n_real, int n_fft,
                                     fft_win_t win, float *mag)
{
    if (!x || !mag || n_real < 2 || n_fft < n_real) return;
    if (n_fft > FFT_MAX_N || (n_fft & (n_fft - 1))) return;

    /* 窗加在**真实样点**上，补零的部分不加窗（本来就是 0） */
    for (int i = 0; i < n_real; i++) {
        s_re[i] = x[i] * win_coef(win, i, n_real);
        s_im[i] = 0.0f;
    }
    for (int i = n_real; i < n_fft; i++) {
        s_re[i] = 0.0f;
        s_im[i] = 0.0f;
    }

    fft_radix2(s_re, s_im, n_fft);

    /* 归一化按 n_real 算，不是 n_fft —— 补零不改变信号能量。
     * 与 fft_magnitude_spectrum() 同口径：单频 A*sin() 峰值读出来约等于 A。 */
    const float scale = 4.0f / (float)n_real;
    const int   half  = n_fft >> 1;

    for (int k = 0; k < half; k++) {
        const float m = sqrtf(s_re[k] * s_re[k] + s_im[k] * s_im[k]);
        mag[k] = m * scale;
    }
}

void fft_magnitude_spectrum(const float *x, int n, float *mag)
{
    if (!x || !mag || n <= 1 || n > FFT_MAX_N) return;
    if (n & (n - 1)) return;

    /* 加 Hann 窗（相干增益 0.5，下面归一化时补偿回来） */
    for (int i = 0; i < n; i++) {
        const float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1)));
        s_re[i] = x[i] * w;
        s_im[i] = 0.0f;
    }

    fft_radix2(s_re, s_im, n);

    /* 单边幅度谱：|X[k]| * 2/n，再 *2 补偿 Hann 窗的 0.5 相干增益 */
    const float scale = 4.0f / (float)n;
    const int   half  = n >> 1;

    for (int k = 0; k < half; k++) {
        const float m = sqrtf(s_re[k] * s_re[k] + s_im[k] * s_im[k]);
        mag[k] = m * scale;
    }
}

/* 兼容旧调用：Hann */
void fft_magnitude_spectrum_zero(const float *x, int n_real, int n_fft, float *mag)
{
    fft_magnitude_spectrum_zero_win(x, n_real, n_fft, FFT_WIN_HANN, mag);
}
