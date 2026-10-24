/*
 * fft.h —— 极简 FFT（radix-2，原地运算）
 *
 * 点数少（512）且只做幅度谱，所以自己写一个就够了，
 * 不用引 kissFFT / FFTW，省掉一个外部依赖。
 */
#ifndef FFT_H
#define FFT_H

/* 支持的 FFT 最大点数。V32CAP3 一帧 4096 点，再 2 倍补零到 8192，
 * 所以这里取 8192（两个 8192 的 float 数组 = 64 KB 静态内存）。 */
#define FFT_MAX_N   8192

/* 原地复数 FFT。n 必须是 2 的幂。 */
void fft_radix2(float *re, float *im, int n);

/*
 * 实数序列 → 幅度谱。
 *   x   : 输入，n 点实数
 *   mag : 输出，n/2 点（bin 0 .. n/2-1）
 *
 * 内部先加 Hann 窗抑制频谱泄漏，再归一化 ——
 * 单频正弦 A*sin() 会在对应 bin 上读出约等于 A 的峰值，可直接当幅度看。
 */
void fft_magnitude_spectrum(const float *x, int n, float *mag);

/*
 * 补零版：输入 n_real 点实数，补零到 n_fft 点再做 FFT，输出 n_fft/2 点幅度。
 *
 * ⚠️ 补零**不提高频率分辨率**，只是把频谱插值成更密的网格。
 *    真实分辨率始终由 n_real 决定：df_真 = fs / n_real。
 *
 * V32CAP3 为什么用这个：一帧 4096 点 @65 MSPS，直接做 FFT 的 bin 宽是
 * 15.87 kHz；补零到 8192 后网格变成 7.935 kHz，正好和参考稿 UI 上写的
 * 「8192 POINTS / BIN WIDTH 7.935 kHz」对得上，峰值读数也更平滑。
 */
/*
 * 显示/分析用的窗（系数取自 SciPy general_cosine，与 FPGA 交付包
 * linux/spectrum.c 一致）。20261024 协作包把窗函数枚举扩到 7 种
 * （RECT/HANN/HAMMING/BLACKMAN/FLATTOP/KAISER/NUTTALL），本地这四档
 * 是照协议补齐的 —— 协议 id 到这里的映射在 ui.c（两套编号不同，别混用）。
 *
 * 统一写成余弦级数 w = Σ a[k]·cos(k·2π·n/N)。
 */
typedef enum {
    FFT_WIN_HANN = 0,      /* 0.5 - 0.5cos */
    FFT_WIN_BLACKMAN,      /* 0.42 - 0.5cos + 0.08cos2 */
    FFT_WIN_FLATTOP,       /* 5 项 Flat Top */
    FFT_WIN_RECT,          /* 矩形（不加窗） */
    FFT_WIN_HAMMING,       /* 0.54 - 0.46cos */
    FFT_WIN_KAISER,        /* β=8.6，I0 级数近似 */
    FFT_WIN_NUTTALL        /* 4 项最小旁瓣 */
} fft_win_t;

void fft_magnitude_spectrum_zero_win(const float *x, int n_real, int n_fft,
                                     fft_win_t win, float *mag);

/* 兼容旧调用：等价于 FFT_WIN_HANN */
void fft_magnitude_spectrum_zero(const float *x, int n_real, int n_fft, float *mag);

/*
 * 把 bin 序号换算成频率（Hz）。仅为方便 UI 打刻度用。
 */
static inline float fft_bin_to_hz(int bin, int n, int sample_rate)
{
    return (float)bin * (float)sample_rate / (float)n;
}

#endif /* FFT_H */
