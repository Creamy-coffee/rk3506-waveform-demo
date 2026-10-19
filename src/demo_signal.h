/*
 * demo_signal.h —— 演示信号发生器（照抄参考稿的数学式）
 *
 * 参考稿是纯前端 canvas 原型，画面里所有曲线都是现算的，没有真实采集。
 * 这个模块就是把那些 JS 公式原样搬到 C 里，让板子上的观感和参考稿一致。
 *
 * 公式来源：v32_linux_touch_ui-1.html 的 <script>
 *   noiseFn() / drawScope() / drawFFT() / drawIQ() / drawLock() / drawAI()
 *
 * 唯一的改写是把「canvas 像素坐标 y（向下为正）」换算成
 * 「归一化纵坐标 y01（0=贴底，1=贴顶）」，换算关系：
 *   y01 = 1 - y_px / h
 * 这样 plot.c 只管按 0..1 铺满绘图区，不用知道原来画布多高。
 *
 * ⚠️ 这是**演示数据**。接真实波形请用 wave_source.h 那一层（见 ui.c 里的
 *    「数据来源切换点」注释），本模块只负责把界面喂饱。
 */
#ifndef DEMO_SIGNAL_H
#define DEMO_SIGNAL_H

#include <stdbool.h>

/* 参考稿的窗口选项：窗函数下拉框 */
typedef enum {
    DEMO_WIN_HANN = 0,
    DEMO_WIN_HAMMING,
    DEMO_WIN_BLACKMAN
} demo_window_t;

typedef struct {
    int    freq_mhz;      /* 信号频率下拉：1 / 5 / 10 / 20 / 30 */
    float  timebase_us;   /* 时基下拉：0.5 / 1 / 2 / 5 / 10 µs/div */
    int    noise_pct;     /* 噪声滑块：0..100 */
    float  phase_deg;     /* 相位差读数（也是 IQ 矢量的角度） */
    int    window;        /* demo_window_t */
    float  t;             /* 动画相位，每帧 +0.02（暂停时不动） */
} demo_sig_t;

/* 参考稿里 trace() 认定的画布宽度，noiseFn 的横坐标按它缩放 */
#define DEMO_REF_W   760.0f

/* ---- SCOPE 页：两条时域曲线，输出 y01[0..1] ---- */
void demo_scope_ch1(const demo_sig_t *c, float *y01, int n);
void demo_scope_ch2(const demo_sig_t *c, float *y01, int n);

/* ---- FFT 页：一条频谱曲线 ----
 * 同时回吐峰值位置（peak_x 归一化横坐标 0..1，peak_y01 归一化纵坐标），
 * 供 plot_set_peak() 打箭头、以及右栏算 PEAK LEVEL。 */
void demo_fft(const demo_sig_t *c, float *y01, int n,
              float *peak_x, float *peak_y01);

/* ---- LOCK-IN 页 ----
 * a：锁相输出时间历程（参考稿的青色那条）
 * b：参考通道（黄色那条） */
void demo_lock_a(const demo_sig_t *c, float *y01, int n);
void demo_lock_b(const demo_sig_t *c, float *y01, int n);
/* I/Q 矢量角度：直接用 c->phase_deg */

/* ---- AI 页：raw=true 原始输入，raw=false 输出（ai_on 决定是否降噪） ---- */
void demo_ai(const demo_sig_t *c, float *y01, int n, bool raw, bool ai_on);

/* ---- 由 y01 曲线估算峰峰值（mV），给右栏 MEASUREMENTS 用 ----
 * 口径：整个绘图区高度 = divy 格，每格 V32_DEMO_MV_PER_DIV 毫伏。
 * 参考稿默认 CH1 量程下算出来 ≈ 20.8 mV，和它写死的 20.4 mV 基本对得上。 */
#define V32_DEMO_MV_PER_DIV  5.0f
float demo_vpp_mv(const float *y01, int n, int divy);

/* ---- 频谱读数：峰值电平 / 底噪，单位 dBFS ---- */
/* 口径：纵轴满刻度 = 0 dBFS，纵轴底部 = -100 dBFS */
#define V32_DEMO_DB_FULLSCALE  100.0f
float demo_dbfs_from_y01(float y01);
/* 取曲线中位数当底噪 */
float demo_noise_floor_dbfs(const float *y01, int n);

#endif /* DEMO_SIGNAL_H */
