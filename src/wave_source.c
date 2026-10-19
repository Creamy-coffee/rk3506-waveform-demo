/*
 * wave_source.c —— 假数据源实现（占位用，以后换成真实采集）
 *
 * 做法：100Hz 基波 + 300Hz 三次谐波 + 500Hz 五次谐波 + 带限噪声。
 * 三次谐波的幅度按 0.4Hz 缓慢起伏（0.20 ~ 0.70），
 * 于是频谱图上的第二根柱子会持续上下动，画面不会死板。
 */
#include "wave_source.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------- 可调参数 ---------------- */
#define DEMO_SAMPLE_RATE  4000  /* 演示源的采样率，和真实 65 MSPS 无关 */
#define DEMO_FRAME_LEN    512   /* 演示源每帧点数 */

#define DEMO_F1_HZ      100.0   /* 基波频率 */
#define DEMO_F2_HZ      300.0   /* 三次谐波 */
#define DEMO_F3_HZ      500.0   /* 五次谐波 */

#define DEMO_A1         1.00    /* 基波幅度 */
#define DEMO_A2_BASE    0.45    /* 三次谐波幅度基准 */
#define DEMO_A2_SWING   0.25    /* 三次谐波幅度起伏量 */
#define DEMO_A2_MOD_HZ  0.4     /* 起伏速度 */
#define DEMO_A3         0.20    /* 五次谐波幅度 */
#define DEMO_NOISE      0.04    /* 噪声幅度 */

/* ---------------- 内部状态 ---------------- */
static double   s_t       = 0.0;        /* 连续时间轴，保证帧间相位连续 */
static uint32_t s_rng     = 0x12345678u;
static float    s_noise_lp = 0.0f;

/* xorshift32 → [0,1) */
static float rnd01(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return (float)(s_rng & 0xFFFFFFu) / (float)0x1000000u;
}

/* 一阶低通，让噪声看起来像带限噪声，而不是逐点乱跳 */
static float noise_slow(void)
{
    float n = (rnd01() - 0.5f) * 2.0f;
    s_noise_lp += 0.25f * (n - s_noise_lp);
    return s_noise_lp;
}

static int demo_init(void)
{
    s_t        = 0.0;
    s_rng      = 0x12345678u;
    s_noise_lp = 0.0f;
    return 0;
}

static int demo_read_frame(float *buf, int len)
{
    const double dt = 1.0 / (double)DEMO_SAMPLE_RATE;

    if (!buf || len <= 0) return 0;
    if (len > DEMO_FRAME_LEN) len = DEMO_FRAME_LEN;

    for (int i = 0; i < len; i++) {
        double t  = s_t + (double)i * dt;
        double a2 = DEMO_A2_BASE + DEMO_A2_SWING * sin(2.0 * M_PI * DEMO_A2_MOD_HZ * t);

        double v = DEMO_A1 * sin(2.0 * M_PI * DEMO_F1_HZ * t)
                 + a2      * sin(2.0 * M_PI * DEMO_F2_HZ * t)
                 + DEMO_A3 * sin(2.0 * M_PI * DEMO_F3_HZ * t)
                 + DEMO_NOISE * (double)noise_slow();

        buf[i] = (float)v;
    }

    /* 时间轴往前推一整帧。因为一帧不是基波周期的整数倍，
     * 所以每帧的起始相位都在变 —— 屏幕上波形会持续滚动。 */
    s_t += (double)len * dt;

    return len;
}

static void demo_deinit(void)
{
}

const wave_source_t wave_source_demo = {
    .name          = "demo",
    .frame_len     = DEMO_FRAME_LEN,
    .sample_rate   = DEMO_SAMPLE_RATE,
    .init          = demo_init,
    .read_frame    = demo_read_frame,
    .read_frame2   = NULL,      /* 单通道合成信号，没有第二路 */
    .read_spectrum = NULL,      /* 频谱交给 UI 层本地算 */
    .deinit        = demo_deinit,
};

/* 当前生效的数据源。默认演示源；main.c 解析 --source 后会改它。 */
static const wave_source_t *s_active = &wave_source_demo;

void wave_source_select(const wave_source_t *s)
{
    if (s) s_active = s;
}

const wave_source_t *wave_source_get(void)
{
    return s_active;
}
