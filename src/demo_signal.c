/*
 * demo_signal.c —— 演示信号发生器
 *
 * 逐行对照参考稿的 JS 写，注释里保留原始 JS 表达式，方便以后核对。
 */
#include "demo_signal.h"

#include <math.h>
#include <string.h>

/* 参考稿：function noiseFn(x,seed=0){
 *   return sin(x*.32+seed)*.48 + sin(x*1.87+seed*2)*.26
 *        + sin(x*4.19+seed)*.18 + sin(x*9.1+seed)*.08 } */
static float noise_fn(float x, float seed)
{
    return sinf(x * 0.32f + seed) * 0.48f
         + sinf(x * 1.87f + seed * 2.0f) * 0.26f
         + sinf(x * 4.19f + seed) * 0.18f
         + sinf(x * 9.10f + seed) * 0.08f;
}

/* 参考稿：const cyc = frequency*timebase*10*.18;
 *
 * 那个 0.18 是原型作者塞进去的**视觉缩放系数**：按物理算，
 * 10 MHz 在 2 µs/div 下每格 20 个周期，10 格就是 200 个周期，
 * 在那个宽度下会糊成一片。乘 0.18 才好看。
 * 这里保持原值，保证和参考稿长得一样。 */
#define DEMO_CYC_FUDGE   0.18f

/*
 * ⚠️ tick 与 t 的区别 —— 这里踩过一个很隐蔽的坑。
 *
 * 参考稿有两个时间量：
 *     tick  —— 帧计数，每帧 +1（render() 里 if(running) tick++）
 *     t     —— 秒，只在 drawScope 里定义：const t = tick*.02
 * 也就是说  tick = t / 0.02 = 50t。
 *
 * 而参考稿各页用的**不是同一个**：
 *     drawScope 用 t      -> noiseFn(x + t)
 *     drawFFT   用 tick   -> noiseFn(x + tick*.1)
 *     drawLock  用 tick   -> sin(u*7 + tick*.018)
 *     drawAI    用 tick   -> sin(u*PI*15 + tick*.03), noiseFn(x + tick*.22)
 *
 * 移植时如果一律把 tick 换成 t，那几个系数就要乘 50 才对得上 ——
 * 漏乘的后果不是"看起来不一样"，而是**数值直接错**：
 * FFT 页的 PEAK LEVEL 因此从 -12.8 dBFS 变成 -18.5 dBFS（噪声相位差了 50 倍）。
 * 所以下面凡是要用 tick 的地方，统一写成 `c->t * (DEMO_TICK_PER_SEC * <原系数>)`，
 * 让"我这里是 tick 不是 t"这件事在代码里看得见。
 */
#define DEMO_TICK_PER_SEC  50.0f    /* tick = t/0.02 */

static float cycles_of(const demo_sig_t *c)
{
    return (float)c->freq_mhz * c->timebase_us * 10.0f * DEMO_CYC_FUDGE;
}

static float noise_amp(const demo_sig_t *c)
{
    float n = (float)c->noise_pct / 100.0f;
    if(n < 0.f) n = 0.f;
    if(n > 1.f) n = 1.f;
    return n;
}

/* 把 y_px/h 形式的结果统一转成 y01 */
static inline float to_y01(float y_over_h)
{
    float v = 1.0f - y_over_h;
    if(v < 0.f) v = 0.f;
    if(v > 1.f) v = 1.f;
    return v;
}

/* ==================== SCOPE ==================== */
/* drawScope():
 *   const t = tick*.02;
 *   ch1: h*.47 - sin(u*PI*2*cyc + t)*h*.26 - noiseFn(x+t)*h*.055*n
 *   ch2: h*.55 - sin(u*PI*2*cyc + t + phase*PI/180)*h*.22 - noiseFn(x+t+10)*h*.045*n
 * （x 是像素列号，u = x/w） */
/*
 * AM 调制波（2026-10-24 起默认）：载波乘 (1 + 0.6·sin(包络)) 的包络，
 * 满屏放 2.5 个调制周期 —— 与原型 scope_proto_1024.html 的 wave('am')
 * （载频×0.05 调制、深度 0.6、500µs/格 视角）一致。
 * 峰值 0.26×1.6 = 0.416，0.47±0.416 = 0.054..0.886，仍在 0..1 画布内。
 * 发生器实测台当前就是 1 kHz 载频 + 1 kHz AM 50% 的信号，demo 也对齐成 AM。
 */
#define DEMO_AM_MOD_CYC   2.5f    /* 满屏调制周期数 */
#define DEMO_AM_DEPTH     0.6f    /* 调制深度 */
#define DEMO_AM_MPH       0.8f    /* 包络相位速度（t 的系数） */

static float am_env(float u, float t, float mph)
{
    return 1.0f + DEMO_AM_DEPTH * sinf(u * 6.2831853f * DEMO_AM_MOD_CYC + t * mph);
}

void demo_scope_ch1(const demo_sig_t *c, float *y01, int n)
{
    if(y01 == NULL || n < 2) return;
    const float cyc = cycles_of(c);
    const float amp = noise_amp(c);
    const float t   = c->t;

    for(int i = 0; i < n; i++) {
        const float u = (float)i / (float)(n - 1);
        const float x = u * DEMO_REF_W;
        const float y = 0.47f
                      - sinf(u * 6.2831853f * cyc + t) * 0.26f * am_env(u, t, DEMO_AM_MPH)
                      - noise_fn(x + t, 0.0f) * 0.055f * amp;
        y01[i] = to_y01(y);
    }
}

void demo_scope_ch2(const demo_sig_t *c, float *y01, int n)
{
    if(y01 == NULL || n < 2) return;
    const float cyc = cycles_of(c);
    const float amp = noise_amp(c);
    const float t   = c->t;
    const float ph  = c->phase_deg * 3.14159265f / 180.0f;

    for(int i = 0; i < n; i++) {
        const float u = (float)i / (float)(n - 1);
        const float x = u * DEMO_REF_W;
        /* 两路包络相位错开 1/4 周期，像两路独立的 AM，XY 档看也不重合 */
        const float y = 0.55f
                      - sinf(u * 6.2831853f * cyc + t + ph) * 0.22f
                        * am_env(u, t, DEMO_AM_MPH + 1.5708f)
                      - noise_fn(x + t + 10.0f, 0.0f) * 0.045f * amp;
        y01[i] = to_y01(y);
    }
}

/* ==================== FFT ==================== */
/* drawFFT():
 *   let peak = frequency/32.5, peak2 = peak*2, peak3 = peak*3;
 *   h*.87 - ( exp(-((u-peak )/.007)^2)*h*.70
 *           + exp(-((u-peak2)/.008)^2)*h*.37
 *           + exp(-((u-peak3)/.009)^2)*h*.18 )
 *         - (noiseFn(x+tick*.1)+1)*h*.017*(1+noise/70)
 *
 * 注意第三项在 y_px 里是**减**，转到 y01 就是**加**。 */
void demo_fft(const demo_sig_t *c, float *y01, int n,
              float *peak_x, float *peak_y01)
{
    if(y01 == NULL || n < 2) return;

    const float peak  = (float)c->freq_mhz / 32.5f;
    const float peak2 = peak * 2.0f;
    const float peak3 = peak * 3.0f;
    const float nf    = 1.0f + (float)c->noise_pct / 70.0f;

    int   best_i = 0;
    float best_v = -1e30f;

    for(int i = 0; i < n; i++) {
        const float u = (float)i / (float)(n - 1);
        const float x = u * DEMO_REF_W;

        const float a = (u - peak)  / 0.007f;
        const float b = (u - peak2) / 0.008f;
        const float d = (u - peak3) / 0.009f;
        const float gauss = expf(-(a * a)) * 0.70f
                          + expf(-(b * b)) * 0.37f
                          + expf(-(d * d)) * 0.18f;

        float y = 0.13f
                + gauss
                /* 参考稿：noiseFn(x + tick*.1) —— 用的是帧计数，不是秒 */
                + (noise_fn(x + c->t * (DEMO_TICK_PER_SEC * 0.1f), 0.0f) + 1.0f)
                  * 0.017f * nf;

        if(y < 0.f) y = 0.f;
        if(y > 1.f) y = 1.f;
        y01[i] = y;

        /* 只看基波那一段找峰，避免被 3 次谐波抢走 */
        if(u < peak + 0.06f && y > best_v) { best_v = y; best_i = i; }
    }

    if(peak_x)   *peak_x   = (float)best_i / (float)(n - 1);
    if(peak_y01) *peak_y01 = y01[best_i];
}

/* ==================== LOCK-IN ==================== */
/* drawLock():
 *   h*.45 - sin(u*7 + tick*.018)*h*.13 - sin(u*26 + tick*.03)*h*.013   （青）
 *   h*.62 - cos(u*7 + tick*.018)*h*.09                                 （黄） */
void demo_lock_a(const demo_sig_t *c, float *y01, int n)
{
    if(y01 == NULL || n < 2) return;
    for(int i = 0; i < n; i++) {
        const float u = (float)i / (float)(n - 1);
        const float y = 0.45f
                      - sinf(u * 7.0f  + c->t * 0.9f)  * 0.13f
                      - sinf(u * 26.0f + c->t * 1.5f)  * 0.013f;
        y01[i] = to_y01(y);
    }
}

void demo_lock_b(const demo_sig_t *c, float *y01, int n)
{
    if(y01 == NULL || n < 2) return;
    for(int i = 0; i < n; i++) {
        const float u = (float)i / (float)(n - 1);
        const float y = 0.62f - cosf(u * 7.0f + c->t * 0.9f) * 0.09f;
        y01[i] = to_y01(y);
    }
}

/* ==================== AI ==================== */
/* drawAI(raw):
 *   h*.52 - sin(u*PI*15 + tick*.03)*h*.28
 *         - (raw ? noiseFn(x+tick*.22)*h*.23*noise/70
 *                : (ai ? noiseFn(x+tick*.22)*h*.025*noise/70
 *                      : noiseFn(x+tick*.22)*h*.23*noise/70)) */
void demo_ai(const demo_sig_t *c, float *y01, int n, bool raw, bool ai_on)
{
    if(y01 == NULL || n < 2) return;

    const float noise = (float)c->noise_pct;
    const float denoise = ai_on ? 0.025f : 0.23f;
    const float k = (raw ? 0.23f : denoise) * noise / 70.0f;
    const float t = c->t;

    for(int i = 0; i < n; i++) {
        const float u = (float)i / (float)(n - 1);
        const float x = u * DEMO_REF_W;
        const float y = 0.52f
                      /* 参考稿：sin(u*PI*15 + tick*.03) */
                      - sinf(u * 3.14159265f * 15.0f
                             + t * (DEMO_TICK_PER_SEC * 0.03f)) * 0.28f
                      /* 参考稿：noiseFn(x + tick*.22) */
                      - noise_fn(x + t * (DEMO_TICK_PER_SEC * 0.22f), 0.0f) * k;
        y01[i] = to_y01(y);
    }
}

/* ==================== 读数换算 ==================== */

float demo_vpp_mv(const float *y01, int n, int divy)
{
    if(y01 == NULL || n < 2) return 0.f;
    float lo = y01[0], hi = y01[0];
    for(int i = 1; i < n; i++) {
        if(y01[i] < lo) lo = y01[i];
        if(y01[i] > hi) hi = y01[i];
    }
    if(divy < 1) divy = 1;
    return (hi - lo) * (float)divy * V32_DEMO_MV_PER_DIV;
}

float demo_dbfs_from_y01(float y01)
{
    if(y01 < 0.f) y01 = 0.f;
    if(y01 > 1.f) y01 = 1.f;
    return -(1.0f - y01) * V32_DEMO_DB_FULLSCALE;
}

float demo_noise_floor_dbfs(const float *y01, int n)
{
    if(y01 == NULL || n < 1) return 0.f;
    float sum = 0.f;
    for(int i = 0; i < n; i++) sum += y01[i];
    return demo_dbfs_from_y01(sum / (float)n);
}
