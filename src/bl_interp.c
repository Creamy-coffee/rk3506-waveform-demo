/*
 * bl_interp.c —— 带限（sinc 多相 FIR）重采样实现
 *
 * 数学：先按 P=20 倍补零上采样，再过一个截止在原奈奎斯特的低通，
 * 等价于对每个输出点做一次加窗 sinc 卷积。系数见 bl_coef.h（自动生成）。
 *
 *   y(t) = Σ_j x[n0-(K-1)+j] · coef[p][j]
 *     u  = t + 64            （64 = 滤波器群延迟，单位输入样点）
 *     n0 = floor(u), p = round((u-n0)·20)   ← 相位（20 档，量化误差 < 1/40 样点）
 *
 * 复杂度：每输出点 129 次乘加。1024 点 × 2 通道 ≈ 26 万次，RK3506 上不到 1 ms，
 * 相对 5.6 帧/秒的串口节奏完全可以忽略。
 */
#include "bl_interp.h"
#include "bl_coef.h"

#include <math.h>

void bl_resample(const float *x, int n, float *y, int nout, float t0, float dt,
                 bool limit_overshoot)
{
    if (!x || !y || nout <= 0 || n < 2) return;

    for (int i = 0; i < nout; i++) {
        const float t = t0 + dt * (float)i;

        /* 换到「滤波器中心」坐标，再拆成整数位置 + 相位 */
        const float u = t + (float)BL_DELAY;
        int         n0 = (int)floorf(u);

        /*
         * 相位**线性插值**，不再直接取最近的一相。
         *
         * 20 个相位意味着分数延迟被量化到 1/20 = 0.05 个采样点，
         * 最大误差 0.025 个采样点。听起来很小，但 25 MHz @65 MSPS 每个周期
         * 只有 2.57 个采样点、斜率极陡（斜率 ≈ 2.44·A／采样点），
         * 于是 0.025 × 2.44 ≈ **6% 的幅度误差**，画出来就是一条抖动的正弦
         * —— 比原始点直连还"毛"，用户看到的就是这个。
         *
         * 在 p0 / p0+1 两组系数之间按小数部分线性混合，等效相位数上千，
         * 误差降到 0.1% 量级。代价只是内层循环跑两遍（129 抽头 ×2，仍然便宜）。
         */
        const float fr = (u - (float)n0) * (float)BL_PHASES;
        int   p0 = (int)fr;
        float w  = fr - (float)p0;
        if (p0 < 0)            { p0 = 0; w = 0.0f; }
        if (p0 >= BL_PHASES)   { p0 = 0; n0++; w = 0.0f; }
        int   p1 = p0 + 1;
        int   n1 = n0;
        if (p1 >= BL_PHASES) { p1 = 0; n1 = n0 + 1; }

        if (n0 - (BL_TAPS - 1) >= 0 && n1 < n) {
            /* 正常区：系数和时间顺序同向，内层循环两边都顺着读 */
            const float *c0  = s_bl_coef + p0 * BL_TAPS;
            const float *c1  = s_bl_coef + p1 * BL_TAPS;
            const float *xp0 = x + (n0 - (BL_TAPS - 1));
            const float *xp1 = x + (n1 - (BL_TAPS - 1));
            float a0 = 0.0f, a1 = 0.0f;
            for (int j = 0; j < BL_TAPS; j++) {
                a0 += xp0[j] * c0[j];
                a1 += xp1[j] * c1[j];
            }
            float acc = a0 + w * (a1 - a0);
            y[i] = acc;

            if (limit_overshoot) {
                /*
                 * 钳位到「以输出点为中心 ±1 个原始样点」的极值范围内。
                 *
                 * ⚠️ 窗口必须**小于一个平台**，否则钳不住。
                 *    10 MHz @65 MSPS 每周期只有 6.5 点，高/低电平平台各约 3 点；
                 *    之前取 ±3（7 个点）比平台还宽，窗口里同时含了两个电平，
                 *    等于没钳，结果平顶中央出现下凹（吉布斯）。
                 *    取 ±1（3 个点）正好落在平台内：平台处 lo==hi ⇒ 钉死；
                 *    跳变处窗口横跨两个电平 ⇒ 保留重建的过渡。
                 */
                int cc = (int)floorf(t + 0.5f);
                int a = cc - 1, b = cc + 1;
                if (a < 0) a = 0;
                if (b > n - 1) b = n - 1;
                float lo = x[a], hi = x[a];
                for (int k = a; k <= b; k++) {
                    if (x[k] < lo) lo = x[k];
                    if (x[k] > hi) hi = x[k];
                }
                if (y[i] < lo) y[i] = lo;
                if (y[i] > hi) y[i] = hi;
            }
        } else {
            /* 边界兜底：前后各 BL_DELAY 个样点没有完整支撑，
             * 硬算会掉幅度，退回线性插值（这一段本来就看不清）。 */
            int a = (int)floorf(t);
            if (a < 0)        a = 0;
            if (a > n - 2)    a = n - 2;
            const float f = t - (float)a;
            y[i] = x[a] + (x[a + 1] - x[a]) * f;
        }
    }
}
