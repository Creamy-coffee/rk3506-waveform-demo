/*
 * bl_selftest.c —— 带限重建的数值自检（跑在编译机 x86 上，不占板子）
 *
 * 用 30.0012 MHz / 65 MSPS 的合成正弦（正是「每周期只有 2.17 个样点」那种最坏情况），
 * 拿 bl_resample() 按 0.5 样点步进重建，再和解析真值比。
 * 编译机上的 gcc 直接编，秒出结果；坐标系和板子完全一样（同一份 bl_interp.c）。
 *
 *   gcc -O2 -o /tmp/bltest tools/_test/bl_selftest.c src/bl_interp.c -Isrc -lm
 */
#include <stdio.h>
#include <math.h>

#include "bl_interp.h"

#define PI_F 3.14159265358979323846f

int main(void)
{
    enum { N = 4096 };
    static float x[N], y[512];

    const float fs = 65000000.0f;
    const float f0 = 30001200.0f;          /* 交接包实测的那个频率 */

    for (int i = 0; i < N; i++)
        x[i] = 1000.0f * sinf(2.0f * PI_F * f0 * (float)i / fs);

    /* 从样点 100 起，每 0.5 个样点重建一个点 —— 相当于放大 20 倍看 30 MHz */
    const int nout = 400;
    bl_resample(x, N, y, nout, 100.0f, 0.5f);

    double se = 0.0;
    float  mx = 0.0f;
    for (int i = 0; i < nout; i++) {
        const float t   = 100.0f + 0.5f * (float)i;
        const float ref = 1000.0f * sinf(2.0f * PI_F * f0 * t / fs);
        const float e   = fabsf(y[i] - ref);
        se += (double)e * e;
        if (e > mx) mx = e;
    }
    printf("30MHz @ 65MSPS 带限重建: RMS %.4f counts, 峰值误差 %.4f (信号幅度 1000)\n",
           sqrt(se / (double)nout), mx);

    /* 顺带量一下单点耗时对应的乘加次数：nout * 129 */
    printf("每帧计算量: %d 点 × 129 抽头 = %d 次乘加\n", nout, nout * 129);

    return 0;
}
