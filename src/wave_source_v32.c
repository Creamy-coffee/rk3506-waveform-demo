/*
 * wave_source_v32.c —— V32CAP3 真实数据源（FPGA 经 UART 送来的 4096 点双通道帧）
 *
 * 这一层只做两件事：
 *   1. 把 v32cap3.c 的「新帧」标志变成「随时可读的最新帧」（读两次拿到同一帧，
 *      因为 UI 的 fetch_scope 会按通道调两次，不能第一次读走了第二次就没数据）
 *   2. 把 int16 ADC 码值转成 float，并做 min/max 抽取
 *
 * 抽取为什么要 min/max 而不是等间隔抽点：
 *   4096 点压到 1024 点画在 740 px 宽的图上，等间隔抽点会把窄脉冲、
 *   毛刺、过零细节整片丢掉，屏幕上看起来「干净」但那是假的。
 *   min/max 每桶保留最早出现的极值和另一个极值，包络不会被抹掉 ——
 *   示波器前端芯片（如 HMCAD15xx 的 peak detect）就是这么干的。
 */
#include "wave_source.h"
#include "v32cap3.h"

#include <stdio.h>
#include <string.h>

#define V32_DEV_DEFAULT   "/dev/ttyS1"
#define V32_BAUD_DEFAULT  921600

static char        s_spec[192];
static int         s_baud = V32_BAUD_DEFAULT;
static v32_frame_t s_cur;
static int         s_have;
static int         s_inited;

void wave_source_v32_set_spec(const char *spec)
{
    if (!spec || !*spec) { s_spec[0] = '\0'; return; }
    snprintf(s_spec, sizeof s_spec, "%s", spec);
}

/*
 * 波特率只能改「还没初始化」的那一次 —— 串口一旦打开，速率就固定了，
 * 运行中改需要关掉重开、读线程、缓冲区全要重来，没有必要。
 */
void wave_source_v32_set_baud(int baud)
{
    if (baud > 0) s_baud = baud;
}

int wave_source_v32_get_baud(void)
{
    const int actual = v32_uart_baud();
    return actual > 0 ? actual : s_baud;
}

double wave_source_v32_frame_rate_hz(void)
{
    return s_have && s_cur.decimation ? (double)V32_ADC_FS_HZ/s_cur.decimation : 0.0;
}

/*
 * nsrc 点 -> ndst 点。ndst >= nsrc 时原样拷贝（不做任何抽取）。
 * ndst 为偶数时每 2 个输出点对应一个桶（先出现的极值在前，保持时间顺序）。
 */
static void decimate_minmax(const int16_t *src, int nsrc, float *dst, int ndst)
{
    if (ndst >= nsrc) {
        for (int i = 0; i < nsrc; i++) dst[i] = (float)src[i];
        return;
    }

    const int buckets = ndst / 2;
    for (int b = 0; b < buckets; b++) {
        int i0 = (int)((long long)b * nsrc / buckets);
        int i1 = (int)((long long)(b + 1) * nsrc / buckets);
        if (i1 <= i0) i1 = i0 + 1;
        if (i1 > nsrc) i1 = nsrc;

        int imn = i0, imx = i0;
        for (int i = i0 + 1; i < i1; i++) {
            if (src[i] < src[imn]) imn = i;
            if (src[i] > src[imx]) imx = i;
        }
        /* 保持时间顺序：先发生的那个先画，否则波形会被画成锯齿 */
        if (imn <= imx) {
            dst[2 * b]     = (float)src[imn];
            dst[2 * b + 1] = (float)src[imx];
        } else {
            dst[2 * b]     = (float)src[imx];
            dst[2 * b + 1] = (float)src[imn];
        }
    }

    /* ndst 是奇数时，最后一个点取末尾桶的中点值 */
    if (ndst & 1) {
        int i0 = (int)((long long)(ndst - 1) * nsrc / ndst);
        if (i0 >= nsrc) i0 = nsrc - 1;
        dst[ndst - 1] = (float)src[i0];
    }
}

static int v32_src_init(void)
{
    const char *spec = s_spec[0] ? s_spec : ("uart:" V32_DEV_DEFAULT);
    int rc;

    if (strncmp(spec, "file:", 5) == 0) {
        rc = v32_init_file(spec + 5);
    } else {
        const char *dev = (strncmp(spec, "uart:", 5) == 0) ? (spec + 5) : spec;
        rc = v32_init_uart(dev, s_baud);
    }

    if (rc != 0) {
        fprintf(stderr, "[v32] 初始化失败 (%s): %s\n", spec, v32_last_error());
        return rc;
    }

    printf("[v32] 数据源 = %s (%s @ %d Bd), RAW缓存上限 %d 点, ADC标称 %.3f MHz（实际点数/抽取率以有效帧头为准）\n",
           v32_source_name(), v32_is_live() ? "实时串口" : "文件回放",
           v32_is_live() ? wave_source_v32_get_baud() : 0,
           V32_POINTS, (double)V32_ADC_FS_HZ / 1e6);

    s_inited = 1;
    return 0;
}

static int v32_src_read2(float *ch0, float *ch1, int len)
{
    if (!s_inited || !ch0 || len <= 0) return 0;

    /* 有新帧才刷新缓存；没有新帧时继续吐上一帧，保证两个通道读到的是同一份数据 */
    v32_frame_t tmp;
    if (v32_copy_latest(&tmp)) {
        s_cur  = tmp;
        s_have = 1;
    }
    if (!s_have) return 0;

    /* 用**本帧实际点数**，不是编译期的 4096 —— FPGA 可以按需送 1024 换取帧率 */
    const int src_n = (s_cur.n > 1 && s_cur.n <= V32_POINTS) ? s_cur.n : V32_POINTS;
    if (len > src_n) len = src_n;
    decimate_minmax(s_cur.ch0, src_n, ch0, len);
    if (ch1) decimate_minmax(s_cur.ch1, src_n, ch1, len);
    return len;
}

static int v32_src_read(float *buf, int len)
{
    return v32_src_read2(buf, NULL, len);
}

static void v32_src_deinit(void)
{
    v32_shutdown();
    s_inited = 0;
}

/* 非 const：抽取倍数 dec>1 时 sample_rate 要改成 fs/dec（见 v32cap3.c） */
wave_source_t wave_source_v32 = {
    .name          = "v32cap3",
    .frame_len     = V32_POINTS,
    .sample_rate   = (int)V32_ADC_FS_HZ,
    .init          = v32_src_init,
    .read_frame    = v32_src_read,
    .read_frame2   = v32_src_read2,
    .read_spectrum = NULL,      /* 帧里没有频谱，由 UI 本地做 FFT */
    .deinit        = v32_src_deinit,
};
