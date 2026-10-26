/*
 * wave_source.h —— 波形数据源抽象层
 *
 * 现在有两个实现：
 *   wave_source_v32   V32CAP3 真实帧（UART 或文件回放），4096 点 × 双通道 @ 65 MSPS
 *   wave_source_demo  数学合成的演示信号，512 点单通道 @ 4 kSPS
 *
 * 加新数据源只需要：
 *   1. 照着现有实现写一个文件
 *   2. 在 wave_source_get() 里按需要返回它
 * UI 层和 FFT 层不用动。
 *
 * ⚠️ 每个数据源必须声明自己的 frame_len / sample_rate。
 *    这两个值决定时间轴和频率轴的刻度，写错了屏幕上的读数是错的
 *    （比如拿 65 MSPS 的点数去按 4 kSPS 算，频率会差 1.6 万倍）。
 */
#ifndef WAVE_SOURCE_H
#define WAVE_SOURCE_H

/* 帧缓冲容量上限。取所有数据源里最大的那个 —— V32CAP3 一帧 4096 点。
 * 调用方按这个尺寸开数组就够用，不用管当前源实际多少点。 */
#define WAVE_MAX_FRAME      4096

typedef struct wave_source {
    const char *name;

    /* 本数据源每帧的点数（必须是 2 的幂，FFT 要求） */
    int         frame_len;
    /* 本数据源的采样率 Hz */
    int         sample_rate;

    /* 初始化。返回 0 表示成功。 */
    int  (*init)(void);

    /* 取一帧最新样本写入 buf（容量 len）。返回实际点数，<= 0 表示这一轮没取到。 */
    int  (*read_frame)(float *buf, int len);

    /* 【可选】双通道版本。V32CAP3 一帧本来就带两路，用它避免把同一帧读两次。
     * 返回实际点数，<= 0 表示没数据。不需要就填 NULL。 */
    int  (*read_frame2)(float *ch0, float *ch1, int len);

    /* 【可选】数据源自带频谱时用（上游 DSP 已算好）。
     * 返回 <= 0 时 UI 自动退回「拿时域数据本地做 FFT」。不需要就填 NULL。 */
    int  (*read_spectrum)(float *mag, int bins);

    /* 释放资源，可为 NULL。 */
    void (*deinit)(void);
} wave_source_t;

/* 当前生效的数据源 */
const wave_source_t *wave_source_get(void);
/* 切换数据源（main.c 解析 --source 之后调用） */
void wave_source_select(const wave_source_t *s);

/* 内置假数据源：100 / 300 / 500 Hz 谐波叠加 + 带限噪声 */
extern const wave_source_t wave_source_demo;

/* V32CAP3 真实数据源。
 * 由 main.c 的 --source 参数选择串口或文件，先调 wave_source_v32_set_spec()
 * 再让 wave_source_get() 返回它。 */
/* 非 const：FPGA 抽取倍数变化时 sample_rate 要跟着改（见 v32cap3.c） */
extern wave_source_t wave_source_v32;
/* spec 形如 "uart:/dev/ttyS1" 或 "file:/path/to/frame.bin" */
void wave_source_v32_set_spec(const char *spec);
/* 串口波特率，默认 921600。必须在 init 之前调；init 后改无效。
 * 921600 下一帧 16428 B 要 178 ms（≈5.6 帧/秒），2000000 下 82 ms（≈12 帧/秒）。 */
void wave_source_v32_set_baud(int baud);
/* 当前生效的波特率（已打开的串口返回驱动真实值） */
int  wave_source_v32_get_baud(void);
/* UI-thread cached RAW frame rate, before integer-Hz rounding. 0 if unavailable. */
double wave_source_v32_frame_rate_hz(void);

#endif /* WAVE_SOURCE_H */
