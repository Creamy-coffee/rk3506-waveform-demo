/*
 * v32cap3.h —— V32CAP3 波形帧协议（FPGA -> RK3506）
 *
 * 协议依据：`V32CAP3_Linux_FPGA_帧协议与接收工具_20261017.zip` 里的
 *           README + v32cap3.py，以及 samples/real_frame_000001.bin 实测帧。
 *
 * 线上格式（全部 little-endian，字节偏移从 0 开始）：
 *
 *   偏移     长度     字段
 *   ------------------------------------------------------------------
 *   0        8       magic = "V32CAP3!"  (56 33 32 43 41 50 33 21)
 *   8        4       version            uint32 = 3
 *   12       4       frame_id           uint32，模 2^32 递增
 *   16       4       adc_fs_hz          uint32 = 65000000（标称，未校准）
 *   20       4       decimation         uint32 = 1
 *   24       4       points_per_channel uint32 = 4096
 *   28       4       flags              uint32 = 0
 *   32       4       start_index_low    uint32
 *   36       4       start_index_high   uint32
 *   40       16384   samples            4096 组交错 {CH0 int16, CH1 int16}
 *   16424    4       crc32              覆盖 [0,16424)，zlib 语义
 *   ------------------------------------------------------------------
 *   合计     16428   无帧尾魔数、无换行、无结束符
 *
 * 样点语义：12 位有符号 ADC 码值符号扩展到 int16，范围 −2048..2047。
 *          不是浮点、不是电压、也不是两个 12 位紧凑拼在一个 16 位里。
 *          第 i 组：CH0 在字节 40+4i，CH1 在 42+4i，i = 0..4095。
 *
 * ⚠️ 三个必须记住的约束：
 *   1. **只有 FPGA→Linux 单向**。协议里没有配置命令、没有应答。
 *      要从 Linux 改参考频率/积分时间/触发，得另外加 FPGA RX 命令通道。
 *   2. **帧间有死区**。单帧内 4096 点连续（63.015 µs），但帧与帧之间不连续，
 *      不能拼成连续的 65 MSPS 信号。921600/8N1 传 16428 字节理想下限 ~178 ms，
 *      所以上限约 5.6 帧/秒，实际更慢。
 *   3. 帧里**没有** FFT / 锁相 / AI 字段，只有 RAW 时域数据。
 *
 * 本模块只负责「字节流 -> 校验过的帧」，不含任何 UI 逻辑。
 */
#ifndef V32CAP3_H
#define V32CAP3_H

#include <stdbool.h>
#include <stdint.h>

#include "harmonic_receiver.h"   /* 谐波帧 V32HRM1! 的解包结构体 */

#define V32_MAGIC_LEN        8
#define V32_HEADER_BYTES     40
/*
 * 每帧样点数**不再是固定 4096**。
 *
 * 显示只需要约 740 个点（屏宽），送 4096 点是 5.5 倍过采样，
 * 白白占掉带宽，导致帧率上不去。FPGA 可以按需送 1024 点：
 *   1024 点 → 一帧 4140 B → 921600 下 44.9 ms → **22 帧/秒**
 *   4096 点 → 一帧 16428 B → 178 ms → 5.6 帧/秒
 * 解析器接受 [V32_POINTS_MIN, V32_POINTS] 之间的 2 的幂，
 * 上层按实际帧长自适应，所以两种点数的固件都能用。
 */
#define V32_POINTS           4096            /* 上限（缓冲区按这个开） */
#define V32_POINTS_MIN       256             /* 下限 */
#define V32_PAYLOAD_BYTES    (V32_POINTS * 4)          /* 16384（最大） */
#define V32_PACKET_BYTES     (V32_HEADER_BYTES + V32_PAYLOAD_BYTES + 4)  /* 16428（最大） */
#define V32_PAYLOAD_BYTES    (V32_POINTS * 4)          /* 16384 */
#define V32_PACKET_BYTES     (V32_HEADER_BYTES + V32_PAYLOAD_BYTES + 4)  /* 16428 */

#define V32_VERSION          3u
#define V32_ADC_FS_HZ        65000000u
#define V32_DECIMATION       1u      /* 默认；运行时允许 1..256 的 2 的幂 */
/* 抽取比上限。FPGA 侧权威规定（linux_ab5_receiver_handoff）：
   「V32CAP3 的 decimation 允许双向、逐帧变化，合法值为 1..32768 的 2 的幂」。
   之前写死 256，实测帧 dec=512 会被当头错误整帧丢弃 —— 外在表现就是
   "串口助手有数据、屏幕全是 —"。上限必须按协议来，不能按经验猜。 */
#define V32_DECIMATION_MAX   32768u
#define V32_ADC_MIN          (-2048)
#define V32_ADC_MAX          2047

/* 单帧时间跨度：4096 / 65e6 = 63.015 µs */
#define V32_FRAME_SECONDS    ((float)V32_POINTS / (float)V32_ADC_FS_HZ)

typedef struct {
    uint32_t frame_id;
    uint64_t start_index;     /* 复位后 ADC 域时钟计数，不是 Unix 时间 */
    uint32_t crc32;
    uint32_t flags;           /* 帧头 flags，原样带上来（bit0 预留：payload 是 FPGA 端算的频谱） */
    uint32_t decimation;      /* Same validated RAW frame; not the latest reader-thread setting. */
    int      n;               /* 本帧实际样点数（256..4096） */
    int16_t  ch0[V32_POINTS];
    int16_t  ch1[V32_POINTS];
} v32_frame_t;

typedef struct {
    uint32_t frames_ok;        /* CRC 通过且序列合法的帧数 */
    uint32_t crc_errors;       /* CRC 不符（丢弃后重新找魔数） */
    uint32_t header_errors;    /* 魔数对但头字段不符（version/fs/dec/points/flags） */
    uint32_t sample_errors;    /* 样点越界（不在 −2048..2047） */
    uint32_t sequence_errors;  /* 重复帧号 / 倒序 / 样点窗口重叠 */
    uint32_t missing_frames;   /* 帧号跳号累计 */
    uint32_t eagain_cnt;      /* 非阻塞轮询空转次数（阻塞读应为 0） */
    uint32_t discarded_bytes;  /* 为了找魔数扔掉的字节 */
    uint32_t trailing_bytes;   /* 缓冲区里尚未成帧的尾巴 */
    uint32_t last_frame_id;
    uint32_t fps_milli;        /* 最近 1 秒收到的帧数 ×1000 */
} v32_stats_t;

/* ---- 打开数据源 ----------------------------------------------------
 * 两种模式，互斥：
 *   v32_init_uart("/dev/ttyS1", 921600)  真实串口，起一个读线程
 *   v32_init_file("samples/xxx.bin")     回放：把文件喂一遍解析器后就不再进数据
 * 返回 0 成功，非 0 失败（失败时 v32_last_error() 有原因）。
 */
int  v32_init_uart(const char *dev, int baud);
int  v32_init_file(const char *path);
void v32_shutdown(void);

/* 取最新一帧。返回 1 = 拿到新帧（自上次调用之后有更新），0 = 没有新数据。
 * 调用方给 out 传 NULL 时只查询是否有新帧，不拷贝。 */
int  v32_copy_latest(v32_frame_t *out);

/*
 * 抽取倍数的**反向通道**（主机 -> FPGA）。
 *
 * 命令 2 字节：0xA5, k    其中 dec = 2^k（k = 0..15，即 1..32768；FPGA 20261025 开放）
 * FPGA 侧实现一个 UART 接收状态机即可；收到后切换抽取并把新的 dec
 * 填进后续帧的帧头 decimation 字段（主机就是靠这个字段确认生效的）。
 *
 * 未实现时该函数只是把 2 个字节发出去，对现有单向链路没有任何副作用。
 */
int  v32_send_decimation(int dec);
/* 最近一帧帧头里的 decimation（FPGA 实际在用多少）。没收到帧时返回 0。 */
int  v32_last_decimation(void);

const v32_stats_t *v32_get_stats(void);
const char        *v32_source_name(void);   /* "/dev/ttyS1" 或文件名 */
bool               v32_is_live(void);       /* true=UART，false=文件回放 */
const char        *v32_last_error(void);

/* 驱动实际生效的波特率（tty 打开后从 termios 读回；文件回放返回 0）。
 * 内核凑不出请求的速率时会往下取整，界面上要显示这个值而不是请求值。 */
int                v32_uart_baud(void);

/* UART 环回自检：把 TX 内部接到 RX，写 bytes 个字节再读回逐字节比对。
 * 没有对端设备时用它确认「这个波特率在这块板子上真的跑得动」。
 * 返回 0 = 全部一致；-1 = 失败（原因见 v32_last_error() 或 stdout）。 */
int                v32_uart_loopback_test(const char *dev, int baud, int bytes);

/* 把 int16 ADC 码值转成毫伏。
 *
 * ⚠️ 这个换算**依赖模拟前端量程**，协议里只给码值，没给伏特。
 *
 * 实测标定（不要按数据手册猜，这里已经踩过一次）：
 *   · 交接包实测证据 `30m_100mv_normal_01`：发生器 100 mVpp
 *     → 拟合 Vpp 41.23 码 → 2.426 mV/码
 *   · 板上复核：发生器 1 Vpp → 界面读数 100 mV，正好差 10 倍
 * 两个来源互相印证，说明前端有 10:1 衰减（或 ADC 等效满量程 10 Vpp）：
 *        1000 mV × 10 / 4096 = 2.441 mV/码
 *
 * 以前这里写的是 0.2441（按「AD9238 差分 1 Vpp 满量程」从手册推的），
 * 后果是**所有电压读数都小 10 倍** —— 1 Vpp 显示成 100 mV，
 * 连带着每格毫伏、Vpp 读数、纵向量程全错。
 *
 * 换前端板卡必须重新标定：给一个已知幅度的正弦，用读数反算这一个常数。
 */
#define V32_ADC_MV_PER_LSB   2.441f

/* ============ 锁相帧 V32MEAS!（AB5 proto 12，紧跟在波形帧之后）============
 * 128 字节，独立 CRC32。字段偏移与判定规则见 FPGA 交付包
 * docs/V32_LINUX_LINK_SPEC.md，这里只做接收与复核。
 */
#define V32_MEAS_MAGIC    "V32MEAS!"
#define V32_MEAS_BYTES    128
#define V32_MEAS_CRC_OFF  124
#define V32_MEAS_VERSION  1u

/* status 位（偏移 64） */
#define V32_ST_USABLE       0x00000001u
#define V32_ST_PRESENT      0x00000002u
#define V32_ST_RATIO_VALID  0x00000004u
#define V32_ST_AFC_READY    0x00000008u
#define V32_ST_SAME_SOURCE  0x00000010u
#define V32_ST_SETTLING     0x00000020u
#define V32_ST_CAL_CLIP     0x00000100u
#define V32_ST_ADC_OTR      0x00000200u
#define V32_ST_REQUIRED     0x0000001Fu   /* 低 5 位必须全 1 */
#define V32_ST_FORBIDDEN    0x00000F20u   /* settling/cal_clip/adc_otr/保留 必须全 0 */
#define V32_MAX_AGE_TICKS   65000000u     /* 1.0 s */

typedef struct {
    int      valid;             /* 最近一帧是否新鲜（1.5 s 内收到过） */
    int      usable;            /* 按规格复核：幅相能不能显示 */
    uint32_t seq;               /* 每收到一帧 +1，上层用它判断有没有更新 */
    uint32_t wave_frame_id;     /* 关联的波形帧 */
    uint32_t status;
    uint8_t  qual, flags, afc_state, afc_reason, guard0, guard1, decim_log2;
    uint8_t  fft_valid;
    int64_t  amp0_millicounts;  /* Q12 → 毫码 */
    int64_t  amp1_millicounts;
    int64_t  ratio_milli;       /* Q16 → 千分之一 */
    int64_t  pdiff_millideg;    /* 整周 Q16 → 毫度 */
    int64_t  tracked_hz_x1000;  /* FCW → 毫 Hz（锁相频率，比 FFT 栅格准） */
    int64_t  resid0_hz_x1000, resid1_hz_x1000;
    uint32_t fft_hz0, fft_hz1;
    uint32_t age_ms;            /* result_age 换算成毫秒 */
} v32_meas_view_t;

/* 取锁相读数。返回 1 = 有新鲜数据（能不能显示还要看 usable）。 */
int v32_get_meas(v32_meas_view_t *out);

/* ============ 频谱帧 V32SPEC!（FPGA 端 FFT 结果，三帧组里的第三帧）============
 * 44 B 头 + bins×u8 + 4 B CRC。频点 k 的频率 = k × fs_eff_hz / nfft，
 * 幅度 dBFS ≈ byte × scale_dbfs / 255 − scale_dbfs（满刻度 2048 码 = 0 dBFS）。
 * 当前固件：nfft=4096、bins=2048、window=0(Hann)、scale_dbfs=120、整帧 2096 B。
 */
#define V32_SPEC_MAGIC    "V32SPEC!"
#define V32_SPEC_HDR      44
#define V32_SPEC_MAX_BINS 2048

typedef struct {
    int      valid;              /* 新鲜（1.5 s 内收到过） */
    uint32_t seq;
    uint32_t wave_frame_id;
    uint32_t nfft;
    uint32_t bins;
    uint32_t fs_eff_hz;
    uint32_t window;             /* 0=Hann 1=Blackman 2=FlatTop（FPGA 实际用的） */
    uint32_t scale_dbfs;
    uint32_t got;                /* 实际存下来的 bin 数 */
    uint8_t  data[V32_SPEC_MAX_BINS];
} v32_spec_view_t;

int v32_get_spec(v32_spec_view_t *out);
/* dBFS 换算（整数运算，返回 -12000..0 的"百分之一 dB"） */
int v32_spec_dbfs_centi(int byte, uint32_t scale_dbfs);

/* ============ 解调 I/Q 帧 V32IQ01!（模式 1..7）============ */
#define V32_IQ_MAGIC      "V32IQ01!"
#define V32_IQ_HDR        40
#define V32_IQ_MAX_POINTS 256

typedef struct {
    int      valid;
    uint32_t seq;
    uint32_t frame_id;
    uint32_t fs_hz;
    uint32_t decim;
    uint32_t points;
    uint32_t flags;              /* bit0 = FIFO 丢样 */
    uint32_t cfg_id;
    int16_t  i[V32_IQ_MAX_POINTS];
    int16_t  q[V32_IQ_MAX_POINTS];
} v32_iq_view_t;

int v32_get_iq(v32_iq_view_t *out);

/* ============ 新锁相测量帧 V32DDC3!（协作包 20261024，72 B）============
 *
 * 与 V32MEAS!（AB5 proto 12，128 B）是**两个不同的帧**：
 *   · V32MEAS!  现行固件在发，字段多、带 usable 判定，与波形帧强关联；
 *   · V32DDC3!  20261024 协作包新增（字段对齐 SPI 0x10..0x1B），
 *               魔数/布局为冻结前候选 —— 新固件上线后由它接管锁相读数。
 * 布局（全部 LE，CRC32 覆盖 [0,68)）：
 *   0  magic "V32DDC3!"   8  version=3   12 frame_id   16 adc_fs_hz
 *   20 config_seq（配置提交序号，与 AA5 COMMIT 对齐；不是波形帧号！）
 *   24 fcw                28 ch0_i  32 ch0_q  36 ch1_i  40 ch1_q  (i32, counts×4096)
 *   44 ch0_amp  48 ch1_amp (u32, Q12)  52 ch0_phase  56 ch1_phase (i32, 整圈 65536)
 *   60 timestamp(65MHz 计数,~66 s 回绕)  64 flags_word([9:8]=phase_valid,[7:0]=ddc_flags)
 *   68 crc32
 * 换算：幅度码 = amp_q12/4096；相位° = phase_signed*360/65536。
 */
#define V32_DDC_MAGIC    "V32DDC3!"
#define V32_DDC_BYTES    72
#define V32_DDC_VERSION  3u

typedef struct {
    int      valid;               /* 1.5 s 内收到过 */
    uint32_t seq;
    uint32_t frame_id;            /* ==0 表示当前配置尚未产生完整结果 */
    uint32_t adc_fs_hz;
    uint32_t config_seq;
    uint32_t fcw;
    int64_t  ref_hz_x1000;        /* fcw × fs / 2^32，毫 Hz */
    int32_t  ch0_i_q12, ch0_q_q12, ch1_i_q12, ch1_q_q12;
    int64_t  ch0_amp_mc, ch1_amp_mc;        /* 毫码（amp_q12×1000/4096） */
    int64_t  ch0_phase_millideg, ch1_phase_millideg;
    int64_t  pdiff_millideg;      /* (ch1-ch0) 环形差，±180° */
    uint32_t timestamp;
    uint8_t  ddc_flags;           /* [0]adc_gap [1]cfg_illegal [2]busy_drop */
    uint8_t  phase_valid;         /* bit0=CH0 bit1=CH1 */
} v32_ddc_view_t;

int v32_get_ddc(v32_ddc_view_t *out);

/* ============ 调制分析帧 V32ANA1!（交付包 603695b2 v2，20261025）============
 * 48 B 头 + 256×u32 样点 + CRC32 = 1076 B。样点按 kind 解释：
 *   kind0 (QPSK/BPSK): 低16=I(i16)，高16=Q(i16) —— 原始 I/Q，非恢复星座
 *   kind1 (AM/OOK):    低24 无符号 Q8 模长（值/256=DDC 码值）；bit31=无效
 *   kind2 (FM/2FSK):   低24 是 16 位环绕相位差的符号扩展；Hz=v*65e6/(R*65536)
 *   kind3 (PM):        低24 相位符号扩展；度=v*360/65536，[-180,180)
 * flags≠0 整帧只作诊断；cfg_id/mode/channel/R 变化清分析历史。
 * mode 沿用 A5 E0：1AM 2FM 3FSK 4QPSK 5OOK 6BPSK 7PM。
 */
#define V32_ANA_MAGIC     "V32ANA1!"
#define V32_ANA_BYTES     1076
#define V32_ANA_VERSION   1u
#define V32_ANA_POINTS    256

typedef struct {
    int      valid;            /* 1 s 内收到过 */
    uint32_t seq;
    uint32_t frame_id;         /* 配置内帧号 */
    uint32_t cfg_id;           /* 配置编号（重配变化，复位可复用） */
    uint32_t mode;             /* 1..7 */
    uint32_t channel;          /* 0/1 */
    uint32_t kind;             /* 0..3 */
    uint32_t decim;            /* 总抽取 R（8192/4096/2048） */
    uint32_t flags;            /* bit0 结果FIFO丢样 bit2 计算器丢样 */
    uint32_t fcw;              /* DDC 中心 FCW */
    float    center_hz;        /* fcw×65e6/2^32 */
    int      yvalid[V32_ANA_POINTS];  /* 样点有效位（bit31/相位跳变） */
    float    y[V32_ANA_POINTS];       /* kind 解释后的值（Hz/度/模长/I） */
    float    q[V32_ANA_POINTS];       /* 仅 kind0 的 Q 分量 */
    int      nvalid;           /* 有效样点数 */
} v32_ana_view_t;

int v32_get_ana(v32_ana_view_t *out);

/* RAW 时基焦点：A5 E5 f，f=0/1/2 = CH0/CH1/自动（不改分析通道，不递增 cfg_id） */
int v32_cmd_focus(int f);

/* ============ 谐波帧 V32HRM1!（谐波候选 16556a03，交付包 20261028，216 B）============
 *
 * 与 V32CAP3/MEAS/SPEC/ANA 是**并列的另一种帧**，走同一个串口、同一个读线程，
 * 由 v32_feed 的魔数扫描分发到这里；本模块不新开读者、不建线程。
 *
 * 布局（全部小端，CRC32 覆盖 [0,212)，与 zlib.crc32 同语义）：
 *   0   char[8] "V32HRM1!"    8  version(1|2)   12 length=216
 *   16  result_id   20 cfg_id   24 adc_fs   28 nfft   32 reference_fcw
 *   36  decimation  40 age_ticks  44 flags   48 channel  52 window
 *   56  valid_mask  60 noise_mask  64 v2固定0/v1的THD Q24  68 dropped_frames
 *   72  band_power[5] u64   112 noise_power[5] u64   152 signal_x6[5] u64
 *   192 peak_bin[5] u32     212 crc32
 *
 * 三个必须记住的点（都是对方文档明确划的红线）：
 *   1. **v2 线上 thd_q24 字段恒为 0**，必须用 harmonic_receiver.c 里的整数
 *      开方自己算；直接显示这个 0 就是"0% 失真"的假读数。
 *   2. **HARM 的 cfg_id / result_id 与 ANA 独立**，不能拿 ANA 的 cfg 变化
 *      去结束谐波的"切换中"状态。
 *   3. 有限二至五阶 THD，不是 THD+N / SINAD / ADC 性能认证。
 */
#define V32_HRM_MAGIC     "V32HRM1!"
#define V32_HRM_BYTES     216

/* 取最近一包谐波结果。
 * out 为 NULL 时只查有没有收到过。返回 1 = 有可读结果（是否可显示还要看
 * out->thd_valid / thd_below_noise / 年龄）。*age_ms 是"距第一次收到这个
 * result_id 过去了多久"——重复包不会把它刷新回 0，这是协议要求的。*/
int v32_get_harm(v32_harmonic_measurement *out, uint32_t *age_ms);

/* 谐波配置（A5 F0..F3）。fcw = round(f_ref*2^32/65e6)。
 * 顺序固定：先 F3 00 关，再 F0 ch、F1 高16位、F2 低16位提交，最后 F3 01 开。
 * 与 ANA 的 E0..E5 配置彼此独立，不改 DDC 制式/通道/速率/cfg。*/
int v32_cmd_harm_config(uint32_t channel, uint32_t fcw);
/* 只关不开：A5 F3 00 */
int v32_cmd_harm_off(void);
/* 累计发出/收到的谐波包数（诊断用，界面上可显示"未启用/无数据"） */
uint32_t v32_harm_rx_count(void);

/* 测试专用：把字节直接喂给解析器（等价于读线程收到一块数据）。
 * 主机端在没有板子/串口时用它验证碎片拼接、错 CRC 重同步。 */
void v32_test_feed_bytes(const uint8_t *data, int n);

/* ============ 下行控制帧 V32AA5C!（协作包 20261024，只发不答）============
 * 24 B 头（magic/ver/cmd/seq/plen）+ payload + CRC32。设备不回 ACK；
 * 配置类命令写暂存寄存器，必须跟一条 COMMIT 才生效 —— 下面的 set_* 帮手
 * 内部已经自动跟 COMMIT，调用方不用（也不要）再单独发。
 */
#define V32_AA5_MAGIC       "V32AA5C!"
#define V32_AA5_CMD_SET_WINDOW      1u
#define V32_AA5_CMD_SET_MODULATION  2u
#define V32_AA5_CMD_SET_REF_FREQ    3u
#define V32_AA5_CMD_SET_LOCKIN_DECIM 4u
#define V32_AA5_CMD_SET_RUN_MODE    5u
#define V32_AA5_CMD_COMMIT          6u

/* 窗函数枚举（协议 id；冻结前候选，与 FPGA 实现对齐后再定稿） */
enum {
    V32_WIN_RECT = 0, V32_WIN_HANN = 1, V32_WIN_HAMMING = 2,
    V32_WIN_BLACKMAN = 3, V32_WIN_FLATTOP = 4, V32_WIN_KAISER = 5,
    V32_WIN_NUTTALL = 6, V32_WIN_COUNT = 7
};
/* 调制/解调模式枚举（协议 id） */
enum {
    V32_MOD_AUTO = 0, V32_MOD_AM = 1, V32_MOD_FM = 2, V32_MOD_PM = 3,
    V32_MOD_LOCKIN_NCO = 4, V32_MOD_RAW_BASEBAND = 5, V32_MOD_COUNT = 6
};

/* 返回 0 = 两帧（配置+COMMIT）都写进串口；-1 = 串口不可用/写失败 */
int v32_aa5_set_window(uint32_t window_type_id);
int v32_aa5_set_modulation(uint32_t modulation_mode_id);
int v32_aa5_set_ref_freq(uint32_t fcw);
int v32_aa5_commit(void);
/* 窗函数协议 id -> 显示名（"HANN"…，未知 id 返回 "WIN?"） */
const char *v32_aa5_win_name(unsigned id);

/* ============ 控制命令（Linux → FPGA，原始十六进制字节）============
 * 全部无 CRC / 无 ACK。命令之间保持连续发送，FPGA 对不完整命令会超时丢弃。
 * 详见交付包 PROTOCOL.md。
 */
#define V32_CMD_MODE_SCOPE  0
#define V32_CMD_MODE_AM     1
#define V32_CMD_MODE_FM     2
#define V32_CMD_MODE_2FSK   3
#define V32_CMD_MODE_QPSK   4
#define V32_CMD_MODE_OOK    5
#define V32_CMD_MODE_BPSK   6
#define V32_CMD_MODE_PM     7

int v32_cmd_mode(int mode);            /* A5 E0 mm  0=示波 1..7=调制 */
int v32_cmd_channel(int ch);           /* A5 E3 cc  cc=0/1 */
int v32_cmd_rate(int rr);              /* A5 E4 rr  rr=0/1/2 -> 8192/4096/2048 */
int v32_cmd_dec(int k);                /* A5 kk     k=0..15，普通波形抽取 2^k */
int v32_cmd_auto_dec(void);            /* A5 FF     恢复普通波形自动抽取 */
int v32_cmd_fcw(uint32_t fcw);         /* A5 E1 hi16 + A5 E2 lo16（分两帧发） */
uint32_t v32_fcw_from_hz(double hz);   /* round(hz * 2^32 / 65e6) */
int v32_pop_ana(v32_ana_view_t *out); /* queued frame; out->valid reports receive freshness */
int v32_cmd_apply(int mode,int channel,int rate,uint32_t fcw,int focus); /* one write, no implicit OFF */

#endif /* V32CAP3_H */

