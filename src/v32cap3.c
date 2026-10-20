/*
 * v32cap3.c —— V32CAP3 帧解析 + UART 传输
 *
 * 分三层，彼此不耦合：
 *   1) crc32 / rd32        纯工具
 *   2) v32_feed()          字节流 -> 校验过的帧（可被任意次任意长度喂）
 *   3) 传输                UART 读线程 或 文件回放
 *
 * 线程模型：
 *   UART 模式起一个读线程，阻塞 read() 后喂 v32_feed()；
 *   成帧后把 s_stage 拷到 s_latest（受 s_lock 保护），并置 s_new。
 *   UI 线程只调 v32_copy_latest()，不会碰串口，也不会被 read 卡住。
 *
 *   ⚠️ s_st 里的计数器是读线程写、UI 读，没有加锁 —— 都是对齐的 32 位
 *      整数，ARM 上不会撕裂，最多看到差一拍的数值。做状态显示足够，
 *      不要拿它做严格统计。
 */
#include "v32cap3.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* 两帧多一点，保证即使一次读进来一帧半也能装下 */
#define V32_RXBUF   (V32_PACKET_BYTES * 2)

static const uint8_t s_magic[V32_MAGIC_LEN] = {
    'V', '3', '2', 'C', 'A', 'P', '3', '!'
};

static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;

static uint8_t     s_buf[V32_RXBUF];
static int         s_len;

static v32_stats_t s_st;
static v32_frame_t s_stage;      /* 解析中，不受锁保护 */
static v32_frame_t s_latest;     /* 受 s_lock 保护 */
static int         s_new;

static int         s_fd = -1;
static pthread_t   s_th;
static int         s_th_running;
static volatile int s_stop;
static bool        s_live;
static char        s_name[128];
static char        s_err[160];
static int         s_baud_actual;   /* 驱动实际生效的波特率 */

static uint32_t    s_last_id;
static uint64_t    s_last_index;
static int         s_seq_rejects;   /* 连续被判「序列不对」的次数，用于自动重新对齐 */
static int         s_have_last;
static int         s_idxdbg = -1;   /* -1 = 还没查环境变量；见 WAVE_IDXDBG */

/* 帧率统计窗口 */
static struct timespec s_fps_t0;
static uint32_t        s_fps_cnt;

/* =========================================================
 * 工具
 * ========================================================= */

static uint32_t s_crc_tab[256];
static int      s_crc_ready;

/* 反射多项式 0xEDB88320，初值 0xFFFFFFFF，末尾异或 0xFFFFFFFF。
 * 等价于 zlib.crc32()。标准检查向量 "123456789" -> 0xCBF43926。 */
static void crc_init(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        s_crc_tab[i] = c;
    }
    s_crc_ready = 1;
}

static uint32_t crc32_calc(const uint8_t *p, int n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (int i = 0; i < n; i++) {
        c = s_crc_tab[(c ^ p[i]) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static uint32_t ms_since(const struct timespec *t0)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint32_t)((now.tv_sec - t0->tv_sec) * 1000
                    + (now.tv_nsec - t0->tv_nsec) / 1000000);
}

/* 每成一帧调一次，累计到 1 秒就刷新一次帧率读数 */
static void fps_tick(void)
{
    const uint32_t dt = ms_since(&s_fps_t0);

    /*
     * 空闲很久之后才来一帧时，别把那段空档算进平均。
     * 否则「停发 20 秒再发一帧」会算成 1/20000 帧每秒，
     * 读数会在界面上趴在 0 附近很久才爬回来。
     */
    if (dt >= 2000u) {
        s_fps_cnt = 1;
        clock_gettime(CLOCK_MONOTONIC, &s_fps_t0);
        s_st.fps_milli = 0;
        return;
    }

    s_fps_cnt++;
    if (dt >= 1000u) {
        /* 1e6 而不是 1e3：字段是「帧/s ×1000」，dt 单位是 ms（本身已带 1e-3） */
        s_st.fps_milli = (uint32_t)(((uint64_t)s_fps_cnt * 1000000u) / dt);
        s_fps_cnt = 0;
        clock_gettime(CLOCK_MONOTONIC, &s_fps_t0);
    }
}

/* =========================================================
 * 解析
 * ========================================================= */

static void v32_feed(const uint8_t *data, int n)
{
    if (!data || n <= 0) return;

    /* 缓冲区装不下就先丢老的字节（正常情况下不会走到） */
    if (s_len + n > V32_RXBUF) {
        int drop = s_len + n - V32_RXBUF;
        if (drop > s_len) drop = s_len;
        memmove(s_buf, s_buf + drop, (size_t)(s_len - drop));
        s_len -= drop;
        s_st.discarded_bytes += (uint32_t)drop;
        if (n > V32_RXBUF - s_len) n = V32_RXBUF - s_len;   /* 仍装不下就截断本次 */
    }
    memcpy(s_buf + s_len, data, (size_t)n);
    s_len += n;

    for (;;) {
        /* ---- 找魔数 ---- */
        int at = -1;
        for (int i = 0; i + V32_MAGIC_LEN <= s_len; i++) {
            if (memcmp(s_buf + i, s_magic, V32_MAGIC_LEN) == 0) { at = i; break; }
        }
        if (at < 0) {
            /* 保留可能是「被拆开的魔数前缀」的尾巴，其余丢弃 */
            int drop = s_len - (V32_MAGIC_LEN - 1);
            if (drop > 0) {
                memmove(s_buf, s_buf + drop, (size_t)(s_len - drop));
                s_len -= drop;
                s_st.discarded_bytes += (uint32_t)drop;
            }
            break;
        }
        if (at > 0) {
            memmove(s_buf, s_buf + at, (size_t)(s_len - at));
            s_len -= at;
            s_st.discarded_bytes += (uint32_t)at;
        }

        if (s_len < V32_HEADER_BYTES) break;   /* 头还没收全 */

        /* ---- 固定头校验 ---- */
        const uint32_t version = rd32(s_buf + 8);
        const uint32_t fid     = rd32(s_buf + 12);
        const uint32_t fs      = rd32(s_buf + 16);
        const uint32_t dec     = rd32(s_buf + 20);
        const uint32_t pts     = rd32(s_buf + 24);
        const uint32_t flags   = rd32(s_buf + 28);
        const uint32_t lo      = rd32(s_buf + 32);
        const uint32_t hi      = rd32(s_buf + 36);

        if (version != V32_VERSION || fs != V32_ADC_FS_HZ || dec != V32_DECIMATION
            || pts != V32_POINTS || flags != 0u) {
            s_st.header_errors++;
            s_st.discarded_bytes++;
            memmove(s_buf, s_buf + 1, (size_t)(s_len - 1));
            s_len--;
            continue;
        }

        if (s_len < V32_PACKET_BYTES) break;   /* 整帧还没收全 */

        /* ---- CRC ---- */
        const uint32_t want = rd32(s_buf + V32_PACKET_BYTES - 4);
        if (crc32_calc(s_buf, V32_PACKET_BYTES - 4) != want) {
            s_st.crc_errors++;
            s_st.discarded_bytes++;
            memmove(s_buf, s_buf + 1, (size_t)(s_len - 1));
            s_len--;
            continue;                          /* 只右移 1 字节，重新找魔数 */
        }

        /* ---- 解样点 ---- */
        const uint8_t *p = s_buf + V32_HEADER_BYTES;
        int bad = 0;
        for (int i = 0; i < V32_POINTS; i++) {
            const int16_t a = (int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
            const int16_t b = (int16_t)(uint16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
            if (a < V32_ADC_MIN || a > V32_ADC_MAX
                || b < V32_ADC_MIN || b > V32_ADC_MAX) { bad = 1; break; }
            s_stage.ch0[i] = a;
            s_stage.ch1[i] = b;
            p += 4;
        }

        const uint64_t start_idx = ((uint64_t)hi << 32) | (uint64_t)lo;

        /* 无论接收与否都要把这帧从缓冲区吃掉 */
        memmove(s_buf, s_buf + V32_PACKET_BYTES, (size_t)(s_len - V32_PACKET_BYTES));
        s_len -= V32_PACKET_BYTES;

        if (bad) {
            s_st.sample_errors++;
            continue;
        }

        /* ---- 序列检查：拒绝重复帧、倒序帧、样点窗口重叠 ---- */
        if (s_have_last) {
            const uint32_t delta = fid - s_last_id;
            if (delta == 0u || delta >= 0x80000000u
                || start_idx < s_last_index + V32_POINTS) {
                s_st.sequence_errors++;

                /*
                 * **自动重新对齐。**
                 *
                 * FPGA 一旦复位/重新加载比特流，帧号 fid 和 start_idx 都会从
                 * 小数字重来。这时 delta = fid - s_last_id 会下溢成一个巨大的
                 * 值（≥ 0x80000000），被当成"倒序帧"；start_idx 也小于上次的值
                 * —— 于是**之后每一帧都被拒绝，永远锁死**，只能重启进程才恢复。
                 * 用户遇到的"FPGA 重来之后就没波形了"就是这个。
                 *
                 * 连续拒绝几次就认为对面换了一台/重启了，丢掉旧基准重新对齐。
                 */
                if (++s_seq_rejects >= 4) {
                    s_have_last   = 0;
                    s_seq_rejects = 0;
                    if (s_idxdbg > 0)
                        fprintf(stderr, "[idx] 连续拒绝，重新对齐序列\n");
                }
                continue;
            }
            s_seq_rejects = 0;
            s_st.missing_frames += delta - 1u;
        }

        /*
         * 帧连续性诊断（WAVE_IDXDBG=1 打开）。
         *
         * 协议头的 start_idx 是这个窗口在 ADC 样点序列里的绝对下标。
         *   d = 4096  ⇒ 相邻帧在时间上严丝合缝，理论上可以拼接成更长的一段
         *   d ≫ 4096 ⇒ 帧与帧之间有空洞（串口来不及传，中间全丢了），
         *              那就**绝对不能**把两帧接起来当成连续信号画。
         * 这个判断决定了「能不能在软件里看更长的时间窗」，所以留着这个开关。
         */
        if (s_idxdbg < 0) s_idxdbg = getenv("WAVE_IDXDBG") ? 1 : 0;
        if (s_idxdbg) {
            fprintf(stderr, "[idx] fid=%u start=%llu d=%lld decim=%u\n",
                    (unsigned)fid, (unsigned long long)start_idx,
                    s_have_last ? (long long)(start_idx - s_last_index) : -1LL,
                    (unsigned)dec);
        }

        s_stage.frame_id    = fid;
        s_stage.start_index = start_idx;
        s_stage.crc32       = want;

        s_last_id    = fid;
        s_last_index = start_idx;
        s_have_last  = 1;
        s_st.frames_ok++;
        s_st.last_frame_id = fid;
        fps_tick();

        pthread_mutex_lock(&s_lock);
        s_latest = s_stage;
        s_new    = 1;
        pthread_mutex_unlock(&s_lock);
    }

    s_st.trailing_bytes = (uint32_t)s_len;
}

/* =========================================================
 * UART 传输
 * ========================================================= */

static void *reader_main(void *arg)
{
    uint8_t tmp[4096];
    (void)arg;

    while (!s_stop) {
        const ssize_t n = read(s_fd, tmp, sizeof tmp);
        if (n > 0) {
            v32_feed(tmp, (int)n);
        } else if (n == 0) {
            usleep(5000);
        } else if (errno == EINTR) {
            continue;
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            usleep(2000);          /* 2 ms 轮询一次，CPU 占用可忽略 */
        } else {
            /* EIO 等：USB 串口拔掉会走到这里，别把线程打死 */
            usleep(20000);
        }
    }
    return NULL;
}

/*
 * 高速率那几档部分 libc 的 <termios.h> 里没定义（glibc 要 _GNU_SOURCE 才给），
 * 但 Linux 的 c_cflag 里速率就是这几个编码（asm-generic/termbits.h），
 * 直接补数值即可，不用依赖宏。ARM 走的就是 asm-generic，编码是通用的。
 */
#ifndef B1000000
#define B1000000 0010010
#endif
#ifndef B1500000
#define B1500000 0010012
#endif
#ifndef B2000000
#define B2000000 0010013
#endif
#ifndef B2500000
#define B2500000 0010014
#endif
#ifndef B3000000
#define B3000000 0010015
#endif
#ifndef B4000000
#define B4000000 0010017
#endif

/* glibc 的 <sys/ioctl.h> 里没有 TIOCM_LOOP（只有内核 header 有），
 * 值是 0x8000；serial8250_set_mctrl() 见到它会置 UARTMCR.LOOPBACK。 */
#ifndef TIOCM_LOOP
#define TIOCM_LOOP 0x8000
#endif

static speed_t baud_const(int baud)
{
    switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 4000000: return B4000000;
    default:      return B921600;
    }
}

/* 把驱动实际生效的波特率读回来 —— 内核如果因为分频凑不出来，会往下取整，
 * 只看自己设的参数会以为跑在 2 M，实际可能不是。 */
static int uart_actual_baud(void)
{
    struct termios t;
    if (tcgetattr(s_fd, &t) != 0) return 0;
    const speed_t sp = cfgetospeed(&t);
    switch (sp) {
    case B9600:    return 9600;
    case B19200:   return 19200;
    case B38400:   return 38400;
    case B57600:   return 57600;
    case B115200:  return 115200;
    case B230400:  return 230400;
    case B460800:  return 460800;
    case B921600:  return 921600;
    case B1000000: return 1000000;
    case B1500000: return 1500000;
    case B2000000: return 2000000;
    case B2500000: return 2500000;
    case B3000000: return 3000000;
    case B4000000: return 4000000;
    default:       return 0;
    }
}

int v32_init_uart(const char *dev, int baud)
{
    if (!s_crc_ready) crc_init();

    s_fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (s_fd < 0) {
        snprintf(s_err, sizeof s_err, "open %s: %s", dev, strerror(errno));
        return -1;
    }

    struct termios tio;
    memset(&tio, 0, sizeof tio);
    if (tcgetattr(s_fd, &tio) != 0) {
        snprintf(s_err, sizeof s_err, "tcgetattr %s: %s", dev, strerror(errno));
        close(s_fd);
        s_fd = -1;
        return -1;
    }

    /* 手写 raw 配置，不用 cfmakeraw（避免 _DEFAULT_SOURCE 的依赖） */
    tio.c_iflag &= (tcflag_t)~(IGNBRK | BRKINT | PARMRK | ISTRIP
                              | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tio.c_oflag &= (tcflag_t)~OPOST;
    tio.c_lflag &= (tcflag_t)~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
    tio.c_cflag &= (tcflag_t)~(PARENB | CSTOPB | CRTSCTS);
    tio.c_cflag &= (tcflag_t)~CSIZE;
    tio.c_cflag |= (tcflag_t)CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    const speed_t sp = baud_const(baud);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    if (tcsetattr(s_fd, TCSANOW, &tio) != 0) {
        snprintf(s_err, sizeof s_err, "tcsetattr %s: %s", dev, strerror(errno));
        close(s_fd);
        s_fd = -1;
        return -1;
    }
    tcflush(s_fd, TCIFLUSH);

    /* 读回真正生效的速率：UART 时钟分频凑不齐时内核会往下取整，
     * 这时候界面上继续写 2000000 就是在骗人。 */
    s_baud_actual = uart_actual_baud();
    if (s_baud_actual && s_baud_actual != baud)
        fprintf(stderr, "[v32] 注意：请求 %d Bd，实际生效 %d Bd（UART 时钟分频限制）\n",
                baud, s_baud_actual);

    s_live = true;
    snprintf(s_name, sizeof s_name, "%s", dev);
    clock_gettime(CLOCK_MONOTONIC, &s_fps_t0);

    s_stop = 0;
    if (pthread_create(&s_th, NULL, reader_main, NULL) != 0) {
        snprintf(s_err, sizeof s_err, "pthread_create 失败");
        close(s_fd);
        s_fd = -1;
        return -1;
    }
    s_th_running = 1;
    return 0;
}

/* =========================================================
 * 文件回放（没有硬件时验证解析链路用）
 * ========================================================= */

int v32_init_file(const char *path)
{
    if (!s_crc_ready) crc_init();

    FILE *f = fopen(path, "rb");
    if (!f) {
        snprintf(s_err, sizeof s_err, "open %s: %s", path, strerror(errno));
        return -1;
    }
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); snprintf(s_err, sizeof s_err, "seek 失败"); return -1; }
    const long sz = ftell(f);
    rewind(f);
    if (sz <= 0) { fclose(f); snprintf(s_err, sizeof s_err, "%s 是空文件", path); return -1; }

    uint8_t *buf = (uint8_t *)malloc((size_t)sz);
    if (!buf) { fclose(f); snprintf(s_err, sizeof s_err, "内存不足"); return -1; }

    const size_t got = fread(buf, 1, (size_t)sz, f);
    fclose(f);

    /* 一次性喂完；解析器内部会自己搜魔数、跳前导垃圾 */
    v32_feed(buf, (int)got);
    free(buf);

    s_live = false;
    snprintf(s_name, sizeof s_name, "%s", path);
    return 0;
}

void v32_shutdown(void)
{
    if (s_th_running) {
        s_stop = 1;
        pthread_join(s_th, NULL);
        s_th_running = 0;
    }
    if (s_fd >= 0) {
        close(s_fd);
        s_fd = -1;
    }
}

int v32_copy_latest(v32_frame_t *out)
{
    int has;
    pthread_mutex_lock(&s_lock);
    has = s_new;
    if (has && out) {
        *out  = s_latest;
        s_new = 0;          /* 只有真的拷走了才清标志 */
    }
    pthread_mutex_unlock(&s_lock);
    return has;
}

const v32_stats_t *v32_get_stats(void) { return &s_st; }
const char        *v32_source_name(void) { return s_name[0] ? s_name : "-"; }
bool               v32_is_live(void) { return s_live; }
const char        *v32_last_error(void) { return s_err; }
int                v32_uart_baud(void) { return s_baud_actual; }

/* =========================================================
 * 环回自检
 * ========================================================= */

/*
 * 打开 UART 内部环回（UARTMCR.LOOPBACK），自己写自己读，逐字节比对。
 *
 * 为什么需要它：换到 2 Mbps 这类高速档时，"termios 设置成功" 不等于
 * "硬件真能跑"——分频凑不出来时要么报错、要么悄悄偏一点，只有真把字节
 * 发出去再收回来才能确认。没有对端设备（FPGA 还没焊/没通电）时也能测。
 *
 * 注意：环回期间 RX 引脚与外部断开，所以这个测试是独占的、跑完就关。
 */
int v32_uart_loopback_test(const char *dev, int baud, int bytes)
{
    if (bytes <= 0) bytes = V32_PACKET_BYTES;

    const int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        snprintf(s_err, sizeof s_err, "open %s: %s", dev, strerror(errno));
        return -1;
    }

    struct termios tio;
    memset(&tio, 0, sizeof tio);
    if (tcgetattr(fd, &tio) != 0) {
        snprintf(s_err, sizeof s_err, "tcgetattr: %s", strerror(errno));
        close(fd);
        return -1;
    }
    tio.c_iflag &= (tcflag_t)~(IGNBRK | BRKINT | PARMRK | ISTRIP
                              | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tio.c_oflag &= (tcflag_t)~OPOST;
    tio.c_lflag &= (tcflag_t)~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
    tio.c_cflag &= (tcflag_t)~(PARENB | CSTOPB | CRTSCTS);
    tio.c_cflag &= (tcflag_t)~CSIZE;
    tio.c_cflag |= (tcflag_t)CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 5;            /* 0.5 s 无数据就当超时 */
    cfsetispeed(&tio, baud_const(baud));
    cfsetospeed(&tio, baud_const(baud));
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        snprintf(s_err, sizeof s_err, "tcsetattr @%d: %s", baud, strerror(errno));
        close(fd);
        return -1;
    }

    int mstat = 0;
    if (ioctl(fd, TIOCMGET, &mstat) != 0) {
        snprintf(s_err, sizeof s_err, "TIOCMGET: %s", strerror(errno));
        close(fd);
        return -1;
    }
    mstat |= TIOCM_LOOP;
    if (ioctl(fd, TIOCMSET, &mstat) != 0) {
        snprintf(s_err, sizeof s_err, "打开环回失败(TIOCM_LOOP): %s", strerror(errno));
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);

    const int CH = 64;
    uint8_t out[64], in[64];
    int total = 0, first_bad = -1, got_bad = -1, timed_out = 0;

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    while (total < bytes) {
        int n = bytes - total;
        if (n > CH) n = CH;
        for (int i = 0; i < n; i++) out[i] = (uint8_t)((total + i) * 7 + 0x5A);

        const int w = (int)write(fd, out, (size_t)n);
        if (w != n) {
            snprintf(s_err, sizeof s_err, "write 只写了 %d/%d: %s", w, n, strerror(errno));
            break;
        }

        int got = 0;
        while (got < n) {
            const int r = (int)read(fd, in + got, (size_t)(n - got));
            if (r <= 0) break;
            got += r;
        }
        if (got != n) { timed_out = 1; break; }

        for (int i = 0; i < n; i++) {
            if (in[i] != out[i]) {
                if (first_bad < 0) { first_bad = total + i; got_bad = in[i]; }
            }
        }
        total += n;
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);
    const double sec = (double)(t1.tv_sec - t0.tv_sec)
                     + (double)(t1.tv_nsec - t0.tv_nsec) / 1e9;

    /* 关掉环回，恢复成外部引脚 */
    mstat &= ~TIOCM_LOOP;
    ioctl(fd, TIOCMSET, &mstat);
    tcflush(fd, TCIOFLUSH);
    close(fd);

    printf("[loopback] %s @ %d Bd：请求 %d 字节，收回 %d 字节，用时 %.3f s（%.1f kB/s）\n",
           dev, baud, bytes, total, sec,
           sec > 0.0 ? (double)total / 1000.0 / sec : 0.0);

    if (timed_out) {
        printf("[loopback] 结果：失败 —— 收到 %d 字节后超时（多半是波特率/时钟不支持）\n", total);
        return -1;
    }
    if (first_bad >= 0) {
        printf("[loopback] 结果：失败 —— 第 %d 字节起不一致（期望 %02X 实收 %02X），"
               "高速率下分频误差过大\n",
               first_bad, (unsigned)((first_bad * 7 + 0x5A) & 0xFF), (unsigned)got_bad);
        return -1;
    }
    printf("[loopback] 结果：通过 —— %d 字节全部逐位一致，%.1f kB/s（理论上限 %.1f kB/s）\n",
           total, (double)total / 1000.0 / sec, (double)baud / 10.0);
    return 0;
}
