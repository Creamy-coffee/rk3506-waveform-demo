/*
 * main.c —— V32 信号分析仪界面 demo 入口
 *
 *   ./waveform_demo [设备节点] [--view <页>] [--autoswitch]
 *
 * 不指定设备节点时自动探测：先试 /dev/fb0，没有就退回 /dev/dri/card0(DRM)。
 * 触摸自动扫描 /dev/input/event*。
 * 若没找到触摸设备，会自动在 5 个页面之间定时轮播，方便无人值守演示。
 *
 * 排错 / 自动化用的子命令：
 *   --drm-info          打印 DRM 拓扑后退出
 *   --selftest          不碰显示，只跑数据源+FFT
 *   --tap X Y [MS]      注入一次触摸点击（验证按钮）
 *   --pinch CX CY D0 D1 [N] [MS]   注入一次双指捏合（验证缩放）
 *   --snapshot SEC F    跑一会儿截图后退出
 *   --snap-file F       收到 SIGUSR1 时截图
 *
 * 环境变量：WAVE_SRC=real 让 SCOPE/FFT 改走 wave_source 抽象层（真实数据链路）
 */
#include "lvgl.h"
#include "ui.h"
#include "port.h"
#include "drm_disp.h"
#include "wave_source.h"
#include "v32cap3.h"       /* v32_last_error()：数据源打不开时报原因 */
#include "fft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <linux/input.h>

/*
 * 界面调度周期。旧100ms把显示限制在10Hz；33ms允许及时展示新到帧，
 * 不改变ADC采样率或UART实际到帧率。到帧数与DRM翻页数分别统计。
 * 如果CPU跟不上，会自动被主循环节流，
 * 不会卡死 —— 只是帧率降下来。
 */
/*
 * 数据刷新周期：45ms 时每秒 22 次全量 plot 重绘，叠加五种帧的接收
 * 线程后 CPU 撑不住（实测调制分析开着就卡）。100ms = 10fps，
 * 与数据帧率同量级，渲染负载直接减半，肉眼看不出差别。
 */
#define REFRESH_MS      33      /* UI scheduling; acquisition rate remains measured separately. */
#define AUTOSWITCH_MS   8000    /* 无触摸时的自动轮播周期 */

static volatile sig_atomic_t s_quit = 0;
static volatile sig_atomic_t s_snap_req = 0;
static volatile sig_atomic_t s_dump_req = 0;
static const char *s_snap_path = NULL;      /* SIGUSR1 截图目标 */

static void on_signal(int sig)
{
    (void)sig;
    s_quit = 1;
}

static void on_sigusr1(int sig)
{
    (void)sig;
    s_snap_req = 1;
}
static void on_sigusr2(int sig){(void)sig;s_dump_req=1;}

static void do_snapshot(const char *path)
{
    if (!path || !*path) return;
    if (drm_display_snapshot_ppm(path) != 0)
        fprintf(stderr, "[app] 截图失败: %s\n", path);
}

static uint32_t tick_cb(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ULL
                    + (uint64_t)ts.tv_nsec / 1000000ULL);
}

/* 单调时钟（纳秒），用来量单帧耗时 */
static uint64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void usage(const char *argv0)
{
    printf("usage: %s [device] [--view <page>] [--autoswitch] [--fps [SEC]] [--tap X Y [MS]]\n", argv0);
    printf("  device            显式指定显示节点；不填则自动探测\n");
    printf("                    /dev/fbX -> framebuffer, 其它 -> DRM card (如 /dev/dri/card0)\n");
    printf("  --autoswitch      强制定时轮播 5 个页面，不管有没有触摸\n");
    printf("  --view <page>     启动时直接进入指定页，可选：\n");
    printf("                      scope (wave) | fft | lock | ai | system\n");
    printf("  --drm-info        只打印 DRM 拓扑信息后退出（排错用）\n");
    printf("  --selftest        不碰显示，只跑数据源+FFT 并打印结果（排错用）\n");
    printf("  --fps [SEC]       实测帧率：先按正常节奏跑 SEC 秒，再强制每轮重绘跑\n");
    printf("                    SEC 秒，分别报出「实际运行帧率」和「渲染上限」（默认 5 秒）\n");
    printf("  --tap X Y [MS]    往触摸设备注入一次点击(X,Y 为屏幕像素坐标,\n");
    printf("                    MS 为按住时长,默认 150ms)，注入完就退出\n");
    printf("                    —— 没有真手指时用它验证按钮点击是否真的通\n");
    printf("  --pinch CX CY D0 D1 [N] [MS]\n");
    printf("                    注入一次双指捏合：两指以 (CX,CY) 为中心，水平间距由 D0\n");
    printf("                    变到 D1（D1>D0 张开），N 步，每步 MS 毫秒（默认 12 步 / 45ms）。\n");
    printf("                    横向捏 = 时基（µs/div），注入完就退出\n");
    printf("  --pinch-v CX CY D0 D1 [N] [MS]\n");
    printf("                    同上，但两指沿**竖直**方向分开：纵向捏 = 幅度（mV/div）\n");
    printf("  --freeze [T]      冻结动画并把波形相位定死为 T（等价参考稿的 tick=T/0.02,\n");
    printf("                    默认 4.0 即 tick=200）。截图比对时必须用，否则每次\n");
    printf("                    截图相位都不同，两张图叠不上\n");
    printf("  --dump-layout     打印整棵控件树的真实坐标后退出。用于排查\n");
    printf("                    「控件不见了 / 文字被截了」这类 flex 尺寸问题\n");
    printf("  --snapshot SEC F  跑 SEC 秒后把上屏画面存成 PPM(P6) 到 F，然后退出\n");
    printf("  --snap-file F     运行中收到 SIGUSR1 时截图存到 F\n");
    printf("  --source <spec>   数据源，默认 demo：\n");
    printf("                      demo                    数学合成的演示波形\n");
    printf("                      uart[:/dev/ttyS1]       V32CAP3 真帧（FPGA 经 UART）\n");
    printf("                      file:<path>             回放一帧 .bin（没有硬件时验证解析）\n");
    printf("  --baud <rate>     串口波特率，默认 921600。可选 921600 / 2000000 等：\n");
    printf("                      921600   一帧 16428 B 要 178 ms  -> 约 5.6 帧/秒\n");
    printf("                      2000000  一帧 16428 B 要  82 ms  -> 约 12 帧/秒\n");
    printf("                    两端和 FPGA 必须一致，否则收到的全是乱码\n");
    printf("  --uart-loopback [BYTES]\n");
    printf("                    UART 内部环回自检（TX 直连 RX），自己写自己读逐字节比对。\n");
    printf("                    没有对端设备时用它确认某个波特率真能跑；不影响其它实例，\n");
    printf("                    但测试期间该串口的 RX 引脚会断开，跑完自动恢复\n");
    printf("\n环境变量: WAVE_SRC=real    SCOPE/FFT 改走 wave_source 抽象层\n");
    printf("          WAVE_DRM_CARD=/dev/dri/card0  WAVE_DRM_CONNECTOR=<id>\n");
}

/* =========================================================
 *  --tap：往 evdev 注入一次「按下 -> 保持 -> 抬起」
 * ========================================================= */

static int write_ev(int fd, uint16_t type, uint16_t code, int32_t value)
{
    struct input_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.type  = type;
    ev.code  = code;
    ev.value = value;

    const ssize_t n = write(fd, &ev, sizeof(ev));
    if (n != (ssize_t)sizeof(ev)) {
        fprintf(stderr, "[tap] write(type=%u code=%u) 失败: %s\n",
                (unsigned)type, (unsigned)code, strerror(errno));
        return -1;
    }
    return 0;
}

/*
 * 走 evdev 的 write() 注入路径（内核 input_inject_event，evemu 就是这个原理）。
 * ⚠️ 内核会**静默丢弃设备未声明的 ABS 码**：这块板子的 goodix-ts 没有
 *    ABS_MT_SLOT（能力位只有 48/50/53/54/57），所以 ABS_MT_SLOT 写了也白写，
 *    必须按 Protocol A 组织事件：每个触点「TRACKING_ID + 坐标 + SYN_MT_REPORT」，
 *    整帧以 SYN_REPORT 结束；抬起就是发一个不含触点的空 SYN_REPORT。
 *
 *     （这也是之前捏合没反应的根因：注入按 B 写，两个触点都被并到同一个 slot，
 *       驱动侧永远只看到一根手指。）
 *
 * hold_ms 不能太短：LVGL 的 indev 读取周期是 LV_DEF_REFR_PERIOD(33ms)，
 * 若「按下」和「抬起」落在同一次 read 循环里被一次读走，LVGL 只会看到
 * 「在 (x,y) 处抬起」而没看到按下，LV_EVENT_CLICKED 就不会触发。
 */
static int inject_tap(const char *dev, int x, int y, int hold_ms)
{
    const int fd = open(dev, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        fprintf(stderr, "[tap] open %s: %s\n", dev, strerror(errno));
        return 1;
    }

    int rc = 0;
    rc |= write_ev(fd, EV_ABS, ABS_MT_TRACKING_ID, 0);
    rc |= write_ev(fd, EV_ABS, ABS_MT_POSITION_X, x);
    rc |= write_ev(fd, EV_ABS, ABS_MT_POSITION_Y, y);
    rc |= write_ev(fd, EV_SYN, SYN_MT_REPORT, 0);
    rc |= write_ev(fd, EV_KEY, BTN_TOUCH, 1);
    rc |= write_ev(fd, EV_SYN, SYN_REPORT, 0);
    if (rc != 0) { close(fd); return 1; }

    printf("[tap] %s  down (%d,%d)\n", dev, x, y);
    fflush(stdout);
    usleep((useconds_t)hold_ms * 1000);

    rc = 0;
    rc |= write_ev(fd, EV_KEY, BTN_TOUCH, 0);
    rc |= write_ev(fd, EV_SYN, SYN_REPORT, 0);
    close(fd);

    if (rc != 0) return 1;
    printf("[tap] up  (按住 %d ms)\n", hold_ms);
    return 0;
}

/*
 * --pinch：注入一次双指捏合（两指沿水平方向从 d0 分开到 d1）。
 * 用来在真机上验证「两指捏放缩放波形」，不必真的用手指。
 *
 * 按 Protocol A 组织：同一帧里放两个触点，各以 SYN_MT_REPORT 结尾，
 * 最后一个 SYN_REPORT 提交；抬起发空帧。详见 inject_tap 上面的说明。
 *
 * step_ms 要大于 LVGL 的 indev 轮询周期（约 33 ms），否则中间位置会被
 * 一次 read 全部吃掉，只剩首尾两点 —— 还能缩放，但看不出是连续捏合。
 */
static int inject_pinch(const char *dev, int cx, int cy, int d0, int d1,
                        int steps, int step_ms, int axis)
{
    if (steps < 1) steps = 1;
    if (step_ms < 1) step_ms = 1;

    const int fd = open(dev, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        fprintf(stderr, "[pinch] open %s: %s\n", dev, strerror(errno));
        return 1;
    }

    /*
     * 发一帧两触点。axis=0 两指沿水平方向分开（改时基），
     * axis=1 沿垂直方向分开（改幅度），和真机上「横捏/竖捏」的判定一致。
     */
    #define PINCH_PX(d, sgn) (axis ? (cx) : (cx) + (sgn) * (d) / 2)
    #define PINCH_PY(d, sgn) (axis ? (cy) + (sgn) * (d) / 2 : (cy))
    #define PINCH_FRAME(d)                                                    \
        do {                                                                  \
            rc |= write_ev(fd, EV_ABS, ABS_MT_TRACKING_ID, 10);                \
            rc |= write_ev(fd, EV_ABS, ABS_MT_POSITION_X, PINCH_PX(d, -1));    \
            rc |= write_ev(fd, EV_ABS, ABS_MT_POSITION_Y, PINCH_PY(d, -1));    \
            rc |= write_ev(fd, EV_SYN, SYN_MT_REPORT, 0);                      \
            rc |= write_ev(fd, EV_ABS, ABS_MT_TRACKING_ID, 11);                \
            rc |= write_ev(fd, EV_ABS, ABS_MT_POSITION_X, PINCH_PX(d, 1));     \
            rc |= write_ev(fd, EV_ABS, ABS_MT_POSITION_Y, PINCH_PY(d, 1));     \
            rc |= write_ev(fd, EV_SYN, SYN_MT_REPORT, 0);                      \
            rc |= write_ev(fd, EV_KEY, BTN_TOUCH, 1);                          \
            rc |= write_ev(fd, EV_SYN, SYN_REPORT, 0);                         \
        } while (0)

    int rc = 0;
    PINCH_FRAME(d0);
    if (rc != 0) { close(fd); return 1; }

    printf("[pinch] 落指 %s d=%d 中心(%d,%d) -> d=%d，%d 步 × %d ms\n",
           axis ? "纵向（幅度）" : "横向（时基）",
           d0, cx, cy, d1, steps, step_ms);
    fflush(stdout);
    usleep((useconds_t)(step_ms * 2) * 1000);   /* 先稳住，别和首帧挤在一起 */

    for (int s = 1; s <= steps; s++) {
        const int d = d0 + (int)((long)(d1 - d0) * s / steps);
        rc = 0;
        PINCH_FRAME(d);
        if (rc != 0) { close(fd); return 1; }
        usleep((useconds_t)step_ms * 1000);
    }

    rc = 0;
    rc |= write_ev(fd, EV_KEY, BTN_TOUCH, 0);
    rc |= write_ev(fd, EV_SYN, SYN_REPORT, 0);   /* 空帧 = 全部抬起 */
    close(fd);

    #undef PINCH_FRAME

    if (rc != 0) return 1;
    printf("[pinch] 抬起\n");
    return 0;
}

/*
 * --selftest：不碰显示，只跑「数据源 -> FFT」，把频谱关键点打出来。
 * 用来把「DSP 链路」和「LVGL 显示」两段分开定位问题。
 */
static int run_selftest(void)
{
    const wave_source_t *src = wave_source_get();
    if (!src || !src->read_frame) {
        fprintf(stderr, "[selftest] 没有可用数据源\n");
        return 1;
    }
    if (src->init) src->init();

    static float frame[WAVE_MAX_FRAME];
    static float mag[FFT_MAX_N / 2];
    const int fn = src->frame_len;

    printf("source=%s  n=%d  fs=%d  df=%.3f Hz\n",
           src->name ? src->name : "?", fn, src->sample_rate,
           (double)src->sample_rate / (double)fn);

    for (int f = 0; f < 3; f++) {
        int n = src->read_frame(frame, fn);
        if (n <= 0) { fprintf(stderr, "[selftest] read_frame 返回 %d\n", n); return 1; }

        float mn = frame[0], mx = frame[0];
        for (int i = 1; i < n; i++) {
            if (frame[i] < mn) mn = frame[i];
            if (frame[i] > mx) mx = frame[i];
        }

        fft_magnitude_spectrum(frame, fn, mag);

        int pk = 2;
        for (int i = 3; i < fn / 2; i++)
            if (mag[i] > mag[pk]) pk = i;

        const double df = (double)src->sample_rate / (double)fn;
        printf("frame %d: 时域 min=%+.4f max=%+.4f | mag[13]=%.6f mag[38]=%.6f "
               "mag[64]=%.6f | 峰值 bin=%d (%.1f Hz) mag=%.6f\n",
               f, (double)mn, (double)mx,
               (double)mag[13], (double)mag[38], (double)mag[64],
               pk, pk * df, (double)mag[pk]);
    }
    return 0;
}

/*
 * --fps SEC：实测帧率。
 *
 * 「帧率」其实是两个不同的数，所以分两段测：
 *
 *   阶段 1「实际运行」—— 就是正常跑的样子：UI每REFRESH_MS调度一次，
 *                        LVGL 显示定时器 33ms 一次，只在画面真的变了才翻页。
 *                        量出来的才是你眼睛看到的刷新节奏。
 *
 *   阶段 2「渲染上限」—— 每轮都强制重绘，并直接调 lv_refr_now() 绕开 LVGL 的
 *                        33ms 显示定时器，量这条「软件渲染 + DRM 翻页」流水线
 *                        最快能跑到多少。这个数通常被面板 vblank 卡住。
 *
 * 顺便报「平均单次翻页耗时」：如果它接近 1/面板刷新率，说明瓶颈是等 vblank
 * （也就是已经跑满面板了），而不是软件渲染慢 —— 这两件事的优化方向完全不同。
 */
static int run_fps_probe(int sec)
{
    const uint32_t ms = (uint32_t)sec * 1000u;
    uint32_t f0, f1, f2;
    uint64_t n0, n1, n2;

    /* ---------------- 阶段 1：正常节奏 ---------------- */
    drm_display_flip_stats(&f0, &n0);
    const uint32_t t0 = tick_cb();
    uint32_t last_data = t0;
    const uint32_t raw0=v32_get_stats()->frames_ok;

    while (tick_cb() - t0 < ms) {
        lv_timer_handler();

        const uint32_t now = tick_cb();
        if (now - last_data >= REFRESH_MS) {
            ui_tick();
            last_data = now;
        }
        usleep(3000);
    }

    drm_display_flip_stats(&f1, &n1);
    const uint32_t t1 = tick_cb();
    const uint32_t raw1=v32_get_stats()->frames_ok;

    /* ---------------- 阶段 2：强制每轮重绘 ---------------- */
    /*
     * 这一轮刻意**不调 lv_timer_handler()**：LVGL 的显示定时器自己也可能触发
     * 刷新，混进来会让「一次迭代 = 一帧」这个对应关系不成立，统计就不干净了。
     * 只跑 5 秒，不让定时器转没影响（程序紧接着就退出）。
     */
    uint64_t refr_ns = 0;       /* lv_refr_now 累计耗时 = 软件渲染 + DRM 翻页 */
    uint32_t iters   = 0;
    while (tick_cb() - t1 < ms) {
        ui_tick();                      /* 换一帧数据，同时让 chart 失效 */
        lv_obj_invalidate(lv_screen_active()); /* Explicit full redraw, even if setters detect unchanged data. */
        const uint64_t r0 = mono_ns();
        lv_refr_now(NULL);              /* 立刻渲染 + 翻页，不等 33ms 显示定时器 */
        refr_ns += mono_ns() - r0;
        iters++;
    }

    drm_display_flip_stats(&f2, &n2);
    const uint32_t t2 = tick_cb();
    const uint32_t raw2=v32_get_stats()->frames_ok;

    const uint32_t d1 = f1 - f0, e1 = t1 - t0;
    const uint32_t d2 = f2 - f1, e2 = t2 - t1;

    const double fps_run = e1 ? (double)d1 * 1000.0 / (double)e1 : 0.0;
    const double fps_max = e2 ? (double)d2 * 1000.0 / (double)e2 : 0.0;

    /* 单帧拆解（阶段 2 每次迭代就是一帧） */
    const uint64_t flip_ns_2 = n2 - n1;
    const double per_frame  = iters ? (double)refr_ns / (double)iters / 1e6 : 0.0;
    const double per_flip   = iters ? (double)flip_ns_2 / (double)iters / 1e6 : 0.0;
    const double per_render = per_frame - per_flip;

    int W = 0, H = 0, HZ = 0;
    drm_display_mode_get(&W, &H, &HZ);

    printf("\n================ 帧率实测（每段 %d 秒）================\n", sec);
    printf("面板模式       : %dx%d @%dHz   硬件上限 %d fps\n", W, H, HZ, HZ);

    printf("1) 实际运行    : %6.1f fps   (%u 帧 / %u ms)\n", fps_run, d1, e1);
    printf("RAW CRC-good arrival: normal %.2f frames/s, forced %.2f frames/s (not display FPS)\n",
           e1?(raw1-raw0)*1000.0/e1:0.,e2?(raw2-raw1)*1000.0/e2:0.);
    printf("                  ↑ 数据每 %d ms 刷一次、画面变了才翻页，这是肉眼看到的节奏\n",
           REFRESH_MS);

    printf("2) 强制重绘上限: %6.1f fps   (%u 帧 / %u ms)\n", fps_max, d2, e2);
    printf("                  ↑ 每轮都重绘并立刻翻页（绕开显示定时器），流水线极限\n");

    printf("每帧耗时拆解   : %6.2f ms = 软件渲染 %.2f ms + DRM 翻页 %.2f ms\n",
           per_frame, per_render, per_flip);

    if (HZ > 0 && per_frame > 0.0) {
        const double vblank_ms = 1000.0 / (double)HZ;
        printf("                 面板一帧 %.2f ms；", vblank_ms);
        if (per_render > per_flip)
            printf("瓶颈在**软件渲染**（占 %.0f%%），降点数/缩小刷新区域还有余量\n",
                   per_render * 100.0 / per_frame);
        else
            printf("瓶颈在**等 vblank**（翻页占 %.0f%%），已经跑满面板了\n",
                   per_flip * 100.0 / per_frame);
    }
    return 0;
}

int main(int argc, char **argv)
{
    const char *devpath = NULL;
    int force_autoswitch = 0;
    int start_page = 0;                 /* 0 = SCOPE */
    const char *snap_file = NULL;       /* --snapshot 的输出文件 */
    int         snap_after = -1;        /* --snapshot 的延时秒数 */
    int         tap_req  = 0;           /* --tap */
    int         tap_x = 0, tap_y = 0, tap_hold = 150;
    int         pinch_req = 0;          /* --pinch */
    int         pinch_cx = 512, pinch_cy = 300, pinch_d0 = 120, pinch_d1 = 360;
    int         pinch_steps = 12, pinch_step_ms = 45;   /* 12 步 × 45 ms 约 0.5 s */
    int         pinch_axis = 0;         /* 0 = 横向捏（时基），1 = 纵向捏（幅度） */
    int         fps_sec = 0;            /* --fps：实测帧率，跑完就退出 */
    int         dump_req = 0;           /* --dump-layout */
    int         freeze_req = 0;         /* --freeze */
    float       freeze_t = 4.0f;        /*   默认等价参考稿 tick=200 */
    const char *src_spec = NULL;        /* --source */
    int         src_baud = 0;           /* --baud，0 = 用默认值 */
    int         loopback_req = 0;       /* --uart-loopback [字节数] */
    int         loopback_bytes = 0;
    int         selftest_req = 0;       /* --selftest（推迟到解析完 --source 之后跑） */

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--autoswitch") == 0) {
            force_autoswitch = 1;
        } else if (strcmp(argv[i], "--view") == 0 && i + 1 < argc) {
            const char *v = argv[++i];
            const int p = ui_page_from_name(v);
            if (p < 0) {
                fprintf(stderr, "[app] 未知视图 '%s'（scope|fft|lock|ai|mod|system）\n", v);
                return 2;
            }
            start_page = p;
        } else if (strcmp(argv[i], "--freeze") == 0) {
            freeze_req = 1;
            /* 可选的相位值：后面跟着纯数字/小数点就当它是 */
            if (i + 1 < argc && (isdigit((unsigned char)argv[i + 1][0]) || argv[i + 1][0] == '.'))
                freeze_t = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--dump-layout") == 0) {
            dump_req = 1;
        } else if (strcmp(argv[i], "--drm-info") == 0) {
            const char *c = getenv("WAVE_DRM_CARD");
            return drm_display_info(c && *c ? c : "/dev/dri/card0") == 0 ? 0 : 1;
        } else if (strcmp(argv[i], "--selftest") == 0) {
            selftest_req = 1;
        } else if (strcmp(argv[i], "--source") == 0 && i + 1 < argc) {
            src_spec = argv[++i];
        } else if (strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            src_baud = atoi(argv[++i]);
            if (src_baud <= 0) {
                fprintf(stderr, "[app] --baud 参数无效：%s\n", argv[i]);
                return 2;
            }
        } else if (strcmp(argv[i], "--uart-loopback") == 0) {
            loopback_req = 1;
            if (i + 1 < argc && isdigit((unsigned char)argv[i + 1][0]))
                loopback_bytes = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--tap") == 0 && i + 2 < argc) {
            tap_x = atoi(argv[++i]);
            tap_y = atoi(argv[++i]);
            /* 可选的按住时长：后面若还跟着纯数字就当它 */
            if (i + 1 < argc && isdigit((unsigned char)argv[i + 1][0]))
                tap_hold = atoi(argv[++i]);
            tap_req = 1;
        } else if (strcmp(argv[i], "--pinch") == 0 && i + 4 < argc) {
            pinch_cx = atoi(argv[++i]);
            pinch_cy = atoi(argv[++i]);
            pinch_d0 = atoi(argv[++i]);
            pinch_d1 = atoi(argv[++i]);
            if (i + 1 < argc && isdigit((unsigned char)argv[i + 1][0]))
                pinch_steps = atoi(argv[++i]);
            if (i + 1 < argc && isdigit((unsigned char)argv[i + 1][0]))
                pinch_step_ms = atoi(argv[++i]);
            pinch_req = 1;
        } else if (strcmp(argv[i], "--pinch-v") == 0 && i + 4 < argc) {
            pinch_cx = atoi(argv[++i]);
            pinch_cy = atoi(argv[++i]);
            pinch_d0 = atoi(argv[++i]);
            pinch_d1 = atoi(argv[++i]);
            if (i + 1 < argc && isdigit((unsigned char)argv[i + 1][0]))
                pinch_steps = atoi(argv[++i]);
            if (i + 1 < argc && isdigit((unsigned char)argv[i + 1][0]))
                pinch_step_ms = atoi(argv[++i]);
            pinch_axis = 1;
            pinch_req  = 1;
        } else if (strcmp(argv[i], "--fps") == 0) {
            fps_sec = 5;
            if (i + 1 < argc && isdigit((unsigned char)argv[i + 1][0]))
                fps_sec = atoi(argv[++i]);
            if (fps_sec < 1) fps_sec = 1;
        } else if (strcmp(argv[i], "--snapshot") == 0 && i + 2 < argc) {
            snap_after  = atoi(argv[++i]);
            snap_file   = argv[++i];
            s_snap_path = snap_file;
        } else if (strcmp(argv[i], "--snap-file") == 0 && i + 1 < argc) {
            s_snap_path = argv[++i];
        } else {
            devpath = argv[i];
        }
    }

    /*
     * --source 要在 init / selftest / UI 之前定下来，这三处都靠 wave_source_get() 取源。
     * "uart" 是简写，等价于 "uart:/dev/ttyS1" —— 那正是 JP4 第 10 脚
     * (UART1_RX) 对应的 ttyS1。
     */
    if (src_spec && strcmp(src_spec, "demo") != 0) {
        const char *spec = src_spec;
        if (strcmp(spec, "uart") == 0) spec = "uart:/dev/ttyS1";
        wave_source_v32_set_spec(spec);
        /* --baud 必须赶在数据源 init 之前生效，串口一打开速率就定死了 */
        if (src_baud > 0) wave_source_v32_set_baud(src_baud);
        wave_source_select(&wave_source_v32);
    }

    if (selftest_req) return run_selftest();

    /*
     * --uart-loopback：独占串口做内部环回自检，必须在数据源 init 之前返回，
     * 否则读线程和测试会抢同一个 fd。
     */
    if (loopback_req) {
        const char *dev = "/dev/ttyS1";
        if (src_spec && strncmp(src_spec, "uart:", 5) == 0 && src_spec[5]) dev = src_spec + 5;
        return v32_uart_loopback_test(dev, src_baud > 0 ? src_baud : 921600,
                                      loopback_bytes) == 0 ? 0 : 1;
    }

    /*
     * --tap 只注入事件，不碰显示：板子上已经有一个实例占着 DRM 了，
     * 这里再开一次显示会抢不到卡。所以放在 lv_init() 之前直接返回。
     */
    if (tap_req) {
        const char *tdev = port_touch_scan();
        if (!tdev) {
            fprintf(stderr, "[tap] 没在 /dev/input 下找到触摸设备\n");
            return 1;
        }
        return inject_tap(tdev, tap_x, tap_y, tap_hold);
    }

    /*
     * --pinch：注入一次双指捏合（两指水平分开/靠拢）。
     * 同样是只写 evdev、不碰显示，可以在主实例跑着的时候执行。
     */
    if (pinch_req) {
        const char *tdev = port_touch_scan();
        if (!tdev) {
            fprintf(stderr, "[pinch] 没在 /dev/input 下找到触摸设备\n");
            return 1;
        }
        return inject_pinch(tdev, pinch_cx, pinch_cy, pinch_d0, pinch_d1,
                            pinch_steps, pinch_step_ms, pinch_axis);
    }

    /*
     * 板子上是 `nohup ./waveform_demo > /tmp/app.log 2>&1 &` 这样跑的，
     * stdout 一重定向就变成「全缓冲」，几条启动日志（[port] touch ...）会一直
     * 卡在 4KB 缓冲里不落盘，排查时 tail 日志什么都看不到，很容易误判成
     * 「程序没跑起来 / 日志没打」。改成行缓冲，日志实时可看。
     */
    setvbuf(stdout, NULL, _IOLBF, 0);

    printf("=== waveform demo ===\n");
    lv_init();
    lv_tick_set_cb(tick_cb);

    if (port_display_init(devpath) != 0) {
        fprintf(stderr, "[app] display init failed\n");
        return 1;
    }

    const int has_touch = (port_input_init() == 0);
    if (!has_touch) {
        printf("[app] no touch -> auto-switch every %d s\n", AUTOSWITCH_MS / 1000);
    }

    const wave_source_t *src = wave_source_get();
    if (src && src->init && src->init() != 0) {
        /*
         * 串口打不开（设备节点不存在 / 权限不对）时**不再退回演示源**。
         * 演示曲线在真实设备上会被当成「有信号」，反而误导；
         * 保持 v32 数据源、只是拿不到帧，界面就会画一条零线，
         * 同时 SYSTEM 页的 /dev/ttyS1、丢包计数能说明问题出在哪。
         */
        fprintf(stderr, "[app] 数据源初始化失败（%s），界面将显示零线\n", v32_last_error());
    }
    printf("[app] wave source: %s, fs=%d Hz, frame=%d\n",
           (src && src->name) ? src->name : "?",
           (src && src->sample_rate) ? src->sample_rate : 0,
           (src && src->frame_len) ? src->frame_len : 0);

    ui_create();
    if (start_page) ui_set_page(start_page);

    /*
     * --freeze：把波形相位定死。
     * 必须在 ui_set_page 之后、第一次真正上屏之前调，这样连第一帧就是冻结相位，
     * 截出来的图和 tools/render_ref.py 渲染的参考稿能逐像素对上。
     */
    if (freeze_req) {
        printf("[app] 冻结动画，相位 t=%.3f（参考稿 tick=%.0f）\n",
               (double)freeze_t, (double)(freeze_t / 0.02f));
        ui_freeze(freeze_t);
    }

    /*
     * --dump-layout：把控件树真实坐标打出来。
     * 先空跑几轮 lv_timer_handler 让布局/尺寸解析彻底收敛，再 dump ——
     * 否则可能拿到 LV_SIZE_CONTENT 还没解析完的中间态，比不 dump 更误导。
     */
    if (dump_req) {
        for (int i = 0; i < 10; i++) {
            lv_timer_handler();
            usleep(5000);
        }
        ui_dump_layout();
        return 0;
    }

    /* --fps：界面已建好，接着就跑实测，跑完退出（不进正常主循环） */
    if (fps_sec > 0) return run_fps_probe(fps_sec);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGUSR1, on_sigusr1);        /* kill -USR1 <pid> 即截图 */
    signal(SIGUSR2, on_sigusr2);        /* Queue a layout dump on the UI thread. */

    uint32_t last_data = tick_cb();
    uint32_t last_sw   = last_data;
    const uint32_t t0  = last_data;

    while (!s_quit) {
        lv_timer_handler();

        const uint32_t now = tick_cb();

        if (now - last_data >= REFRESH_MS) {
            ui_tick();
            last_data = now;
        }

        if ((force_autoswitch || !has_touch) && (now - last_sw >= AUTOSWITCH_MS)) {
            ui_set_page((ui_get_page() + 1) % ui_page_count());
            last_sw = now;
        }

        /* 运行中收到 SIGUSR1 -> 立刻截图 */
        if(s_dump_req){s_dump_req=0;lv_obj_update_layout(lv_screen_active());ui_dump_layout();fflush(stdout);}
        if (s_snap_req) {
            s_snap_req = 0;
            do_snapshot(s_snap_path ? s_snap_path : "/tmp/wave_frame.ppm");
        }

        /* --snapshot 定时截图后收工 */
        if (snap_after >= 0 && now - t0 >= (uint32_t)snap_after * 1000u) {
            do_snapshot(snap_file);
            break;
        }

        usleep(3000);
    }

    /* Drain an optional diagnostic capture and join the existing UART reader. */
    const wave_source_t *finished_source=wave_source_get();
    if(finished_source && finished_source->deinit)finished_source->deinit();
    printf("\n[app] exit\n");
    return 0;
}
