/*
 * port.c —— Linux 板级移植层
 *
 * 显示：优先 Linux framebuffer（/dev/fb0）；板子上没有 fb 设备时
 *       自动退回 DRM（/dev/dri/card0，见 drm_disp.c）。
 * 输入：evdev 触摸（/dev/input/eventN）。
 * 两条路都不依赖 Rockchip 的 RKADK，所以可以脱离 buildroot
 * 单独编译成一个可执行文件直接丢到板子上跑。
 */
#include "port.h"

#include "lvgl.h"
#include "drm_disp.h"
#include "ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <linux/input.h>

/* =========================================================
 *  显示
 * ========================================================= */

/* WAVE_DRM_CONNECTOR=<id> 可以强制指定 DRM connector，默认自动挑 */
static int drm_connector_pref(void)
{
    const char *s = getenv("WAVE_DRM_CONNECTOR");
    if (!s || !*s) return -1;
    return atoi(s);
}

static int try_fbdev(const char *fb)
{
    lv_display_t *disp = lv_linux_fbdev_create();
    if (!disp) {
        fprintf(stderr, "[port] lv_linux_fbdev_create() failed\n");
        return -1;
    }

    lv_linux_fbdev_set_file(disp, fb);

    printf("[port] display: %s (fbdev)  %dx%d\n", fb,
           (int)lv_display_get_horizontal_resolution(disp),
           (int)lv_display_get_vertical_resolution(disp));
    return 0;
}

/*
 * devpath 传 NULL 时自动探测。
 *   /dev/fb*   → framebuffer
 *   其它路径    → 当作 DRM card 节点
 */
int port_display_init(const char *devpath)
{
    const char *card;

    /* 显式指定了 DRM 节点 */
    if (devpath && strncmp(devpath, "/dev/fb", 7) != 0) {
        return drm_display_init(devpath, drm_connector_pref());
    }

    /* 1) 有 framebuffer 就用 framebuffer */
    {
        const char *fb = devpath ? devpath : "/dev/fb0";
        if (access(fb, F_OK) == 0) return try_fbdev(fb);
    }

    /* 2) 没有 fb（RK3506 这块板子就是这种），退回 DRM */
    card = getenv("WAVE_DRM_CARD");
    if (!card || !*card) card = "/dev/dri/card0";

    if (access(card, F_OK) != 0) {
        fprintf(stderr, "[port] 既没有 %s，也没有 %s\n",
                devpath ? devpath : "/dev/fb0", card);
        return -1;
    }

    printf("[port] 没有 framebuffer 设备，改用 DRM: %s\n", card);
    return drm_display_init(card, drm_connector_pref());
}

/* =========================================================
 *  触摸（evdev 自动发现）
 * ========================================================= */

static char s_touch_path[300];

static int test_bit(const unsigned long *bits, int bit)
{
    return (int)((bits[bit / (int)(8 * sizeof(unsigned long))] >>
                  (bit % (int)(8 * sizeof(unsigned long)))) & 1UL);
}

static int fd_is_touch(int fd)
{
    unsigned long absbits[(ABS_MAX / (int)(8 * sizeof(unsigned long))) + 2];
    unsigned long keybits[(KEY_MAX / (int)(8 * sizeof(unsigned long))) + 2];

    memset(absbits, 0, sizeof(absbits));
    memset(keybits, 0, sizeof(keybits));

    if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(absbits)), absbits) < 0) return 0;
    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits) < 0) return 0;

    /* 多点/单点触摸屏的典型特征 */
    const int has_mt  = test_bit(absbits, ABS_MT_POSITION_X);
    const int has_x   = test_bit(absbits, ABS_X);
    const int has_btn = test_bit(keybits, BTN_TOUCH);

    return (has_mt && has_btn) || (has_x && has_btn);
}

/* 先枚举目录，比硬猜 event0..31 更稳 */
static const char *scan_touch(void)
{
    DIR *d = opendir("/dev/input");
    if (!d) return NULL;

    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (strncmp(ent->d_name, "event", 5) != 0) continue;

        char path[300];
        snprintf(path, sizeof(path), "/dev/input/%s", ent->d_name);

        const int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;

        const int ok = fd_is_touch(fd);
        close(fd);

        if (ok) {
            snprintf(s_touch_path, sizeof(s_touch_path), "%s", path);
            closedir(d);
            return s_touch_path;
        }
    }
    closedir(d);
    return NULL;
}

const char *port_touch_scan(void)
{
    return scan_touch();
}

const char *port_touch_device(void)
{
    return s_touch_path[0] ? s_touch_path : NULL;
}

/*
 * 取触摸设备的原始坐标量程。
 *
 * ⚠️ 为什么要自己取，而不是靠 lv_evdev 内部那套？
 *
 * lv_evdev_create() 里只会用 EVIOCGABS(ABS_X) / EVIOCGABS(ABS_Y) 拿标定范围：
 *
 *     if(ioctl(fd, EVIOCGABS(ABS_X), &absinfo) == 0) {
 *         dsc->min_x = absinfo.minimum;  dsc->max_x = absinfo.maximum;
 *     }
 *
 * 而 GT9xx / goodix 这类「多点 Protocol B」触摸屏**根本不报 ABS_X/ABS_Y**，
 * 只报 ABS_MT_POSITION_X / ABS_MT_POSITION_Y（本机 event0 实测就只有
 * 48/50/53/54/57 这几个码）。于是这两个 ioctl 全部失败，min/max 保持
 * lv_malloc_zeroed 出来的 0，标定函数 _evdev_calibrate() 因为
 * in_min == in_max 直接跳过缩放，把原始值原样透传。
 *
 * 这块 7 寸屏的原始坐标恰好是 X 0~1024、Y 0~600，跟分辨率一模一样，
 * 所以「蒙对了」。但换一块分辨率不同的屏就会静默错位（不报错、只是点不准）。
 * 这里改成按设备真实量程显式标定，做到与分辨率解耦。
 */
static int read_touch_range(const char *dev, int *minx, int *maxx, int *miny, int *maxy)
{
    const int fd = open(dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return -1;

    struct input_absinfo ax, ay;
    memset(&ax, 0, sizeof(ax));
    memset(&ay, 0, sizeof(ay));

    /* 优先多点协议；退而求其次再试单点的 ABS_X/ABS_Y */
    int okx = (ioctl(fd, EVIOCGABS(ABS_MT_POSITION_X), &ax) == 0);
    int oky = (ioctl(fd, EVIOCGABS(ABS_MT_POSITION_Y), &ay) == 0);
    if (!okx && ioctl(fd, EVIOCGABS(ABS_X), &ax) == 0) okx = 1;
    if (!oky && ioctl(fd, EVIOCGABS(ABS_Y), &ay) == 0) oky = 1;
    close(fd);

    if (!okx || !oky) return -1;
    if (ax.maximum <= ax.minimum || ay.maximum <= ay.minimum) return -1;

    *minx = ax.minimum; *maxx = ax.maximum;
    *miny = ay.minimum; *maxy = ay.maximum;
    return 0;
}

/* =========================================================
 *  触摸：自己读 evdev（单点喂 LVGL + 双指捏合缩放）
 *
 * 为什么不用 lv_evdev_create()：
 *   LVGL 9.1 的 lv_evdev 只记「最近一个 ABS_X / ABS_MT_POSITION_X 的值」，
 *   既不认 ABS_MT_SLOT，也没有多触点的概念（见 drivers/evdev/lv_evdev.c）。
 *   它拿不到第二根手指，天然做不了捏合。
 *   而一个 evdev 节点只能有一个读者（事件读走就没了，dup 也没用），
 *   所以必须自己接管：一套读循环同时喂 LVGL 和捏合判定。
 * ========================================================= */

#define MT_SLOTS        5
#define PINCH_MIN       8.0f      /* 两指在某轴上的间距小于这个像素数不作数 */
#define PINCH_HYST      1.06f     /* 缩放死区，滤掉手抖 */
#define PINCH_AXIS_MIN  24.0f     /* 间距长到这么大才判定「这次是横捏还是竖捏」 */
#define PINCH_IDLE_MS   600u      /* 手势中这么久没事件就认为已经结束 */

typedef struct {
    int     fd;
    bool    is_mt;            /* 有 ABS_MT_POSITION_X */
    bool    slot_mode;        /* 有 ABS_MT_SLOT -> Protocol B；否则 Protocol A */
    int     hres, vres;
    int     minx, maxx, miny, maxy;
    int     slot;             /* Protocol B：当前 slot */

    /* Protocol A：一帧之内的临时触点表。
     * f_neg 记录该触点是否带着 TRACKING_ID(-1)——那是「抬起标记」而不是手指。 */
    int     f_idx;
    bool    f_has_xy[MT_SLOTS];
    bool    f_neg[MT_SLOTS];
    int32_t f_x[MT_SLOTS], f_y[MT_SLOTS];

    /* Protocol B 的下压表（按 slot 索引） */
    bool    down[MT_SLOTS];

    /* 最近一帧解析出来的结果：紧凑排列的活触点 */
    int     nd;
    int32_t rx[MT_SLOTS], ry[MT_SLOTS];

    /* 手势状态 */
    bool     pinch_active;
    int      pinch_axis;      /* -1 未定；0 横向 = 时基；1 纵向 = 幅度 */
    float    pinch_prev;
    bool     suppress_click;  /* 捏合后残留的那根手指不许当点击 */
    uint32_t last_evt_ms;
} touch_t;

static touch_t s_touch;

/*
 * 手势诊断开关：WAVE_TDBG=1 启动时，把每一帧双指/缩放事件打到 stderr。
 *
 * 为什么要留这个：触摸手势出问题时（比如"没碰屏幕波形自己缩小了"），
 * 光看界面分不清是**驱动报了假触点**、还是**我们自己算错了比例**。
 * 打开它就能看到每一帧的两点坐标和实际应用的 ratio，一次就能定位。
 * 默认关闭，零开销（只在启动时 getenv 一次）。
 */
static int s_tdbg = -1;
static int tdbg_on(void)
{
    if (s_tdbg < 0) s_tdbg = getenv("WAVE_TDBG") ? 1 : 0;
    return s_tdbg;
}

static int map_axis(int v, int in_min, int in_max, int out)
{
    if (in_max <= in_min) return v;
    if (v < in_min) v = in_min;
    if (v > in_max) v = in_max;
    return (int)(((long)(v - in_min) * out) / (in_max - in_min));
}

static int map_x(const touch_t *t, int v) { return map_axis(v, t->minx, t->maxx, t->hres); }
static int map_y(const touch_t *t, int v) { return map_axis(v, t->miny, t->maxy, t->vres); }

/* Protocol A：把本帧收集到的触点压缩成「活触点」数组。
 * 只有「有坐标」且「没带 TRACKING_ID(-1)」的才算手指，其余一律当抬起标记丢弃。
 * 这样无论是「空帧表示抬起」还是「带 -1 表示抬起」，都不会把手指数量算错。 */
static void frame_commit_a(touch_t *t)
{
    int n = t->f_idx + 1;
    if (n > MT_SLOTS) n = MT_SLOTS;

    int k = 0;
    for (int i = 0; i < n; i++) {
        if (!t->f_has_xy[i] || t->f_neg[i]) continue;
        t->rx[k] = t->f_x[i];
        t->ry[k] = t->f_y[i];
        k++;
    }
    t->nd = k;

    t->f_idx = 0;
    for (int i = 0; i < MT_SLOTS; i++) {
        t->f_has_xy[i] = false;
        t->f_neg[i]    = false;
    }
}

/* Protocol B：按 slot 的下压表压缩（k <= i，原地搬不会覆盖） */
static void frame_commit_b(touch_t *t)
{
    int k = 0;
    for (int i = 0; i < MT_SLOTS; i++) {
        if (!t->down[i]) continue;
        if (k != i) { t->rx[k] = t->rx[i]; t->ry[k] = t->ry[i]; }
        k++;
    }
    t->nd = k;
}

static int32_t s_last_tx, s_last_ty;   /* 上一次触点的屏幕坐标 */

void port_touch_last_point(int32_t *x, int32_t *y)
{
    if(x) *x = s_last_tx;
    if(y) *y = s_last_ty;
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    touch_t *t = (touch_t *)lv_indev_get_user_data(indev);

    data->continue_reading = false;
    data->state            = LV_INDEV_STATE_RELEASED;
    if (!t || t->fd < 0) return;

    /* ---- 1) 把缓冲区里的事件全部吃掉 ---- */
    bool got_ev = false;
    struct input_event ev;
    while (read(t->fd, &ev, sizeof ev) == (ssize_t)sizeof ev) {
        got_ev = true;

        if (ev.type == EV_SYN) {
            if (ev.code == SYN_MT_REPORT) {
                /* 一个触点结束，换下一个 */
                if (!t->slot_mode && t->f_idx < MT_SLOTS - 1) t->f_idx++;
            } else if (ev.code == SYN_REPORT) {
                /* 一帧结束：这一帧有几个活触点就是几根手指 */
                if (t->slot_mode) frame_commit_b(t);
                else              frame_commit_a(t);
            }
            continue;
        }

        if (ev.type == EV_ABS) {
            switch (ev.code) {
            case ABS_MT_SLOT:
                if (ev.value >= 0 && ev.value < MT_SLOTS) t->slot = (int)ev.value;
                break;

            case ABS_MT_TRACKING_ID:
                if (t->slot_mode) {
                    if (t->slot < MT_SLOTS) t->down[t->slot] = (ev.value >= 0);
                } else {
                    /* Protocol A：-1 是抬起标记，这个触点不是手指 */
                    t->f_neg[t->f_idx] = (ev.value < 0);
                }
                break;

            case ABS_MT_POSITION_X:
            case ABS_X:
                if (t->slot_mode) {
                    if (t->slot < MT_SLOTS) t->rx[t->slot] = ev.value;
                } else {
                    t->f_x[t->f_idx] = ev.value;
                    t->f_has_xy[t->f_idx] = true;
                }
                break;

            case ABS_MT_POSITION_Y:
            case ABS_Y:
                if (t->slot_mode) {
                    if (t->slot < MT_SLOTS) t->ry[t->slot] = ev.value;
                } else {
                    t->f_y[t->f_idx] = ev.value;
                    t->f_has_xy[t->f_idx] = true;
                }
                break;

            default: break;
            }
        } else if (ev.type == EV_KEY && ev.code == BTN_TOUCH) {
            if (!t->is_mt) {
                t->down[0] = (ev.value != 0);
            } else if (ev.value == 0 && t->slot_mode) {
                /* Protocol B 的全局抬起；Protocol A 由帧内容表达，不能用这个清，
                 * 否则两指时先抬一根会把另一根也抹掉。 */
                for (int i = 0; i < MT_SLOTS; i++) t->down[i] = false;
            }
        }
    }

    if (got_ev) t->last_evt_ms = lv_tick_get();

    /*
     * 兜底：手势状态只能靠事件推进，一旦驱动漏发抬手帧，状态就会永远卡住
     * （表现就是「捏完之后按钮全点不动」）。这里用「一段时间完全没有事件」
     * 作为「手上已经没有手指了」的旁证，强制复位。
     */
    if (!got_ev && (uint32_t)(lv_tick_get() - t->last_evt_ms) > PINCH_IDLE_MS) {
        t->pinch_active   = false;
        t->pinch_axis     = -1;
        t->pinch_prev     = 0.f;
        t->suppress_click = false;
    }

    const int nd = t->nd;

    /* ---- 2) 双指：横向捏 = 时基，纵向捏 = 幅度 ---- */
    if (nd >= 2) {
        const float x1 = (float)map_x(t, t->rx[0]), y1 = (float)map_y(t, t->ry[0]);
        const float x2 = (float)map_x(t, t->rx[1]), y2 = (float)map_y(t, t->ry[1]);
        const float adx = fabsf(x2 - x1), ady = fabsf(y2 - y1);

        if (!t->pinch_active) {
            t->pinch_active = true;
            t->pinch_axis   = -1;
            t->pinch_prev   = 0.f;
            /* 第二根手指落下时取消第一根可能已经产生的按压，
             * 否则松手会在按钮上触发一次点击（捏合时误切页面）。 */
            lv_indev_wait_release(indev);
        }

        if (t->pinch_axis < 0) {
            /* 刚落下时 dx/dy 都是噪声，等手势长到 24 px 再定轴，
             * 免得竖捏被误判成横捏。定了就不再改，斜着捏也不会来回跳。 */
            if (adx > PINCH_AXIS_MIN || ady > PINCH_AXIS_MIN) {
                t->pinch_axis = (adx >= ady) ? 0 : 1;
                t->pinch_prev = (t->pinch_axis == 0) ? adx : ady;
            }
        } else {
            const float metric = (t->pinch_axis == 0) ? adx : ady;
            if (t->pinch_prev >= PINCH_MIN && metric >= PINCH_MIN) {
                const float ratio = metric / t->pinch_prev;
                if (ratio > PINCH_HYST || ratio < 1.f / PINCH_HYST) {
                    if (tdbg_on())
                        fprintf(stderr, "[tdbg] nd=2 p0=(%.0f,%.0f) p1=(%.0f,%.0f) "
                                        "adx=%.0f ady=%.0f axis=%d ratio=%.3f\n",
                                (double)x1, (double)y1, (double)x2, (double)y2,
                                (double)adx, (double)ady, t->pinch_axis, (double)ratio);
                    if (t->pinch_axis == 0) {
                        const float anchor = ((x1 + x2) * 0.5f) /
                                             (float)(t->hres > 0 ? t->hres : 1);
                        ui_pinch_zoom(ratio, anchor);
                    } else {
                        ui_pinch_gain(ratio);
                    }
                    t->pinch_prev = metric;
                }
            } else {
                t->pinch_prev = metric;
            }
        }

        data->point.x = (int32_t)x1;
        data->point.y = (int32_t)y1;
        data->state   = LV_INDEV_STATE_RELEASED;   /* 双指期间不产生点击 */
        return;
    }

    /*
     * ---- 3) 两指变一指/零指：这次手势就结束了，立刻复位 ----
     *
     * 以前只在 nd==0 时才清 in_pinch。只要漏掉一帧抬手，状态就永远卡住：
     *   · 下一次捏合会拿上次的最终间距当基准 -> 张开放大却变成缩小；
     *   · 单指永远被当成「捏合中的残留手指」-> 底部按钮再也点不动。
     */
    if (t->pinch_active) {
        t->pinch_active   = false;
        t->pinch_axis     = -1;
        t->pinch_prev     = 0.f;
        t->suppress_click = true;      /* 残留那根手指抬起时不算点击 */
    }
    if (nd <= 0) t->suppress_click = false;

    /* ---- 4) 单指：照常给 LVGL ---- */
    if (nd >= 1 && !t->suppress_click) {
        data->point.x = (int32_t)map_x(t, t->rx[0]);
        data->point.y = (int32_t)map_y(t, t->ry[0]);
        data->state   = LV_INDEV_STATE_PRESSED;
        /* 记一份原始触点：lv_indev_get_point() 在事件回调里取到的值不可靠
         * （实测拿到的是控件左上角），需要精确落点的地方直接用这个。 */
        s_last_tx = data->point.x;
        s_last_ty = data->point.y;
    } else {
        data->point.x = (int32_t)(nd >= 1 ? map_x(t, t->rx[0]) : 0);
        data->point.y = (int32_t)(nd >= 1 ? map_y(t, t->ry[0]) : 0);
        data->state   = LV_INDEV_STATE_RELEASED;
    }
}

static int touch_open(const char *dev, int have_range,
                      int minx, int maxx, int miny, int maxy)
{
    memset(&s_touch, 0, sizeof(s_touch));
    s_touch.pinch_axis = -1;          /* -1 = 这次手势还没定轴 */
    s_touch.last_evt_ms = lv_tick_get();
    s_touch.fd = open(dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (s_touch.fd < 0) {
        fprintf(stderr, "[port] open(%s) failed: %s\n", dev, strerror(errno));
        return -1;
    }

    unsigned long absbits[(ABS_MAX / (int)(8 * sizeof(unsigned long))) + 2];
    memset(absbits, 0, sizeof(absbits));
    if (ioctl(s_touch.fd, EVIOCGBIT(EV_ABS, sizeof(absbits)), absbits) >= 0) {
        s_touch.is_mt   = test_bit(absbits, ABS_MT_POSITION_X);
        /* 有 ABS_MT_SLOT 才是 Protocol B。这块板子的 goodix-ts 只报
         * 48/50/53/54/57，没有 SLOT，是 Protocol A —— 每个触点发一段，
         * 靠 SYN_MT_REPORT 分隔、SYN_REPORT 结束一帧。
         * 以前按 B 解析，第二根手指会被写进同一个 slot，永远只有单点。 */
        s_touch.slot_mode = test_bit(absbits, ABS_MT_SLOT);
    }

    if (have_range) {
        s_touch.minx = minx; s_touch.maxx = maxx;
        s_touch.miny = miny; s_touch.maxy = maxy;
    }

    lv_display_t *d = lv_display_get_default();
    s_touch.hres = d ? (int)lv_display_get_horizontal_resolution(d) : 1024;
    s_touch.vres = d ? (int)lv_display_get_vertical_resolution(d) : 600;

    /* 丢掉启动前积压的事件，否则第一下可能落在很久以前的坐标上 */
    struct input_event ev;
    while (read(s_touch.fd, &ev, sizeof ev) == (ssize_t)sizeof ev) { }

    lv_indev_t *indev = lv_indev_create();
    if (!indev) {
        fprintf(stderr, "[port] lv_indev_create() failed\n");
        return -1;
    }
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);
    lv_indev_set_user_data(indev, &s_touch);
    lv_indev_set_display(indev, d);

    printf("[port] touch mode: %s, screen %dx%d\n",
           !s_touch.is_mt ? "single-touch"
                          : (s_touch.slot_mode ? "MT Protocol B (slots)"
                                               : "MT Protocol A (per-contact syn)"),
           s_touch.hres, s_touch.vres);
    return 0;
}

int port_input_init(void)
{
    const char *dev = port_touch_scan();
    if (!dev) {
        fprintf(stderr, "[port] no touch device found under /dev/input\n");
        return -1;
    }

    int minx = 0, maxx = 0, miny = 0, maxy = 0;
    const int have_range = (read_touch_range(dev, &minx, &maxx, &miny, &maxy) == 0);

    touch_open(dev, have_range, minx, maxx, miny, maxy);
    printf("[port] touch: %s\n", dev);
    if (have_range)
        printf("[port] touch 量程: x %d..%d  y %d..%d (已显式标定)\n",
               minx, maxx, miny, maxy);
    else
        fprintf(stderr, "[port] 读不到触摸量程，按 1:1 处理\n");

    return 0;
}
