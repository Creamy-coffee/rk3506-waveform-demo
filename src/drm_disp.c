/*
 * drm_disp.c —— 内核 DRM UAPI 直驱显示后端（无 libdrm 依赖）
 *
 * 流程（全走 legacy API；atomic-only 驱动上内核会自动用
 * drm_atomic_helper_set_config 兼容层接管）：
 *
 *   open(/dev/dri/card0)
 *     → DRM_IOCTL_SET_MASTER          抢占 master（官方 lv_demo 已被杀掉）
 *     → MODE_GETRESOURCES             拿 connector / crtc 列表
 *     → MODE_GETCONNECTOR             找 CONNECTED 且有 mode 的那一条
 *     → MODE_GETENCODER               由 encoder 反查 crtc_id
 *     → MODE_CREATE_DUMB x2           XRGB8888 dumb buffer
 *     → MODE_MAP_DUMB + mmap          CPU 可直接写的线性内存
 *     → MODE_ADDFB2 x2                变成驱动认得的 framebuffer
 *     → MODE_SETCRTC                  点亮面板，并作为「翻页」手段
 *
 * 两块 dumb buffer 直接作为 LVGL 的 DIRECT 绘制缓冲：
 *   LVGL 往「背面」那块物理内存里画 → flush 回调里 SETCRTC 换 fb_id → 翻页。
 *   全程零拷贝。
 */
#define _GNU_SOURCE

#include "drm_disp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_fourcc.h>

/* 这几个枚举属于内核内部头，UAPI 不导出，自己补 */
#ifndef DRM_MODE_CONNECTED
#define DRM_MODE_CONNECTED          1
#define DRM_MODE_DISCONNECTED       2
#define DRM_MODE_UNKNOWNCONNECTION  3
#endif
#ifndef DRM_DISPLAY_MODE_LEN
#define DRM_DISPLAY_MODE_LEN        32
#endif
/* atomic 能力不是 DRM_CAP_*，而是 client cap，用 SET_CLIENT_CAP 探测 */
#ifndef DRM_CLIENT_CAP_ATOMIC
#define DRM_CLIENT_CAP_ATOMIC       1
#endif

#define DBUF_MAX     2
#define MAX_CONNS    32
#define MAX_MODES    32
#define MAX_ENCS     32
#define MAX_CRTCS    32

typedef struct {
    uint32_t handle;        /* gem handle      */
    uint32_t fb_id;         /* framebuffer id  */
    uint32_t pitch;         /* 行字节数         */
    uint64_t size;          /* 总字节数         */
    void    *map;           /* CPU 映射地址     */
} dbuf_t;

typedef struct {
    uint32_t conn_id;
    uint32_t conn_type;
    uint32_t conn_type_id;
    uint32_t crtc_id;
    uint32_t mm_w, mm_h;
    struct drm_mode_modeinfo mode;
} probe_t;

static struct {
    int      fd;
    uint32_t conn_id;
    uint32_t crtc_id;
    struct drm_mode_modeinfo mode;
    int      width, height;
    dbuf_t   buf[DBUF_MAX];
    int      nbuf;
    int      front;         /* 当前正在扫描输出的缓冲下标 */
    int      ready;

    /* ---- 帧率统计（只读诊断用，不影响显示路径）---- */
    uint32_t flips;         /* 成功翻页次数，即真正上屏的帧数 */
    uint64_t flip_ns;       /* 累计花在 SETCRTC 里的时间 */
} g = { .fd = -1 };

/* 单调时钟，单位纳秒 */
static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* =========================================================
 *  底层小工具
 * ========================================================= */

static const char *conn_type_name(uint32_t t)
{
    switch (t) {
    case DRM_MODE_CONNECTOR_VGA:        return "VGA";
    case DRM_MODE_CONNECTOR_DVII:       return "DVI-I";
    case DRM_MODE_CONNECTOR_DVID:       return "DVI-D";
    case DRM_MODE_CONNECTOR_DVIA:       return "DVI-A";
    case DRM_MODE_CONNECTOR_Composite:  return "Composite";
    case DRM_MODE_CONNECTOR_LVDS:       return "LVDS";
    case DRM_MODE_CONNECTOR_Component:  return "Component";
    case DRM_MODE_CONNECTOR_DisplayPort:return "DP";
    case DRM_MODE_CONNECTOR_HDMIA:      return "HDMI-A";
    case DRM_MODE_CONNECTOR_HDMIB:      return "HDMI-B";
    case DRM_MODE_CONNECTOR_TV:         return "TV";
    case DRM_MODE_CONNECTOR_eDP:        return "eDP";
    case DRM_MODE_CONNECTOR_VIRTUAL:    return "Virtual";
    case DRM_MODE_CONNECTOR_DSI:        return "DSI";
    case DRM_MODE_CONNECTOR_DPI:        return "DPI";
    default:                            return "?";
    }
}

static const char *conn_status_name(uint32_t s)
{
    switch (s) {
    case DRM_MODE_CONNECTED:         return "connected";
    case DRM_MODE_DISCONNECTED:      return "disconnected";
    case DRM_MODE_UNKNOWNCONNECTION: return "unknown";
    default:                         return "?";
    }
}

/* 拉一条 connector 的完整信息（含 mode 列表 + encoder 列表），两段式 ioctl */
static int get_connector(int fd, uint32_t id,
                         struct drm_mode_get_connector *out,
                         uint32_t *encs, uint32_t *nencs,
                         struct drm_mode_modeinfo *modes, uint32_t *nmodes)
{
    struct drm_mode_get_connector c;

    memset(&c, 0, sizeof c);
    c.connector_id = id;
    if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &c) < 0) return -1;

    uint32_t nm = c.count_modes;    if (nm > *nmodes) nm = *nmodes;
    uint32_t ne = c.count_encoders; if (ne > *nencs) ne = *nencs;

    memset(&c, 0, sizeof c);
    c.connector_id   = id;
    c.count_modes    = nm;
    c.modes_ptr      = (uint64_t)(uintptr_t)modes;
    c.count_encoders = ne;
    c.encoders_ptr   = (uint64_t)(uintptr_t)encs;

    if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &c) < 0) return -1;

    *out    = c;
    *nmodes = c.count_modes;
    *nencs  = c.count_encoders;
    return 0;
}

/* 由 connector 反查/挑选一个可用的 crtc_id */
static uint32_t pick_crtc(int fd,
                          const struct drm_mode_get_connector *c,
                          const uint32_t *encs, uint32_t nencs,
                          const uint32_t *crtcs, uint32_t ncrtcs)
{
    struct drm_mode_get_encoder e;

    /* 1) 已经绑在某个 crtc 上就直接沿用（最稳，不改动既有拓扑） */
    if (c->encoder_id) {
        memset(&e, 0, sizeof e);
        e.encoder_id = c->encoder_id;
        if (ioctl(fd, DRM_IOCTL_MODE_GETENCODER, &e) == 0 && e.crtc_id)
            return e.crtc_id;
    }

    /* 2) 遍历这条 connector 的 encoder，找已绑定的 crtc */
    for (uint32_t i = 0; i < nencs; i++) {
        memset(&e, 0, sizeof e);
        e.encoder_id = encs[i];
        if (ioctl(fd, DRM_IOCTL_MODE_GETENCODER, &e) < 0) continue;
        if (e.crtc_id) return e.crtc_id;
    }

    /* 3) 还没绑定就用 possible_crtcs 位图挑第一个能接的 */
    for (uint32_t i = 0; i < nencs; i++) {
        memset(&e, 0, sizeof e);
        e.encoder_id = encs[i];
        if (ioctl(fd, DRM_IOCTL_MODE_GETENCODER, &e) < 0) continue;
        for (uint32_t j = 0; j < ncrtcs && j < 32; j++) {
            if (e.possible_crtcs & (1u << j)) return crtcs[j];
        }
    }

    return ncrtcs ? crtcs[0] : 0;
}

/* 一段式取资源列表；返回实际拿到的 connector / crtc 数 */
static int get_resources(int fd, uint32_t *conns, uint32_t *nc,
                         uint32_t *crtcs, uint32_t *nr,
                         struct drm_mode_card_res *info)
{
    struct drm_mode_card_res res;

    memset(&res, 0, sizeof res);
    if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) < 0) {
        fprintf(stderr, "[drm] MODE_GETRESOURCES: %s\n", strerror(errno));
        return -1;
    }

    uint32_t a = res.count_connectors; if (a > MAX_CONNS) a = MAX_CONNS;
    uint32_t b = res.count_crtcs;      if (b > MAX_CRTCS) b = MAX_CRTCS;

    memset(&res, 0, sizeof res);
    res.count_connectors   = a; res.connector_id_ptr = (uint64_t)(uintptr_t)conns;
    res.count_crtcs        = b; res.crtc_id_ptr      = (uint64_t)(uintptr_t)crtcs;
    if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) < 0) {
        fprintf(stderr, "[drm] MODE_GETRESOURCES(2): %s\n", strerror(errno));
        return -1;
    }

    *nc = res.count_connectors;
    *nr = res.count_crtcs;
    if (info) *info = res;
    return 0;
}

/*
 * 探测 DRM 拓扑，挑出要用的 connector / crtc / mode。
 * 打分规则（分高者胜）：
 *   +100  用户显式指定的 connector_id
 *   +20   DSI 类型（这块板子的 7 寸 MIPI 屏）
 *   +5    该 connector 带 PREFERRED 标志的 mode
 */
static int probe_fd(int fd, int want_conn, probe_t *out, int verbose)
{
    struct drm_mode_card_res res;
    uint32_t conns[MAX_CONNS], crtcs[MAX_CRTCS];
    uint32_t nc = 0, nr = 0;
    int best = -1;

    if (get_resources(fd, conns, &nc, crtcs, &nr, &res) != 0) return -1;

    if (verbose) {
        fprintf(stderr, "[drm] %u connector(s), %u crtc(s), %u encoder(s); size %ux%u..%ux%u\n",
                nc, nr, res.count_encoders, res.min_width, res.min_height,
                res.max_width, res.max_height);
    }

    for (uint32_t i = 0; i < nc; i++) {
        struct drm_mode_get_connector c;
        struct drm_mode_modeinfo modes[MAX_MODES];
        uint32_t encs[MAX_ENCS];
        uint32_t nm = MAX_MODES, ne = MAX_ENCS;
        uint32_t crtc_id;
        int score = 0;

        if (want_conn >= 0 && (uint32_t)want_conn != conns[i]) continue;

        if (get_connector(fd, conns[i], &c, encs, &ne, modes, &nm) != 0) continue;

        if (verbose) {
            fprintf(stderr, "[drm]   conn %u: %s-%u [%s] %u mode(s)\n",
                    c.connector_id, conn_type_name(c.connector_type),
                    c.connector_type_id, conn_status_name(c.connection), nm);
        }

        if (c.connection != DRM_MODE_CONNECTED || nm == 0) continue;

        /* 优先 PREFERRED 的 mode，没有就第一个 */
        uint32_t pick = 0, preferred = 0;
        for (uint32_t m = 0; m < nm; m++) {
            if (modes[m].type & DRM_MODE_TYPE_PREFERRED) { pick = m; preferred = 1; break; }
        }

        crtc_id = pick_crtc(fd, &c, encs, ne, crtcs, nr);
        if (!crtc_id) {
            if (verbose) fprintf(stderr, "[drm]   conn %u 找不到可用 crtc，跳过\n", c.connector_id);
            continue;
        }

        if (want_conn >= 0)                                    score += 100;
        if (c.connector_type == DRM_MODE_CONNECTOR_DSI)         score += 20;
        if (preferred)                                          score += 5;

        if (score > best) {
            best = score;
            memset(out, 0, sizeof *out);
            out->conn_id      = c.connector_id;
            out->conn_type    = c.connector_type;
            out->conn_type_id = c.connector_type_id;
            out->crtc_id      = crtc_id;
            out->mm_w         = c.mm_width;
            out->mm_h         = c.mm_height;
            memcpy(&out->mode, &modes[pick], sizeof out->mode);
        }
    }

    if (best < 0) return -1;

    fprintf(stderr, "[drm] 选中 conn=%u (%s-%u) crtc=%u  mode=\"%s\" %ux%u @%uHz\n",
            out->conn_id, conn_type_name(out->conn_type), out->conn_type_id,
            out->crtc_id, out->mode.name,
            out->mode.hdisplay, out->mode.vdisplay, out->mode.vrefresh);
    return 0;
}

/* 申请一块 dumb buffer 并登记成 framebuffer */
static int dbuf_alloc(dbuf_t *b, int w, int h, uint32_t fourcc)
{
    struct drm_mode_create_dumb creq;
    struct drm_mode_map_dumb    mreq;
    struct drm_mode_fb_cmd2     fbreq;

    memset(&creq, 0, sizeof creq);
    creq.width  = (uint32_t)w;
    creq.height = (uint32_t)h;
    creq.bpp    = 32;
    if (ioctl(g.fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0) {
        fprintf(stderr, "[drm] MODE_CREATE_DUMB: %s\n", strerror(errno));
        return -1;
    }
    b->handle = creq.handle;
    b->pitch  = creq.pitch;
    b->size   = creq.size;

    memset(&mreq, 0, sizeof mreq);
    mreq.handle = creq.handle;
    if (ioctl(g.fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0) {
        fprintf(stderr, "[drm] MODE_MAP_DUMB: %s\n", strerror(errno));
        return -1;
    }

    b->map = mmap(NULL, creq.size, PROT_READ | PROT_WRITE, MAP_SHARED, g.fd, mreq.offset);
    if (b->map == MAP_FAILED) {
        b->map = NULL;
        fprintf(stderr, "[drm] mmap(%llu): %s\n",
                (unsigned long long)creq.size, strerror(errno));
        return -1;
    }
    memset(b->map, 0, creq.size);   /* 全黑起步，避免开机闪白 */

    memset(&fbreq, 0, sizeof fbreq);
    fbreq.width        = (uint32_t)w;
    fbreq.height       = (uint32_t)h;
    fbreq.pixel_format = fourcc;
    fbreq.handles[0]   = creq.handle;
    fbreq.pitches[0]   = creq.pitch;
    fbreq.offsets[0]   = 0;
    if (ioctl(g.fd, DRM_IOCTL_MODE_ADDFB2, &fbreq) < 0) {
        fprintf(stderr, "[drm] MODE_ADDFB2: %s\n", strerror(errno));
        return -1;
    }
    b->fb_id = fbreq.fb_id;
    return 0;
}

/* 把某块 framebuffer 推上屏：首次是 modeset，之后等价于翻页 */
static int do_modeset(uint32_t fb_id)
{
    struct drm_mode_crtc req;

    memset(&req, 0, sizeof req);
    req.crtc_id            = g.crtc_id;
    req.fb_id              = fb_id;
    req.x                  = 0;
    req.y                  = 0;
    req.count_connectors   = 1;
    req.set_connectors_ptr = (uint64_t)(uintptr_t)&g.conn_id;
    req.mode_valid         = 1;
    memcpy(&req.mode, &g.mode, sizeof g.mode);

    if (ioctl(g.fd, DRM_IOCTL_MODE_SETCRTC, &req) < 0) {
        fprintf(stderr, "[drm] MODE_SETCRTC(fb=%u): %s\n", fb_id, strerror(errno));
        return -1;
    }
    return 0;
}

/* =========================================================
 *  LVGL flush：把「刚画完的那块」推上屏
 * ========================================================= */

static void drm_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    LV_UNUSED(area);

    if (lv_display_flush_is_last(disp)) {
        int idx = -1;
        for (int i = 0; i < g.nbuf; i++) {
            if ((uint8_t *)g.buf[i].map == px_map) { idx = i; break; }
        }
        if (idx >= 0 && idx != g.front) {
            const uint64_t t0 = now_ns();
            if (do_modeset(g.buf[idx].fb_id) == 0) {
                g.front = idx;
                /*
                 * 把 SETCRTC 的耗时也记下来：如果驱动是等 vblank 才返回的，
                 * 这个数就会接近 1/刷新率（比如 49Hz -> 约 20ms），
                 * 也就说明帧率上限被面板 vblank 卡住了。
                 */
                g.flip_ns += now_ns() - t0;
                g.flips++;
            }
        }
    }
    lv_display_flush_ready(disp);
}

/* =========================================================
 *  对外接口
 * ========================================================= */

int drm_display_init(const char *card, int connector_id)
{
    probe_t pi;

    if (g.ready) return 0;
    if (!card) card = "/dev/dri/card0";

    g.fd = open(card, O_RDWR | O_CLOEXEC);
    if (g.fd < 0) {
        fprintf(stderr, "[drm] open %s: %s\n", card, strerror(errno));
        return -1;
    }

    /* 官方 lv_demo 被杀掉后 master 空着，这里抢过来 */
    if (ioctl(g.fd, DRM_IOCTL_SET_MASTER, 0) < 0)
        fprintf(stderr, "[drm] SET_MASTER: %s（继续尝试）\n", strerror(errno));

    if (probe_fd(g.fd, connector_id, &pi, 1) != 0) {
        fprintf(stderr, "[drm] 没有可用的 connector / crtc\n");
        goto fail;
    }

    g.conn_id = pi.conn_id;
    g.crtc_id = pi.crtc_id;
    memcpy(&g.mode, &pi.mode, sizeof g.mode);
    g.width   = pi.mode.hdisplay;
    g.height  = pi.mode.vdisplay;

    if (g.width <= 0 || g.height <= 0) {
        fprintf(stderr, "[drm] 取到非法分辨率 %dx%d\n", g.width, g.height);
        goto fail;
    }

    for (int i = 0; i < DBUF_MAX; i++) {
        if (dbuf_alloc(&g.buf[i], g.width, g.height, DRM_FORMAT_XRGB8888) != 0) {
            fprintf(stderr, "[drm] 第 %d 块 dumb buffer 申请失败\n", i);
            goto fail;
        }
        /* LVGL 的 32bpp 绘制缓冲假设每行恰好 width*4 字节。
         * dumb buffer 的 pitch 由驱动决定，通常是紧密的，但必须核实。 */
        if (g.buf[i].pitch != (uint32_t)g.width * 4u) {
            fprintf(stderr, "[drm] pitch=%u 与 width*4=%u 不一致，无法当绘制缓冲\n",
                    g.buf[i].pitch, (uint32_t)g.width * 4u);
            goto fail;
        }
        g.nbuf++;
    }

    /* 先让 buf[0] 上屏，LVGL 从 buf[1] 那块「背面」开始画 */
    if (do_modeset(g.buf[0].fb_id) != 0) {
        fprintf(stderr, "[drm] 初始 modeset 失败\n");
        goto fail;
    }
    g.front = 0;

    lv_display_t *disp = lv_display_create(g.width, g.height);
    if (!disp) {
        fprintf(stderr, "[drm] lv_display_create 失败\n");
        goto fail;
    }

    lv_display_set_buffers(disp, g.buf[1].map, g.buf[0].map,
                           (uint32_t)g.buf[1].size, LV_DISPLAY_RENDER_MODE_DIRECT);
    lv_display_set_flush_cb(disp, drm_flush_cb);

    g.ready = 1;
    printf("[drm] display: %s conn=%u crtc=%u %dx%d @%uHz  fb=%u/%u pitch=%u\n",
           card, g.conn_id, g.crtc_id, g.width, g.height,
           pi.mode.vrefresh, g.buf[0].fb_id, g.buf[1].fb_id, g.buf[0].pitch);
    return 0;

fail:
    for (int i = 0; i < DBUF_MAX; i++) {
        if (g.buf[i].map) { munmap(g.buf[i].map, g.buf[i].size); g.buf[i].map = NULL; }
    }
    if (g.fd >= 0) { close(g.fd); g.fd = -1; }
    return -1;
}

int drm_display_connector_id(void)
{
    return g.ready ? (int)g.conn_id : -1;
}

/*
 * 帧率统计：flips = 真正推上屏的帧数（每次成功翻页算一帧），
 * flip_ns = 这些翻页在 SETCRTC 里累计花掉的时间。
 *
 * 实测帧率 = (两次采样之间的 flips 差值) / 间隔秒数。
 * 平均单次翻页耗时 = flip_ns / flips —— 若它接近 1/面板刷新率，
 * 说明瓶颈在等 vblank，而不是我们的软件渲染。
 */
void drm_display_flip_stats(uint32_t *flips, uint64_t *flip_ns)
{
    if (flips)   *flips   = g.flips;
    if (flip_ns) *flip_ns = g.flip_ns;
}

/* 面板分辨率与刷新率（测速时用来和实测值对照） */
void drm_display_mode_get(int *w, int *h, int *hz)
{
    if (w)  *w  = g.ready ? g.width  : 0;
    if (h)  *h  = g.ready ? g.height : 0;
    if (hz) *hz = g.ready ? (int)g.mode.vrefresh : 0;
}

/*
 * 导出当前上屏内容为 PPM(P6)。
 * 缓冲是 DRM_FORMAT_XRGB8888：小端下一个像素 4 字节依次是 B, G, R, X。
 */
int drm_display_snapshot_ppm(const char *path)
{
    if (!g.ready || !path) return -1;

    const dbuf_t *b = &g.buf[g.front];
    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "[drm] 打不开 %s: %s\n", path, strerror(errno));
        return -1;
    }

    fprintf(f, "P6\n%d %d\n255\n", g.width, g.height);
    for (int y = 0; y < g.height; y++) {
        const uint8_t *row = (const uint8_t *)b->map + (size_t)y * b->pitch;
        for (int x = 0; x < g.width; x++) {
            const uint8_t *p = row + (size_t)x * 4u;
            fputc(p[2], f);     /* R */
            fputc(p[1], f);     /* G */
            fputc(p[0], f);     /* B */
        }
    }
    fclose(f);

    printf("[drm] 截图 -> %s  (%dx%d, fb=%u)\n", path, g.width, g.height, b->fb_id);
    return 0;
}

int drm_display_info(const char *card)
{
    if (!card) card = "/dev/dri/card0";

    int fd = open(card, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        fprintf(stderr, "[drm] open %s: %s\n", card, strerror(errno));
        return -1;
    }

    /* 驱动版本 */
    {
        struct drm_version v;
        char name[64] = {0}, date[64] = {0}, desc[128] = {0};
        memset(&v, 0, sizeof v);
        v.name_len = sizeof name; v.name = name;
        v.date_len = sizeof date; v.date = date;
        v.desc_len = sizeof desc; v.desc = desc;
        if (ioctl(fd, DRM_IOCTL_VERSION, &v) == 0) {
            printf("driver      : %s %d.%d.%d  (%s)\n", name,
                   v.version_major, v.version_minor, v.version_patchlevel, desc);
        }
    }

    /* 客户端能力 */
    {
        struct { uint64_t cap_id; uint64_t value; } cq;

        cq.cap_id = DRM_CAP_DUMB_BUFFER; cq.value = 0;
        if (ioctl(fd, DRM_IOCTL_GET_CAP, &cq) == 0)
            printf("dumb buffer : %s\n", cq.value ? "支持" : "不支持");

        cq.cap_id = DRM_CAP_ADDFB2_MODIFIERS; cq.value = 0;
        if (ioctl(fd, DRM_IOCTL_GET_CAP, &cq) == 0)
            printf("addfb2 mod  : %s\n", cq.value ? "支持" : "不支持");

        /* 本程序走 legacy SETCRTC，不依赖 atomic；这里只是把能力打出来排错用 */
        {
            struct { uint64_t capability; uint64_t value; } ca;
            ca.capability = DRM_CLIENT_CAP_ATOMIC; ca.value = 1;
            printf("atomic cap  : %s\n",
                   ioctl(fd, DRM_IOCTL_SET_CLIENT_CAP, &ca) == 0 ? "支持" : "不支持");
        }
    }

    /* 拓扑 */
    {
        struct drm_mode_card_res res;
        uint32_t conns[MAX_CONNS], crtcs[MAX_CRTCS];
        uint32_t nc = 0, nr = 0;

        if (get_resources(fd, conns, &nc, crtcs, &nr, &res) != 0) { close(fd); return -1; }

        printf("display     : %u connector(s), %u crtc(s), %u encoder(s), %u fb(s)\n",
               nc, nr, res.count_encoders, res.count_fbs);
        printf("size range  : %ux%u .. %ux%u\n",
               res.min_width, res.min_height, res.max_width, res.max_height);

        for (uint32_t i = 0; i < nc; i++) {
            struct drm_mode_get_connector c;
            struct drm_mode_modeinfo modes[MAX_MODES];
            uint32_t encs[MAX_ENCS];
            uint32_t nm = MAX_MODES, ne = MAX_ENCS;

            if (get_connector(fd, conns[i], &c, encs, &ne, modes, &nm) != 0) continue;

            printf("  conn id=%u type=%s-%u status=%s mm=%ux%u encoders=%u modes=%u\n",
                   c.connector_id, conn_type_name(c.connector_type), c.connector_type_id,
                   conn_status_name(c.connection), c.mm_width, c.mm_height, ne, nm);
            for (uint32_t m = 0; m < nm && m < 8; m++) {
                printf("      mode[%u] %-16s %ux%u @%uHz  clock=%u type=0x%x%s\n",
                       m, modes[m].name, modes[m].hdisplay, modes[m].vdisplay,
                       modes[m].vrefresh, modes[m].clock, modes[m].type,
                       (modes[m].type & DRM_MODE_TYPE_PREFERRED) ? " PREFERRED" : "");
            }
            printf("      -> crtc=%u encoder=%u\n",
                   pick_crtc(fd, &c, encs, ne, crtcs, nr), c.encoder_id);
        }
    }

    /* 最关键的一条假设：dumb buffer 的 pitch 是否等于 width*4 */
    {
        probe_t pi;
        if (probe_fd(fd, -1, &pi, 0) == 0) {
            struct drm_mode_create_dumb creq;
            memset(&creq, 0, sizeof creq);
            creq.width  = pi.mode.hdisplay;
            creq.height = pi.mode.vdisplay;
            creq.bpp    = 32;
            if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) == 0) {
                printf("dumb probe  : %ux%u bpp32 -> pitch=%u size=%llu (期望 pitch=%u)\n",
                       pi.mode.hdisplay, pi.mode.vdisplay, creq.pitch,
                       (unsigned long long)creq.size, pi.mode.hdisplay * 4u);
                struct drm_mode_destroy_dumb dreq;
                memset(&dreq, 0, sizeof dreq);
                dreq.handle = creq.handle;
                ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
            } else {
                printf("dumb probe  : CREATE_DUMB 失败: %s\n", strerror(errno));
            }
        }
    }

    close(fd);
    return 0;
}
