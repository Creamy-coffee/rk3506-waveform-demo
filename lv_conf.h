/*
 * lv_conf.h —— LVGL 9.1 配置（只写要覆盖的项，其余走 lv_conf_internal.h 默认值）
 *
 * 目标平台：RK3506 EVB + 7 寸 MIPI 屏（1024x600），DRM 显示 + evdev 触摸。
 */
#ifndef LV_CONF_H
#define LV_CONF_H

/* ============ 基础 ============ */
/* 32bpp -> LV_COLOR_FORMAT_XRGB8888，正好对应 DRM 的 DRM_FORMAT_XRGB8888 */
#define LV_COLOR_DEPTH      32
/* DRM 走 DIRECT 模式，绘制缓冲由 drm_disp.c 直接提供（就是 dumb buffer 本体），
 * 这里的内存池只用于控件/图层等零碎分配。 */
#define LV_MEM_SIZE         (1024 * 1024)
#define LV_USE_OS           0
#define LV_DEF_REFR_PERIOD  33

/* ============ 日志（排错期保持打开） ============ */
#define LV_USE_LOG          1
#define LV_LOG_LEVEL        LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF       1

/* ============ 字体 ============ */
#define LV_FONT_MONTSERRAT_14   1
#define LV_FONT_MONTSERRAT_20   1
#define LV_FONT_MONTSERRAT_28   1
#define LV_FONT_DEFAULT         &lv_font_montserrat_20

/* ============ 浮点 ============ */
/*
 * ⚠️ 这一项必须开，否则 lv_label_set_text_fmt(l, "%.1f mV", v) 会打出 "f mV"。
 *
 * 原因（读 LVGL 源码确认，不是猜的）：
 *   src/stdlib/builtin/lv_sprintf_builtin.c:41
 *       #define PRINTF_DISABLE_SUPPORT_FLOAT  (!LV_USE_FLOAT)
 *   LV_USE_FLOAT 默认 0 -> PRINTF_SUPPORT_FLOAT 未定义 -> 那个 switch 里的
 *   `case 'f':` 连同 `case 'e'/'g'` **整段被条件编译掉**。于是 "%.1f" 的
 *   '%' 被当转换引导符消费掉，'.1' 被当精度吃掉，最后落到 default 分支，
 *   'f' 就按普通字符原样输出 —— 屏幕上正好是 "f mV"。
 *   它不会报错、不会告警，非常容易误判成「LVGL 不支持 %f」。
 *
 * 顺带好处：LV_USE_FLOAT=1 会把 lv_value_precise_t 从 int32_t 变成 float，
 * 于是 lv_point_precise_t / lv_draw_line_dsc_t.p1-p2 能保留小数坐标，
 * 自绘波形（plot.c）的折线拐点不会再被截断成整数像素。
 */
#define LV_USE_FLOAT         1

/* ============ 控件 ============ */
#define LV_USE_CHART         1

/*
 * 关于 flex/grid：LVGL 9.1 里 LV_USE_FLEX / LV_USE_GRID **仍然是配置宏**
 * （默认都是 1，见 src/lv_conf_internal.h），并不是「v9 已改成非配置项」。
 * 本工程用到 flex，所以这里显式写出来锁住，免得以后有人误关。
 */
#define LV_USE_FLEX          1
#define LV_USE_GRID          1

/* ============ 显示 ============ */
/* framebuffer 路径：板子有 /dev/fb0 时走这里 */
#define LV_USE_LINUX_FBDEV            1
#define LV_LINUX_FBDEV_RENDER_MODE    LV_DISPLAY_RENDER_MODE_PARTIAL
#define LV_LINUX_FBDEV_BUFFER_COUNT   1
/* 一行 = 一个水平扫描行。1024 宽 x 120 行 x 4B ≈ 480KB 绘制缓冲 */
#define LV_LINUX_FBDEV_BUFFER_SIZE    120

/* 刻意关掉 LVGL 自带的 DRM 后端：
 *   它要 libdrm.so，而交叉工具链 sysroot 里没有 libdrm.a，会破坏静态链接；
 *   且它按「第一个格式匹配的 plane」选平面，在 Rockchip VOP 上未必是 primary。
 *   本工程改用 src/drm_disp.c（纯内核 UAPI ioctl，零用户态依赖）。 */
#define LV_USE_LINUX_DRM     0

/* ============ 输入：evdev 触摸 ============ */
#define LV_USE_EVDEV         1

/* ============ 关掉用不上的，省编译时间和体积 ============ */
#define LV_USE_DEMO_WIDGETS      0
#define LV_USE_DEMO_BENCHMARK    0
#define LV_USE_DEMO_MUSIC        0
#define LV_USE_DEMO_STRESS       0
#define LV_USE_FREETYPE          0
#define LV_USE_SDL               0
#define LV_USE_LIBINPUT          0
#define LV_USE_X11               0
#define LV_USE_WAYLAND           0
#define LV_USE_OPENGLES          0
#define LV_USE_VECTOR_GRAPHIC    0

#endif /* LV_CONF_H */
