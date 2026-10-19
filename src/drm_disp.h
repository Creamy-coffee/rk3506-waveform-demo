/*
 * drm_disp.h —— 直接用内核 DRM UAPI 驱动的显示后端（不依赖 libdrm）
 *
 * 为什么不用 LVGL 自带的 lv_linux_drm？
 *   1) 它需要 libdrm.so，而交叉工具链 sysroot 里只有 libdrm.so、没有 libdrm.a，
 *      会破坏我们「单个静态可执行文件」的部署方式；
 *   2) 它走 atomic 提交、且用「第一个格式匹配的 plane」，在 Rockchip VOP 上
 *      选中 overlay plane 的风险不好排除。
 *   本文件只用 <drm/drm.h> 这些纯内核 UAPI 头（编译期不需要任何用户态库），
 *   走 legacy SETCRTC 路径（内核会用 drm_atomic_helper_set_config 兼容层接管），
 *   代码可控，静态链接照样成立。
 */
#ifndef DRM_DISP_H
#define DRM_DISP_H

#include "lvgl.h"

/*
 * 打开 card（如 "/dev/dri/card0"），挑一条已连接的 connector，
 * 建两块 dumb buffer 当 LVGL 的 DIRECT 绘制缓冲，并做一次初始 modeset。
 *
 * connector_id 传 -1 表示自动挑选。
 * 成功返回 0（此时默认 display 已就绪），失败返回 -1。
 */
int drm_display_init(const char *card, int connector_id);

/* 只打印 DRM 拓扑信息（connector / mode / crtc / 内存 pitch），不做任何 LVGL 初始化。 */
int drm_display_info(const char *card);

/*
 * 把「当前正在上屏的那块缓冲」导出成 PPM(P6) 图片，便于无显示器时核对画面。
 * 成功返回 0。初始化未完成时返回 -1。
 */
int drm_display_snapshot_ppm(const char *path);

/* 初始化成功时有效：当前 connector 的 DRM id（供日志/调试） */
int drm_display_connector_id(void);

/*
 * 帧率统计（诊断用，不影响显示路径）。
 *   flips    —— 成功翻页次数，也就是真正上屏的帧数
 *   flip_ns  —— 这些翻页在 DRM_IOCTL_MODE_SETCRTC 里累计花掉的时间
 *
 * 用法：前后各取一次，
 *     实测帧率 = Δflips / Δt
 *     平均单次翻页耗时 = Δflip_ns / Δflips
 * 后者若接近 1/面板刷新率（本板 49Hz -> 约 20ms），说明帧率被 vblank 卡住，
 * 而不是软件渲染慢。
 * 两个指针都可以传 NULL 表示不关心该项。
 */
void drm_display_flip_stats(uint32_t *flips, uint64_t *flip_ns);

/* 初始化成功时有效：面板分辨率与刷新率（测速时用来和实测值对照）。 */
void drm_display_mode_get(int *w, int *h, int *hz);

#endif /* DRM_DISP_H */
