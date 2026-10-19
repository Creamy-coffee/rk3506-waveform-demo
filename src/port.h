/*
 * port.h —— Linux 板级移植：framebuffer/DRM 显示 + evdev 触摸
 */
#include <stdint.h>

#ifndef PORT_H
#define PORT_H

/*
 * 初始化显示。devpath 可为 NULL（自动探测）。取值含义：
 *   NULL       → 先试 /dev/fb0，没有就退回 /dev/dri/card0(DRM)
 *   /dev/fbX   → 强制用 framebuffer
 *   其它路径    → 当作 DRM card 节点（如 /dev/dri/card0）
 * 另外可用环境变量覆盖：WAVE_DRM_CARD、WAVE_DRM_CONNECTOR。
 * 返回 0 成功。
 */
int port_display_init(const char *devpath);

/*
 * 扫描 /dev/input/event*，返回触摸设备节点（如 /dev/input/event0）。
 * 判定条件：带 ABS_X 或 ABS_MT_POSITION_X，且带 BTN_TOUCH。
 * 没找到返回 NULL。结果缓存在内部静态缓冲里。
 *
 * 单独暴露出来是因为 --tap 注入工具只需要设备路径，不需要建 indev。
 */
const char *port_touch_scan(void);

/*
 * 初始化触摸。内部会自己扫描 /dev/input/event*。
 * 返回 0 成功，-1 表示没找到触摸设备（此时界面仍能显示，只是按钮点不了）。
 */
int port_input_init(void);

/* 上一次端口初始化时用到的触摸设备节点名，没找到返回 NULL。仅用于打印日志。 */
const char *port_touch_device(void);

#endif /* PORT_H */

/* 上一次触摸落点的屏幕坐标（读数游标等需要用精确位置的地方） */
void port_touch_last_point(int32_t *x, int32_t *y);
