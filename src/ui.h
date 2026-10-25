/*
 * ui.h —— 界面层
 *
 * 界面是参考稿 v32_linux_touch_ui-1.html 的 1:1 移植（1024×600 触摸仪表盘）：
 *
 *   顶栏   48px  V32 SIGNAL ANALYZER 品牌 · DEMO MODE 徽标 · ADC 65 MSPS · RUNNING · 时钟
 *   主区  404px  五个页面，右侧 235px 读数面板
 *     ⌁ SCOPE    双通道时域 + MEASUREMENTS/CHANNELS
 *     ▥ FFT      频谱 + SPECTRUM ANALYSIS
 *     ◎ LOCK-IN  I/Q 矢量 + 时程 + SYNCHRONOUS DETECTION
 *     ✧ AI       原始 / 降噪对比 + AI PROCESSOR
 *     ⚙ SYSTEM   设备状态 + 诊断（这两个面板显示的是**真实**数据）
 *   控制条 92px  信号频率 / 噪声 / 页内参数 / 暂停 / 复位 / 全屏
 *   导航   56px  5 个页面按钮
 */
#ifndef UI_H
#define UI_H

#include "lvgl.h"

/* 页面编号（顺序 = 底部导航从左到右） */
enum {
    UI_PAGE_SCOPE = 0,
    UI_PAGE_FFT,
    UI_PAGE_LOCK,
    UI_PAGE_AI,
    UI_PAGE_MOD,       /* 调制分析（V32ANA1!，交付包 603695b2） */
    UI_PAGE_SYSTEM,
    UI_PAGE_COUNT
};

/* 建界面。调用前必须先 lv_init() 且 display 已创建。 */
void ui_create(void);

/* 刷一帧。建议 15~20Hz 左右调用（主循环里是 100ms 一次数据更新）。 */
void ui_tick(void);

/* ---- 页面 ---- */
int  ui_page_count(void);
int  ui_get_page(void);
void ui_set_page(int page);/* 按名字选页：scope|wave|fft|lock|lockin|ai|system；返回 -1 表示不认识 */
int  ui_page_from_name(const char *name);

/* ---- 兼容旧接口：0 = 波形(scope)，1 = 频谱(fft) ---- */
int  ui_get_view(void);
void ui_set_view(int view);

/*
 * 两指捏放缩放波形（由触摸驱动的双指手势调用）。
 *   scale  > 1 = 手指张开 = 放大（可见点数变少）
 *   anchor = 捏合中心在绘图区里的水平比例 0..1，缩放时该处样点保持不动
 * 只影响显示窗口，不改变实际收到的数据；纵向量程仍按整帧自适应。
 * 横向捏放 = 时基（µs/div），纵向捏放 = 幅度（mV/div），见 ui_pinch_gain()。
 */
void ui_pinch_zoom(float scale, float anchor);

/*
 * 纵向增益，等价示波器的 V/div。
 *   ratio > 1 = 两指上下张开 = 波形变高（每格毫伏变小）
 * 和 ui_pinch_zoom 各管一轴，同时使用不冲突。
 */
void ui_pinch_gain(float ratio);

/* ---- 排错 / 自动化 ---- */

/*
 * 冻结动画并把波形相位定为 t（对应参考稿里的 `tick*.02`，所以传 4.0f
 * 等价于参考稿的 tick=200）。
 * 用途：截图比对。不冻结的话每次截图相位都不同，两张图叠不上。
 */
void ui_freeze(float t);

/*
 * 把整棵控件树的真实坐标打到 stdout。
 * 用途：LVGL 的 flex / 内容自适应尺寸出问题时，屏幕上只表现为
 * 「文字少了一截」或「控件整个不见了」，看截图判不出是尺寸算错、
 * 被裁剪、还是压根没创建。打出真实 coords 一眼就清楚。
 */
void ui_dump_layout(void);

#endif /* UI_H */
