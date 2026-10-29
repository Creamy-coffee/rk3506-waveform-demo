/*
 * plot.h —— 自绘波形控件
 *
 * 为什么不用 lv_chart：
 *   参考稿的画布不是「数据点 → 柱子」，而是带网格、中心虚线、触发线、同心圆、
 *   峰值箭头和矢量线的一张仪表图。lv_chart 表达不了这些，硬凑要挂一堆插件。
 *   所以照 LVGL 官方推荐的路子：普通 lv_obj + LV_EVENT_DRAW_MAIN 回调，
 *   在回调里用 lv_draw_line / lv_draw_arc / lv_draw_rect / lv_draw_label 直接画。
 *
 * 和 lv_chart 的一致性：lv_chart 自己画折线也是「一段一根 lv_draw_line」
 *   （见 lvgl/src/widgets/chart/lv_chart.c: draw_series_line → lv_draw_line），
 *   所以绘制成本和官方控件同量级，没有额外的坑。
 *
 * 坐标系约定（和 lv_chart 一样）：
 *   LV_EVENT_DRAW_MAIN 里的坐标是**屏幕绝对坐标**，从 lv_obj_get_coords() 拿
 *   控件矩形再加自己的边距。不要用相对坐标，否则会画歪。
 *
 * 数据约定：
 *   plot_set_trace() 传进来的 y[] 是**归一化**的，0.0 = 贴底，1.0 = 贴顶。
 *   传原始电压/ADC 值也行 —— 先用 plot_set_range() 告诉控件量程，
 *   或者直接用 wave 工具函数 plot_norm() 把一段数据压到 0..1。
 */
#ifndef V32_PLOT_H
#define V32_PLOT_H

#include "lvgl.h"

/* 一条曲线的最大点数。
 * 绘图区宽约 740 px，1024 点足够做到「每像素至少一个点」；
 * 演示源只发 512 点、V32CAP3 一帧 4096 点会先被压到 1024。 */
#define PLOT_MAX_PTS   1024

/* Optional true-bin horizontal positions for an FFT trace, normalized 0..1. */
void plot_set_fft_positions(lv_obj_t *plot,const float *positions,int n);
void plot_set_fft_axes(lv_obj_t *plot,float range_db,float nyquist_hz);
/* 一个波形区最多几条曲线（参考稿里 SCOPE/LOCK-IN 是两条，其余一条） */
#define PLOT_MAX_TRACE 3   /* 2 路 + 1 条 Math */

typedef enum {
    PLOT_SCOPE = 0,   /* 双通道折线 + 触发线 + 中心虚线，10×8 网格 */
    PLOT_FFT,         /* 单条频谱曲线 + 峰值箭头，13×8 网格 */
    PLOT_IQ,          /* 同心圆 + 矢量线（Q/I 轴），8×8 网格 */
    PLOT_XY,          /* 李萨如：trace0 作 X 轴、trace1 作 Y 轴 */
    PLOT_HIST,        /* 双通道时间历程（锁相输出），10×8 网格 */
    PLOT_AI           /* 单条曲线（AI 原始/降噪），10×4 网格 */
} plot_mode_t;

/* ---- 建/删 ---- */
/* 返回的 lv_obj 已经设好 .plot 的背景色、圆角、1px 边框、不可滚动。 */
lv_obj_t *plot_create(lv_obj_t *parent, plot_mode_t mode);
/* 换模式（比如复用同一个控件切页面）*/
void      plot_set_mode(lv_obj_t *plot, plot_mode_t mode);

/* ---- 网格 ---- */
/* divx=10, divy=8 是参考稿 grid() 的默认值；center_dash = 画中间那条虚线 */
void plot_set_grid(lv_obj_t *plot, int divx, int divy, bool center_dash);

/* ---- 曲线 ---- */
/*
 * idx   : 0 或 1
 * y     : 归一化数据，0.0=底 1.0=顶；内部会拷贝一份，调用方可以复用缓冲
 * n     : 点数（会被截断到 PLOT_MAX_PTS）
 * color : 线色
 * width : 线宽，轴对齐的 1px 线用 1 最省（LVGL 有 1px 快速路径）
 */
void plot_set_trace(lv_obj_t *plot, int idx, const float *y, int n,
                    lv_color_t color, int32_t width);
void plot_hide_trace(lv_obj_t *plot, int idx);

/* ---- SCOPE 专用：触发线 ----
 * plot_set_trigger      : 竖直的位置标记（FFT 页还在用；SCOPE 页画成
 *                          顶部小三角，照 scope_proto_1024.html）
 * plot_set_trigger_level: 水平的触发电平虚线（scope_proto 的洋红长虚线：
 *                          两端 22px 实线头 + 左端小三角），y01: 0=底 1=顶 */
/* x 是归一化横坐标 0..1（参考稿画的固定 0.46），会在竖线右侧写个 "T" */
void plot_set_trigger(lv_obj_t *plot, bool vis, float x);
void plot_set_trigger_level(lv_obj_t *plot, bool vis, float y01);

/* ---- FFT 专用：峰值箭头 + 文字 ---- */
/* 会在 (x, 曲线高度) 处画一个向下的三角形，并在旁边写 txt。
 * txt 会被拷进控件内部（lv_draw_label 渲染时才读，不能传栈上缓冲）。 */
void plot_set_peak(lv_obj_t *plot, bool vis, float x, const char *txt);

/* ---- IQ 专用：矢量角度（数学约定，0°=+I 向右，逆时针为正）---- */
void plot_set_iq(lv_obj_t *plot, float phase_deg);
void plot_hide_iq(lv_obj_t *plot);
bool plot_get_data_area(lv_obj_t *plot,lv_area_t *area);
void plot_set_voltage_reference(lv_obj_t *plot,int channel,float zero0,float zero1,bool show0,bool show1);

/* ---- 画布内的角落文字（参考稿 FFT 页的 "0 MHz" / "32.5 MHz"）---- */
void plot_set_axis_labels(lv_obj_t *plot, const char *left, const char *right);

/* ---- SCOPE 专用：AM 上下包络叠加（金色虚线, 照原型 AM 视图）----
 * up/dn 为归一化 0..1 曲线(0=底 1=顶), show=false 或 n<2 清除。
 * 绘制在迹线之上、光标之下。 */
void plot_set_envelope(lv_obj_t *plot, const float *up, const float *dn,
                       int n, bool show);
/* Optional amplitude-region shading, not sampled carrier data. Off by default. */
void plot_set_envelope_fill(lv_obj_t *plot, bool show);
/* For baseband sample distributions: actual samples, optional equal pixel scales. */
void plot_set_xy_style(lv_obj_t *plot, bool points, bool equal_axes);

/* 测量光标：n<=4，xs 是竖线的相对位置 0..1，ys 是横线的相对位置 0..1。
 * 示波器页用 2 竖 + 2 横（时间光标 + 电压光标）。 */
void plot_set_cursors(lv_obj_t *plot, int n, const float *xs, const float *ys, bool show);

/* 绘图区顶部那行示波器信息：A1=1.00V/格  A2=GND  Time/Div=202µs  Trig=A1::Auto
 * 照 HTML 原型 scope_proto_1024.html 的版式。 */
void plot_set_info(lv_obj_t *plot, const char *line);

/*
 * 分段配色的信息行（scope_proto 的画法：标签绿、数值白、触发洋红）。
 * plot.c 会把文本拷进控件内部，调用方的缓冲可以复用；最多 12 段。
 */
typedef struct {
    char       txt[28];
    lv_color_t color;
} plot_info_seg_t;
void plot_set_info_segs(lv_obj_t *plot, const plot_info_seg_t *segs, int n);

/* 刻度文字：x 轴 6 个（0,2,4,6,8,10 格）、y 轴 5 个（0,2,4,6,8 格）。
 * 传入后绘图区会自动留出左 42px / 下 18px 的边距画刻度，跟 HTML 原型一致。 */
void plot_set_ticks(lv_obj_t *plot, const char *const *xl, int nx,
                    const char *const *yl, int ny);
/* 两条曲线右端的小图例（可选，写 NULL 不画） */
void plot_set_legends(lv_obj_t *plot, const char *l0, const char *l1);

/* ---- 工具：把一段任意量程的数据压到 0..1 ---- */
/* 自动找 min/max 并留 margin（0.05），返回 0 成功；数据全等或 n<2 时返回 -1。 */
int plot_norm(const float *src, int n, float *dst, int dst_n, float margin);

#endif /* V32_PLOT_H */
