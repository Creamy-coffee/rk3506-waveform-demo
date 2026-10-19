/*
 * ui.c —— V32 SIGNAL ANALYZER 界面（参考稿 v32_linux_touch_ui-1.html 的 1:1 移植）
 *
 * 尺寸和配色全部来自 src/theme.h（那份从参考稿 CSS 逐条抄的）。
 * 曲线数据来自 src/demo_signal.c（照抄参考稿的 JS 数学式）。
 *
 * 布局是「LVGL flex 管横向 + 显式像素管纵向」的混合：
 *   横向会变（点「⛶ 全屏」后右栏隐藏，主绘图区要撑满）→ 交给 flex_grow；
 *   纵向是死的（1024×600 固定面板）→ 直接写死像素，省得百分比解析出意外。
 * 唯一用 LV_PCT 的地方是 flex 容器里交叉轴的宽度。
 *
 * ---------- 接真实数据 ----------
 * 默认走 demo_signal 的演示曲线。想试真实数据链路：
 *     WAVE_SRC=real /tmp/waveform_demo
 * 这时 SCOPE 的波形和 FFT 的谱线改从 wave_source.h 那一层取
 * （read_frame / read_spectrum），走的是和真实 ADC 完全相同的代码路径。
 */
#include "lvgl.h"
#include "ui.h"
#include "theme.h"
#include "plot.h"
#include "demo_signal.h"
#include "wave_source.h"
#include "v32cap3.h"       /* SYSTEM 页要读 UART 的帧数/CRC 错误统计 */
#include "fft.h"
#include "drm_disp.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* =========================================================
 * 布局常量（由 theme.h 里的 CSS 数字算出来）
 *
 * ⚠️ 坐标一律相对 **root 的内容区**，不要再自己加 V32_SHELL_X/Y。
 *    root 上挂了 1px 外框（模拟 .shell 的 border），LVGL 已经把子控件
 *    整体内缩了 1px —— 再手动加一次就变成 (2,2)，整屏又偏回去。
 *    （实测 dump：顶栏原本落在 [2,2]，应该 [1,1]）
 * 反解自 out/ref/ref_*.png（屏幕绝对坐标）：
 *   top 内容 y 1..47，border-bottom 48      -> 49 起是 .main
 *   .main 行 y 49..450（402），内容 y 58..442（385）
 *   .controls 行 y 451..542（92），border-top 451
 *   .nav 行 y 543..598（56），border-top 543
 *   x: .main 内容 13..1010（998），右栏 776..1010（235），绘图区 13..766（754）
 * ========================================================= */
#define MAIN_X0   V32_MAIN_PAD_X                                            /* +1 = 13 */
#define MAIN_Y0   (V32_ROW_TOP + V32_MAIN_PAD_T)                            /* +1 = 58 */
#define MAIN_W    (V32_SHELL_W - 2 * V32_MAIN_PAD_X)                        /* 998 */
#define MAIN_H    (V32_ROW_MAIN - V32_MAIN_PAD_T - V32_MAIN_PAD_B)          /* 385 */
#define CTRL_Y0   (V32_ROW_TOP + V32_ROW_MAIN)                              /* +1 = 451 */
#define NAV_Y0    (CTRL_Y0 + V32_ROW_CTRL)                                  /* +1 = 543 */

#define ASIDE_W   V32_RIGHT_W
/*
 * .metric 一行的高度。
 *
 * 39 是从参考稿渲染图上**量出来的**，不是算出来的：在 ref_scope.png 右栏里
 * 找那 5 条分隔线（#23364a 的整行像素），y = 130/169/208/247/286，行距正好 39。
 * 按 CSS 硬算（padding 7+7 + 18px 行高 + 1px 边框）只能得 37 左右，
 * 差在浏览器 normal line-height 与实际字体度量的出入上 —— 所以以实测为准。
 * 我们改行高之后，第一条分隔线落在 y=129，参考稿 130，只差 1px。
 */
#define METRIC_H  39
/* .kv 一行：13px 字体行高 15 + 上下各 6 + 1px 下边框 */
#define KV_H      28
/* .panel-title 那一行 + margin-bottom */
#define TITLE_H   (15 + V32_PANEL_TITLE_MB)

/* =========================================================
 * 选项表（对照参考稿的 <select>）
 * ========================================================= */
static const int   FREQ_MHZ[5]     = { 1, 5, 10, 20, 30 };
static const char  FREQ_OPTS[]     = "1 MHz\n5 MHz\n10 MHz\n20 MHz\n30 MHz";
static const int   FREQ_DEF        = 2;    /* 10 MHz */

static const float TIMEBASE_US[5]  = { 0.5f, 1.f, 2.f, 5.f, 10.f };
static const char  TIMEBASE_OPTS[] = "0.5 µs/格\n1 µs/格\n2 µs/格\n5 µs/格\n10 µs/格";
static const int   TIMEBASE_DEF    = 2;    /* 2 µs/div */

static const char  WIN_OPTS[]      = "HANN\nHAMMING\nBLACKMAN";
static const char *WIN_NAME[3]     = { "HANN", "HAMMING", "BLACKMAN" };
static const int   WIN_DEF         = 0;

static const int   TAU_MS[4]       = { 1, 10, 100, 1000 };
static const char  TAU_OPTS[]      = "1 ms\n10 ms\n100 ms\n1 s";
static const int   TAU_DEF         = 1;    /* 10 ms */

/* =========================================================
 * 全局状态（变量名故意和参考稿 JS 里的对应）
 * ========================================================= */
static struct {
    int   page;
    bool  running;
    bool  ch1, ch2, ai;
    int   noise;        /* 0..100 */
    int   freq_idx;
    int   tb_idx;
    int   win_idx;
    int   tau_idx;
    float phase;        /* ° */
    bool  fullscreen;
    bool  use_real;     /* WAVE_SRC=real */
    bool  auto_fit;     /* 按下「自动」后等下一帧真实数据到位再算窗口 */
    demo_sig_t sig;
} S;

static lv_obj_t *s_page[UI_PAGE_COUNT];
static lv_obj_t *s_nav[UI_PAGE_COUNT];
static lv_obj_t *s_aside[UI_PAGE_COUNT];      /* 可 NULL（SYSTEM 页没有右栏） */

static lv_obj_t *s_plot_scope, *s_plot_fft, *s_plot_iq, *s_plot_lock;
static lv_obj_t *s_plot_raw, *s_plot_den;

static lv_obj_t *s_lbl_scope_freq, *s_lbl_v1, *s_lbl_v2, *s_lbl_phase;
static lv_obj_t *s_lbl_scope_axis;
static lv_obj_t *s_lbl_fft_peakf, *s_lbl_fft_peakl, *s_lbl_fft_floor, *s_lbl_fft_win;
static lv_obj_t *s_lbl_lock_ref, *s_lbl_lock_amp, *s_lbl_lock_phase, *s_lbl_lock_tau;
static lv_obj_t *s_lbl_ai_stat;
static lv_obj_t *s_lbl_fps, *s_lbl_uptime;
static lv_obj_t *s_lbl_v32_rate, *s_lbl_v32_loss;   /* FPGA 链路实时统计 */
static lv_obj_t *s_lbl_clock, *s_lbl_run_status;

static lv_obj_t *s_chip_ch1, *s_chip_ch2, *s_chip_ai, *s_chip_run;
static lv_obj_t *s_dd_freq;
static lv_obj_t *s_dd_tb;              /* 时基下拉框，真实模式下要跟着窗口同步 */

/*
 * 原来这里还有 s_slider_noise / s_lbl_noise（控制条上的「噪声」滑条）。
 * 接真实链路之后它没有任何作用 —— 噪声是 FPGA 采回来的，UI 这边改一个
 * 滑条值既不影响波形也不影响读数，纯属摆设，按需求整条删掉。
 * 演示用的 S.noise 字段保留（demo_signal 还读它），固定 40%，界面上不再暴露。
 */
#define DEMO_NOISE_PCT   40

static lv_obj_t *s_ctrl_grp[UI_PAGE_COUNT];   /* 页内参数的 5 个变体 */

/* 三条横栏的句柄，只给 ui_dump_layout() 排错用 */
static lv_obj_t *s_top, *s_ctrlbar, *s_navbar;

/* 事件回调前置声明：页面构造函数（build_scope 等）写在回调定义之前 */
static void cb_ch1(lv_event_t *e);
static void cb_ch2(lv_event_t *e);

/* =========================================================
 * 小工具
 * ========================================================= */

/* 一个「什么都没有」的容器：透明、无边框、无内边距、不滚动 */
static lv_obj_t *mk(lv_obj_t *parent, int x, int y, int w, int h)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

/* .panel{ background:#101d2b; border:1px solid #2a3b4b; border-radius:8px; padding:10px } */
static lv_obj_t *mk_panel(lv_obj_t *parent, int x, int y, int w, int h)
{
    lv_obj_t *o = mk(parent, x, y, w, h);
    lv_obj_set_style_bg_color(o, V32_C_PANEL, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(o, V32_C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(o, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(o, V32_PANEL_RADIUS, LV_PART_MAIN);
    lv_obj_set_style_pad_all(o, V32_PANEL_PAD, LV_PART_MAIN);
    return o;
}

static lv_obj_t *mk_label(lv_obj_t *parent, int x, int y, const char *txt,
                          const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_remove_style_all(l);
    lv_obj_set_size(l, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_text_font(l, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(l, color, LV_PART_MAIN);
    lv_label_set_text(l, txt);
    lv_obj_set_pos(l, x, y);
    lv_obj_remove_flag(l, LV_OBJ_FLAG_SCROLLABLE);
    return l;
}

/* .panel-title{ font-size:11px; letter-spacing:1px; color:#8ca6bf } */
static lv_obj_t *mk_title(lv_obj_t *parent, int x, int y, const char *txt)
{
    lv_obj_t *l = mk_label(parent, x, y, txt, V32_FONT_SMALL, V32_C_PANEL_TITLE);
    lv_obj_set_style_text_letter_space(l, 1, LV_PART_MAIN);
    return l;
}

/*
 * .metric 用的是 CSS `align-items:baseline` —— 12px 的小标签要和 18px 的
 * 读数**基线对齐**，不是垂直居中。LVGL 没有基线对齐，用 MID 居中会让
 * 小标签整体偏高。
 *
 * 偏移量是拿参考稿渲染图**量出来的**，不是硬算字体 ascent/descent 得来的：
 *   标签字腹  ref 109..120 / 我们 105..116  -> 差 4px
 *   数值字腹  ref 104..118 / 我们 103..115  -> 差 1px
 *   （5 行读数全部一致，是系统性偏移而不是随机误差）
 * 跨字体硬算 ascent/descent 很容易差几像素，量出来最准。
 */
#define V32_METRIC_LABEL_DY   4
#define V32_METRIC_VALUE_DY   1

/* .metric —— 左灰标题、右大字读数 */
static lv_obj_t *mk_metric(lv_obj_t *parent, int y, const char *name,
                           const char *val, lv_color_t val_color)
{
    lv_obj_t *row = mk(parent, 0, y, V32_RIGHT_W - V32_PANEL_PAD * 2, METRIC_H);
    lv_obj_set_style_border_color(row, V32_C_METRIC_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 1, LV_PART_MAIN);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);

    lv_obj_t *k = mk_label(row, 0, 0, name, V32_FONT_SMALL, V32_C_METRIC_LABEL);
    lv_obj_align(k, LV_ALIGN_LEFT_MID, 0, V32_METRIC_LABEL_DY);

    lv_obj_t *v = mk_label(row, 0, 0, val, V32_FONT_BASE_B, val_color);
    lv_obj_align(v, LV_ALIGN_RIGHT_MID, 0, V32_METRIC_VALUE_DY);
    return v;
}

/* .kv —— SYSTEM 页的键值行 */
static lv_obj_t *mk_kv(lv_obj_t *parent, int y, int w, const char *key,
                       const char *val, lv_color_t val_color)
{
    lv_obj_t *row = mk(parent, 0, y, w, KV_H);
    lv_obj_set_style_border_color(row, V32_C_KV_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 1, LV_PART_MAIN);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);

    lv_obj_t *k = mk_label(row, 0, 0, key, V32_FONT_SMALL, V32_C_KV_KEY);
    lv_obj_align(k, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *v = mk_label(row, 0, 0, val, V32_FONT_SMALL_B, val_color);
    lv_obj_align(v, LV_ALIGN_RIGHT_MID, 0, 0);
    return v;
}

/* .note —— 面板底部小字，按宽度换行（LVGL 把 CJK 当断行字符，中文能正常折） */
static lv_obj_t *mk_note(lv_obj_t *parent, int y, int w, const char *txt)
{
    lv_obj_t *l = mk_label(parent, 0, y, txt, V32_FONT_SMALL, V32_C_NOTE);
    lv_obj_set_width(l, w);
    lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_line_space(l, 2, LV_PART_MAIN);
    return l;
}

/* .chip */
static void chip_styles(lv_obj_t *b)
{
    lv_obj_set_style_bg_color(b, V32_C_CHIP_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(b, V32_C_CHIP_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(b, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(b, V32_CHIP_RADIUS, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(b, V32_CHIP_PAD_X, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(b, V32_CHIP_PAD_Y, LV_PART_MAIN);
    /* 按下变暗（CSS 是 .chip:active{ transform:scale(.98) }，缩放不好做）。
     * 白底主题下"变暗"是压深一档，不能沿用深色主题的 #152436（那样按下去会变黑洞）。 */
    lv_obj_set_style_bg_color(b, V32_C_CHIP_BG_PRESS, LV_STATE_PRESSED);
}

static void chip_selected(lv_obj_t *b, bool on)
{
    lv_obj_set_style_bg_color(b, on ? V32_C_CHIP_ON_BG : V32_C_CHIP_BG, LV_PART_MAIN);
    lv_obj_set_style_border_color(b, on ? V32_C_CHIP_ON_BORDER : V32_C_CHIP_BORDER, LV_PART_MAIN);
    lv_obj_set_style_text_color(b, on ? V32_C_CHIP_ON_FG : V32_C_CHIP_FG, LV_PART_MAIN);
    /* 文字在子 label 上，得单独刷一次 */
    lv_obj_t *l = lv_obj_get_child(b, 0);
    if(l) lv_obj_set_style_text_color(l, on ? V32_C_CHIP_ON_FG : V32_C_CHIP_FG, LV_PART_MAIN);
}

static lv_obj_t *mk_chip(lv_obj_t *parent, const char *txt, lv_event_cb_t cb)
{
    lv_obj_t *b = lv_button_create(parent);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, LV_SIZE_CONTENT, V32_CHIP_H);
    chip_styles(b);

    lv_obj_t *l = lv_label_create(b);
    lv_obj_remove_style_all(l);
    /* .chip 没有 font-size，继承 body 的 16px —— 见 theme.h 的 V32_FONT_CHIP */
    lv_obj_set_style_text_font(l, V32_FONT_CHIP, LV_PART_MAIN);
    lv_obj_set_style_text_color(l, V32_C_CHIP_FG, LV_PART_MAIN);
    lv_label_set_text(l, txt);
    lv_obj_center(l);

    if(cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
    return b;
}

static void chip_set_text(lv_obj_t *b, const char *txt)
{
    lv_obj_t *l = lv_obj_get_child(b, 0);
    if(l) lv_label_set_text(l, txt);
}

/* select{ background:#1b3043; border:1px solid #36526b; border-radius:6px } */
static void style_dropdown(lv_obj_t *dd)
{
    lv_obj_set_style_bg_color(dd, V32_C_SEL_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dd, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(dd, V32_C_SEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(dd, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(dd, 6, LV_PART_MAIN);
    lv_obj_set_style_text_color(dd, V32_C_SEL_FG, LV_PART_MAIN);
    lv_obj_set_style_text_font(dd, V32_FONT_CHIP, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(dd, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(dd, 8, LV_PART_MAIN);
}

/*
 * 下拉弹出列表是**懒创建**的（第一次 open 才建），
 * 所以建完试一次，点击展开时再补一次。
 */
static void style_dropdown_popup(lv_obj_t *dd)
{
    lv_obj_t *list = lv_dropdown_get_list(dd);
    if(list == NULL) return;

    lv_obj_set_style_bg_color(list, V32_C_SEL_BG2, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(list, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(list, V32_C_SEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(list, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(list, 6, LV_PART_MAIN);
    lv_obj_set_style_text_color(list, V32_C_SEL_FG, LV_PART_MAIN);
    lv_obj_set_style_text_font(list, V32_FONT_CHIP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(list, V32_C_CHIP_ON_BG, LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(list, LV_OPA_COVER, LV_PART_SELECTED);
    lv_obj_set_style_text_color(list, V32_C_CHIP_ON_FG, LV_PART_SELECTED);
}

static void dd_open_cb(lv_event_t *e)
{
    style_dropdown_popup(lv_event_get_target(e));
}

/* select 的左右内边距：select{ padding:8px } */
#define V32_SEL_PAD_X   8
/* ▼ 和文字之间留的空隙 */
#define V32_SEL_ARROW_GAP 4

/*
 * 把下拉框宽度按最长选项算出来。
 *
 * 为什么非得自己算：
 *   LVGL 的 lv_dropdown 只定义了 height_def = LV_SIZE_CONTENT，**没有宽度自适应**
 *   （源码 lv_dropdown.c 里只在 open 时对 list 做了 set_width，没有对本体算宽度）。
 *   所以：
 *     不给宽度 -> 吃主题默认值，实测 130px（参考稿是 98 / 113，整体偏宽）
 *     给 LV_SIZE_CONTENT -> 塌成 16px，选项文字全被裁掉（更糟）
 *   参考稿的 <select> 是浏览器按最长选项 auto-size 的，这里用 lv_text_get_size
 *   把最长选项量出来，加上 padding 和 ▼ 的占位，等价复现。
 */
static void dropdown_fit_width(lv_obj_t *dd, const char *opts)
{
    lv_point_t sz, sy;
    lv_text_get_size(&sz, opts, V32_FONT_CHIP, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    lv_text_get_size(&sy, "▼", V32_FONT_CHIP, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    lv_obj_set_width(dd, sz.x + V32_SEL_PAD_X * 2 + sy.x + V32_SEL_ARROW_GAP);
}

static lv_obj_t *mk_dropdown(lv_obj_t *parent, const char *opts, int sel,
                            lv_event_cb_t cb)
{
    lv_obj_t *dd = lv_dropdown_create(parent);
    lv_dropdown_set_options(dd, opts);
    lv_dropdown_set_selected(dd, (uint32_t)sel);
    /* 用我们自己字体里有的 ▼，默认的 LV_SYMBOL_DOWN 在 Montserrat 里 */
    lv_dropdown_set_symbol(dd, "▼");
    style_dropdown(dd);
    dropdown_fit_width(dd, opts);
    style_dropdown_popup(dd);
    lv_obj_add_event_cb(dd, dd_open_cb, LV_EVENT_CLICKED, NULL);
    if(cb) lv_obj_add_event_cb(dd, cb, LV_EVENT_VALUE_CHANGED, NULL);
    return dd;
}

/* .group —— 一行「标签 + 控件」，横向排列、垂直居中 */
static lv_obj_t *mk_group(lv_obj_t *parent, const char *label_txt)
{
    lv_obj_t *g = mk(parent, 0, 0, LV_SIZE_CONTENT, V32_CHIP_H);
    lv_obj_set_flex_flow(g, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(g, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(g, V32_GROUP_GAP, LV_PART_MAIN);
    if(label_txt) mk_label(g, 0, 0, label_txt, V32_FONT_SMALL, V32_C_LABEL);
    return g;
}

/* .plot-title / .plot-foot 这种「一行多个不同颜色的小字」用 flex 横排拼 */
static lv_obj_t *mk_title_row(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *r = mk(parent, x, y, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(r, 8, LV_PART_MAIN);

    /*
     * 整行加一块和绘图区同色的不透明底板。
     *
     * 接真实数据之后纵向是自动量程，波形几乎顶满整个绘图区，
     * 就会从「原始输入 · CH1」这些标题文字上压过去，白底上看着像乱码。
     * 底板颜色取 V32_C_PLOT（和绘图区完全一致），所以只有在波形压过来时
     * 才看得出来 —— 相当于给标题留了一块「不画波形」的地方。
     * 和上一版给坐标轴文字加 opaque 底板是同一个思路（见 plot.c 的 text_plated）。
     */
    lv_obj_set_style_bg_color(r, V32_C_PLOT, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(r, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(r, 3, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(r, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(r, 1, LV_PART_MAIN);
    return r;
}

/* =========================================================
 * 数据来源
 * ========================================================= */

/* ---------------------------------------------------------
 * 真实数据（V32CAP3）缓存
 *
 * 一帧只在「帧号变了」的时候算一次，SCOPE 和 FFT 两页共用同一份结果，
 * 避免两个页面各做一遍 8192 点 FFT。
 * --------------------------------------------------------- */

/* 4096 点补零到 8192：bin 宽 = 65e6/8192 = 7.935 kHz，
 * 正好和参考稿 UI 上写的 "8192 POINTS / BIN WIDTH 7.935 kHz" 对上。
 * ⚠️ 补零只是插值，真实分辨率仍是 65e6/4096 = 15.87 kHz。 */
#define REAL_FFT_N              8192

/* 单边幅度谱归一化后满刻度 = 2048 码（12 位有符号），所以 dBFS 是
 * 20*log10(mag/2048)，不再是参考稿里那个写死的 -12.8。 */
#define REAL_FULLSCALE_COUNTS   2048.0f

static bool     s_real_valid;
static uint32_t s_real_fid = 0xFFFFFFFFu;
static int      s_real_n;
static float    s_real_d0[PLOT_MAX_PTS];    /* CH0，ADC 码值 */
static float    s_real_d1[PLOT_MAX_PTS];    /* CH1，ADC 码值 */
static float    s_real_lo;                  /* y01 = (v - lo) * sc */
static float    s_real_sc;
static float    s_real_vpp0_mv, s_real_vpp1_mv;

static float    s_real_mag[REAL_FFT_N / 2];
static int      s_real_bins;
static float    s_real_peak_hz;
static float    s_real_peak_dbfs;
static float    s_real_floor_dbfs;
static float    s_real_phase_deg;           /* CH1 相对 CH0 的相位差 */

/*
 * 断链判定。
 * 真实链路只有约 5.6 帧/秒，所以 1 秒没新帧就肯定是断了（或 FPGA 停了）。
 * 以前这里「帧号没变就沿用上一帧」，看起来像正常，其实屏幕冻在最后一帧 ——
 * 用户明确要求断开之后归零，不能保持现状。
 */
#define REAL_STALE_MS   1000u
static uint32_t s_real_t_last;          /* 最近一次收到新帧的时刻 */

/* 整帧原始数据，缩放窗口要在这上面取，不能在抽取后的 1024 点上取 */
static float    s_full0[WAVE_MAX_FRAME];
static float    s_full1[WAVE_MAX_FRAME];
static int      s_full_n;

/* AI 页「降噪后」的整帧缓存。长度与 s_full0 一致，这样 map_window()
 * 用的 s_zoom_span / s_zoom_start 对两条曲线是同一套下标，上下两图
 * 时间轴严格对齐 —— 否则放大后两张图会错位，对比就失去意义。 */
static float    s_den[WAVE_MAX_FRAME];

/*
 * 缩放窗口（两指捏放）。
 *   s_zoom_span = 可见点数，0 表示还没初始化（= 整帧）
 *   s_zoom_start = 窗口起点对应的样点下标
 *
 * 纵向另有一个独立增益 s_gain（两指上下捏），相当于示波器的 V/div：
 * 横向捏放改「看多长一段」，纵向捏放改「一格多少毫伏」，两件事互不影响。
 */
static int      s_zoom_span;
static int      s_zoom_start;

#define ZOOM_GAIN_MIN   0.05f
#define ZOOM_GAIN_MAX   200.0f

/*
 * 横向最少显示多少个样点（= 最大放大倍率）。
 * 原来是 64 点（≈0.98 µs），10 MHz 以下根本放不到 1 个周期里看细节。
 * 现在放开到 8 点（≈0.123 µs），配合下面的线性插值分支，能贴到单个样点看。
 * 上限就是整帧 4096 点 —— 一帧只有 63 µs，这是接收端的硬边界，不是缩放限制。
 */
#define ZOOM_SPAN_MIN   8

static float    s_gain = 1.0f;          /* 1.0 = 按整帧满幅铺满 */
/* 程序里同步「时基」下拉框时置位，避免 set_selected 反过来触发 cb_timebase
 * 把刚捏好的窗口又按档位拍扁（会形成「捏不动」的假象）。 */
static bool     s_tb_lock;

/* 抽样中位数当底噪：每 16 个 bin 取一个，插入排序足够快，
 * 而且不会被单根强谱线带偏（用平均值就会）。 */
static float noise_floor_mag(const float *mag, int bins)
{
    static float t[512];
    int n = 0;
    for(int i = 0; i < bins && n < (int)(sizeof t / sizeof t[0]); i += 16)
        t[n++] = mag[i];
    if(n <= 0) return 0.f;

    for(int i = 1; i < n; i++) {
        const float v = t[i];
        int j = i - 1;
        while(j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
        t[j + 1] = v;
    }
    return t[n / 2];
}

/*
 * 两路在同一个频点上的相位差（CH1 - CH0），单位度，范围 (-180, 180]。
 *
 * 只算一个频点，用 Goertzel 递推，O(N) 的几次乘加 —— 比再补两次 8192 点 FFT
 * 便宜一个数量级，而这个读数本来也只需要峰值那一根谱线。
 *
 * 递推结束后 X(k) = (s1 - cos(w)s2) + j·sin(w)s2，其辐角就是该频点的初相，
 * 这一点在 Goertzel 的标准推导里是直接的结论，不用再造窗/归一化。
 */
static float phase_diff_deg(const float *x0, const float *x1, int n,
                            float f_hz, float fs_hz)
{
    if(n < 16 || fs_hz <= 0.f) return 0.f;

    /* 取最近的整数 bin：f 必须落在这个 bin 上，否则 Goertzel 的辐角会带泄漏偏差 */
    int k = (int)(f_hz / fs_hz * (float)n + 0.5f);
    if(k < 1) k = 1;
    if(k > n / 2 - 1) k = n / 2 - 1;

    const float pi  = 3.14159265358979f;
    const float w   = 2.0f * pi * (float)k / (float)n;
    const float co  = 2.0f * cosf(w);
    const float cw  = cosf(w), sw = sinf(w);
    float a1 = 0.f, a2 = 0.f, b1 = 0.f, b2 = 0.f;

    for(int i = 0; i < n; i++) {
        float s = x0[i] + co * a1 - a2; a2 = a1; a1 = s;
        s       = x1[i] + co * b1 - b2; b2 = b1; b1 = s;
    }

    const float p0 = atan2f(sw * a2, a1 - cw * a2);
    const float p1 = atan2f(sw * b2, b1 - cw * b2);

    float d = p1 - p0;
    while(d >  pi) d -= 2.0f * pi;
    while(d < -pi) d += 2.0f * pi;
    return d * 180.0f / pi;
}

static void update_axis_label(void);
static void sync_timebase_dd(float us_div);

/*
 * 「自动」（AUTO / 自动设置）真正要做的两件事：
 *   ① 横向：用 FFT 峰值频率把窗口框成**正好 10 个完整周期**
 *   ② 纵向：满刻度 = 峰峰值的 **110%**（波形居中，上下各留 5%）
 *
 * 为什么不放在 cb_auto() 里直接算：峰值频率是 real_update() 里 FFT 算出来的，
 * 按下 AUTO 的那一刻它多半还是上一帧的值。放到 real_update() 末尾，
 * 用的就是同一帧刚算好的 s_real_peak_hz，窗口和读数天然一致。
 *
 * 只执行一次，算完清标志。之后两指捏放立即接管 —— ui_pinch_zoom/gain 会把
 * S.auto_fit 清掉，不然你捏完松手它又跳回 10 个周期，会以为是手势乱了。
 */
static void auto_fit_apply(int fn)
{
    if(fn < 8) return;

    /* ① 10 个完整周期 */
    if(s_real_peak_hz > 1.0f) {
        const wave_source_t *src = wave_source_get();
        const float fs = (src && src->sample_rate > 0)
                       ? (float)src->sample_rate : 65000000.0f;
        int span = (int)(10.0f * fs / s_real_peak_hz + 0.5f);
        if(span < 4)  span = 4;
        if(span > fn) span = fn;
        s_zoom_span  = span;
        s_zoom_start = (fn - span) / 2;
    }

    /* ② 满刻度 = 峰峰值的 110%。
     *    峰峰值按**显示窗口**统计，不按整帧 —— 整帧里有个毛刺就会把波形压小，
     *    那就不是「纵轴最高到峰峰值的 110%」了。 */
    {
        const int st0 = s_zoom_start;
        const int n   = (s_zoom_span > 0) ? s_zoom_span : fn;
        float mn = s_full0[st0], mx = s_full0[st0];
        for(int i = 0; i < n && st0 + i < fn; i++) {
            const float v0 = s_full0[st0 + i];
            const float v1 = s_full1[st0 + i];
            if(v0 < mn) mn = v0;
            if(v0 > mx) mx = v0;
            if(v1 < mn) mn = v1;
            if(v1 > mx) mx = v1;
        }
        float span = mx - mn;
        if(span < 1e-3f) span = 1e-3f;
        s_real_lo = mn - span * 0.05f;
        s_real_sc = 1.0f / (span * 1.10f);
    }

    s_gain = 1.0f;
    S.auto_fit = false;
    update_axis_label();
}

static void real_update(void)
{
    /*
     * 暂停 = 冻结。
     *
     * 这里必须**在置 s_real_valid = false 之前**返回：正常路径第一件事就是
     * 把 valid 清掉、然后靠新帧再置回来，如果暂停时也走这一步，屏幕上就会
     * 瞬间变成一条零线 —— 那不叫暂停，那叫断开。
     *
     * 返回之后不再调用 read_frame2，所以 s_full0/s_full1、各项读数、
     * FFT 幅度谱全部停在按下暂停那一刻；两指捏放仍然改 s_zoom_*，
     * 也就是可以对着这一屏静止的数据继续放大、平移查看。
     * 数据源那边的读线程照常收帧（wave_source_v32 只保留最新一帧），
     * 恢复运行时拿到的自然是最新数据。
     */
    if(!S.running) return;

    s_real_valid = false;
    if(!S.use_real) return;

    const wave_source_t *src = wave_source_get();
    if(!src || !src->read_frame2) return;

    /* 帧号没变就不重算。5.6 帧/秒的速率下重算 8192 点 FFT 纯属浪费。 */
    const v32_stats_t *st = v32_get_stats();
    if(s_real_n >= 2 && s_real_fid == st->last_frame_id) {
        /* 但没有新帧超过 1 秒，就是链路断了：保持 s_real_valid = false，
         * 让上层画零线，而不是把最后一帧冻在屏幕上。 */
        if((uint32_t)(lv_tick_get() - s_real_t_last) > REAL_STALE_MS) return;
        s_real_valid = true;
        /* 按了 AUTO 但这一帧没变（比如刚按完就暂停）也要把窗口算出来，
         * 否则标志一直挂着，等下一帧才生效，看着像 AUTO 没反应。 */
        if(S.auto_fit) auto_fit_apply(s_full_n);
        return;
    }

    static float d0[PLOT_MAX_PTS], d1[PLOT_MAX_PTS];
    static float f0[WAVE_MAX_FRAME], f1[WAVE_MAX_FRAME];

    /* ① 抽取后的显示数据。数据源用 min/max 抽取，极值不会丢，
     *    所以拿它算 Vpp 和纵向量程是准的。 */
    const int dn = src->read_frame2(d0, d1, PLOT_MAX_PTS);
    if(dn < 2) return;

    /* ② 整帧原始数据（两路都要）。数据源内部缓存同一帧，这次调用不会把它读走。 */
    const int fn = src->read_frame2(f0, f1, src->frame_len);
    if(fn < 64) return;

    /* 存一份整帧，供缩放窗口重采样 */
    if(fn <= WAVE_MAX_FRAME) {
        memcpy(s_full0, f0, (size_t)fn * sizeof(float));
        memcpy(s_full1, f1, (size_t)fn * sizeof(float));
        s_full_n = fn;
    }

    /* 帧长变了（换数据源/重启）就让缩放回到整帧 */
    if(s_zoom_span <= 0 || s_zoom_span > fn) { s_zoom_span = fn; s_zoom_start = 0; }
    if(s_zoom_start > fn - s_zoom_span) s_zoom_start = fn - s_zoom_span;
    if(s_zoom_start < 0) s_zoom_start = 0;

    float a_lo = d0[0], a_hi = d0[0], b_lo = d1[0], b_hi = d1[0];
    for(int i = 1; i < dn; i++) {
        if(d0[i] < a_lo) a_lo = d0[i];
        if(d0[i] > a_hi) a_hi = d0[i];
        if(d1[i] < b_lo) b_lo = d1[i];
        if(d1[i] > b_hi) b_hi = d1[i];
    }

    s_real_vpp0_mv = (a_hi - a_lo) * V32_ADC_MV_PER_LSB;
    s_real_vpp1_mv = (b_hi - b_lo) * V32_ADC_MV_PER_LSB;

    /* 两路共用同一纵向量程 —— 否则屏幕上 CH1 和 CH2 谁大谁小是假的 */
    {
        const float mn = (a_lo < b_lo) ? a_lo : b_lo;
        const float mx = (a_hi > b_hi) ? a_hi : b_hi;
        float span = mx - mn;
        if(span < 1e-3f) span = 1e-3f;
        s_real_lo = mn - span * 0.05f;
        s_real_sc = 1.0f / (span * 1.10f);
    }

    memcpy(s_real_d0, d0, (size_t)dn * sizeof(float));
    memcpy(s_real_d1, d1, (size_t)dn * sizeof(float));
    s_real_n = dn;

    /* ③ 频谱。用**交流成分大的那一路**做 FFT —— 另一路可能没接或者只有直流，
     *    拿它算频率会得到一个恒在 bin 1（≈0 Hz）的假读数。 */
    const float *spec_src = (s_real_vpp1_mv >= s_real_vpp0_mv) ? f1 : f0;
    fft_magnitude_spectrum_zero(spec_src, fn, REAL_FFT_N, s_real_mag);
    s_real_bins = REAL_FFT_N / 2;

    int pk = 1;
    for(int i = 2; i < s_real_bins; i++)
        if(s_real_mag[i] > s_real_mag[pk]) pk = i;

    s_real_peak_hz   = (float)pk * (float)src->sample_rate / (float)REAL_FFT_N;
    s_real_peak_dbfs = 20.0f * log10f((s_real_mag[pk] + 1e-9f) / REAL_FULLSCALE_COUNTS);
    s_real_floor_dbfs = 20.0f * log10f((noise_floor_mag(s_real_mag, s_real_bins) + 1e-9f)
                                       / REAL_FULLSCALE_COUNTS);
    s_real_phase_deg = phase_diff_deg(f0, f1, fn, s_real_peak_hz, (float)src->sample_rate);

    s_real_fid   = st->last_frame_id;
    s_real_t_last = lv_tick_get();
    s_real_valid = true;

    if(S.auto_fit) auto_fit_apply(fn);
}

/*
 * 把 [s_zoom_start, s_zoom_start + s_zoom_span) 这段样点映射成 n 个 0..1 的点。
 *
 * 窗口比输出点多（缩小时）用 min/max 分桶，一桶出两个点（先出现者在前），
 * 这样窄脉冲不会像等间隔抽点那样被抽没；窗口比输出点少（放大时）用线性插值。
 * 纵向用的是**整帧**的 `s_real_lo/s_real_sc`，不随缩放变 ——
 * 否则 Vpp 读数会跟着缩放跳，看起来像信号自己在变。
 */
static void map_window(const float *src, int src_n, float *y01, int n)
{
    if(src_n < 2 || n < 2) return;

    int span = s_zoom_span > 0 ? s_zoom_span : src_n;
    if(span > src_n) span = src_n;
    if(span < 2) span = 2;

    int start = s_zoom_start;
    if(start < 0) start = 0;
    if(start > src_n - span) start = src_n - span;

    if(span >= n * 2) {
        const int buckets = n / 2;
        for(int b = 0; b < buckets; b++) {
            const int s = start + (int)((long)b * span / buckets);
            int e = start + (int)((long)(b + 1) * span / buckets);
            if(e <= s) e = s + 1;
            if(e > src_n) e = src_n;

            int imin = s, imax = s;
            for(int k = s; k < e; k++) {
                if(src[k] < src[imin]) imin = k;
                if(src[k] > src[imax]) imax = k;
            }
            /* 极值按时间先后输出，避免把波形画成锯齿 */
            y01[2 * b]     = src[imin < imax ? imin : imax];
            y01[2 * b + 1] = src[imin < imax ? imax : imin];
        }
    } else {
        for(int i = 0; i < n; i++) {
            const float t = (float)i * (float)(span - 1) / (float)(n - 1);
            const int   a = start + (int)t;
            const float f = t - (float)(int)t;
            const int   b = (a + 1 < start + span) ? (a + 1) : a;
            y01[i] = src[a] + (src[b] - src[a]) * f;
        }
    }

    for(int i = 0; i < n; i++) {
        float y = (y01[i] - s_real_lo) * s_real_sc;
        /* 纵向增益绕中线展开：gain>1 放大（波形超出屏幕就削顶，和示波器一样） */
        y = 0.5f + (y - 0.5f) * s_gain;
        if(y < 0.f) y = 0.f;
        if(y > 1.f) y = 1.f;
        y01[i] = y;
    }
}

static void fetch_scope(int ch, float *y01, int n)
{
    if(S.use_real) {
        if(s_real_valid && s_full_n >= 2) {
            map_window(ch ? s_full1 : s_full0, s_full_n, y01, n);
            return;
        }
        /* 接了串口但没有数据 / 链路已断：平直零线，不再退回演示曲线 */
        const float zero = 0.5f;      /* y01 中点 = ADC 码值 0 */
        for(int i = 0; i < n; i++) y01[i] = zero;
        return;
    }

    /* 只有显式用 --source demo 时才画演示曲线 */
    if(ch) demo_scope_ch2(&S.sig, y01, n);
    else   demo_scope_ch1(&S.sig, y01, n);
}

/* 时基 / 幅度标签跟着缩放窗口和增益走，不再是写死的 6.302。
 * 只在文本真的变了才 set，避免每帧重绘这一小块。
 *
 * 暂停时前面挂一个「⏸ 已冻结」：冻结之后时基不用再变，但用户需要一眼看出
 * 屏幕上这一屏是停住的（否则和"链路断了"看起来一样）。
 * 缓冲区按字节算：一个汉字 3 字节，整串约 45 字节，留到 96 才够。 */
static void update_axis_label(void)
{
    const wave_source_t *src = wave_source_get();
    const float fs = (src && src->sample_rate > 0) ? (float)src->sample_rate : 1.0f;
    const int   span = s_zoom_span > 0 ? s_zoom_span
                                       : ((src && src->frame_len > 0) ? src->frame_len : V32_POINTS);
    const float us_div = (float)span / fs * 1e6f / (float)V32_GRID_X;

    static char last[96];
    char core[64];
    char buf[96];

    if(S.use_real && s_real_sc > 0.f) {
        /* 满屏码值 = 1/sc，除纵向格数、再除增益 = 每格多少码 -> 换 mV */
        const float mv_div = (1.0f / s_real_sc) / (float)V32_GRID_Y / s_gain
                             * V32_ADC_MV_PER_LSB;
        snprintf(core, sizeof core, "%.3f µs/格 · %.2f mV/格",
                 (double)us_div, (double)mv_div);
    } else {
        snprintf(core, sizeof core, "%.3f µs/格 · 触发 自动", (double)us_div);
    }
    snprintf(buf, sizeof buf, "%s%s", S.running ? "" : "⏸ 已冻结 · ", core);

    if(strcmp(buf, last) != 0) {
        lv_label_set_text(s_lbl_scope_axis, buf);
        snprintf(last, sizeof last, "%s", buf);
    }

    sync_timebase_dd(us_div);
}

/* 把「时基」下拉框拨到和当前真实窗口最接近的那一档。
 *
 * 为什么不干脆不显示：这个下拉框现在是真的能改窗口的（cb_timebase），
 * 如果它停在「2 µs/格」而左边读数写着 6.302，用户会以为又被骗了。
 * 它是**档位**不是精确值，精确值看左下角那行。 */
static void sync_timebase_dd(float us_div)
{
    if(!s_dd_tb || !S.use_real) return;

    int   best  = 0;
    float bestd = 1e9f;
    for(int i = 0; i < 5; i++) {
        float d = us_div - TIMEBASE_US[i];
        if(d < 0.f) d = -d;
        if(d < bestd) { bestd = d; best = i; }
    }
    if((int)lv_dropdown_get_selected(s_dd_tb) == best) return;

    S.tb_idx  = best;
    s_tb_lock = true;
    lv_dropdown_set_selected(s_dd_tb, (uint32_t)best);
    s_tb_lock = false;
}

/*
 * 两指捏放缩放。scale > 1 表示手指张开（放大 = 可见点数变少）。
 * anchor 是捏合中心在绘图区里的水平比例 0..1，缩放时保持该处的样点不动。
 * 由 port.c 的触摸回调调用，见 touch_read_cb()。
 */
void ui_pinch_zoom(float scale, float anchor)
{
    const int total = s_full_n > 0 ? s_full_n : V32_POINTS;
    if(scale <= 0.f) return;
    if(anchor < 0.f) anchor = 0.f;
    if(anchor > 1.f) anchor = 1.f;

    if(s_zoom_span <= 0 || s_zoom_span > total) s_zoom_span = total;

    int span = (int)((float)s_zoom_span / scale + 0.5f);
    if(span < ZOOM_SPAN_MIN) span = ZOOM_SPAN_MIN;   /* 最小 8 点 ≈ 0.123 µs */
    if(span > total) span = total;

    const int anchor_sample = s_zoom_start + (int)(anchor * (float)s_zoom_span);

    int start = anchor_sample - (int)(anchor * (float)span);
    if(start < 0) start = 0;
    if(start > total - span) start = total - span;

    s_zoom_span  = span;
    s_zoom_start = start;
    S.auto_fit = false;          /* 手动缩放立即接管，别让 AUTO 又跳回 10 个周期 */
    update_axis_label();
}

/*
 * 纵向增益（示波器的 V/div）。ratio > 1 = 两指上下张开 = 波形变高。
 * 由 port.c 的触摸回调调用；和横向捏放各管一轴，同时用也不冲突。
 */
void ui_pinch_gain(float ratio)
{
    if(ratio <= 0.f) return;
    s_gain *= ratio;
    if(s_gain < ZOOM_GAIN_MIN) s_gain = ZOOM_GAIN_MIN;
    if(s_gain > ZOOM_GAIN_MAX) s_gain = ZOOM_GAIN_MAX;
    S.auto_fit = false;          /* 纵向手动调整同样接管，理由见 ui_pinch_zoom */
    if(S.ch1 || S.ch2) update_axis_label();
}

static void fetch_spectrum(float *y01, int n, float *peak_x, float *peak_y01)
{
    /*
     * 真实链路：直接用 real_update() 算好的幅度谱。
     * 纵轴是**绝对** dBFS（满刻度 2048 码 = 0 dBFS），不是相对峰值归一化 ——
     * 这样 PEAK LEVEL / NOISE FLOOR 两个读数才有物理意义。
     */
    if(S.use_real && s_real_valid && s_real_bins > 2) {
        int   bi = 1;
        float bv = -1e30f;
        for(int i = 0; i < n; i++) {
            const int si = (int)((long)i * (s_real_bins - 1) / (n > 1 ? n - 1 : 1));
            const float db = 20.0f * log10f((s_real_mag[si] + 1e-9f) / REAL_FULLSCALE_COUNTS);
            float y = (db + V32_DEMO_DB_FULLSCALE) / V32_DEMO_DB_FULLSCALE;
            if(y < 0.f) y = 0.f;
            if(y > 1.f) y = 1.f;
            y01[i] = y;
            if(y > bv) { bv = y; bi = i; }
        }
        if(peak_x)   *peak_x   = (float)bi / (float)(n - 1);
        if(peak_y01) *peak_y01 = bv;
        return;
    }

    /* 接了串口但没有数据 / 断链：地板线，读数后面会归零 */
    if(S.use_real) {
        for(int i = 0; i < n; i++) y01[i] = 0.f;
        if(peak_x)   *peak_x   = 0.f;
        if(peak_y01) *peak_y01 = 0.f;
        return;
    }

    demo_fft(&S.sig, y01, n, peak_x, peak_y01);
}

/* =========================================================
 * 页面 1：SCOPE
 * ========================================================= */
static void build_scope(lv_obj_t *pg)
{
    lv_obj_set_flex_flow(pg, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(pg, V32_PAGE_GAP, LV_PART_MAIN);

    s_plot_scope = plot_create(pg, PLOT_SCOPE);
    lv_obj_set_flex_grow(s_plot_scope, 1);
    lv_obj_set_height(s_plot_scope, MAIN_H);

    lv_obj_t *tr = mk_title_row(s_plot_scope, V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_t *t0 = mk_label(tr, 0, 0, "时域波形", V32_FONT_SMALL, V32_C_PLOT_TITLE);
    lv_obj_set_style_text_letter_space(t0, 1, LV_PART_MAIN);
    mk_label(tr, 0, 0, "● CH1", V32_FONT_SMALL, V32_C_CH1);
    mk_label(tr, 0, 0, "● CH2", V32_FONT_SMALL, V32_C_CH2);

    s_lbl_scope_axis = mk_label(s_plot_scope, 0, 0, "2 µs/格 · 触发 自动",
                                V32_FONT_SMALL, V32_C_PLOT_FOOT);
    /* 垫一块和画布同色的底板：这行字压在左下角，波形经常从它上面穿过去，
     * 白底主题下不垫就读不出来（FFT 页的角标同理，见 plot.c 的 text_plated）。 */
    lv_obj_set_style_bg_color(s_lbl_scope_axis, V32_C_PLOT, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_lbl_scope_axis, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_lbl_scope_axis, 3, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(s_lbl_scope_axis, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(s_lbl_scope_axis, 1, LV_PART_MAIN);
    lv_obj_align(s_lbl_scope_axis, LV_ALIGN_BOTTOM_LEFT, V32_PLOT_FOOT_X, -V32_PLOT_FOOT_Y);

    /* ---- 右侧读数栏 ---- */
    lv_obj_t *aside = mk(pg, 0, 0, ASIDE_W, MAIN_H);
    s_aside[UI_PAGE_SCOPE] = aside;
    lv_obj_set_flex_flow(aside, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(aside, V32_RIGHT_GAP, LV_PART_MAIN);

    lv_obj_t *p1 = mk_panel(aside, 0, 0, 0, 0);
    lv_obj_set_width(p1, LV_PCT(100));
    lv_obj_set_flex_grow(p1, 1);
    mk_title(p1, 0, 0, "测量读数");
    s_lbl_v1         = mk_metric(p1, TITLE_H + METRIC_H * 0, "CH1 峰峰值",  "20.4 mV",    V32_C_CH1);
    s_lbl_v2         = mk_metric(p1, TITLE_H + METRIC_H * 1, "CH2 峰峰值",  "18.7 mV",    V32_C_CH2);
    s_lbl_scope_freq = mk_metric(p1, TITLE_H + METRIC_H * 2, "频率",       "10.000 MHz", V32_C_METRIC_VALUE);
    s_lbl_phase      = mk_metric(p1, TITLE_H + METRIC_H * 3, "相位差",     "43.7°",      V32_C_METRIC_VALUE);
    mk_metric(p1, TITLE_H + METRIC_H * 4, "触发", "自动", V32_C_GREEN);

    /*
     * ⚠️ 这里必须把面板自己的 2px 边框算进去（V32_PANEL_BW * 2）。
     *    CSS 默认 box-sizing:content-box 的直觉在这儿会坑人：
     *    参考稿 .panel 的高度是「内容 + padding + **border**」，
     *    而 LVGL 的 height 是 border-box（边框算在里面）。
     *    少算这 2px，这块面板就矮 2px，而这个高度又反过来把上面
     *    flex:1 的 MEASUREMENTS 撑高 2px —— 两块的边框线一起偏 2px。
     *    （反解 ref_scope.png：CHANNELS 面板 y=355..442 共 88px）
     */
    lv_obj_t *p2 = mk_panel(aside, 0, 0, 0,
                            TITLE_H + V32_CHIP_H + V32_PANEL_PAD * 2
                            + V32_PANEL_BW * 2);
    lv_obj_set_width(p2, LV_PCT(100));
    mk_title(p2, 0, 0, "通道");

    lv_obj_t *row = mk(p2, 0, TITLE_H, 0, V32_CHIP_H);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 7, LV_PART_MAIN);
    s_chip_ch1 = mk_chip(row, "CH1 开", cb_ch1);
    s_chip_ch2 = mk_chip(row, "CH2 开", cb_ch2);
    /*
     * ⚠️ 必须在这里补一次"选中"着色。
     *    mk_chip() 建出来的是 chip_styles() 的**未选中**配色，选中态只在
     *    cb_ch1/cb_ch2/cb_ai/cb_reset 这几个回调里刷。而 S.ch1/S.ch2/S.ai
     *    初始就是 true，于是不补这一下，开机时两个按钮会显示成未选中的
     *    灰字暗边（参考稿里是亮青字 #70e8f5 + 亮青边 #56d8e8）。
     *    这个 bug 很隐蔽：点一下按钮颜色就正常了，所以只看静态截图才抓得到。
     */
    chip_selected(s_chip_ch1, S.ch1);
    chip_selected(s_chip_ch2, S.ch2);
}

/* =========================================================
 * 页面 2：FFT
 * ========================================================= */
static void build_fft(lv_obj_t *pg)
{
    lv_obj_set_flex_flow(pg, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(pg, V32_PAGE_GAP, LV_PART_MAIN);

    s_plot_fft = plot_create(pg, PLOT_FFT);
    lv_obj_set_flex_grow(s_plot_fft, 1);
    lv_obj_set_height(s_plot_fft, MAIN_H);
    plot_set_axis_labels(s_plot_fft, "0 MHz", "32.5 MHz");

    lv_obj_t *tr = mk_title_row(s_plot_fft, V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_t *t0 = mk_label(tr, 0, 0, "FFT 频谱 · 8192 点",
                            V32_FONT_SMALL, V32_C_PLOT_TITLE);
    lv_obj_set_style_text_letter_space(t0, 1, LV_PART_MAIN);

    /*
     * 左下角原来还有一行 LVGL 标签 "0 — 32.5 MHz · dBFS"。
     * 但画布自己已经用 plot_set_axis_labels() 在**同一行**画了 "0 MHz" 和
     * "32.5 MHz"（见 plot_set_axis_labels 的调用），两串字正好叠在一起：
     * 白底上放大看就是 "0OMHz32.5 MHz dB" 一团。删掉这行，只留画布角标。
     */

    lv_obj_t *aside = mk(pg, 0, 0, ASIDE_W, MAIN_H);
    s_aside[UI_PAGE_FFT] = aside;
    lv_obj_set_flex_flow(aside, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *p1 = mk_panel(aside, 0, 0, 0, 0);
    lv_obj_set_width(p1, LV_PCT(100));
    lv_obj_set_flex_grow(p1, 1);
    mk_title(p1, 0, 0, "频谱分析");
    s_lbl_fft_peakf = mk_metric(p1, TITLE_H + METRIC_H * 0, "峰值频率",   "10.000 MHz", V32_C_GREEN);
    s_lbl_fft_peakl = mk_metric(p1, TITLE_H + METRIC_H * 1, "峰值电平",   "-12.8 dBFS", V32_C_METRIC_VALUE);
    s_lbl_fft_floor = mk_metric(p1, TITLE_H + METRIC_H * 2, "底噪",      "-79.4 dBFS", V32_C_METRIC_VALUE);
    mk_metric(p1, TITLE_H + METRIC_H * 3, "频率分辨率", "7.935 kHz", V32_C_METRIC_VALUE);
    s_lbl_fft_win   = mk_metric(p1, TITLE_H + METRIC_H * 4, "窗函数",    "HANN",      V32_C_METRIC_VALUE);
}

/* =========================================================
 * 页面 3：LOCK-IN
 * ========================================================= */
static void build_lock(lv_obj_t *pg)
{
    lv_obj_set_flex_flow(pg, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(pg, V32_PAGE_GAP, LV_PART_MAIN);

    /* .twocol：两块等宽面板 */
    lv_obj_t *two = mk(pg, 0, 0, 0, MAIN_H);
    lv_obj_set_flex_grow(two, 1);
    lv_obj_set_height(two, MAIN_H);
    lv_obj_set_flex_flow(two, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(two, V32_PAGE_GAP, LV_PART_MAIN);

    lv_obj_t *L = mk_panel(two, 0, 0, 0, MAIN_H);
    lv_obj_set_flex_grow(L, 1);
    lv_obj_set_height(L, MAIN_H);
    lv_obj_set_flex_flow(L, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(L, V32_SECTION_GAP, LV_PART_MAIN);
    lv_obj_t *lt = mk_title(L, 0, 0, "锁相矢量 · I / Q");
    lv_obj_set_width(lt, LV_PCT(100));
    s_plot_iq = plot_create(L, PLOT_IQ);
    lv_obj_set_width(s_plot_iq, LV_PCT(100));
    lv_obj_set_flex_grow(s_plot_iq, 1);
    lv_obj_set_style_border_width(s_plot_iq, 0, LV_PART_MAIN);   /* style="border:0" */

    lv_obj_t *R = mk_panel(two, 0, 0, 0, MAIN_H);
    lv_obj_set_flex_grow(R, 1);
    lv_obj_set_height(R, MAIN_H);
    lv_obj_set_flex_flow(R, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(R, V32_SECTION_GAP, LV_PART_MAIN);
    lv_obj_t *rt = mk_title(R, 0, 0, "锁相输出 · 时间历程");
    lv_obj_set_width(rt, LV_PCT(100));
    s_plot_lock = plot_create(R, PLOT_HIST);
    lv_obj_set_width(s_plot_lock, LV_PCT(100));
    lv_obj_set_flex_grow(s_plot_lock, 1);
    lv_obj_set_style_border_width(s_plot_lock, 0, LV_PART_MAIN);

    lv_obj_t *aside = mk(pg, 0, 0, ASIDE_W, MAIN_H);
    s_aside[UI_PAGE_LOCK] = aside;
    lv_obj_set_flex_flow(aside, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *p1 = mk_panel(aside, 0, 0, 0, 0);
    lv_obj_set_width(p1, LV_PCT(100));
    lv_obj_set_flex_grow(p1, 1);
    mk_title(p1, 0, 0, "同步检波");
    s_lbl_lock_ref   = mk_metric(p1, TITLE_H + METRIC_H * 0, "参考频率",  "10.000 MHz", V32_C_METRIC_VALUE);
    s_lbl_lock_amp   = mk_metric(p1, TITLE_H + METRIC_H * 1, "幅度",      "8.6 mV",     V32_C_CH1);
    s_lbl_lock_phase = mk_metric(p1, TITLE_H + METRIC_H * 2, "相位",      "43.7°",      V32_C_CH2);
    s_lbl_lock_tau   = mk_metric(p1, TITLE_H + METRIC_H * 3, "时间常数",  "10 ms",      V32_C_METRIC_VALUE);
    mk_metric(p1, TITLE_H + METRIC_H * 4, "锁定状态", "跟踪中", V32_C_GREEN);
    mk_note(p1, TITLE_H + METRIC_H * 5 + 6, ASIDE_W - V32_PANEL_PAD * 2,
            "矢量图与积分输出为演示数据，实际结果以 FPGA DDC/CORDIC 为准。");
}

/* =========================================================
 * 页面 4：AI
 * ========================================================= */
static void build_ai(lv_obj_t *pg)
{
    lv_obj_set_flex_flow(pg, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(pg, V32_PAGE_GAP, LV_PART_MAIN);

    /* .stack：上下两块，各一半高 */
    lv_obj_t *stack = mk(pg, 0, 0, 0, MAIN_H);
    lv_obj_set_flex_grow(stack, 1);
    lv_obj_set_height(stack, MAIN_H);
    lv_obj_set_flex_flow(stack, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(stack, V32_PAGE_GAP, LV_PART_MAIN);

    const int half = (MAIN_H - V32_PAGE_GAP) / 2;      /* 188 */

    s_plot_raw = plot_create(stack, PLOT_AI);
    lv_obj_set_width(s_plot_raw, LV_PCT(100));
    lv_obj_set_height(s_plot_raw, half);
    lv_obj_t *tr1 = mk_title_row(s_plot_raw, V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_t *t1 = mk_label(tr1, 0, 0, "原始输入 · CH1", V32_FONT_SMALL, V32_C_PLOT_TITLE);
    lv_obj_set_style_text_letter_space(t1, 1, LV_PART_MAIN);
    mk_label(tr1, 0, 0, "●", V32_FONT_SMALL, V32_C_CH1);

    s_plot_den = plot_create(stack, PLOT_AI);
    lv_obj_set_width(s_plot_den, LV_PCT(100));
    lv_obj_set_height(s_plot_den, MAIN_H - V32_PAGE_GAP - half);
    lv_obj_t *tr2 = mk_title_row(s_plot_den, V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_t *t2 = mk_label(tr2, 0, 0, "降噪输出 · 中值 5 点", V32_FONT_SMALL, V32_C_PLOT_TITLE);
    lv_obj_set_style_text_letter_space(t2, 1, LV_PART_MAIN);
    mk_label(tr2, 0, 0, "●", V32_FONT_SMALL, V32_C_GREEN);

    lv_obj_t *aside = mk(pg, 0, 0, ASIDE_W, MAIN_H);
    s_aside[UI_PAGE_AI] = aside;
    lv_obj_set_flex_flow(aside, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *p1 = mk_panel(aside, 0, 0, 0, 0);
    lv_obj_set_width(p1, LV_PCT(100));
    lv_obj_set_flex_grow(p1, 1);
    mk_title(p1, 0, 0, "降噪处理");
    mk_metric(p1, TITLE_H + METRIC_H * 0, "算法",     "中值 5 点", V32_C_METRIC_VALUE);
    s_lbl_ai_stat = mk_metric(p1, TITLE_H + METRIC_H * 1, "状态", "已启用",   V32_C_GREEN);
    mk_metric(p1, TITLE_H + METRIC_H * 2, "窗口",  "77 ns*",    V32_C_METRIC_VALUE);
    mk_metric(p1, TITLE_H + METRIC_H * 3, "延迟",   "0 帧",      V32_C_METRIC_VALUE);
    mk_metric(p1, TITLE_H + METRIC_H * 4, "SNR 增益",  "—",         V32_C_METRIC_VALUE);
    mk_note(p1, TITLE_H + METRIC_H * 5 + 6, ASIDE_W - V32_PANEL_PAD * 2,
            "*5 点 @65 MSPS。两图都是串口真实帧。中值会把过渡样点当离群点压掉，"
            "边沿反而变陡；边沿宽度不能以本页为准。");
}

/* =========================================================
 * 页面 5：SYSTEM（这一页显示真实数据）
 * ========================================================= */
static void build_system(lv_obj_t *pg)
{
    lv_obj_set_flex_flow(pg, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(pg, V32_PAGE_GAP, LV_PART_MAIN);

    /* .twocol，没有右栏 */
    lv_obj_t *two = mk(pg, 0, 0, MAIN_W, MAIN_H);
    lv_obj_set_flex_flow(two, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(two, V32_PAGE_GAP, LV_PART_MAIN);

    lv_obj_t *L = mk_panel(two, 0, 0, 0, MAIN_H);
    lv_obj_set_flex_grow(L, 1);
    lv_obj_set_height(L, MAIN_H);
    const int lw = (MAIN_W - V32_PAGE_GAP) / 2 - V32_PANEL_PAD * 2;

    mk_title(L, 0, 0, "设备状态");
    mk_kv(L, TITLE_H + KV_H * 0, lw, "主控 CPU",    "Rockchip RK3506",       V32_C_METRIC_VALUE);
    mk_kv(L, TITLE_H + KV_H * 1, lw, "操作系统",     "Buildroot / Linux 6.1", V32_C_METRIC_VALUE);
    mk_kv(L, TITLE_H + KV_H * 2, lw, "显示屏",       "1024 × 600 / MIPI DSI", V32_C_METRIC_VALUE);
    mk_kv(L, TITLE_H + KV_H * 3, lw, "ADC 目标",    "65 MSPS / 12 bit × 2",  V32_C_METRIC_VALUE);
    /* 链路那一行要写**驱动实际生效**的波特率：--baud 2000000 若被 UART 时钟
     * 分频取整，这里就会显示真实值，而不是我们请求的值。 */
    static char s_link_txt[32];
    if(S.use_real) snprintf(s_link_txt, sizeof s_link_txt, "串口 %d 8N1",
                            wave_source_v32_get_baud());
    mk_kv(L, TITLE_H + KV_H * 4, lw, "FPGA 链路",
          S.use_real ? s_link_txt : "模拟",
          S.use_real ? V32_C_GREEN : V32_C_WARN);
    mk_kv(L, TITLE_H + KV_H * 5, lw, "数据源",
          S.use_real ? v32_source_name() : "DemoSource", V32_C_METRIC_VALUE);

    lv_obj_t *R = mk_panel(two, 0, 0, 0, MAIN_H);
    lv_obj_set_flex_grow(R, 1);
    lv_obj_set_height(R, MAIN_H);

    mk_title(R, 0, 0, "诊断");
    s_lbl_fps    = mk_kv(R, TITLE_H + KV_H * 0, lw, "渲染帧率",         "—",             V32_C_GREEN);
    /* 这两个原来是写死的 "— (DEMO)"。接上 V32CAP3 之后填的是 UART 实测值：
     *   Network Throughput = 帧率 × 16428 字节/帧 的净荷速率
     *   Packet Loss        = CRC 错误 + 序列错误（丢掉的帧）
     * 标签名沿用参考稿，不动布局。 */
    s_lbl_v32_rate = mk_kv(R, TITLE_H + KV_H * 1, lw, "链路吞吐",
                           S.use_real ? "—" : "—（演示）", V32_C_METRIC_VALUE);
    s_lbl_v32_loss = mk_kv(R, TITLE_H + KV_H * 2, lw, "丢包统计",
                           S.use_real ? "—" : "—（演示）", V32_C_METRIC_VALUE);
    mk_kv(R, TITLE_H + KV_H * 3, lw, "FPGA 固件",
          S.use_real ? "V32CAP3 v3 / 4096" : "未连接", V32_C_METRIC_VALUE);
    s_lbl_uptime = mk_kv(R, TITLE_H + KV_H * 4, lw, "运行时长", "00:00",      V32_C_METRIC_VALUE);
    /*
     * 参考稿原文是「此网页是 UI 原型，不会自动连接实际 FPGA…」。
     * 接上真实链路之后这句话已经不成立了（板子连的就是 /dev/ttyS1 上的
     * V32CAP3 帧流，上面几行全是实测值），所以改写成现在真正的事实：
     * 帧率/吞吐/丢包是板载实测，温度和网络没有接入。
     */
    mk_note(R, TITLE_H + KV_H * 5 + 6, lw,
            "渲染帧率、链路吞吐与丢包均为板载实测值；设备温度与网络状态未接入。");
}

/* =========================================================
 * 顶栏
 * ========================================================= */
/*
 * 从右往左依次摆放一组控件：right_edge 是最右边界（屏幕绝对坐标），
 * gaps[i] 是 items[i] 与 items[i+1] 之间的间距（长度 n-1，所以要支持不一样）。
 * 每项按**自身真实宽度**（含 padding）定位并垂直居中。
 *
 * ⚠️ 为什么不用 flex 干这件事（这是实测踩出来的坑，别再改回去）：
 *    LVGL 9.1 里「LV_SIZE_CONTENT 的 flex 容器」算出来的**自身**尺寸是错的。
 *    实测顶栏状态串：4 个子项内容宽 336（flex 也确实把它们摆到了正确的
 *    绝对坐标 672/774/868/952），但容器自己报的却是 90x22 —— 正好等于
 *    第一个子项 badge 的宽度。
 *    而 LVGL 默认会把子项裁剪到父容器边界内（LV_OBJ_FLAG_OVERFLOW_VISIBLE
 *    默认是关的），于是 672..918 那段整段被裁掉，屏幕上只剩最后一个子项
 *    尾巴上的 "ING"。lv_obj_update_layout() 也救不回来。
 *    改成自己量、自己算 x，就完全不依赖容器的尺寸推导了。
 */
static void layout_right_to_left(lv_obj_t *parent, int32_t right_edge,
                                 const int32_t *gaps,
                                 lv_obj_t *const *items, int n)
{
    LV_UNUSED(parent);
    int32_t x = right_edge;
    for(int i = n - 1; i >= 0; i--) {
        lv_obj_t *o = items[i];
        if(o != NULL) {
            x -= lv_obj_get_width(o);
            lv_obj_align(o, LV_ALIGN_LEFT_MID, x, 0);
        }
        if(i > 0) x -= gaps[i - 1];
    }
}

/* ── 顶部状态串：DEMO MODE · ADC 65 MSPS · ● RUNNING · 00:00:00 ── */
static lv_obj_t *s_st_badge, *s_st_adc, *s_st_adc2, *s_st_live, *s_st_clock;
static lv_obj_t *s_st_items[5];              /* 顺序即从左到右 */
#define V32_TOP_PAD_X   16                   /* .top 的 padding:0 16px */
#define V32_ST_GAP      12                   /* .statuses 的 gap:12px */
/*
 * 品牌区两行的纵向落点。
 *
 * .top 是 `align-items:center`，但**不能照着做**：LVGL 居中依赖控件自身高度，
 * 而我们的字体不是参考稿的 Inter/系统字体，行高不一样，居中后墨迹位置必然偏。
 * 实测（反解 ref_scope.png 的墨迹行段）：
 *     ref  主标题墨迹 y=10..24    副标题墨迹 y=33..40
 *     我们 主标题墨迹 y= 8..21    副标题墨迹 y=28..36
 * 也就是主标题高了 2px、副标题高了 5px —— 顺带说明两行的间距也小 3px。
 * 所以改成**按顶部绝对定位**，两个数各自可调：
 *   V32_BRAND_DY   ：主标题行框相对顶栏内容区顶部（y=1）的偏移
 *   V32_BRAND_GAP  ：主标题行框底到副标题行框顶的间距
 * 这两个值是用 "行框顶 = 墨迹顶 - 字体墨迹上沿偏移" 反推的，改动后必须
 * 重跑 tools/grid_probe.py 核对，不要凭感觉调。
 */
#define V32_BRAND_DY    7
#define V32_BRAND_GAP   3

/* "ADC " 和 "65 MSPS" 是一组，中间只有词间距，不该插 12px */
static const int32_t s_st_gaps[4] = { V32_ST_GAP, 0, V32_ST_GAP, V32_ST_GAP };

/* 文本改了宽度就变了，必须重新量、重新排，否则会盖到左边或留下空隙 */
static void layout_topbar_status(void)
{
    lv_obj_update_layout(s_top);             /* 先把新文本的宽度刷出来 */
    /* 右边界相对顶栏自身：内区宽 1022，padding 16 -> 1006（不是 1024-16） */
    layout_right_to_left(s_top, V32_SHELL_W - V32_TOP_PAD_X, s_st_gaps,
                         s_st_items, 5);
}

static void build_topbar(lv_obj_t *root)
{
    /*
     * 参考稿 .top{ display:flex; align-items:center; justify-content:space-between;
     *              padding:0 16px; gap:12px }
     *
     * 左右两端都是「按内容宽度贴边」，而 LVGL 里凡是落点依赖自身宽度的对齐
     * （LV_ALIGN_RIGHT_MID 之类）都很容易被内容宽度解析时序坑到，
     * 所以这里两端都用与宽度无关的方式定位：
     *   左：LV_ALIGN_LEFT_MID —— 落点只看左边界，跟自身宽度无关
     *   右：layout_topbar_status() —— 自己量宽度从右往左排
     */
    lv_obj_t *top = mk(root, 0, 0, V32_SHELL_W, V32_ROW_TOP);
    s_top = top;
    lv_obj_set_style_bg_color(top, V32_C_TOPBAR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(top, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(top, V32_C_TOPBAR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(top, 1, LV_PART_MAIN);
    lv_obj_set_style_border_side(top, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);

    /* ---- 左：品牌两行 ---- */
    lv_obj_t *brand = mk(top, 0, 0, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(brand, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(brand, V32_BRAND_GAP, LV_PART_MAIN);
    /* 按顶部定位而不是居中：居中会因为字体行高与参考稿不同而整体偏移（见上） */
    lv_obj_align(brand, LV_ALIGN_TOP_LEFT, V32_TOP_PAD_X, V32_BRAND_DY);

    /* .brand 的 "V32 " 是品牌青、后面是深墨蓝，整行 font-weight:800。
     * LVGL 9 把 v8 的 lv_label_set_recolor 删了，所以拆两个 label 横排。 */
    lv_obj_t *line1 = mk(brand, 0, 0, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(line1, LV_FLEX_FLOW_ROW);
    lv_obj_t *b1 = mk_label(line1, 0, 0, "V32 ", V32_FONT_BIG_B, V32_C_BRAND);
    lv_obj_set_style_text_letter_space(b1, 1, LV_PART_MAIN);
    lv_obj_t *b2 = mk_label(line1, 0, 0, "信号分析仪", V32_FONT_BIG_B, V32_C_BRAND_TEXT);
    lv_obj_set_style_text_letter_space(b2, 1, LV_PART_MAIN);

    lv_obj_t *b3 = mk_label(brand, 0, 0, "双通道 · FPGA + Linux",
                            V32_FONT_SMALL, V32_C_BRAND_SMALL);
    lv_obj_set_style_text_letter_space(b3, 2, LV_PART_MAIN);

    /* ---- 右：状态串 ---- */

    /* .mode-badge{ background:#3b3321; color:#ffd36b; border-radius:4px }
     * 接串口跑的时候不能还挂「DEMO MODE」——用户明确说上电就是读串口，
     * 没有示例数据了。 */
    s_st_badge = mk_label(top, 0, 0, S.use_real ? "串口模式" : "演示模式",
                          V32_FONT_SMALL, V32_C_BADGE_FG);
    lv_obj_set_style_bg_color(s_st_badge, V32_C_BADGE_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_st_badge, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_st_badge, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(s_st_badge, 7, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(s_st_badge, 4, LV_PART_MAIN);

    /* 参考稿这里是 `ADC <b>65 MSPS</b>` —— 只有后半段加粗。
     * 拆成两个紧挨着的 label（间距 0），不要合成一条，
     * 否则要么整条都不粗、要么整条都粗，和参考稿对不上。 */
    s_st_adc   = mk_label(top, 0, 0, "ADC ", V32_FONT_SMALL, V32_C_STATUS);
    s_st_adc2  = mk_label(top, 0, 0, "65 MSPS", V32_FONT_SMALL_B, V32_C_STATUS);
    /* "● RUNNING" 合成一个 label。参考稿里是 <i class="dot"></i> + <span>，
     * 两者同色、间距只有 4px，合成一个 label 视觉上等价，
     * 却省掉了「一个小圆点 + 一段文字」这对需要特例处理的定位。 */
    s_st_live  = mk_label(top, 0, 0, "● 运行中", V32_FONT_SMALL, V32_C_LIVE);
    s_st_clock = mk_label(top, 0, 0, "00:00:00", V32_FONT_SMALL, V32_C_STATUS);

    s_st_items[0] = s_st_badge;
    s_st_items[1] = s_st_adc;
    s_st_items[2] = s_st_adc2;
    s_st_items[3] = s_st_live;
    s_st_items[4] = s_st_clock;

    s_lbl_run_status = s_st_live;
    s_lbl_clock      = s_st_clock;

    layout_topbar_status();
}

/* 改运行状态文字。文本长度会变（运行中/已暂停），所以必须重排一遍。 */
static void set_run_status(bool running)
{
    if(s_lbl_run_status == NULL) return;
    lv_label_set_text(s_lbl_run_status, running ? "● 运行中" : "● 已暂停");
    lv_obj_set_style_text_color(s_lbl_run_status,
                                running ? V32_C_LIVE : V32_C_WARN, LV_PART_MAIN);
    layout_topbar_status();
}

/*
 * 原来这里有一行右下角常驻注记：
 *   "V32 · 可触摸交互的 1024×600 参赛 UI 原型 · 所有数值为模拟数据"
 * （对应参考稿的 .fullscreen-note，position:fixed 压在导航条上，横跨 x≈500..1012、
 *   底边 y=588，正好盖住 ⚙ SYSTEM 按钮。）
 *
 * 现在它不只多余，而且是**错的**：接上真实链路后「所有数值为模拟数据」不成立，
 * 按用户要求整行删除。ui_create() 里也不再调用。
 */

/* =========================================================
 * 控制条
 * ========================================================= */

/* ---- 事件回调 ---- */
static void cb_nav(lv_event_t *e)
{
    ui_set_page((int)(intptr_t)lv_event_get_user_data(e));
}

static void cb_freq(lv_event_t *e)
{
    S.freq_idx = (int)lv_dropdown_get_selected(lv_event_get_target(e));
    ui_tick();
}

static void cb_timebase(lv_event_t *e)
{
    if(s_tb_lock) return;        /* 程序在同步档位，不是用户点的 */
    S.tb_idx = (int)lv_dropdown_get_selected(lv_event_get_target(e));

    if(S.use_real) {
        /*
         * 真实链路：这个下拉框**现在真的改时间窗口**。
         * 以前这里只是把标签覆盖回去，点了等于没反应 —— 用户会以为控件坏了。
         *
         *   一屏 = V32_GRID_X 格  ⇒  span = (µs/格) × 格数 × fs
         *
         * 上限是整帧：4096 点 @65 MSPS 只有 63 µs，想看更长的时间窗
         * 只能让 FPGA 端做抽取（协议头里有 decimation 字段，见 v32cap3.h）。
         */
        const wave_source_t *src = wave_source_get();
        const float fs = (src && src->sample_rate > 0) ? (float)src->sample_rate : 65000000.0f;
        const int   total = s_full_n > 0 ? s_full_n : V32_POINTS;

        int span = (int)(TIMEBASE_US[S.tb_idx] * 1e-6f * fs * (float)V32_GRID_X + 0.5f);
        if(span < ZOOM_SPAN_MIN) span = ZOOM_SPAN_MIN;
        if(span > total)         span = total;

        /* 以当前窗口中心为基准缩放，改时基时波形不会突然跳到一边 */
        const int c = s_zoom_start + ((s_zoom_span > 0 ? s_zoom_span : total) / 2);
        int start = c - span / 2;
        if(start < 0) start = 0;
        if(start > total - span) start = total - span;

        s_zoom_span  = span;
        s_zoom_start = start;
        S.auto_fit   = false;        /* 手动改时基就退出 AUTO 跟踪 */
        update_axis_label();
    } else {
        lv_label_set_text_fmt(s_lbl_scope_axis, "%g µs/格 · 触发 自动",
                              (double)TIMEBASE_US[S.tb_idx]);
    }
    ui_tick();
}

static void cb_window(lv_event_t *e)
{
    S.win_idx = (int)lv_dropdown_get_selected(lv_event_get_target(e));
    ui_tick();
}

static void cb_tau(lv_event_t *e)
{
    S.tau_idx = (int)lv_dropdown_get_selected(lv_event_get_target(e));
    if(TAU_MS[S.tau_idx] >= 1000) lv_label_set_text(s_lbl_lock_tau, "1 s");
    else                          lv_label_set_text_fmt(s_lbl_lock_tau, "%d ms", TAU_MS[S.tau_idx]);
}

static void cb_ch1(lv_event_t *e)
{
    LV_UNUSED(e);
    S.ch1 = !S.ch1;
    chip_set_text(s_chip_ch1, S.ch1 ? "CH1 开" : "CH1 关");
    chip_selected(s_chip_ch1, S.ch1);
    ui_tick();
}

static void cb_ch2(lv_event_t *e)
{
    LV_UNUSED(e);
    S.ch2 = !S.ch2;
    chip_set_text(s_chip_ch2, S.ch2 ? "CH2 开" : "CH2 关");
    chip_selected(s_chip_ch2, S.ch2);
    ui_tick();
}

static void cb_ai(lv_event_t *e)
{
    LV_UNUSED(e);
    S.ai = !S.ai;
    chip_set_text(s_chip_ai, S.ai ? "降噪 开" : "降噪 关");
    chip_selected(s_chip_ai, S.ai);
    lv_label_set_text(s_lbl_ai_stat, S.ai ? "已启用" : "旁路");
    ui_tick();
}

/*
 * 暂停 = 冻结这一屏。
 *
 * 真正的"停止取数"在 real_update() 开场那一句 `if(!S.running) return;`，
 * 这里只负责界面反馈：
 *   - 按钮文字 暂停/运行
 *   - 顶栏 运行中/已暂停
 *   - 左下角时基前加「⏸ 已冻结」，让人一眼看出这一屏是停住的而不是断了
 * 冻结期间 s_full0/s_full1 原样保留，两指捏放照旧生效，
 * 所以「保留一定时间长度的信息来查看」= 冻住之后还能放大看细节。
 */
static void cb_run(lv_event_t *e)
{
    LV_UNUSED(e);
    S.running = !S.running;
    chip_set_text(s_chip_run, S.running ? "⏸ 暂停" : "▶ 运行");
    set_run_status(S.running);
    update_axis_label();
}

/*
 * 自动 —— 对应示波器的 Auto Set。
 *
 * 一路捏放/暂停之后，用户很可能把波形捏成了满天飞或者压成一条线，
 * 这时候要有一个"一键回到看得见"的入口。做法是：
 *   横向回到整帧（s_zoom_span = 0 表示整帧）、纵向增益回 1:1、
 *   两个通道强制打开、并恢复运行。
 * 纵向量程本来就是每帧按 min/max 自动算的（s_real_lo/s_real_sc），
 * 所以增益回到 1.0 就等于「自动量程铺满 8 格」。
 */
static void cb_auto(lv_event_t *e)
{
    LV_UNUSED(e);

    if(S.use_real) {
        /* 真实链路：只置标志。窗口由 real_update() 末尾的 auto_fit_apply()
         * 用**同一帧**刚算好的 FFT 峰值频率去框 10 个完整周期，
         * 这里自己改 s_zoom_span 的话会跟真实帧对不上。 */
        S.auto_fit = true;
    } else {
        /* 演示源没有真实 FFT，保持原来的「整帧 + 1:1」 */
        s_zoom_span  = 0;
        s_zoom_start = 0;
    }
    s_gain = 1.0f;

    S.running = true;
    S.ch1 = true;
    S.ch2 = true;

    chip_set_text(s_chip_run, "⏸ 暂停");
    set_run_status(true);
    chip_set_text(s_chip_ch1, "CH1 开"); chip_selected(s_chip_ch1, true);
    chip_set_text(s_chip_ch2, "CH2 开"); chip_selected(s_chip_ch2, true);

    if(S.use_real) update_axis_label();
    ui_tick();
}

static void cb_reset(lv_event_t *e)
{
    LV_UNUSED(e);

    S.running = true; S.ch1 = true; S.ch2 = true; S.ai = true;
    S.noise = DEMO_NOISE_PCT;
    S.freq_idx = FREQ_DEF; S.tb_idx = TIMEBASE_DEF;
    S.win_idx = WIN_DEF; S.tau_idx = TAU_DEF; S.phase = 43.7f;
    S.sig.t = 0.f;

    /* 显示缩放回位：整帧 + 纵向 1:1（这是「复位」，不是「自动」——
     * 自动走的是 auto_fit_apply()，会按 FFT 峰值频率框 10 个周期） */
    s_zoom_span  = 0;
    s_zoom_start = 0;
    s_gain       = 1.0f;
    S.auto_fit   = false;
    /* 控件回位（设了下拉框会触发 VALUE_CHANGED，顺手把状态也同步了） */
    lv_dropdown_set_selected(s_dd_freq, (uint32_t)FREQ_DEF);
    {
        lv_obj_t *dd = lv_obj_get_child(s_ctrl_grp[UI_PAGE_SCOPE], 1);
        if(dd) lv_dropdown_set_selected(dd, (uint32_t)TIMEBASE_DEF);
        dd = lv_obj_get_child(s_ctrl_grp[UI_PAGE_FFT], 1);
        if(dd) lv_dropdown_set_selected(dd, (uint32_t)WIN_DEF);
        dd = lv_obj_get_child(s_ctrl_grp[UI_PAGE_LOCK], 1);
        if(dd) lv_dropdown_set_selected(dd, (uint32_t)TAU_DEF);
    }
    chip_set_text(s_chip_ch1, "CH1 开"); chip_selected(s_chip_ch1, true);
    chip_set_text(s_chip_ch2, "CH2 开"); chip_selected(s_chip_ch2, true);
    chip_set_text(s_chip_ai,  "降噪 开"); chip_selected(s_chip_ai, true);
    chip_set_text(s_chip_run, "⏸ 暂停");
    set_run_status(true);
    lv_label_set_text(s_lbl_ai_stat, "已启用");
    /* 时基/幅度由 update_axis_label() 统一写，别在这里写死字符串：
     * 真实链路下写死 "2 µs/格" 会和实际帧长（63 µs 满屏）自相矛盾。 */
    update_axis_label();
    lv_label_set_text(s_lbl_lock_tau, "10 ms");
    lv_label_set_text(s_lbl_fft_win, WIN_NAME[WIN_DEF]);

    ui_set_page(UI_PAGE_SCOPE);
}

static void cb_fullscreen(lv_event_t *e)
{
    LV_UNUSED(e);
    S.fullscreen = !S.fullscreen;
    for(int i = 0; i < UI_PAGE_COUNT; i++) {
        lv_obj_t *a = s_aside[i];
        if(a == NULL) continue;                 /* SYSTEM 页没有右栏 */
        if(S.fullscreen) lv_obj_add_flag(a, LV_OBJ_FLAG_HIDDEN);
        else             lv_obj_remove_flag(a, LV_OBJ_FLAG_HIDDEN);
    }
}

static void ctrl_set_page(int page)
{
    for(int i = 0; i < UI_PAGE_COUNT; i++) {
        if(s_ctrl_grp[i] == NULL) continue;
        if(i == page) lv_obj_remove_flag(s_ctrl_grp[i], LV_OBJ_FLAG_HIDDEN);
        else          lv_obj_add_flag(s_ctrl_grp[i], LV_OBJ_FLAG_HIDDEN);
    }
}

static void build_controls(lv_obj_t *root)
{
    lv_obj_t *bar = mk(root, 0, CTRL_Y0, V32_SHELL_W, V32_ROW_CTRL);
    s_ctrlbar = bar;
    lv_obj_set_style_bg_color(bar, V32_C_CTRL_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(bar, V32_C_CTRL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(bar, 1, LV_PART_MAIN);
    lv_obj_set_style_border_side(bar, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(bar, V32_CTRL_GAP, LV_PART_MAIN);
    lv_obj_set_style_pad_left(bar, V32_CTRL_PAD_X, LV_PART_MAIN);
    lv_obj_set_style_pad_right(bar, V32_CTRL_PAD_X, LV_PART_MAIN);

    /* ---- 信号频率 ---- */
    lv_obj_t *g1 = mk_group(bar, "信号频率");
    s_dd_freq = mk_dropdown(g1, FREQ_OPTS, FREQ_DEF, cb_freq);
    lv_obj_set_style_max_width(s_dd_freq, V32_SEL_MAX_W, LV_PART_MAIN);
    /*
     * 真实数据时这个下拉框改不动任何东西 —— 信号频率是 FFT 从真实帧里量出来的
     * （示波器页左下角「频率」那一项），下拉框只在 --source demo 下有意义。
     * 与其让人点了没反应以为坏了，不如直接置灰。
     */
    if(S.use_real) lv_obj_add_state(s_dd_freq, LV_STATE_DISABLED);

    /*
     * 原来这里还有一组「噪声」滑条（lv_slider + 百分比标签）。
     * 按需求整组删掉了：接真实链路之后它改不动任何东西 —— 噪声是 FPGA
     * 采回来的，UI 端一个滑条既不改波形也不改读数。删掉后控制条还宽出来
     * 约 150px，正好放新加的「自动」按钮。
     */

    /* ---- 页内参数：5 个变体，按当前页显示一个 ---- */
    lv_obj_t *cp = mk(bar, 0, 0, LV_SIZE_CONTENT, V32_CHIP_H);
    lv_obj_set_flex_flow(cp, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(cp, V32_GROUP_GAP, LV_PART_MAIN);

    lv_obj_t *gs = mk_group(cp, "时基");
    lv_obj_t *dd_tb = mk_dropdown(gs, TIMEBASE_OPTS, TIMEBASE_DEF, cb_timebase);
    s_dd_tb = dd_tb;
    lv_obj_set_style_max_width(dd_tb, V32_SEL_MAX_W, LV_PART_MAIN);
    s_ctrl_grp[UI_PAGE_SCOPE] = gs;

    lv_obj_t *gf = mk_group(cp, "窗函数");
    lv_obj_t *dd_w = mk_dropdown(gf, WIN_OPTS, WIN_DEF, cb_window);
    lv_obj_set_style_max_width(dd_w, V32_SEL_MAX_W, LV_PART_MAIN);
    s_ctrl_grp[UI_PAGE_FFT] = gf;

    lv_obj_t *gl = mk_group(cp, "积分时间");
    lv_obj_t *dd_t = mk_dropdown(gl, TAU_OPTS, TAU_DEF, cb_tau);
    lv_obj_set_style_max_width(dd_t, V32_SEL_MAX_W, LV_PART_MAIN);
    s_ctrl_grp[UI_PAGE_LOCK] = gl;

    lv_obj_t *ga = mk(cp, 0, 0, LV_SIZE_CONTENT, V32_CHIP_H);
    s_chip_ai = mk_chip(ga, "降噪 开", cb_ai);
    chip_selected(s_chip_ai, true);
    s_ctrl_grp[UI_PAGE_AI] = ga;

    lv_obj_t *gsys = mk(cp, 0, 0, LV_SIZE_CONTENT, V32_CHIP_H);
    /* 这句原来是写死的「未连接真实硬件」。走真实链路时它会自相矛盾，
     * 所以按数据源切换。 */
    static char s_live_txt[48];
    if(S.use_real) snprintf(s_live_txt, sizeof s_live_txt, "V32CAP3 · 串口 %d · 实时",
                            wave_source_v32_get_baud());
    mk_label(gsys, 0, 0,
             S.use_real ? s_live_txt : "当前为离线演示，未连接真实硬件",
             V32_FONT_SMALL, V32_C_HINT);
    s_ctrl_grp[UI_PAGE_SYSTEM] = gsys;

    /* ---- 四个动作按钮 ----
     * 「自动」放在最前面紧挨着「暂停」：示波器上 Auto Set 和 Run/Stop 是
     * 挨着的两个常用键，捏乱了之后第一反应就是先按自动。 */
    s_chip_run = mk_chip(bar, "⏸ 暂停", cb_run);
    mk_chip(bar, "⊞ 自动", cb_auto);
    mk_chip(bar, "↺ 复位", cb_reset);
    mk_chip(bar, "⛶ 全屏", cb_fullscreen);
}

/* =========================================================
 * 底部导航
 * ========================================================= */
static void nav_selected(lv_obj_t *b, bool on)
{
    /*
     * ⚠️ 未选中的导航按钮**必须有底色**，不能透明。
     *    CSS 里 `.chip,.nav button{ background:#1c3043 }` 是一条共用规则，
     *    选中态才是 `.nav button.active{ background:#205268 }`。
     *    之前这里写的是 bg_opa = TRAMP/TRANSP，于是除当前页之外的 4 个按钮
     *    变成透出导航条底的 #0a111c —— 整个导航条分区均值 ΔL1 = 58，
     *    是全屏最大的一处色差，但单看截图只觉得"导航条有点发暗"。
     */
    lv_obj_set_style_bg_color(b, on ? V32_C_NAV_ON_BG : V32_C_CHIP_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(b, V32_C_NAV_ON_BAR, LV_PART_MAIN);
    lv_obj_set_style_border_width(b, on ? V32_NAV_ACTIVE_BW : 0, LV_PART_MAIN);
    lv_obj_set_style_border_side(b, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);

    lv_obj_t *l = lv_obj_get_child(b, 0);
    if(l) lv_obj_set_style_text_color(l, on ? V32_C_NAV_ON_FG : V32_C_NAV_FG, LV_PART_MAIN);
}

static void build_nav(lv_obj_t *root)
{
    static const char *TXT[UI_PAGE_COUNT] = {
        "⌁ 示波器", "▥ 频谱", "◎ 锁相", "✧ 降噪", "⚙ 系统"
    };

    lv_obj_t *nav = mk(root, 0, NAV_Y0, V32_SHELL_W, V32_ROW_NAV);
    s_navbar = nav;
    lv_obj_set_style_bg_color(nav, V32_C_NAV_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(nav, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(nav, V32_C_NAV_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(nav, 1, LV_PART_MAIN);
    lv_obj_set_style_border_side(nav, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_flex_flow(nav, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(nav, V32_NAV_GAP, LV_PART_MAIN);
    lv_obj_set_style_pad_left(nav, V32_NAV_PAD_X, LV_PART_MAIN);
    lv_obj_set_style_pad_right(nav, V32_NAV_PAD_X, LV_PART_MAIN);
    lv_obj_set_style_pad_top(nav, V32_NAV_PAD_Y, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(nav, V32_NAV_PAD_Y, LV_PART_MAIN);

    for(int i = 0; i < UI_PAGE_COUNT; i++) {
        lv_obj_t *b = lv_button_create(nav);
        lv_obj_remove_style_all(b);
        lv_obj_set_flex_grow(b, 1);
        lv_obj_set_height(b, V32_ROW_NAV - V32_NAV_PAD_Y * 2 - 1);
        lv_obj_set_style_radius(b, V32_CHIP_RADIUS, LV_PART_MAIN);
        lv_obj_add_event_cb(b, cb_nav, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *l = lv_label_create(b);
        lv_obj_remove_style_all(l);
        /* .nav button{ font-size:12px; font-weight:750; letter-spacing:.5px } */
        lv_obj_set_style_text_font(l, V32_FONT_SMALL_B, LV_PART_MAIN);
        lv_obj_set_style_text_letter_space(l, 1, LV_PART_MAIN);
        lv_label_set_text(l, TXT[i]);
        lv_obj_center(l);

        s_nav[i] = b;
        nav_selected(b, i == 0);
    }
}

/* =========================================================
 * 周期刷新
 * ========================================================= */
static uint32_t s_last_sec = 0;
static uint32_t s_fps_t0 = 0, s_fps_f0 = 0;
static uint32_t s_app_t0 = 0;

static void tick_scope(void)
{
    static float ch1[PLOT_MAX_PTS], ch2[PLOT_MAX_PTS];
    const int n = PLOT_MAX_PTS;

    real_update();

    fetch_scope(0, ch1, n);
    fetch_scope(1, ch2, n);

    if(S.ch1) plot_set_trace(s_plot_scope, 0, ch1, n, V32_C_CH1, 2);
    else      plot_hide_trace(s_plot_scope, 0);
    if(S.ch2) plot_set_trace(s_plot_scope, 1, ch2, n, V32_C_CH2, 2);
    else      plot_hide_trace(s_plot_scope, 1);

    plot_set_trigger(s_plot_scope, true, 0.46f);

    const bool use_real = S.use_real;
    const bool real = (use_real && s_real_valid);
    /* 接了串口但没数据时读 0，不显示演示值 */
    const float v1 = real ? s_real_vpp0_mv : (use_real ? 0.f : demo_vpp_mv(ch1, n, V32_GRID_Y));
    const float v2 = real ? s_real_vpp1_mv : (use_real ? 0.f : demo_vpp_mv(ch2, n, V32_GRID_Y));

    lv_label_set_text_fmt(s_lbl_v1, "%.1f mV", (double)v1);
    lv_label_set_text_fmt(s_lbl_v2, "%.1f mV", (double)v2);

    /* 频率：真实链路取 FFT 峰值，演示链路取下拉框 */
    if(real)          lv_label_set_text_fmt(s_lbl_scope_freq, "%.3f MHz", (double)s_real_peak_hz / 1e6);
    else if(use_real) lv_label_set_text(s_lbl_scope_freq, "0.000 MHz");
    else              lv_label_set_text_fmt(s_lbl_scope_freq, "%.3f MHz", (double)FREQ_MHZ[S.freq_idx]);

    /* 相位差也按真实帧算，没有数据时打「—」，不留在演示值上 */
    if(real)          lv_label_set_text_fmt(s_lbl_phase, "%.1f°", (double)s_real_phase_deg);
    else if(use_real) lv_label_set_text(s_lbl_phase, "—°");
    else              lv_label_set_text_fmt(s_lbl_phase, "%.1f°", (double)S.phase);

    /*
     * 真实帧 4096 点 @65 MSPS = 63.015 µs 满屏，10 格 -> 6.302 µs/div。
     * 每帧都跟一遍：纵向是自动量程，Vpp 变了 mV/div 也会变，
     * 只在第一帧写一次的话读数和屏幕上的波形迟早对不上。
     */
    if(use_real) update_axis_label();
}

static void tick_fft(void)
{
    static float y[256];
    static char  peak_txt[24];
    float px = 0.3f, py = 0.8f;

    real_update();

    fetch_spectrum(y, 256, &px, &py);
    plot_set_trace(s_plot_fft, 0, y, 256, V32_C_FFT, 2);

    const bool real = (S.use_real && s_real_valid);

    if(real) {
        /* 峰值/底噪都是**算出来的**绝对 dBFS（满刻度 2048 码 = 0 dBFS），
         * 不再是参考稿里那两个写死的 -12.8 / -79.4 */
        snprintf(peak_txt, sizeof(peak_txt), "▼ %.3f MHz", (double)s_real_peak_hz / 1e6);
        lv_label_set_text_fmt(s_lbl_fft_peakf, "%.3f MHz", (double)s_real_peak_hz / 1e6);
        lv_label_set_text_fmt(s_lbl_fft_peakl, "%.1f dBFS", (double)s_real_peak_dbfs);
        lv_label_set_text_fmt(s_lbl_fft_floor, "%.1f dBFS", (double)s_real_floor_dbfs);
    } else if(S.use_real) {
        /* 没数据：读数归零，不要留着上一帧的数字 */
        snprintf(peak_txt, sizeof(peak_txt), "▼ —");
        lv_label_set_text(s_lbl_fft_peakf, "0.000 MHz");
        lv_label_set_text(s_lbl_fft_peakl, "— dBFS");
        lv_label_set_text(s_lbl_fft_floor, "— dBFS");
    } else {
        snprintf(peak_txt, sizeof(peak_txt), "▼ %d.000 MHz", FREQ_MHZ[S.freq_idx]);
        lv_label_set_text_fmt(s_lbl_fft_peakf, "%.3f MHz", (double)FREQ_MHZ[S.freq_idx]);
        lv_label_set_text_fmt(s_lbl_fft_peakl, "%.1f dBFS", (double)demo_dbfs_from_y01(py));
        lv_label_set_text_fmt(s_lbl_fft_floor, "%.1f dBFS", (double)demo_noise_floor_dbfs(y, 256));
    }

    plot_set_peak(s_plot_fft, true, px, peak_txt);
    lv_label_set_text(s_lbl_fft_win, WIN_NAME[S.win_idx]);
}

static void tick_lock(void)
{
    static float a[PLOT_MAX_PTS], b[PLOT_MAX_PTS];
    const int n = PLOT_MAX_PTS;

    demo_lock_a(&S.sig, a, n);
    demo_lock_b(&S.sig, b, n);
    plot_set_trace(s_plot_lock, 0, a, n, V32_C_CH2, 2);
    plot_set_trace(s_plot_lock, 1, b, n, V32_C_CH1, 2);
    plot_set_iq(s_plot_iq, S.phase);

    lv_label_set_text_fmt(s_lbl_lock_ref,   "%.3f MHz", (double)FREQ_MHZ[S.freq_idx]);
    lv_label_set_text_fmt(s_lbl_lock_amp,   "%.1f mV",  (double)demo_vpp_mv(a, n, V32_GRID_Y));
    lv_label_set_text_fmt(s_lbl_lock_phase, "%.1f°",    (double)S.phase);
}

/*
 * AI 页的「降噪」：5 点中值滤波。
 *
 * 为什么不是滑动平均：
 *   V32CAP3 是 65 MSPS，10 MHz 信号每周期只有 6.5 个点。
 *   3 点滑动平均在 10 MHz 处 |H| = sin(3πf/fs)/(3·sin(πf/fs)) ≈ 0.71，
 *   也就是 -3 dB —— 屏幕上会看到「降噪后幅度明显变小」，
 *   那是滤波器把信号一起削了，不是降噪效果，属于骗人。
 *   中值滤波是幅度保持的（直流增益恒为 1），只压掉窄于窗口宽度的毛刺，
 *   方波的台阶电平原样保留，所以「降噪前/后」的纵向刻度可以互相比。
 *
 * 代价（**上板实测后修正过，和最初想的不一样**）：
 *   原以为中值只是「钝化边沿」，实际看下来正好相反 —— 10 MHz 方波在 65 MSPS
 *   下每周期只有 6.5 点，一次跳变只占 1~2 个样点，这 1~2 个中间值在 5 点窗口里
 *   是少数派，被中值当成离群点直接压掉了，结果边沿**变陡**、台阶变平。
 *   看着更「干净」，但那是假的：真实上升时间被抹掉了。
 *   所以这一页只能用来判断「毛刺有没有被压掉」，**边沿宽度不能以本页为准**。
 */
static void denoise_median5(const float *x, int n, float *y)
{
    for(int i = 0; i < n; i++) {
        float w[5];
        for(int k = -2; k <= 2; k++) {
            int j = i + k;
            if(j < 0)  j = 0;
            if(j >= n) j = n - 1;
            w[k + 2] = x[j];
        }
        /* 只有 5 个元素，插入排序比通用 qsort 快且不用栈 */
        for(int a = 1; a < 5; a++) {
            const float t = w[a];
            int b = a - 1;
            while(b >= 0 && w[b] > t) { w[b + 1] = w[b]; b--; }
            w[b + 1] = t;
        }
        y[i] = w[2];
    }
}

static void tick_ai(void)
{
    static float y[PLOT_MAX_PTS];
    const int n = PLOT_MAX_PTS;

    /* 这一页原来只有 demo_ai()，跟串口一点关系都没有：顶栏写着「串口模式」，
     * 画的却是合成正弦。现在先推进真实链路，再按 S.use_real 分两条路。 */
    real_update();

    if(S.use_real) {
        if(s_real_valid && s_full_n >= 2) {
            /* 上：原始 CH1，与示波器页同一套缩放窗口 */
            map_window(s_full0, s_full_n, y, n);
            plot_set_trace(s_plot_raw, 0, y, n, V32_C_CH1, 2);

            /* 下：中值滤波后的同一段数据；关掉开关就是旁路（与上图重合） */
            if(S.ai) {
                denoise_median5(s_full0, s_full_n, s_den);
                map_window(s_den, s_full_n, y, n);
            } else {
                map_window(s_full0, s_full_n, y, n);
            }
            plot_set_trace(s_plot_den, 0, y, n, V32_C_AI_OUT, 2);
        } else {
            /* 接了串口但没帧 / 链路断了：两条零线，绝不退回演示曲线 */
            for(int i = 0; i < n; i++) y[i] = 0.5f;
            plot_set_trace(s_plot_raw, 0, y, n, V32_C_CH1, 2);
            plot_set_trace(s_plot_den, 0, y, n, V32_C_AI_OUT, 2);
        }
        return;
    }

    /* 只有显式 --source demo 时才走合成曲线 */
    demo_ai(&S.sig, y, n, true, S.ai);
    plot_set_trace(s_plot_raw, 0, y, n, V32_C_CH1, 2);

    demo_ai(&S.sig, y, n, false, S.ai);
    plot_set_trace(s_plot_den, 0, y, n, V32_C_AI_OUT, 2);
}

static void tick_system(void)
{
    const uint32_t now = lv_tick_get();

    /* Render FPS：直接拿 DRM 后端真实的翻页计数算，1 秒更新一次 */
    uint32_t flips = 0;
    uint64_t ns = 0;
    drm_display_flip_stats(&flips, &ns);

    if(s_fps_t0 == 0) { s_fps_t0 = now; s_fps_f0 = flips; }
    else if(now - s_fps_t0 >= 1000) {
        const double fps = (double)(flips - s_fps_f0) * 1000.0 / (double)(now - s_fps_t0);
        lv_label_set_text_fmt(s_lbl_fps, "%.1f FPS", fps);
        s_fps_t0 = now;
        s_fps_f0 = flips;

        /* FPGA 链路统计：同样 1 秒刷一次，免得每帧都触发重绘 */
        if(S.use_real) {
            const v32_stats_t *st = v32_get_stats();
            const double ffps = (double)st->fps_milli / 1000.0;
            lv_label_set_text_fmt(s_lbl_v32_rate, "%.1f kB/s",
                                  ffps * (double)V32_PACKET_BYTES / 1000.0);
            lv_label_set_text_fmt(s_lbl_v32_loss, "%u / %u",
                                  (unsigned)(st->crc_errors + st->sequence_errors),
                                  (unsigned)st->frames_ok);
        }
    }

    const uint32_t up = (now - s_app_t0) / 1000u;
    if(up >= 3600u)
        lv_label_set_text_fmt(s_lbl_uptime, "%u:%02u:%02u",
                             (unsigned)(up / 3600u), (unsigned)((up / 60u) % 60u), (unsigned)(up % 60u));
    else
        lv_label_set_text_fmt(s_lbl_uptime, "%02u:%02u",
                             (unsigned)(up / 60u), (unsigned)(up % 60u));
}

static void tick_clock(void)
{
    const uint32_t now = lv_tick_get();
    if(now - s_last_sec < 1000) return;
    s_last_sec = now;

    char buf[16];
    time_t t = time(NULL);
    struct tm tmv;
    localtime_r(&t, &tmv);
    if(tmv.tm_year + 1900 >= 2020) {
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
    }
    else {
        /* 板子没 RTC 也没联网时系统时间停在 1970，显示开机时长比 00:00:00 有意义 */
        const uint32_t s = (now - s_app_t0) / 1000u;
        snprintf(buf, sizeof(buf), "%02u:%02u:%02u",
                 (unsigned)(s / 3600u), (unsigned)((s / 60u) % 60u), (unsigned)(s % 60u));
    }
    lv_label_set_text(s_lbl_clock, buf);
}

/* =========================================================
 * 对外接口
 * ========================================================= */

void ui_create(void)
{
    /* 默认值对照参考稿 JS 的初值 */
    S.page = UI_PAGE_SCOPE;
    S.running = true;
    S.ch1 = S.ch2 = S.ai = true;
    S.noise = DEMO_NOISE_PCT;      /* 只给演示数据用；界面上已没有滑条 */
    S.freq_idx = FREQ_DEF;
    S.tb_idx = TIMEBASE_DEF;
    S.win_idx = WIN_DEF;
    S.tau_idx = TAU_DEF;
    S.phase = 43.7f;
    S.fullscreen = false;

    /*
     * 数据源在 main.c 里就选好了（--source 参数），这里只判断「选的不是演示源」。
     * 保留 WAVE_SRC 环境变量是为了兼容手动调试。
     */
    const char *env = getenv("WAVE_SRC");
    S.use_real = (wave_source_get() != &wave_source_demo)
              || (env && (strcmp(env, "real") == 0
                       || strcmp(env, "v32")  == 0
                       || strcmp(env, "uart") == 0));
    if(S.use_real) printf("[ui] 真实数据源: %s\n", wave_source_get()->name);

    S.sig.freq_mhz    = FREQ_MHZ[S.freq_idx];
    S.sig.timebase_us = TIMEBASE_US[S.tb_idx];
    S.sig.noise_pct   = S.noise;
    S.sig.phase_deg   = S.phase;
    S.sig.window      = S.win_idx;
    S.sig.t           = 0.f;

    lv_obj_t *root = lv_screen_active();
    lv_obj_remove_style_all(root);
    lv_obj_set_size(root, V32_SCR_W, V32_SCR_H);
    /*
     * 根节点画的是 .shell，不是 <body>：
     *   .shell{ background:#0c1420; border:1px solid #344456 }
     *   body 那层 radial-gradient(#1c3045,#070d15) 被 .shell 完全盖住，永远看不到。
     * 之前这里用的是 #070D15（body 色），后果是 .main 的 padding 区（以及右栏
     * 之外的所有留白）比参考稿暗一档 —— 分区均值实测 ΔL1=23，是当时最大的一处色差。
     */
    lv_obj_set_style_bg_color(root, V32_C_SHELL, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(root, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(root, V32_C_SHELL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(root, V32_SHELL_BW, LV_PART_MAIN);
    lv_obj_remove_flag(root, LV_OBJ_FLAG_SCROLLABLE);

    s_app_t0 = lv_tick_get();

    build_topbar(root);

    /* 五个页面叠在同一位置，靠 HIDDEN 切换 */
    for(int i = 0; i < UI_PAGE_COUNT; i++) {
        s_page[i] = mk(root, MAIN_X0, MAIN_Y0, MAIN_W, MAIN_H);
        lv_obj_add_flag(s_page[i], LV_OBJ_FLAG_HIDDEN);
    }
    build_scope (s_page[UI_PAGE_SCOPE]);
    build_fft   (s_page[UI_PAGE_FFT]);
    build_lock  (s_page[UI_PAGE_LOCK]);
    build_ai    (s_page[UI_PAGE_AI]);
    build_system(s_page[UI_PAGE_SYSTEM]);

    build_controls(root);
    build_nav(root);
    /* 右下角那行原型注记已按需求删除（见 build_fsnote 的注释） */

    ui_set_page(UI_PAGE_SCOPE);

    /* 接了真实源就立刻按真实帧长把时基写对，
     * 否则没有数据时左下角会一直挂着演示时代的 "2 µs/div"，和实际帧对不上。 */
    if(S.use_real) update_axis_label();
}

int  ui_page_count(void) { return UI_PAGE_COUNT; }
int  ui_get_page(void)   { return S.page; }

void ui_set_page(int page)
{
    if(page < 0 || page >= UI_PAGE_COUNT) return;
    S.page = page;

    for(int i = 0; i < UI_PAGE_COUNT; i++) {
        if(i == page) lv_obj_remove_flag(s_page[i], LV_OBJ_FLAG_HIDDEN);
        else          lv_obj_add_flag(s_page[i], LV_OBJ_FLAG_HIDDEN);
        nav_selected(s_nav[i], i == page);
    }
    ctrl_set_page(page);

    /* 切过去立刻补一帧，别让新页面空着等下一个 tick */
    if(page == UI_PAGE_SYSTEM) { s_fps_t0 = 0; tick_system(); }
    ui_tick();
}

int ui_page_from_name(const char *name)
{
    if(name == NULL) return -1;
    if(!strcmp(name, "scope") || !strcmp(name, "wave") || !strcmp(name, "0")) return UI_PAGE_SCOPE;
    if(!strcmp(name, "fft")   || !strcmp(name, "1")) return UI_PAGE_FFT;
    if(!strcmp(name, "lock")  || !strcmp(name, "lockin") || !strcmp(name, "2")) return UI_PAGE_LOCK;
    if(!strcmp(name, "ai")    || !strcmp(name, "3")) return UI_PAGE_AI;
    if(!strcmp(name, "system")|| !strcmp(name, "sys") || !strcmp(name, "4")) return UI_PAGE_SYSTEM;
    return -1;
}

int  ui_get_view(void)     { return S.page == UI_PAGE_FFT ? 1 : 0; }
void ui_set_view(int view) { ui_set_page(view ? UI_PAGE_FFT : UI_PAGE_SCOPE); }

void ui_tick(void)
{
    if(S.running) {
        S.sig.t += 0.02f;
        if(S.sig.t > 1000.f) S.sig.t = 0.f;
    }
    S.sig.freq_mhz    = FREQ_MHZ[S.freq_idx];
    S.sig.timebase_us = TIMEBASE_US[S.tb_idx];
    S.sig.noise_pct   = S.noise;
    S.sig.phase_deg   = S.phase;
    S.sig.window      = S.win_idx;

    switch(S.page) {
        case UI_PAGE_SCOPE:  tick_scope();  break;
        case UI_PAGE_FFT:    tick_fft();    break;
        case UI_PAGE_LOCK:   tick_lock();   break;
        case UI_PAGE_AI:     tick_ai();     break;
        case UI_PAGE_SYSTEM: tick_system(); break;
        default: break;
    }
    tick_clock();
}

/* =========================================================
 * 排错 / 自动化接口
 * ========================================================= */

void ui_freeze(float t)
{
    S.running = false;      /* ui_tick 里不再 t += 0.02 */
    S.sig.t   = t;
    ui_tick();              /* 立刻按这个相位重画一帧 */
}

/* 递归打印控件树。缩进 = 层级，坐标是**绝对屏幕坐标**（LVGL 的 coords 就是绝对的）。 */
static void dump_obj(lv_obj_t *o, int depth)
{
    if(o == NULL || depth > 12) return;

    lv_area_t a;
    lv_obj_get_coords(o, &a);

    const lv_obj_class_t *cls = lv_obj_get_class(o);
    const char *cname = (cls && cls->name) ? cls->name : "?";

    /* label 额外把文本打出来 —— 顶栏出问题时就是靠这个看出
     * 「对象在、坐标也在、只是文字被截了」还是「对象压根没建」 */
    char txt[80];
    txt[0] = '\0';
    if(lv_obj_check_type(o, &lv_label_class)) {
        const char *t = lv_label_get_text(o);
        if(t) snprintf(txt, sizeof(txt), "  \"%.48s\"", t);
    }

    printf("%*s%-12s [%4d,%4d %4dx%-4d]%s%s\n",
           depth * 2, "", cname,
           (int)a.x1, (int)a.y1,
           (int)(a.x2 - a.x1 + 1), (int)(a.y2 - a.y1 + 1),
           lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN) ? "  <HIDDEN>" : "",
           txt);

    const uint32_t n = lv_obj_get_child_count(o);
    for(uint32_t i = 0; i < n; i++)
        dump_obj(lv_obj_get_child(o, (int32_t)i), depth + 1);
}

void ui_dump_layout(void)
{
    printf("=== 控件几何 dump（绝对屏幕坐标 x,y wxh）===\n");
    printf("--- 顶栏 ---\n");
    dump_obj(s_top, 0);
    printf("--- 控制条 ---\n");
    dump_obj(s_ctrlbar, 0);
    printf("--- 导航 ---\n");
    dump_obj(s_navbar, 0);
    printf("--- 当前页 (page=%d) ---\n", S.page);
    if(S.page >= 0 && S.page < UI_PAGE_COUNT)
        dump_obj(s_page[S.page], 0);
    fflush(stdout);
}
