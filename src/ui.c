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
#include "bl_interp.h"
#include "drm_disp.h"
#include "port.h"

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
#define MAIN_Y0   (V32_MAIN_PAD_T)                                          /* 顶栏隐藏后贴顶，原 58 */
#define MAIN_W    (V32_SHELL_W - 2 * V32_MAIN_PAD_X)                        /* 998 */
#define MAIN_H    (V32_ROW_MAIN + V32_ROW_TOP - V32_MAIN_PAD_T - V32_MAIN_PAD_B)  /* 顶栏让出的 48px 全给绘图区 */
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
#define METRIC_H  21   /* 照 HTML：21px 一行，10 项测量 + 光标段 */
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

/* 0.5 µs/格 ~ 500 µs/格：上限要能覆盖 FPGA 抽取后的长帧
 * （dec=32 时一屏 2048 µs，正好一个 500 Hz 周期）。 */
static const float TIMEBASE_US[10] = { 0.5f, 1.f, 2.f, 5.f, 10.f,
                                       20.f, 50.f, 100.f, 200.f, 500.f };
static const char  TIMEBASE_OPTS[] =
    "0.5 µs/格\n1 µs/格\n2 µs/格\n5 µs/格\n10 µs/格"
    "\n20 µs/格\n50 µs/格\n100 µs/格\n200 µs/格\n500 µs/格";
static const int   TIMEBASE_DEF    = 2;    /* 2 µs/div */

/* 只提供交付包里验证过的三种窗：Hamming / 矩形窗不提供，
 * 免得菜单里出现"选了但没验证"的选项。 */
static const char *WIN_NAME[3]     = { "HANN", "BLACKMAN", "FLAT TOP" };
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
    int   fft_ch;        /* FFT 用哪一路：0=自动(交流大的一路) 1=CH1 2=CH2 */
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
static lv_obj_t *s_lbl_m_vmax, *s_lbl_m_vmin, *s_lbl_m_vrms, *s_lbl_m_per, *s_lbl_m_duty;
static lv_obj_t *s_lbl_m_vamp, *s_lbl_m_vavg, *s_lbl_m_rise;
static lv_obj_t *s_lbl_cur_dt, *s_lbl_cur_f, *s_lbl_cur_dv;
static lv_obj_t *s_lbl_scope_axis;
static lv_obj_t *s_lbl_fft_peakf, *s_lbl_fft_peakl, *s_lbl_fft_floor, *s_lbl_fft_win;
static lv_obj_t *s_lbl_lock_ref, *s_lbl_lock_amp, *s_lbl_lock_phase, *s_lbl_lock_tau;
static lv_obj_t *s_lbl_lock_state;      /* 锁定状态（● 锁定 / ○ 原因） */
static lv_obj_t *s_lbl_ai_stat;
static lv_obj_t *s_lbl_fps, *s_lbl_uptime;
static lv_obj_t *s_lbl_v32_rate, *s_lbl_v32_loss;   /* FPGA 链路实时统计 */
static lv_obj_t *s_lbl_clock, *s_lbl_run_status;

static lv_obj_t *s_chip_ch1, *s_chip_ch2, *s_chip_ai, *s_chip_run;
static lv_obj_t *s_dd_freq;
static lv_obj_t *s_lbl_den_title;
static lv_obj_t *s_lbl_recon_badge;      /* 示波器页右上角「重建」牌 */
static lv_obj_t *s_lbl_fft_cur;
static lv_obj_t *s_lbl_fft_title;
static lv_obj_t *s_chip_tb, *s_chip_vg;   /* 时基 / 档位 的值芯片 */
static lv_obj_t *s_chip_cur;    /* 光标开关芯片 */
static lv_obj_t *s_chip_trig, *s_chip_edge, *s_chip_lev;   /* 触发选择芯片 */
static lv_obj_t *s_chip_math;   /* Math 选择芯片 */
static lv_obj_t *s_chip_win;            /* 窗函数：循环切换芯片（这个 UI 里下拉框点不开） */
static lv_obj_t *s_chip_fch[3];          /* 频谱通道：自动/CH1/CH2 */               /* 窗函数下拉框，复位时要回位 */        /* 「FFT 频谱 · CH1」标题 */          /* 频谱页游标读数 */
static float     s_fft_cur_x;            /* 0..1，游标横坐标 */
static bool      s_fft_cur_on;       /* 重建页下方图标题，随能否重建变化 */
static lv_obj_t *s_lbl_wave;           /* 重建页右上角：波形类型 */
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
    /* HTML 原型的测量栏是纯文字（标签左、数值右），没有分隔线 */
    lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);

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
    if(!b) return;                 /* 还没建好就别刷，避免空指针 */
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
static int      s_real_fft_ch;
static bool     s_ai_autoframed;  /* 重建页这次进入是否已经自动框过窗 */   /* 本次频谱实际取自哪一路：0=CH1 1=CH2 */
static float    s_real_peak_hz;
static float    s_real_peak_dbfs;
static float    s_real_floor_dbfs;
static float    s_real_phase_deg;           /* CH1 相对 CH0 的相位差 */
/* FPGA 端频谱（V32SPEC!）的状态 —— 频谱页优先用它 */
static int      s_spec_in_use;              /* 1 = 当前画的是 FPGA 的谱 */
static float    s_spec_peak_hz;
static int      s_spec_win, s_spec_bins, s_spec_nfft;

/*
 * 断链判定。
 * 真实链路只有约 5.6 帧/秒，所以 1 秒没新帧就肯定是断了（或 FPGA 停了）。
 * 以前这里「帧号没变就沿用上一帧」，看起来像正常，其实屏幕冻在最后一帧 ——
 * 用户明确要求断开之后归零，不能保持现状。
 */
#define REAL_STALE_MS   1000u

/*
 * 断链超时可覆盖（WAVE_STALE_MS 环境变量），只为调试。
 *
 * 平时必须是 1 秒：串口 5.6 帧/秒，1 秒没新帧就是真断了，界面要归零。
 * 但 `--source file:` 是把整个文件一次性喂进去的，只产出**一帧**，
 * 过 1 秒就判定断链、屏幕变零线 —— 拿它验证显示/缩放/重建根本来不及操作。
 * 调试时 WAVE_STALE_MS=60000 就能让那一帧"活"一分钟。
 */
static int dbg_on(void)
{
    static int v = -1;
    if(v < 0) v = getenv("WAVE_TDBG") ? 1 : 0;
    return v;
}

static uint32_t stale_ms(void)
{
    static int32_t v = -1;
    if(v < 0) {
        const char *e = getenv("WAVE_STALE_MS");
        v = e ? (int32_t)atoi(e) : (int32_t)REAL_STALE_MS;
        if(v < 100) v = 100;
    }
    return (uint32_t)v;
}
static uint32_t s_real_t_last;          /* 最近一次收到新帧的时刻 */

/* 整帧原始数据，缩放窗口要在这上面取，不能在抽取后的 1024 点上取 */
static float    s_full0[WAVE_MAX_FRAME];
static float    s_full1[WAVE_MAX_FRAME];
static int      s_full_n;

/*
 * 缩放窗口（两指捏放）。
 *   s_zoom_span = 可见点数，0 表示还没初始化（= 整帧）
 *   s_zoom_start = 窗口起点对应的样点下标
 *
 * 纵向另有一个独立增益 s_gain（两指上下捏），相当于示波器的 V/div：
 * 横向捏放改「看多长一段」，纵向捏放改「一格多少毫伏」，两件事互不影响。
 */
static int      s_zoom_span;
static int      s_tb_idx = 4;      /* 当前时基档位下标 */
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
static void cb_scope_touch(lv_event_t *e);   /* 光标拖动：build_scope 里注册，定义在后面 */
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
    if(dbg_on()) fprintf(stderr, "[auto] span=%d start=%d peak=%.0f Hz\n",
                         s_zoom_span, s_zoom_start, (double)s_real_peak_hz);
    update_axis_label();
}

/* ---- 波形类型判定 ------------------------------------------------
 *
 * 只靠谐波比例判，判据很少但足够区分常见的几种，而且**能解释**：
 *   rK = 第 K 次谐波幅度 / 基波幅度
 *     正弦    r2、r3 都很小
 *     方波    r2≈0，r3≈1/3
 *     三角波  r2≈0，r3≈1/9
 *     锯齿波  r2≈1/2 —— **有强的偶次谐波**，这是和方波的关键区别
 *
 * ⚠️ 前提是这些谐波落在奈奎斯特以内。10 MHz 时 3 次谐波 30 MHz 没问题；
 *    30 MHz 时 3 次谐波 90 MHz 早折叠回来了，根本看不到 ——
 *    这时标一个 `*`，表示「判据不足」，不假装认得出来。
 */
enum {
    WF_UNKNOWN = 0,
    WF_DC,
    WF_NOISE,
    WF_SINE,
    WF_SQUARE,
    WF_TRIANGLE,
    WF_SAW
};

static const char *const WF_NAME[] = {
    "未知", "直流", "噪声", "正弦", "方波", "三角波", "锯齿波"
};

static int s_wf_kind  = WF_UNKNOWN;
static int s_wf_alias;          /* 1 = 谐波超出奈奎斯特，判定可能不准 */

/* 在中心 bin 附近找最大值：谱线未必正好落在整数 bin 上 */
static float mag_near_bin(int b, int span)
{
    int lo = b - span, hi = b + span;
    if(lo < 1) lo = 1;
    if(hi > s_real_bins - 1) hi = s_real_bins - 1;
    float m = 0.0f;
    for(int i = lo; i <= hi; i++)
        if(s_real_mag[i] > m) m = s_real_mag[i];
    return m;
}

/*
 * 能不能重建：只有「认得出是什么波形」才行。
 *
 * 这是用户定的原则：不清楚是什么就画一条光滑曲线，等于造假。
 * 所以 未知 / 噪声 / 直流 一律不重建，直接显示原始采样点。
 * 方波/三角波/锯齿波虽然带限重建出来会有振铃，但那是可解释的（谐波被截断），
 * 而且它们的谐波都在带内看得到，属于"已知"，可以重建。
 */
static int wf_can_reconstruct(void)
{
    /*
     * **认出来了就允许重建，认不出来一律不重建。**
     *
     * 这是用户定的规矩（原话「不知道是什么波形还画一条光滑曲线，那是造假」），
     * 所以门槛是"认得出"，不是"带限"。
     *
     * 但方波/三角波/锯齿波**不是带限信号**，谐波按 1/n 或 1/n² 衰减到无穷，
     * 截断到奈奎斯特必然产生吉布斯振铃：平顶上浮、边沿外侧过冲，实测约 9%。
     * **那些过冲不是采到的，是重建算法造的**，界面上必须写明，不能当成测量结果。
     * 只有正弦是单根谱线，重建是唯一且无振铃的解。
     */
    /* 只重建正弦。方波/三角波/锯齿波**不重建**：
     * 它们不是带限信号，谐波按 1/n 衰减到无穷，截断到奈奎斯特必然产生
     * 吉布斯振铃 —— 屏幕上那圈过冲就是"额外加上去的谐波"，看起来已经不像
     * 原来的方波了（用户原话「不要给我加谐波上去都不像了」）。
     * 宁可显示原始采样点，也不显示被改了形状的波形。 */
    return s_wf_kind == WF_SINE   || s_wf_kind == WF_SQUARE
        || s_wf_kind == WF_TRIANGLE || s_wf_kind == WF_SAW;
}

static void classify_wave(void)
{
    s_wf_alias = 0;
    s_wf_kind  = WF_UNKNOWN;

    if(s_real_peak_hz <= 0.0f) return;

    const float vpp = (s_real_vpp0_mv > s_real_vpp1_mv) ? s_real_vpp0_mv : s_real_vpp1_mv;
    if(vpp < 20.0f) { s_wf_kind = WF_DC; return; }

    /* 峰值比底噪高不到 10 dB 就按噪声算，别硬套成某种波形 */
    if(s_real_peak_dbfs < s_real_floor_dbfs + 10.0f) { s_wf_kind = WF_NOISE; return; }

    const wave_source_t *src = wave_source_get();
    const float fs = (src && src->sample_rate > 0) ? (float)src->sample_rate : 65000000.0f;
    const int   b1 = (int)((double)s_real_peak_hz * (double)REAL_FFT_N / (double)fs + 0.5);
    if(b1 < 2 || b1 >= s_real_bins - 2) return;

    /*
     * 主瓣半宽（在 REAL_FFT_N 的 bin 网格上）。
     *
     * ⚠️ 必须跟着**零填充倍数**走：FFT 是 n_real 点补零到 REAL_FFT_N，
     *    Hann 窗主瓣在真实分辨率下约 ±2 bin，补零后网格变密，
     *    主瓣在网格上就被摊宽成 ±2×(REAL_FFT_N/n_real) bin。
     *
     *    4096 点补零到 8192：主瓣 ±4 bin，用旧的 ±7 刚好够；
     *    1024 点补零到 8192：主瓣 ±16 bin，还用 ±7 就会把**基波自己的裙边**
     *    当成杂散（实测 bin 3789 距峰仅 8 bin、比值 -5.9 dB），
     *    于是干净的正弦被判成"未知"、不重建 —— 用户看到的"点数少了重建出问题"。
     */
    const int pad  = (s_real_n > 0) ? (int)((double)REAL_FFT_N / (double)s_real_n + 0.5) : 1;
    int half = 2 * pad + 2;
    if(half < 7) half = 7;              /* 4096 点时保持原来的 7，行为不变 */

    const float a1 = mag_near_bin(b1, half < 2 ? 2 : half);
    if(a1 <= 0.0f) return;

    const int   b2 = 2 * b1, b3 = 3 * b1;
    const float r2 = (b2 < s_real_bins - 1) ? mag_near_bin(b2, 2) / a1 : -1.0f;
    const float r3 = (b3 < s_real_bins - 1) ? mag_near_bin(b3, 2) / a1 : -1.0f;

    if(r2 < 0.0f || r3 < 0.0f) {
        /*
         * 谐波已经折叠回带内，谐波比例这条路走不通了。
         * 这时**不猜**，改用「谱干不干净」这个可以直接观察的证据：
         *   除基波附近外没有明显谱线 ⇒ 就是单音（正弦），可以重建
         *   还有别的谱线          ⇒ 源里仍有超奈奎斯特的成分（方波/锯齿的
         *                            谐波折回来了），判不出来，归「未知」
         * 未知就不重建 —— 在不知道是什么的情况下画一条漂亮的曲线，那是造假。
         */
        s_wf_alias = 1;
        const float a_ref = mag_near_bin(b1, half);
        float spur = 0.0f;
        for(int i = 4; i < s_real_bins; i++) {          /* 跳过 DC 附近的泄漏 */
            if(i >= b1 - half && i <= b1 + half) continue;   /* 跳过基波主瓣（含裙边） */
            if(s_real_mag[i] > spur) spur = s_real_mag[i];
        }
        s_wf_kind = (spur < a_ref * 0.0316f) ? WF_SINE : WF_UNKNOWN;  /* -30 dB */
        return;
    }

    if(r2 < 0.20f && r3 > 0.20f && r3 < 0.60f) { s_wf_kind = WF_SQUARE;   return; }
    if(r2 < 0.20f && r3 > 0.05f && r3 <= 0.20f) { s_wf_kind = WF_TRIANGLE; return; }
    if(r2 > 0.25f && r2 < 0.80f && r3 > 0.15f)  { s_wf_kind = WF_SAW;      return; }
    if(r2 < 0.10f && r3 < 0.05f)                { s_wf_kind = WF_SINE;     return; }
}

/*
 * 自动抽取：用 FFT 测出来的频率决定抽取倍数。
 *
 * 目标：一屏（一帧）大约放 10 个周期。判据用"一帧里有几个周期"：
 *     periods = f × N / fs_eff        （fs_eff = 65MHz / dec）
 *
 * 死区 2~30 个周期：太少（<2）说明信号频率太低、一帧装不下一个完整波形，
 * 要加大抽取；太多（>30）说明抽过头了，波形糊成一团，要减小。
 * 中间这段不动，避免在边界上来回跳。
 *
 * 命令通过 UART 反向通道发给 FPGA（0xA5, k）。FPGA 还没实现接收时，
 * 命令发出去没有任何副作用；靠帧头的 decimation 字段判断有没有被采纳，
 * 连续 3 次没变就认为对面不支持，不再发，直到频率变化超过 2 倍再重试。
 */
static void auto_decimation(void)
{
    static uint32_t s_next_ms;
    static int      s_tries;
    static int      s_blind;      /* 连续「测不准」而加大抽取的次数 */
    static float    s_last_f;

    if(!S.use_real || s_real_peak_hz <= 0.0f || s_full_n < 2) return;

    const int cur = v32_last_decimation();
    if(cur <= 0) return;                       /* 还没收到过帧 */

    /* 频率大幅变化 -> 重新允许尝试 */
    if(s_last_f <= 0.0f ||
       s_real_peak_hz > s_last_f * 2.0f || s_real_peak_hz < s_last_f * 0.5f)
        s_tries = 0;

    const uint32_t now = lv_tick_get();
    if((int32_t)(now - s_next_ms) < 0) return;  /* 800 ms 内不重复动 */

    const wave_source_t *src = wave_source_get();
    const float fs_eff = (src && src->sample_rate > 0) ? (float)src->sample_rate : 0.0f;
    if(fs_eff <= 0.0f) return;

    float periods = s_real_peak_hz * (float)s_full_n / fs_eff;

    int want = cur;

    /*
     * 先判断"这一帧到底测不测得准"。
     *
     * 抽取倍率不够时，一帧里可能只有零点几个周期 —— 这时 FFT 的峰其实是噪声，
     * 报出来的频率毫无意义（实测 500 Hz 在 dec=1 下被报成 63 kHz，正好是奈奎斯特边上）。
     * 拿这种假频率去做"周期数"判断只会越走越偏。
     *
     * 所以：**峰值比底噪高不到 15 dB 就认为没测准，逐级加大抽取重测**，
     * 直到能测出真实频率（或者已经到 256 上限）。
     */
    const float margin = s_real_peak_dbfs - s_real_floor_dbfs;
    if(margin < 15.0f) {
        if(cur < 256 && s_blind < 9) { want = cur << 1; s_blind++; }
    } else {
        s_blind = 0;
        while(periods < 2.0f && want < 256) { want <<= 1; periods *= 2.0f; }
        while(periods > 30.0f && want > 1)  { want >>= 1; periods *= 0.5f; }
    }

    s_next_ms = now + 800;
    s_last_f  = s_real_peak_hz;

    if(want == cur) { s_tries = 0; return; }   /* 已经合适（或已生效） */
    if(dbg_on())
        fprintf(stderr, "[adec] f=%.0f Hz periods=%.1f -> dec %d->%d (第 %d 次)\n",
                (double)s_real_peak_hz, (double)periods, cur, want, s_tries + 1);
    if(s_tries >= 3) return;                   /* 对面不认命令，别再发 */
    if(v32_send_decimation(want) == 0) s_tries++;
}

/* ===== 自动测量（与 tools/scope_proto_1024.html 里同一套算法）=====
 * 输入是 ADC 码值（s_full 系列就是这个单位），显示时再乘 mV/码。
 * 数值一律按**整数个周期**统计 —— 整帧平均会把首尾半个周期算进去，
 * 对称方波会凭空多出几十 mV 的直流，那是假数。
 */
typedef struct {
    float vmax, vmin, vpp, vamp, vrms, vavg;   /* ADC 码 */
    float period_s, freq_hz, duty, rise_s;
    int   cycles;
    int   ok;                                   /* 找到周期 = 1 */
} meas_t;

static void measure_buf(const float *x, int n, float fs, meas_t *m)
{
    memset(m, 0, sizeof *m);
    if (!x || n < 8 || fs <= 0.0f) return;

    float mn = x[0], mx = x[0];
    for (int i = 1; i < n; i++) { if (x[i] < mn) mn = x[i]; if (x[i] > mx) mx = x[i]; }
    m->vmin = mn; m->vmax = mx; m->vpp = mx - mn; m->vamp = (mx - mn) * 0.5f;

    const float mid = (mn + mx) * 0.5f;
    if (m->vpp < 1.0f) return;                  /* 信号太小，周期/占空比没意义 */

    /* 过零计数求周期（中点 + 滞回，避免噪声在阈值上反复触发） */
    int rise[256]; int nr = 0;
    int hi = (x[0] > mid);
    for (int i = 1; i < n && nr < 256; i++) {
        const int now = (x[i] > mid);
        if (!hi && now) rise[nr++] = i;
        hi = now;
    }

    /* 先给个整帧的兜底值 */
    float su = 0.0f, sq = 0.0f;
    for (int i = 0; i < n; i++) { su += x[i]; sq += x[i] * x[i]; }
    m->vavg = su / (float)n;
    m->vrms = sqrtf(sq / (float)n);

    if (nr < 2) return;
    float acc = 0.0f;
    for (int i = 1; i < nr; i++) acc += (float)(rise[i] - rise[i-1]);
    const float avg = acc / (float)(nr - 1);
    m->period_s = avg / fs;
    m->freq_hz  = 1.0f / m->period_s;
    m->ok = 1;

    /* 占空比：中点以上的样点数 / 周期 */
    int hiAcc = 0, seg = 0;
    for (int i = 1; i < nr; i++) {
        int w = 0;
        for (int j = rise[i-1]; j < rise[i]; j++) if (x[j] > mid) w++;
        hiAcc += w; seg++;
    }
    if (seg > 0) m->duty = (float)hiAcc / (float)seg / avg;

    /* 有效值/平均值：整数个周期 */
    const int per = (int)(avg + 0.5f);
    if (per > 2) {
        const int st = rise[0];
        const int en = st + per * ((n - st - 1) / per);
        float s2 = 0.0f, q2 = 0.0f; int c = 0;
        for (int i = st; i < en; i++) { s2 += x[i]; q2 += x[i] * x[i]; c++; }
        if (c > 0) {
            m->vavg = s2 / (float)c;
            m->vrms = sqrtf(q2 / (float)c);
            m->cycles = c / per;
        }
    }

    /* 上升时间 10%→90%（第一个完整边沿） */
    const float lo = mn + m->vpp * 0.1f, hiL = mn + m->vpp * 0.9f;
    for (int i = 1; i < n; i++) {
        if (x[i-1] < lo && x[i] >= lo) {
            for (int j = i; j < n; j++)
                if (x[j] >= hiL) { m->rise_s = (float)(j - (i-1)) / fs; break; }
            break;
        }
    }
}

static meas_t s_m1;      /* CH1 的测量结果 */

/* ===== Math：A+B / A-B（第三条迹线）=====
 * 只做加减：乘除的量纲是 V²/无量纲，跟同一套纵轴放不下（原型里靠独立档位，
 * 设备上这块屏没有位置再放一个档位，先做加法/减法这两个真正实用的）。 */
/* 触发：0=关 1=自动 2=正常；边沿 0=升 1=降；电平按帧内 min..max 归一化 */
/* 测量光标：2 竖（T1/T2）+ 2 横（V1/V2），都是 0..1 的相对位置 */
/* 垂直增益（手动档位）：1× / 2× / 4× / 0.5× / 0.25×，乘在归一化后的 y 上 */
static int   s_vgain;
static const float VGAINS[5] = { 1.0f, 2.0f, 4.0f, 0.5f, 0.25f };

static int   s_cur_on;
static float s_cur_x[2] = { 0.30f, 0.70f };
static float s_cur_y[2] = { 0.35f, 0.65f };

static int   s_trig_mode, s_trig_edge;
static float s_trig_lev = 0.5f;

static int   s_math_mode = 2;    /* 0=关 1=A+B 2=A-B（先默认 A-B，界面上能直接看到） */
static float s_math[PLOT_MAX_PTS];
static float s_math_y[PLOT_MAX_PTS];

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
        if((uint32_t)(lv_tick_get() - s_real_t_last) > stale_ms()) return;
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

    /* ===== 触发：把触发点钉在屏幕左侧 20% 处 =====
     * 用 armed 方式找穿越点：先出现"明确在电平另一侧"的样点才算武装，
     * 之后第一次穿过电平就是触发点。
     * （**不要**写成"相邻两点必须跨过 lv±hys"——采样率低时一个周期内
     *  样点跳不过 2×hys，那会变成一次也触发不到，之前踩过这个坑。）
     */
    if(s_trig_mode && fn > 8) {
        float lo = s_full0[0], hi = s_full0[0];
        for (int i = 1; i < fn; i++) {
            if (s_full0[i] < lo) lo = s_full0[i];
            if (s_full0[i] > hi) hi = s_full0[i];
        }
        if (hi - lo > 1.0f) {
            const float lv = lo + (hi - lo) * s_trig_lev;
            int armed = 0, ti = -1;
            for (int i = 1; i < fn; i++) {
                if (s_trig_edge == 0) {
                    if (s_full0[i-1] <= lv) armed = 1;
                    if (armed && s_full0[i] >= lv) { ti = i; break; }
                } else {
                    if (s_full0[i-1] >= lv) armed = 1;
                    if (armed && s_full0[i] <= lv) { ti = i; break; }
                }
            }
            const int span = (s_zoom_span > 0 && s_zoom_span <= fn) ? s_zoom_span : fn;
            if (ti >= 0) {
                int st = ti - (int)((float)span * 0.2f);
                if (st < 0) st = 0;
                if (st > fn - span) st = fn - span;
                s_zoom_start = st;
            }
        }
    }

    /* 有了样点就算全套测量（跟绘图共用同一帧，读数不会跟波形对不上） */
    {
        const wave_source_t *ms = wave_source_get();
        const float fse = (ms && ms->sample_rate > 0) ? (float)ms->sample_rate : 65000000.0f;
        measure_buf(s_full0, fn, fse, &s_m1);
    }

    /* Math 迹线：算完直接用 CH1 的纵向映射归一化，保证跟着同一套刻度走 */
    if(s_math_mode) {
        for(int i = 0; i < fn; i++)
            s_math[i] = (s_math_mode == 1) ? (s_full0[i] + s_full1[i])
                                           : (s_full0[i] - s_full1[i]);
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

    /* ③ 频谱。默认用**交流成分大的那一路**（另一路可能没接或只有直流，
     *    拿它算频率会得到一个恒在 bin 1 的假读数）；
     *    也可以在频谱页手动指定 CH1 / CH2 —— 两路都接了的时候必须能选，
     *    否则界面上根本看不出这条谱是谁的。 */
    int use_ch;
    if(S.fft_ch == 1)      use_ch = 0;
    else if(S.fft_ch == 2) use_ch = 1;
    else                   use_ch = (s_real_vpp1_mv >= s_real_vpp0_mv) ? 1 : 0;
    s_real_fft_ch = use_ch;
    const float *spec_src = use_ch ? f1 : f0;

    /*
     * 先去掉直流再算谱。
     *
     * ADC 有零点偏置，不过去的话 bin 0 永远是全场最大 —— 后果有两个：
     *   1. 频谱左边缘立着一根直流尖峰，把真正关心的信号压得看不见；
     *   2. fetch_spectrum() 从 i=0 开始找峰值，▼ 箭头永远钉在原点。
     *      （频率读数看着是对的，因为它从 bin 1 开始搜；错的是箭头和画面。）
     * 示波器的频谱本来就是交流耦合的，先减均值是标准做法。
     */
    {
        static float s_fft_tmp[WAVE_MAX_FRAME];
        double sum = 0.0;
        for(int i = 0; i < fn; i++) sum += (double)spec_src[i];
        const float mean = (float)(sum / (double)fn);
        for(int i = 0; i < fn; i++) s_fft_tmp[i] = spec_src[i] - mean;
        fft_magnitude_spectrum_zero_win(s_fft_tmp, fn, REAL_FFT_N,
                                        (fft_win_t)S.win_idx, s_real_mag);
    }
    s_real_bins = REAL_FFT_N / 2;

    int pk = 1;
    for(int i = 2; i < s_real_bins; i++)
        if(s_real_mag[i] > s_real_mag[pk]) pk = i;

    s_real_peak_hz   = (float)pk * (float)src->sample_rate / (float)REAL_FFT_N;
    if(dbg_on())
        fprintf(stderr, "[fft] ch=CH%d(vpp %.0f/%.0f mV, sel=%d) pk_bin=%d (%.3f MHz) mag=%.1f\n",
                s_real_fft_ch+1, (double)s_real_vpp0_mv, (double)s_real_vpp1_mv, S.fft_ch,
                pk, (double)s_real_peak_hz/1e6, (double)s_real_mag[pk]);
    s_real_peak_dbfs = 20.0f * log10f((s_real_mag[pk] + 1e-9f) / REAL_FULLSCALE_COUNTS);
    s_real_floor_dbfs = 20.0f * log10f((noise_floor_mag(s_real_mag, s_real_bins) + 1e-9f)
                                       / REAL_FULLSCALE_COUNTS);
    s_real_phase_deg = phase_diff_deg(f0, f1, fn, s_real_peak_hz, (float)src->sample_rate);

    /* 波形类型：判据只用前三次谐波的比例，见 classify_wave() */
    classify_wave();

    s_real_fid   = st->last_frame_id;
    s_real_t_last = lv_tick_get();
    s_real_valid = true;

    if(S.auto_fit) auto_fit_apply(fn);
    auto_decimation();
}

/*
 * 把 [s_zoom_start, s_zoom_start + s_zoom_span) 这段样点映射成 n 个 0..1 的点。
 *
 * 窗口比输出点多（缩小时）用 min/max 分桶，一桶出两个点（先出现者在前），
 * 这样窄脉冲不会像等间隔抽点那样被抽没；窗口比输出点少（放大时）用线性插值。
 * 纵向用的是**整帧**的 `s_real_lo/s_real_sc`，不随缩放变 ——
 * 否则 Vpp 读数会跟着缩放跳，看起来像信号自己在变。
 */
/*
 * bl = true 时用带限（sinc 多相 FIR）重建，false 时用线性插值。
 *
 * 差别只在「窗口比输出点少」的分支（放大来看细节时）：
 *   线性插值  —— 把两点用直线连起来。10 MHz 方波在 65 MSPS 下只有 6.5 点/周期，
 *                连出来是斜坡；30 MHz 只有 2.17 点/周期，连出来几乎是乱码。
 *   带限重建  —— 30 MHz < 32.5 MHz 奈奎斯特，理论可完全恢复，实测重建误差
 *                0.025 counts（幅度 1000）。方波会带一点吉布斯振铃，那是带限的
 *                必然结果，不是 bug。
 *
 * 缩得很小（span >= n*2）时还是走 min/max 分桶，因为那时一屏几百个周期，
 * 逐点重建既慢又没有意义（每个像素里本来就有好几个完整周期）。
 */
static void map_window(const float *src, int src_n, float *y01, int n, bool bl)
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
    } else if(bl) {
        if(dbg_on())
            fprintf(stderr, "[map] BL span=%d start=%d n=%d kind=%d\n",
                    span, start, n, s_wf_kind);
        /* 输出点恰好等于窗口样点数时 dt=1，重建结果就是原始样点本身 */
        /* 正弦不钳位（重建本来就是精确解）；方波/三角波/锯齿波钳位，
         * 保证原本平整的地方不出现吉布斯波浪。 */
        bl_resample(src, src_n, y01, n,
                    (float)start, (float)(span - 1) / (float)(n - 1),
                    s_wf_kind != WF_SINE);
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
        /*
         * 重建只在一屏点数 < 2048 时才有意义（见 map_window 的分支）：
         * 整帧缩放走 min/max 分桶，上下两图必然一模一样，看着像"重建没了"。
         * 所以**这次进入重建页后，一拿到有效数据就自动框到 10 个周期**，
         * 只框一次 —— 之后用户可以自由缩放，不会被强行拉回。
         * （必须在有峰值之后做：ui_set_page 那一刻还没数据，框不了。）
         */
        if(!s_ai_autoframed && s_real_valid && s_real_peak_hz > 0.0f
           && S.ai && wf_can_reconstruct()
           && (s_zoom_span <= 0 || s_zoom_span >= PLOT_MAX_PTS * 2)) {
            S.auto_fit = true;
            s_ai_autoframed = true;
        }

        if(s_real_valid && s_full_n >= 2) {
            /* 示波器页是否重建，跟着**全局重建开关**走（用户要求）。
             * 关掉 = 原始采样点直连（测量视图，所见即所采）；
             * 打开 = 带限重建，右上角会亮一个「重建」牌，免得看不出来是加工过的。 */
            map_window(ch ? s_full1 : s_full0, s_full_n, y01, n,
                       S.ai && wf_can_reconstruct());
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

    static char last[160];
    char core[112];
    char buf[160];

    if(S.use_real && s_real_sc > 0.f) {
        /* 满屏码值 = 1/sc，除纵向格数、再除增益 = 每格多少码 -> 换 mV */
        const float mv_div = (1.0f / s_real_sc) / (float)V32_GRID_Y / s_gain
                             * V32_ADC_MV_PER_LSB;
        snprintf(core, sizeof core, "%.3f µs/格 · %.2f mV/格",
                 (double)us_div, (double)mv_div);
    } else {
        snprintf(core, sizeof core, "%.3f µs/格 · 触发 自动", (double)us_div);
    }
    if(s_cur_on && s_real_sc > 0.f) {
        /* 光标读数：ΔT 用时间轴换算，ΔV 用当前纵向满屏码值换算 */
        const float dts = fabsf(s_cur_x[1] - s_cur_x[0]) * (float)span / fs;
        const float dvm = fabsf(s_cur_y[1] - s_cur_y[0]) * (1.0f / s_real_sc) * V32_ADC_MV_PER_LSB;
        char cur[64];
        if(dts < 1e-3f) snprintf(cur, sizeof cur, "  ΔT %.2f µs · ΔV %.1f mV", (double)(dts*1e6f), (double)dvm);
        else            snprintf(cur, sizeof cur, "  ΔT %.2f ms · ΔV %.1f mV", (double)(dts*1e3f), (double)dvm);
        strncat(core, cur, sizeof core - strlen(core) - 1);
    }
    snprintf(buf, sizeof buf, "%s%s", S.running ? "" : "⏸ 已冻结 · ", core);

    if(strcmp(buf, last) != 0) {
        lv_label_set_text(s_lbl_scope_axis, buf);
        snprintf(last, sizeof last, "%s", buf);
    }

    /* 绘图区刻度文字（照 HTML 原型）：底部 6 个时间刻度、左侧 5 个电压刻度 */
    if(s_plot_scope && S.use_real && s_real_sc > 0.f) {
        const float mvdiv = (1.0f / s_real_sc) / (float)V32_GRID_Y / s_gain * V32_ADC_MV_PER_LSB;
        static char xb[6][12], yb[5][12];
        const char *xp[6], *yp[5];
        for(int i = 0; i < 6; i++) {
            const float t = (float)(i * 2 - V32_GRID_X / 2) * us_div;      /* µs */
            if(fabsf(t) < 0.5f)        snprintf(xb[i], 12, "0");
            else if(fabsf(t) < 1000.f) snprintf(xb[i], 12, "%.0fµs", (double)t);
            else                       snprintf(xb[i], 12, "%.1fms", (double)(t / 1000.f));
            xp[i] = xb[i];
        }
        for(int j = 0; j < 5; j++) {
            const float v = (float)(V32_GRID_Y / 2 - j * 2) * mvdiv;       /* mV */
            if(fabsf(v) < 0.5f)         snprintf(yb[j], 12, "0");
            else if(fabsf(v) < 1000.f)  snprintf(yb[j], 12, "%.0fmV", (double)v);
            else                        snprintf(yb[j], 12, "%.2fV", (double)(v / 1000.f));
            yp[j] = yb[j];
        }
        plot_set_ticks(s_plot_scope, xp, 6, yp, 5);
    }

    /* 顶部信息行（照 HTML 原型）：A1=… A2=… Time/Div=… Trig=… */
    if(s_plot_scope && S.use_real) {
        char info[96], a1[16], a2[16], td[16];
        if(s_real_sc > 0.f) {
            const float mv = (1.0f / s_real_sc) / (float)V32_GRID_Y / s_gain * V32_ADC_MV_PER_LSB;
            if(mv < 1000.f) snprintf(a1, sizeof a1, "%.2fV", (double)(mv / 1000.f));
            else            snprintf(a1, sizeof a1, "%.1fV", (double)(mv / 1000.f));
        } else snprintf(a1, sizeof a1, "--");
        snprintf(a2, sizeof a2, "%s", S.ch2 ? a1 : "GND");
        if(us_div < 1000.f) snprintf(td, sizeof td, "%.0fµs", (double)us_div);
        else                snprintf(td, sizeof td, "%.1fms", (double)(us_div / 1000.f));
        char curv[40] = "";
        if(s_cur_on && s_real_sc > 0.f) {
            const float dts = fabsf(s_cur_x[1] - s_cur_x[0]) * (float)span / fs;
            const float dvm = fabsf(s_cur_y[1] - s_cur_y[0]) * (1.0f / s_real_sc) * V32_ADC_MV_PER_LSB;
            if(dts < 1e-3f) snprintf(curv, sizeof curv, "  ΔT %.1fµs ΔV %.0fmV", (double)(dts*1e6f), (double)dvm);
            else            snprintf(curv, sizeof curv, "  ΔT %.2fms ΔV %.0fmV", (double)(dts*1e3f), (double)dvm);
        }
        snprintf(info, sizeof info, "A1=%s A2=%s %s/格 Trig=%s%s%s",
                 a1, a2, td,
                 s_trig_mode == 0 ? "关" : (s_trig_mode == 1 ? "Auto" : "Norm"),
                 s_trig_mode ? (s_trig_edge ? "↓" : "↑") : "", curv);
        /* 直接写进左下角那行标签（它自带底板，压在波形上也读得清）；
           plot 内的信息行已关闭，避免同一行字画两遍。 */
        if(s_lbl_scope_axis) lv_label_set_text(s_lbl_scope_axis, info);
    }

    if(s_chip_tb) {
        char tb[16];
        if(us_div < 1.f)       snprintf(tb, sizeof tb, "%.2fµs", (double)us_div);
        else if(us_div < 1000.f) snprintf(tb, sizeof tb, "%.0fµs", (double)us_div);
        else                   snprintf(tb, sizeof tb, "%.1fms", (double)(us_div / 1000.f));
        lv_label_set_text(lv_obj_get_child(s_chip_tb, 0), tb);
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
    /*
     * **优先用 FPGA 端算好的频谱**（V32SPEC!）。
     *
     * 为什么换：FPGA 用 4096 点 FFT + 片上抽取，分辨率比主机拿 1024 点算的高 4 倍；
     * 而且频谱数据和波形来自同一次采集，上下对得上。
     * 收不到频谱帧（旧位流 / 切到调制模式）时才退回本地 FFT —— 不能因为没谱就不显示。
     */
    {
        v32_spec_view_t sp;
        if(S.use_real && v32_get_spec(&sp) && sp.got > 2 && sp.nfft > 0 && sp.fs_eff_hz > 0) {
            int   bi = 2;
            float bv = -1e30f;
            const uint32_t bins = sp.got;
            for(int i = 0; i < n; i++) {
                /* 每点覆盖 [s0,s1) 这一段 bin，取组内最大值 —— 窄谱线不会被抽点漏掉 */
                int s0 = (int)((long)i * (long)(bins - 1) / (long)(n > 1 ? n - 1 : 1));
                int s1 = (int)((long)(i + 1) * (long)(bins - 1) / (long)(n > 1 ? n - 1 : 1));
                if(s1 <= s0) s1 = s0 + 1;
                if(s1 > (int)bins) s1 = (int)bins;

                int mx = 0;
                for(int k = s0; k < s1; k++) if(sp.data[k] > mx) mx = sp.data[k];

                const float db = (float)v32_spec_dbfs_centi(mx, sp.scale_dbfs) / 100.0f;
                float y = (db + V32_DEMO_DB_FULLSCALE) / V32_DEMO_DB_FULLSCALE;
                if(y < 0.f) y = 0.f;
                if(y > 1.f) y = 1.f;
                y01[i] = y;
                if(i >= 2 && y > bv) { bv = y; bi = i; }
            }
            /* 峰值频率直接按 bin 算：k × fs_eff / nfft */
            int pkb = 0;
            for(uint32_t k = 1; k < bins; k++) if(sp.data[k] > sp.data[pkb]) pkb = (int)k;
            s_spec_peak_hz = (float)((double)pkb * (double)sp.fs_eff_hz / (double)sp.nfft);
            s_spec_in_use  = 1;
            s_spec_win     = (int)sp.window;
            s_spec_bins    = (int)bins;
            s_spec_nfft    = (int)sp.nfft;

            if(peak_x)   *peak_x   = (float)bi / (float)(n - 1);
            if(peak_y01) *peak_y01 = bv;
            if(dbg_on())
                fprintf(stderr, "[spec] FPGA 谱 bins=%u nfft=%u 峰值 bin %d = %.3f MHz\n",
                        (unsigned)bins, (unsigned)sp.nfft, pkb, (double)s_spec_peak_hz / 1e6);
            return;
        }
        s_spec_in_use = 0;
    }

    if(S.use_real && s_real_valid && s_real_bins > 2) {
        int   bi = 2;
        float bv = -1e30f;
        for(int i = 0; i < n; i++) {
            /*
             * 每个输出点取**对应这一组 bin 的最大值**，不能只抽一个 bin。
             *
             * 一屏 256 点要覆盖 4096 个 bin，每点平均跨 16 个 bin；
             * 而 30 MHz 这类谱线只占 1~2 个 bin —— 均匀抽点会**整条谱线漏掉**，
             * 画出来的"频谱"全是噪声底，▼ 也会在噪声里乱跳（看着像停在原点）。
             * 取组内最大值才能保证窄谱线一定被画出来。
             */
            int s0 = (int)((long)i * (s_real_bins - 1) / (n > 1 ? n - 1 : 1));
            int s1 = (int)((long)(i + 1) * (s_real_bins - 1) / (n > 1 ? n - 1 : 1));
            if(s1 <= s0) s1 = s0 + 1;
            if(s1 > s_real_bins) s1 = s_real_bins;

            float mx = 0.0f;
            for(int k = s0; k < s1; k++)
                if(s_real_mag[k] > mx) mx = s_real_mag[k];

            const float db = 20.0f * log10f((mx + 1e-9f) / REAL_FULLSCALE_COUNTS);
            float y = (db + V32_DEMO_DB_FULLSCALE) / V32_DEMO_DB_FULLSCALE;
            if(y < 0.f) y = 0.f;
            if(y > 1.f) y = 1.f;
            y01[i] = y;
            if(i >= 2 && y > bv) { bv = y; bi = i; }
        }
        if(peak_x)   *peak_x   = (float)bi / (float)(n - 1);
        if(peak_y01) *peak_y01 = bv;
        if(dbg_on()) {
            /* 列出曲线最高的三个点，确认 ▼ 落点对不对 */
            int b1=-1,b2=-1,b3=-1;
            for(int i=2;i<n;i++){
                if(b1<0||y01[i]>y01[b1]){b3=b2;b2=b1;b1=i;}
                else if(b2<0||y01[i]>y01[b2]){b3=b2;b2=i;}
                else if(b3<0||y01[i]>y01[b3]){b3=i;}
            }
            fprintf(stderr, "[spec] n=%d  top: i=%d y=%.3f / i=%d y=%.3f / i=%d y=%.3f  peak_x=%.3f\n",
                    n, b1, (double)y01[b1], b2, (double)y01[b2], b3, (double)y01[b3],
                    (double)((float)bi/(float)(n-1)));
        }
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
    /* 光标拖动：按下和移动都抓最近那条线 */
    lv_obj_add_event_cb(s_plot_scope, cb_scope_touch, LV_EVENT_PRESSING, NULL);
    lv_obj_set_flex_grow(s_plot_scope, 1);
    lv_obj_set_height(s_plot_scope, MAIN_H);

    lv_obj_t *tr = mk_title_row(s_plot_scope, V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_t *t0 = mk_label(tr, 0, 0, "时域波形", V32_FONT_SMALL, V32_C_PLOT_TITLE);

    /* 右上角「重建」牌：示波器页一旦是加工过的波形，必须让人一眼看出来 */
    s_lbl_recon_badge = mk_label(s_plot_scope, 0, 0, "重建", V32_FONT_SMALL, V32_C_WARN);
    lv_obj_align(s_lbl_recon_badge, LV_ALIGN_TOP_RIGHT, -V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_set_style_bg_color(s_lbl_recon_badge, V32_C_PLOT, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_lbl_recon_badge, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_lbl_recon_badge, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(s_lbl_recon_badge, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(s_lbl_recon_badge, 2, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(t0, 1, LV_PART_MAIN);
    mk_label(tr, 0, 0, "● CH1", V32_FONT_SMALL, V32_C_CH1);
    mk_label(tr, 0, 0, "● CH2", V32_FONT_SMALL, V32_C_CH2);
    /* HTML 原型的网格上方没有这行标题，信息行（A1=… A2=…）就长在那一带，
       所以把这行整行藏掉，让画面跟原型一致 */
    lv_obj_add_flag(tr, LV_OBJ_FLAG_HIDDEN);

    s_lbl_scope_axis = mk_label(s_plot_scope, 0, 0, "",
                                V32_FONT_SMALL, V32_C_PLOT_FOOT);
    /* 垫一块和画布同色的底板：这行字压在左下角，波形经常从它上面穿过去，
     * 白底主题下不垫就读不出来（FFT 页的角标同理，见 plot.c 的 text_plated）。 */
    lv_obj_set_style_bg_color(s_lbl_scope_axis, V32_C_PLOT, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_lbl_scope_axis, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_radius(s_lbl_scope_axis, 3, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(s_lbl_scope_axis, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(s_lbl_scope_axis, 1, LV_PART_MAIN);
    /* 信息行放在**左上角**（HTML 原型的位置），并垫一块画布色底板压住网格 */
    lv_obj_set_width(s_lbl_scope_axis, 620);
    lv_label_set_long_mode(s_lbl_scope_axis, LV_LABEL_LONG_CLIP);
    lv_obj_align(s_lbl_scope_axis, LV_ALIGN_TOP_LEFT, 8, 4);

    /* ---- 右侧读数栏 ---- */
    lv_obj_t *aside = mk(pg, 0, 0, ASIDE_W, MAIN_H);
    s_aside[UI_PAGE_SCOPE] = aside;
    lv_obj_set_flex_flow(aside, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(aside, V32_RIGHT_GAP, LV_PART_MAIN);

    lv_obj_t *p1 = mk_panel(aside, 0, 0, 0, 0);
    lv_obj_set_width(p1, LV_PCT(100));
    lv_obj_set_flex_grow(p1, 1);
    /* 照 HTML：标题「CH1 测量」绿色小字，10 行「标签左/数值右」无框线 */
    mk_title(p1, 0, 0, "CH1 测量");
    s_lbl_v1         = mk_metric(p1, TITLE_H + METRIC_H * 0, "峰峰值 Vpp",  "--", V32_C_METRIC_VALUE);
    s_lbl_m_vmax     = mk_metric(p1, TITLE_H + METRIC_H * 1, "最大值 Vmax", "--", V32_C_METRIC_VALUE);
    s_lbl_m_vmin     = mk_metric(p1, TITLE_H + METRIC_H * 2, "最小值 Vmin", "--", V32_C_METRIC_VALUE);
    s_lbl_m_vamp     = mk_metric(p1, TITLE_H + METRIC_H * 3, "幅度 Vamp",   "--", V32_C_METRIC_VALUE);
    s_lbl_m_vrms     = mk_metric(p1, TITLE_H + METRIC_H * 4, "有效值 Vrms", "--", V32_C_METRIC_VALUE);
    s_lbl_m_vavg     = mk_metric(p1, TITLE_H + METRIC_H * 5, "平均值 Vavg", "--", V32_C_METRIC_VALUE);
    s_lbl_m_per      = mk_metric(p1, TITLE_H + METRIC_H * 6, "周期 T",      "--", V32_C_METRIC_VALUE);
    s_lbl_scope_freq = mk_metric(p1, TITLE_H + METRIC_H * 7, "频率 f",      "--", V32_C_METRIC_VALUE);
    s_lbl_m_duty     = mk_metric(p1, TITLE_H + METRIC_H * 8, "占空比",      "--", V32_C_METRIC_VALUE);
    s_lbl_m_rise     = mk_metric(p1, TITLE_H + METRIC_H * 9, "上升时间",    "--", V32_C_METRIC_VALUE);
    /* 光标段：洋红（照 HTML） */
    s_lbl_cur_dt     = mk_metric(p1, TITLE_H + METRIC_H * 10, "ΔT",   "--", V32_C_TRIGGER);
    s_lbl_cur_f      = mk_metric(p1, TITLE_H + METRIC_H * 11, "1/ΔT", "--", V32_C_TRIGGER);
    s_lbl_cur_dv     = mk_metric(p1, TITLE_H + METRIC_H * 12, "ΔV",   "--", V32_C_TRIGGER);

    /* CH2 峰峰值 / 相位差已并入上面 10 项（HTML 里没有这两行，A2 在信息条里），
       但 tick_scope 还在写这两个句柄 —— 建在面板外，别让它们露出来。 */
    s_lbl_v2    = mk_metric(p1, -80, "CH2 峰峰值", "--", V32_C_CH2);
    s_lbl_phase = mk_metric(p1, -80, "相位差",     "--", V32_C_TRIGGER);

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
/* 频谱页手指点选：把游标挪到点中的位置，读出那一点的频率和幅度。
 * 不做拖拽，只做点一下 —— 拖拽要跟触摸手势状态机抢事件，收益不大。 */
static void cb_fft_cursor(lv_event_t *e)
{
    if(dbg_on()) fprintf(stderr, "[cur] ev=%d\n", (int)lv_event_get_code(e));
    /* lv_indev_get_act() 在 CLICKED 时可能已被清空，返回 NULL；
     * 这时退而求其次取第一个输入设备（板上只有触摸屏一个），
     * 否则永远拿到 (0,0)，游标会一直停在最左边。 */
    lv_indev_t *indev = lv_indev_get_act();
    if(!indev) indev = lv_indev_get_next(NULL);
    if(!indev) return;

    /* 不用 lv_indev_get_point()：实测在事件回调里拿到的是控件左上角(13,0)，
     * 不是手指落点。改用 port.c 里存下来的原始触点。 */
    lv_point_t p;
    { int32_t tx = 0, ty = 0; port_touch_last_point(&tx, &ty); p.x = tx; p.y = ty; }

    lv_area_t a;
    lv_obj_get_coords(s_plot_fft, &a);
    if(p.x < a.x1) p.x = a.x1;
    if(p.x > a.x2) p.x = a.x2;

    const int w = a.x2 - a.x1;
    s_fft_cur_x = (w > 0) ? (float)(p.x - a.x1) / (float)w : 0.0f;
    if(s_fft_cur_x < 0.f) s_fft_cur_x = 0.f;
    if(s_fft_cur_x > 1.f) s_fft_cur_x = 1.f;
    s_fft_cur_on = true;
    if(dbg_on()) fprintf(stderr, "[cur] tap=(%d,%d) plot=(%d..%d) x=%.3f\n",
                         (int)p.x, (int)p.y, (int)a.x1, (int)a.x2, (double)s_fft_cur_x);
    ui_tick();
}

static void build_fft(lv_obj_t *pg)
{
    lv_obj_set_flex_flow(pg, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(pg, V32_PAGE_GAP, LV_PART_MAIN);

    s_plot_fft = plot_create(pg, PLOT_FFT);
    lv_obj_set_flex_grow(s_plot_fft, 1);
    lv_obj_set_height(s_plot_fft, MAIN_H);
    plot_set_axis_labels(s_plot_fft, "0 MHz", "32.5 MHz");

    lv_obj_t *tr = mk_title_row(s_plot_fft, V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_t *t0 = mk_label(tr, 0, 0, "FFT 频谱 · CH1",
                            V32_FONT_SMALL, V32_C_PLOT_TITLE);
    s_lbl_fft_title = t0;
    lv_obj_set_style_text_letter_space(t0, 1, LV_PART_MAIN);

    /* 频谱页游标：手指点哪儿就读哪儿的频率和幅度。
     * 只显示一个尖峰确实浪费 —— 一屏 32.5 MHz 范围内的任意位置都该能读。 */
    s_lbl_fft_cur = mk_label(s_plot_fft, 0, 0, "点击频谱读取", V32_FONT_SMALL, V32_C_WARN);
    lv_obj_align(s_lbl_fft_cur, LV_ALIGN_TOP_RIGHT, -V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_set_style_bg_color(s_lbl_fft_cur, V32_C_PLOT, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_lbl_fft_cur, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_lbl_fft_cur, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(s_lbl_fft_cur, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(s_lbl_fft_cur, 2, LV_PART_MAIN);

    lv_obj_add_flag(s_plot_fft, LV_OBJ_FLAG_CLICKABLE);
    /* 用 PRESSED 而不是 CLICKED：CLICKED 触发时输入设备的坐标可能已被清掉，
     * lv_indev_get_point() 会拿到 0，游标永远停在最左边。 */
    /* ⚠️ lv_obj_add_event_cb 的 filter 是**单个事件码精确匹配**，
     *    写成 (A|B|C) 是一个谁都不等于的值，回调永远不触发（踩过）。
     *    要听多个事件就注册多次。 */
    lv_obj_add_event_cb(s_plot_fft, cb_fft_cursor, LV_EVENT_CLICKED, NULL);
    /* 图上的标题/读数标签会挡住图本身，事件不冒泡，所以页容器也挂一份 */
    lv_obj_add_event_cb(pg, cb_fft_cursor, LV_EVENT_CLICKED, NULL);

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
    s_lbl_lock_state = mk_metric(p1, TITLE_H + METRIC_H * 4, "锁定状态", "跟踪中", V32_C_GREEN);
    mk_note(p1, TITLE_H + METRIC_H * 5 + 6, ASIDE_W - V32_PANEL_PAD * 2,
            "数值来自 FPGA 片上锁相；不可用时显示「—」，沿用旧值会误导。");
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
    lv_obj_t *t1 = mk_label(tr1, 0, 0, "原始采样点 · CH1 / CH2", V32_FONT_SMALL, V32_C_PLOT_TITLE);
    lv_obj_set_style_text_letter_space(t1, 1, LV_PART_MAIN);
    mk_label(tr1, 0, 0, "●", V32_FONT_SMALL, V32_C_CH1);

    /* 右上角：波形类型。底板同色，波形压过来也读得清。 */
    s_lbl_wave = mk_label(s_plot_raw, 0, 0, "—", V32_FONT_SMALL, V32_C_METRIC_VALUE);
    lv_obj_align(s_lbl_wave, LV_ALIGN_TOP_RIGHT, -V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_set_style_bg_color(s_lbl_wave, V32_C_PLOT, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_lbl_wave, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_lbl_wave, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(s_lbl_wave, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(s_lbl_wave, 2, LV_PART_MAIN);

    s_plot_den = plot_create(stack, PLOT_AI);
    lv_obj_set_width(s_plot_den, LV_PCT(100));
    lv_obj_set_height(s_plot_den, MAIN_H - V32_PAGE_GAP - half);
    lv_obj_t *tr2 = mk_title_row(s_plot_den, V32_PLOT_TITLE_X, V32_PLOT_TITLE_Y);
    lv_obj_t *t2 = mk_label(tr2, 0, 0, "重建输出 · 带限 FIR", V32_FONT_SMALL, V32_C_PLOT_TITLE);
    s_lbl_den_title = t2;
    lv_obj_set_style_text_letter_space(t2, 1, LV_PART_MAIN);
    mk_label(tr2, 0, 0, "●", V32_FONT_SMALL, V32_C_GREEN);

    lv_obj_t *aside = mk(pg, 0, 0, ASIDE_W, MAIN_H);
    s_aside[UI_PAGE_AI] = aside;
    lv_obj_set_flex_flow(aside, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *p1 = mk_panel(aside, 0, 0, 0, 0);
    lv_obj_set_width(p1, LV_PCT(100));
    lv_obj_set_flex_grow(p1, 1);
    mk_title(p1, 0, 0, "带限重建");
    mk_metric(p1, TITLE_H + METRIC_H * 0, "算法",     "多相 FIR ×20", V32_C_METRIC_VALUE);
    s_lbl_ai_stat = mk_metric(p1, TITLE_H + METRIC_H * 1, "状态", "已启用",   V32_C_GREEN);
    mk_metric(p1, TITLE_H + METRIC_H * 2, "抽头",  "129/相*",    V32_C_METRIC_VALUE);
    mk_metric(p1, TITLE_H + METRIC_H * 3, "群延迟", "64 点",      V32_C_METRIC_VALUE);
    mk_metric(p1, TITLE_H + METRIC_H * 4, "SNR 增益",  "—",         V32_C_METRIC_VALUE);
    mk_note(p1, TITLE_H + METRIC_H * 5 + 6, ASIDE_W - V32_PANEL_PAD * 2,
            "*正弦直接重建；方波/三角波/锯齿波重建时把输出钳位在原始样点"
            "极值内，保证平整处不出现振铃。认不出来不重建。");
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

/*
 * 频谱通道用**三个常显芯片**，不用下拉框。
 *
 * 理由有两条：
 *   1. 下拉框平时是收起来的，录视频/看一眼根本不知道当前选的是哪一路；
 *      芯片像别的控件一样一直亮着，选中的有高亮。
 *   2. 下拉框的点击实测不可靠（点 CH1 有时仍是"自动"），
 *      而 chip 的点击在整个界面里都是好的。
 */
static void fft_ch_set(int v)
{
    S.fft_ch = v;
    for(int i = 0; i < 3; i++)
        if(s_chip_fch[i]) chip_selected(s_chip_fch[i], i == v);
    ui_tick();
}
static void cb_fch_auto(lv_event_t *e) { LV_UNUSED(e); fft_ch_set(0); }
static void cb_fch_ch1 (lv_event_t *e) { LV_UNUSED(e); fft_ch_set(1); }
static void cb_fch_ch2 (lv_event_t *e) { LV_UNUSED(e); fft_ch_set(2); }

/*
 * 光标拖动：按下/移动时抓**最近**的那条线（±0.08 归一化，约 ±55 px），
 * 只允许在够近时抓，否则手指一点就把线吸过去 —— 用起来会很难受。
 */
static void cb_scope_touch(lv_event_t *e)
{
    if(!s_cur_on || s_plot_scope == NULL) return;
    lv_indev_t *ind = lv_event_get_indev(e);
    if(ind == NULL) return;

    lv_point_t pt;
    lv_indev_get_point(ind, &pt);

    lv_area_t a;
    lv_obj_get_coords(s_plot_scope, &a);
    const int w = a.x2 - a.x1, h = a.y2 - a.y1;
    if(w < 10 || h < 10) return;

    float rx = (float)(pt.x - a.x1) / (float)w;
    float ry = (float)(pt.y - a.y1) / (float)h;
    if(rx < 0.f) rx = 0.f;
    if(rx > 1.f) rx = 1.f;
    if(ry < 0.f) ry = 0.f;
    if(ry > 1.f) ry = 1.f;

    int best = -1; float bd = 0.08f;
    for(int i = 0; i < 2; i++) { float d = fabsf(rx - s_cur_x[i]); if(d < bd) { bd = d; best = i; } }
    for(int i = 0; i < 2; i++) { float d = fabsf(ry - s_cur_y[i]); if(d < bd) { bd = d; best = 2 + i; } }
    if(best < 0) return;

    if(best < 2) s_cur_x[best] = rx;
    else         s_cur_y[best - 2] = ry;
    ui_tick();
}

/* 时基档位表（跟 HTML 原型一致） */
static const float TB_US[12] = { 0.5f,1.f,2.f,5.f,10.f,20.f,50.f,100.f,200.f,500.f,1000.f,5000.f };
static void tb_default(int *idx)
{
    const wave_source_t *src = wave_source_get();
    const float fs = (src && src->sample_rate > 0) ? (float)src->sample_rate : 65000000.0f;
    const int   fn = (src && src->frame_len > 0) ? src->frame_len : V32_POINTS;
    /* 一屏 10 格、整帧放满：us/格 = 帧长/fs*1e6/10 */
    const float want = (float)fn / fs * 1e6f / (float)V32_GRID_X;
    int best = 0; float bd = 1e9f;
    for(int i = 0; i < 12; i++) { float d = fabsf(TB_US[i] - want); if(d < bd) { bd = d; best = i; } }
    *idx = best;
}
static void tb_apply(int idx)
{
    const wave_source_t *src = wave_source_get();
    const float fs = (src && src->sample_rate > 0) ? (float)src->sample_rate : 65000000.0f;
    const int   fn = (src && src->frame_len > 0) ? src->frame_len : V32_POINTS;
    if(idx < 0) idx = 0;
    if(idx > 11) idx = 11;
    s_tb_idx = idx;
    int span = (int)(TB_US[idx] * 1e-6f * (float)V32_GRID_X * fs + 0.5f);
    if(span < 8) span = 8;
    if(span > fn) span = fn;
    s_zoom_span  = span;
    s_zoom_start = (fn - span) / 2;
    update_axis_label();
}
static void cb_tb_dec(lv_event_t *e) { LV_UNUSED(e); tb_apply(s_tb_idx - 1); }
static void cb_tb_inc(lv_event_t *e) { LV_UNUSED(e); tb_apply(s_tb_idx + 1); }
static void cb_tb_auto(lv_event_t *e){ LV_UNUSED(e); int d; tb_default(&d); tb_apply(d); }

static void vg_label(void)
{
    if(s_chip_vg) { chip_set_text(s_chip_vg, s_vgain == 0 ? "自动" : (s_vgain == 3 ? "1/2" : (s_vgain == 4 ? "1/4" : (s_vgain == 2 ? "4x" : "2x"))));
                    chip_selected(s_chip_vg, s_vgain != 0); }
}
static void cb_vg_dec(lv_event_t *e) { LV_UNUSED(e); if(s_vgain > 0) s_vgain--; vg_label(); }
static void cb_vg_inc(lv_event_t *e) { LV_UNUSED(e); if(s_vgain < 4) s_vgain++; vg_label(); }
static void cb_vg_auto(lv_event_t *e){ LV_UNUSED(e); s_vgain = 0; vg_label(); }

static void cb_cur_cycle(lv_event_t *e)
{
    LV_UNUSED(e);
    s_cur_on = !s_cur_on;
    if(s_chip_cur) { chip_set_text(s_chip_cur, s_cur_on ? "开" : "关");
                     chip_selected(s_chip_cur, s_cur_on != 0); }
    ui_tick();
}

static void trig_refresh(void)
{
    static const char *nm[3] = { "关", "自动", "正常" };
    if(s_chip_trig) { chip_set_text(s_chip_trig, nm[s_trig_mode]);
                      chip_selected(s_chip_trig, s_trig_mode != 0); }
    if(s_chip_edge) chip_set_text(s_chip_edge, s_trig_edge ? "↓" : "↑");
    if(s_chip_lev)  lv_label_set_text_fmt(lv_obj_get_child(s_chip_lev, 0), "%.0f%%",
                                          (double)(s_trig_lev * 100.0f));
}
static void cb_trig_mode(lv_event_t *e) { LV_UNUSED(e); s_trig_mode = (s_trig_mode + 1) % 3; trig_refresh(); ui_tick(); }
static void cb_trig_edge(lv_event_t *e) { LV_UNUSED(e); s_trig_edge = !s_trig_edge; trig_refresh(); ui_tick(); }
static void cb_trig_lev_dec(lv_event_t *e){ LV_UNUSED(e); s_trig_lev -= 0.05f; if(s_trig_lev < 0.f) s_trig_lev = 0.f; trig_refresh(); }
static void cb_trig_lev_inc(lv_event_t *e){ LV_UNUSED(e); s_trig_lev += 0.05f; if(s_trig_lev > 1.f) s_trig_lev = 1.f; trig_refresh(); }
static void cb_trig_lev_auto(lv_event_t *e){ LV_UNUSED(e); s_trig_lev = 0.5f; trig_refresh(); }

static void cb_math_cycle(lv_event_t *e)
{
    LV_UNUSED(e);
    s_math_mode = (s_math_mode + 1) % 3;
    static const char *nm[3] = { "关", "A+B", "A-B" };
    if(s_chip_math) { chip_set_text(s_chip_math, nm[s_math_mode]);
                      chip_selected(s_chip_math, s_math_mode != 0); }
    ui_tick();
}

static void cb_win_cycle(lv_event_t *e)
{
    LV_UNUSED(e);
    S.win_idx = (S.win_idx + 1) % 3;
    if(dbg_on())
        fprintf(stderr, "[win] 窗函数 -> %d (%s)\n", S.win_idx, WIN_NAME[S.win_idx]);
    if(s_chip_win) chip_set_text(s_chip_win, WIN_NAME[S.win_idx]);
    /* 该页没有下拉框了，窗名由芯片自己显示 */
    if(s_lbl_fft_win) lv_label_set_text(s_lbl_fft_win, WIN_NAME[S.win_idx]);
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
    chip_set_text(s_chip_ai, S.ai ? "重建 开" : "重建 关");
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

    if(dbg_on()) fprintf(stderr, "[auto] pressed use_real=%d full_n=%d\n", (int)S.use_real, s_full_n);
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
    S.win_idx = WIN_DEF;
    {
        const char *e = getenv("WAVE_FFT_CH");   /* 调试用：直接指定 FFT 通道 0/1/2 */
        S.fft_ch = e ? atoi(e) : 0;
        if(S.fft_ch < 0 || S.fft_ch > 2) S.fft_ch = 0;
    } S.tau_idx = TAU_DEF; S.phase = 43.7f;
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
        if(s_chip_win) { chip_set_text(s_chip_win, WIN_NAME[WIN_DEF]); chip_selected(s_chip_win, true); }
        /* 频谱通道回「自动」 */
        fft_ch_set(0);
        dd = lv_obj_get_child(s_ctrl_grp[UI_PAGE_LOCK], 1);
        if(dd) lv_dropdown_set_selected(dd, (uint32_t)TAU_DEF);
    }
    chip_set_text(s_chip_ch1, "CH1 开"); chip_selected(s_chip_ch1, true);
    chip_set_text(s_chip_ch2, "CH2 开"); chip_selected(s_chip_ch2, true);
    chip_set_text(s_chip_ai,  "重建 开"); chip_selected(s_chip_ai, true);
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
    /* 真实数据下这个下拉框改不动任何东西（频率是量出来的），藏掉腾地方给触发 */
    if(S.use_real) lv_obj_add_flag(g1, LV_OBJ_FLAG_HIDDEN);
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

    /* Math：关 / A+B / A-B（点一下换一个）—— 放在时基组前面，最容易被看到 */
    {
        lv_obj_t *gm = mk_group(cp, "Math");
        s_chip_math = mk_chip(gm, "A-B", cb_math_cycle);
        chip_selected(s_chip_math, true);
    }

    /* 时基：◀ 值 ▶（点中间 = 自动） */
    {
        lv_obj_t *gs3 = mk_group(cp, "时基");
        mk_chip(gs3, "◀", cb_tb_dec);
        s_chip_tb = mk_chip(gs3, "2us", cb_tb_auto);
        mk_chip(gs3, "▶", cb_tb_inc);
    }

    /* 垂直档位：◀ 值 ▶（点中间 = 自动 1×） */
    {
        lv_obj_t *gv = mk_group(cp, "档位");
        mk_chip(gv, "◀", cb_vg_dec);
        s_chip_vg = mk_chip(gv, "自动", cb_vg_auto);
        mk_chip(gv, "▶", cb_vg_inc);
    }

    /* 光标开/关 */
    {
        lv_obj_t *gcu = mk_group(cp, "光标");
        s_chip_cur = mk_chip(gcu, "关", cb_cur_cycle);
        chip_selected(s_chip_cur, false);
    }

    /* 触发：模式 / 边沿 / 电平◀值▶ 各管一件事，别塞成一个 */
    {
        lv_obj_t *gt = mk_group(cp, "触发");
        s_chip_trig = mk_chip(gt, "关", cb_trig_mode);
        s_chip_edge = mk_chip(gt, "↑", cb_trig_edge);
        mk_chip(gt, "◀", cb_trig_lev_dec);
        s_chip_lev  = mk_chip(gt, "50%", cb_trig_lev_auto);
        mk_chip(gt, "▶", cb_trig_lev_inc);
        chip_selected(s_chip_trig, false);
    }

    /* 旧的时基下拉框不再创建：已被上面的 ◀ 值 ▶ 三段式取代。
       （原来这里还兼职当 s_ctrl_grp[UI_PAGE_SCOPE]，所以只 hide 会被
        ctrl_set_page 又显示回来 —— 必须整个不创建。） */
    s_ctrl_grp[UI_PAGE_SCOPE] = NULL;

    lv_obj_t *gc = mk_group(cp, "频谱通道");
    {
        lv_obj_t *rowc = mk(gc, 0, 0, LV_SIZE_CONTENT, V32_CHIP_H);
        lv_obj_set_flex_flow(rowc, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(rowc, 6, LV_PART_MAIN);
        s_chip_fch[0] = mk_chip(rowc, "自动", cb_fch_auto);
        s_chip_fch[1] = mk_chip(rowc, "CH1",  cb_fch_ch1);
        s_chip_fch[2] = mk_chip(rowc, "CH2",  cb_fch_ch2);
        chip_selected(s_chip_fch[0], S.fft_ch == 0);
        chip_selected(s_chip_fch[1], S.fft_ch == 1);
        chip_selected(s_chip_fch[2], S.fft_ch == 2);
    }
    s_ctrl_grp[UI_PAGE_FFT] = gc;

    lv_obj_t *gf2 = mk_group(cp, "窗函数");
    /*
     * ⚠️ 这里原来是个下拉框，实测**点不开**（点下去只重绘了框本身，
     *    弹出列表从来不出现 —— 和之前「频谱通道」下拉框同一个毛病）。
     *    控制条离屏幕底部只有 80 px，列表向下弹会出屏，而这一版 LVGL 不会自动翻到上方。
     *
     *    改成**一个循环切换的芯片**：芯片显示当前窗名，点一下换下一个。
     *    芯片的点击在整个界面里都是可靠的，而且一直看得到当前值。
     */
    s_chip_win = mk_chip(gf2, WIN_NAME[WIN_DEF], cb_win_cycle);
    chip_selected(s_chip_win, true);


    lv_obj_t *gl = mk_group(cp, "积分时间");
    lv_obj_t *dd_t = mk_dropdown(gl, TAU_OPTS, TAU_DEF, cb_tau);
    lv_obj_set_style_max_width(dd_t, V32_SEL_MAX_W, LV_PART_MAIN);
    s_ctrl_grp[UI_PAGE_LOCK] = gl;

    lv_obj_t *ga = mk(cp, 0, 0, LV_SIZE_CONTENT, V32_CHIP_H);
    /* 重建开关是**全局**的（示波器页也跟着变），所以放在主控制条，不放页内组 */
    mk_label(ga, 0, 0, "开关在主控条", V32_FONT_SMALL, V32_C_HINT);
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
    s_chip_ai = mk_chip(bar, "重建 开", cb_ai);   /* 全局开关 */
    chip_selected(s_chip_ai, S.ai);              /* S.ai 初始为 true，补一次选中色 */
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
        "⌁ 示波器", "▥ 频谱", "◎ 锁相", "✧ 重建", "⚙ 系统"
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

    /* 手动垂直档位：在 0..1 归一化之后按倍数放大/缩小，围绕中心线缩放。
       超出 0..1 的部分被裁掉 —— 真示波器超出屏幕也是这样，不会偷偷重新缩放。 */
    if(s_vgain != 0) {
        const float g = VGAINS[s_vgain];
        for(int i = 0; i < n; i++) {
            float v = 0.5f + (ch1[i] - 0.5f) * g; if(v < 0.f) v = 0.f; if(v > 1.f) v = 1.f; ch1[i] = v;
            float w = 0.5f + (ch2[i] - 0.5f) * g; if(w < 0.f) w = 0.f; if(w > 1.f) w = 1.f; ch2[i] = w;
        }
    }

    if(S.ch1) plot_set_trace(s_plot_scope, 0, ch1, n, V32_C_CH1, 2);
    else      plot_hide_trace(s_plot_scope, 0);
    if(S.ch2) plot_set_trace(s_plot_scope, 1, ch2, n, V32_C_CH2, 2);
    else      plot_hide_trace(s_plot_scope, 1);

    /* 测量光标（2 竖 + 2 横） */
    {
        const float xs[2] = { s_cur_x[0], s_cur_x[1] };
        const float ys[2] = { s_cur_y[0], s_cur_y[1] };
        plot_set_cursors(s_plot_scope, 4, xs, ys, s_cur_on != 0);
    }

    /* Math 迹线（亮绿）：A+B / A-B。用自己的量程画，纵向不会被 CH1 压扁。 */
    if(s_math_mode && s_real_valid && s_full_n >= 2) {
        map_window(s_math, s_full_n, s_math_y, n, false);
        plot_set_trace(s_plot_scope, 2, s_math_y, n, V32_C_GREEN, 2);
    } else {
        plot_hide_trace(s_plot_scope, 2);
    }

    plot_set_trigger(s_plot_scope, true, 0.46f);

    const bool use_real = S.use_real;
    const bool real = (use_real && s_real_valid);
    /* 接了串口但没数据时读 0，不显示演示值 */
    const float v1 = real ? s_real_vpp0_mv : (use_real ? 0.f : demo_vpp_mv(ch1, n, V32_GRID_Y));
    const float v2 = real ? s_real_vpp1_mv : (use_real ? 0.f : demo_vpp_mv(ch2, n, V32_GRID_Y));

    lv_label_set_text_fmt(s_lbl_v1, "%.1f mV", (double)v1);
    lv_label_set_text_fmt(s_lbl_v2, "%.1f mV", (double)v2);

    /*
     * 优先用**片上锁相**的读数。
     *
     * 锁相的 tracked_fcw 是 NCO 跟踪出来的频率，精度远高于 FFT 的频率栅格
     * （栅格 = fs/8192 ≈ 7.9 kHz，而锁相可以到 Hz 级）；相位差更是只有锁相
     * 才测得出，本地两路 FFT 只能给到栅格级。
     *
     * 但**必须严格看 usable**：失锁、未稳定、结果超 1 s、没有匹配波形帧，
     * 一律显示「—」，绝不沿用上一次的数值 —— 那是"看着合理其实错位"的读数。
     */
    v32_meas_view_t mm;
    const int  meas_fresh  = v32_get_meas(&mm);
    const bool meas_ok     = meas_fresh && mm.usable;

    if(real) {
        if(meas_ok)
            lv_label_set_text_fmt(s_lbl_scope_freq, "%.4f MHz", (double)mm.tracked_hz_x1000 / 1e9);
        else
            lv_label_set_text_fmt(s_lbl_scope_freq, "%.3f MHz", (double)s_real_peak_hz / 1e6);
    } else if(use_real) {
        lv_label_set_text(s_lbl_scope_freq, "0.000 MHz");
    } else {
        lv_label_set_text_fmt(s_lbl_scope_freq, "%.3f MHz", (double)FREQ_MHZ[S.freq_idx]);
    }

    /* 新增三项 + 光标读数（照 HTML 的 10 项 + 光标段） */
    if(S.use_real && s_real_valid && s_m1.vpp > 0.5f) {
        const float k2 = V32_ADC_MV_PER_LSB / 1000.0f;
        lv_label_set_text_fmt(s_lbl_m_vamp, "%.3f V", (double)(s_m1.vamp * k2));
        lv_label_set_text_fmt(s_lbl_m_vavg, "%.3f V", (double)(s_m1.vavg * k2));
        if(s_m1.ok && s_m1.rise_s > 0.f)
            lv_label_set_text_fmt(s_lbl_m_rise, "%.2f µs", (double)(s_m1.rise_s * 1e6f));
        else
            lv_label_set_text(s_lbl_m_rise, "—");
    } else {
        lv_label_set_text(s_lbl_m_vamp, "—");
        lv_label_set_text(s_lbl_m_vavg, "—");
        lv_label_set_text(s_lbl_m_rise, "—");
    }
    if(s_cur_on && s_real_sc > 0.f) {
        const float dts = fabsf(s_cur_x[1] - s_cur_x[0]) * (float)s_zoom_span
                        / ((wave_source_get() && wave_source_get()->sample_rate > 0)
                           ? (float)wave_source_get()->sample_rate : 65000000.0f);
        const float dvm = fabsf(s_cur_y[1] - s_cur_y[0]) * (1.0f / s_real_sc) * V32_ADC_MV_PER_LSB;
        if(dts < 1e-3f) lv_label_set_text_fmt(s_lbl_cur_dt, "%.1f µs", (double)(dts*1e6f));
        else            lv_label_set_text_fmt(s_lbl_cur_dt, "%.2f ms", (double)(dts*1e3f));
        lv_label_set_text_fmt(s_lbl_cur_f, "%.2f kHz", (double)(1.0f / (dts + 1e-12f) / 1000.0f));
        lv_label_set_text_fmt(s_lbl_cur_dv, "%.1f mV", (double)dvm);
    } else {
        lv_label_set_text(s_lbl_cur_dt, "—");
        lv_label_set_text(s_lbl_cur_f,  "—");
        lv_label_set_text(s_lbl_cur_dv, "—");
    }

    /* 自动测量：单位换算统一在这里，面板只管排版 */
    {
        const bool mv = (S.use_real && s_real_valid && s_m1.vpp > 0.5f);
        char tbuf[24];
        const float k = V32_ADC_MV_PER_LSB / 1000.0f;   /* 码 -> 伏 */
        if (!mv) {
            lv_label_set_text(s_lbl_m_vmax, "—");
            lv_label_set_text(s_lbl_m_vmin, "—");
            lv_label_set_text(s_lbl_m_vrms, "—");
            lv_label_set_text(s_lbl_m_per,  "—");
            lv_label_set_text(s_lbl_m_duty, "—");
        } else {
            lv_label_set_text_fmt(s_lbl_m_vmax, "%.3f V", (double)(s_m1.vmax * k));
            lv_label_set_text_fmt(s_lbl_m_vmin, "%.3f V", (double)(s_m1.vmin * k));
            lv_label_set_text_fmt(s_lbl_m_vrms, "%.3f V", (double)(s_m1.vrms * k));
            if (s_m1.ok) {
                const double T = (double)s_m1.period_s;
                if (T < 1e-3) snprintf(tbuf, sizeof tbuf, "%.2f µs", T * 1e6);
                else          snprintf(tbuf, sizeof tbuf, "%.2f ms", T * 1e3);
                lv_label_set_text(s_lbl_m_per, tbuf);
                lv_label_set_text_fmt(s_lbl_m_duty, "%.1f %%", (double)(s_m1.duty * 100.0f));
            } else {
                lv_label_set_text(s_lbl_m_per,  "—");
                lv_label_set_text(s_lbl_m_duty, "—");
            }
        }
    }

    /* 相位差：只有锁相可用时才显示，否则「—」 */
    if(real) {
        if(meas_ok)
            lv_label_set_text_fmt(s_lbl_phase, "%.2f°", (double)(mm.pdiff_millideg / 1000.0));
        else
            lv_label_set_text(s_lbl_phase, "—°");
    } else if(use_real) {
        lv_label_set_text(s_lbl_phase, "—°");
    } else {
        lv_label_set_text_fmt(s_lbl_phase, "%.1f°", (double)S.phase);
    }

    /*
     * 真实帧 4096 点 @65 MSPS = 63.015 µs 满屏，10 格 -> 6.302 µs/div。
     * 每帧都跟一遍：纵向是自动量程，Vpp 变了 mV/div 也会变，
     * 只在第一帧写一次的话读数和屏幕上的波形迟早对不上。
     */
    if(use_real) update_axis_label();
}

static void fft_cursor_tick(void)
{
    if(!s_fft_cur_on || !s_lbl_fft_cur) return;

    const wave_source_t *src = wave_source_get();
    const float fs = (src && src->sample_rate > 0) ? (float)src->sample_rate : 65000000.0f;
    const float f  = s_fft_cur_x * fs * 0.5f;

    float db = -200.0f;
    if(s_real_bins > 2) {
        int b = (int)(s_fft_cur_x * (float)(s_real_bins - 1) + 0.5f);
        if(b < 1) b = 1;
        if(b > s_real_bins - 1) b = s_real_bins - 1;
        db = 20.0f * log10f((s_real_mag[b] + 1e-9f) / REAL_FULLSCALE_COUNTS);
    }
    lv_label_set_text_fmt(s_lbl_fft_cur, "%.3f MHz · %.1f dBFS",
                          (double)f / 1e6, (double)db);
    plot_set_trigger(s_plot_fft, true, s_fft_cur_x);
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
        /*
         * 画的是 FPGA 的谱还是本地算的谱，读数必须跟着走 ——
         * 混用会出现"曲线峰在这个频率、读数写另一个频率"的自相矛盾。
         */
        const float pf  = s_spec_in_use ? s_spec_peak_hz : s_real_peak_hz;
        const float pdb = s_spec_in_use
                        ? (py * V32_DEMO_DB_FULLSCALE - V32_DEMO_DB_FULLSCALE)
                        : s_real_peak_dbfs;
        snprintf(peak_txt, sizeof(peak_txt), "▼ %.3f MHz %s", (double)pf / 1e6,
                 s_spec_in_use ? "FPGA" : (s_real_fft_ch ? "CH2" : "CH1"));
        lv_label_set_text_fmt(s_lbl_fft_peakf, "%.3f MHz", (double)pf / 1e6);
        lv_label_set_text_fmt(s_lbl_fft_peakl, "%.1f dBFS", (double)pdb);
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

    /*
     * 窗函数读数：画的是 FPGA 的谱就报 FPGA 实际用的窗（协议里窗函数不可远程改，
     * 只有 Hann），画本地谱才报本地选的那个 —— 两者不能混。
     */
    if(s_spec_in_use) {
        char wb[24];
        snprintf(wb, sizeof wb, "%s·FPGA", WIN_NAME[(s_spec_win >= 0 && s_spec_win <= 2) ? s_spec_win : 0]);
        lv_label_set_text(s_lbl_fft_win, wb);
    } else {
        lv_label_set_text(s_lbl_fft_win, WIN_NAME[S.win_idx]);
    }

    /* 标题写明这条谱是谁算的、多少点 */
    if(S.use_real) {
        char tt[40];
        if(s_spec_in_use) snprintf(tt, sizeof tt, "FFT 频谱 · FPGA %d 点", s_spec_nfft);
        else              snprintf(tt, sizeof tt, "FFT 频谱 · 本地 %d 点", s_real_n);
        lv_label_set_text(s_lbl_fft_title, tt);
    } else {
        lv_label_set_text(s_lbl_fft_title, "FFT 频谱 · 8192 点");
    }

    fft_cursor_tick();      /* 手指点选的频率/幅度读数 */
}

static void tick_lock(void)
{
    static float a[PLOT_MAX_PTS], b[PLOT_MAX_PTS];
    const int n = PLOT_MAX_PTS;

    real_update();

    if(S.use_real) {
        if(s_real_valid && s_full_n >= 2) {
            map_window(s_full0, s_full_n, a, n, false);
            map_window(s_full1, s_full_n, b, n, false);
            plot_set_trace(s_plot_lock, 0, a, n, V32_C_CH1, 2);
            plot_set_trace(s_plot_lock, 1, b, n, V32_C_CH2, 2);

            /*
             * 锁定页的读数**全部来自片上锁相**（V32MEAS!）。
             * usable 不成立（失锁 / 未稳定 / 超时 / 没有匹配波形帧）时一律
             * 显示「—」，并用灰色，不能拿上一帧的数凑 —— 那是最容易骗人的地方。
             */
            v32_meas_view_t mm;
            const bool ok = (v32_get_meas(&mm) != 0) && mm.usable;

            if(ok) {
                lv_label_set_text_fmt(s_lbl_lock_ref,   "%.4f MHz",
                                      (double)mm.tracked_hz_x1000 / 1e9);
                lv_label_set_text_fmt(s_lbl_lock_amp,   "%.1f mV",
                                      (double)mm.amp1_millicounts / 1000.0 * V32_ADC_MV_PER_LSB);
                lv_label_set_text_fmt(s_lbl_lock_phase, "%.2f°",
                                      (double)(mm.pdiff_millideg / 1000.0));
                lv_obj_set_style_text_color(s_lbl_lock_ref,   V32_C_METRIC_VALUE, LV_PART_MAIN);
                lv_obj_set_style_text_color(s_lbl_lock_amp,   V32_C_METRIC_VALUE, LV_PART_MAIN);
                lv_obj_set_style_text_color(s_lbl_lock_phase, V32_C_GREEN,        LV_PART_MAIN);
                plot_set_iq(s_plot_iq, (float)(mm.pdiff_millideg / 1000.0));
                lv_label_set_text(s_lbl_lock_state, "● 锁定");
                lv_obj_set_style_text_color(s_lbl_lock_state, V32_C_GREEN, LV_PART_MAIN);
            } else {
                lv_label_set_text(s_lbl_lock_ref,   "—");
                lv_label_set_text(s_lbl_lock_amp,   "—");
                lv_label_set_text(s_lbl_lock_phase, "—");
                lv_obj_set_style_text_color(s_lbl_lock_ref,   V32_C_WARN, LV_PART_MAIN);
                lv_obj_set_style_text_color(s_lbl_lock_amp,   V32_C_WARN, LV_PART_MAIN);
                lv_obj_set_style_text_color(s_lbl_lock_phase, V32_C_WARN, LV_PART_MAIN);
                plot_set_iq(s_plot_iq, 0.0f);
                /* 把原因也说出来：失锁 / 未稳定 / 没数据 */
                const char *why = "○ 未锁定";
                if(v32_get_meas(&mm) == 0)          why = "○ 无锁相数据";
                else if(mm.status & V32_ST_SETTLING) why = "○ 收敛中";
                else if(mm.status & V32_ST_ADC_OTR)  why = "○ ADC 超量程";
                else if(mm.status & V32_ST_CAL_CLIP) why = "○ 校准削顶";
                else if(mm.qual != 3)               why = "○ 单通道";
                lv_label_set_text(s_lbl_lock_state, why);
                lv_obj_set_style_text_color(s_lbl_lock_state, V32_C_WARN, LV_PART_MAIN);
            }
        } else {
            for(int i = 0; i < n; i++) { a[i] = 0.5f; b[i] = 0.5f; }
            plot_set_trace(s_plot_lock, 0, a, n, V32_C_CH1, 2);
            plot_set_trace(s_plot_lock, 1, b, n, V32_C_CH2, 2);
            plot_set_iq(s_plot_iq, 0.0f);
            lv_label_set_text(s_lbl_lock_ref,   "—");
            lv_label_set_text(s_lbl_lock_amp,   "—");
            lv_label_set_text(s_lbl_lock_phase, "—");
            lv_label_set_text(s_lbl_lock_state, "○ 无数据");
            lv_obj_set_style_text_color(s_lbl_lock_state, V32_C_WARN, LV_PART_MAIN);
        }
        return;
    }

    demo_lock_a(&S.sig, a, n);
    demo_lock_b(&S.sig, b, n);
    plot_set_trace(s_plot_lock, 0, a, n, V32_C_CH2, 2);
    plot_set_trace(s_plot_lock, 1, b, n, V32_C_CH1, 2);
    plot_set_iq(s_plot_iq, S.phase);

    lv_label_set_text_fmt(s_lbl_lock_ref,   "%.3f MHz", (double)FREQ_MHZ[S.freq_idx]);
    lv_label_set_text_fmt(s_lbl_lock_amp,   "%.1f mV",  (double)demo_vpp_mv(a, n, V32_GRID_Y));
    lv_label_set_text_fmt(s_lbl_lock_phase, "%.1f°",    (double)S.phase);
    lv_label_set_text(s_lbl_lock_state, "● 锁定");
    lv_obj_set_style_text_color(s_lbl_lock_state, V32_C_GREEN, LV_PART_MAIN);
}

/*
 * 「重建」页：原始采样点 vs 带限重建，同一段数据、同一个时间窗，上下对齐。
 *
 * 这一页原来叫「AI 降噪」，两条曲线都是 demo_ai() 合成的。后来接了真实数据，
 * 下图改成 5 点中值滤波 —— 上板实测发现中值在 65 MSPS 下是**有害**的：
 * 10 MHz 每周期只有 6.5 点，跳变的 1~2 个过渡样点在 5 点窗口里是少数派，
 * 被当成离群点压掉了，结果是边沿被画得**比真实更陡**（假象）。
 * 30 MHz 时每周期只有 2.17 点，中值等于把波形直接毁掉。
 *
 * 现在换成带限重建（bl_interp.c）—— 这才是「65 MSPS 采 30 MHz」的正解：
 * 30 MHz < 32.5 MHz 奈奎斯特，理论可完全恢复，实测重建误差 0.025 counts。
 * 它不降噪、不提高采样率，只是把「采样点」还原成「连续波形」再按像素重取样。
 */
static void tick_ai(void)
{
    static float y[PLOT_MAX_PTS];
    const int n = PLOT_MAX_PTS;

    real_update();

    if(S.use_real) {
        /*
         * 重建只在一屏点数 < 2048 时才有意义（见 map_window 的分支）：
         * 整帧缩放走 min/max 分桶，上下两图必然一模一样，看着像"重建没了"。
         * 所以**这次进入重建页后，一拿到有效数据就自动框到 10 个周期**，
         * 只框一次 —— 之后用户可以自由缩放，不会被强行拉回。
         * （必须在有峰值之后做：ui_set_page 那一刻还没数据，框不了。）
         */
        if(!s_ai_autoframed && s_real_valid && s_real_peak_hz > 0.0f
           && S.ai && wf_can_reconstruct()
           && (s_zoom_span <= 0 || s_zoom_span >= PLOT_MAX_PTS * 2)) {
            S.auto_fit = true;
            s_ai_autoframed = true;
        }

        if(s_real_valid && s_full_n >= 2) {
            /* 上：两个通道的原始点直接连直线 —— 故意不加重建，
             * 让你看清「一周期只有 2 个点」长什么样。 */
            map_window(s_full0, s_full_n, y, n, false);
            plot_set_trace(s_plot_raw, 0, y, n, V32_C_CH1, 2);
            map_window(s_full1, s_full_n, y, n, false);
            plot_set_trace(s_plot_raw, 1, y, n, V32_C_CH2, 2);

            /* 下：两个通道**各自**做带限重建。
             * **认不出是什么波形就不重建** —— 不知道是什么还画光滑曲线是造假。
             * 波形类型是按交流成分大的那一路判的，两路共用同一套判定。 */
            const bool bl_on = S.ai && wf_can_reconstruct();
            if(dbg_on())
                fprintf(stderr, "[ai] ai=%d kind=%d can=%d bl=%d full_n=%d span=%d peak_dbfs=%.1f floor_dbfs=%.1f\n",
                        (int)S.ai, s_wf_kind, (int)wf_can_reconstruct(), (int)bl_on,
                        s_full_n, s_zoom_span,
                        (double)s_real_peak_dbfs, (double)s_real_floor_dbfs);
            map_window(s_full0, s_full_n, y, n, bl_on);
            plot_set_trace(s_plot_den, 0, y, n, V32_C_CH1, 2);
            map_window(s_full1, s_full_n, y, n, bl_on);
            plot_set_trace(s_plot_den, 1, y, n, V32_C_CH2, 2);

            lv_label_set_text(s_lbl_wave, WF_NAME[s_wf_kind]);
            lv_label_set_text(s_lbl_den_title,
                              bl_on ? "重建输出 · CH1 / CH2" : "未重建 · 原始采样点");
            /* 三种"不重建"的原因分开显示，免得让人以为都是因为认不出来 */
            const char *stat;
            lv_color_t  stat_c;
            if(!S.ai) {
                stat = "旁路";               stat_c = V32_C_METRIC_VALUE;
            } else if(bl_on) {
                stat = "已启用";             stat_c = V32_C_GREEN;
            } else if(s_wf_kind == WF_SQUARE || s_wf_kind == WF_TRIANGLE
                      || s_wf_kind == WF_SAW) {
                stat = "不重建";             stat_c = V32_C_WARN;  /* 非带限，重建会振铃 */
            } else {
                stat = "已阻止";             stat_c = V32_C_WARN;  /* 认不出来 */
            }
            lv_label_set_text(s_lbl_ai_stat, stat);
            lv_obj_set_style_text_color(s_lbl_ai_stat, stat_c, LV_PART_MAIN);
        } else {
            lv_label_set_text(s_lbl_wave, "—");
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
    {
        const char *e = getenv("WAVE_FFT_CH");   /* 调试用：直接指定 FFT 通道 0/1/2 */
        S.fft_ch = e ? atoi(e) : 0;
        if(S.fft_ch < 0 || S.fft_ch > 2) S.fft_ch = 0;
    }
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
    /* HTML 原型最上面没有「V32 信号分析仪」这条标题栏，信息行（A1=…）就在最顶上。
       flex 列布局会跳过隐藏成员，把这块高度让给下面的绘图区。 */
    if(root && lv_obj_get_child_count(root) > 0)
        lv_obj_add_flag(lv_obj_get_child(root, 0), LV_OBJ_FLAG_HIDDEN);

    /* 五个页面叠在同一位置，靠 HIDDEN 切换 */
    for(int i = 0; i < UI_PAGE_COUNT; i++) {
        s_page[i] = mk(root, MAIN_X0, MAIN_Y0, MAIN_W, MAIN_H);
        lv_obj_add_flag(s_page[i], LV_OBJ_FLAG_HIDDEN);
    }
    if(getenv("WAVE_TDBG")) fprintf(stderr, "[build] scope\n");
    build_scope (s_page[UI_PAGE_SCOPE]);
    if(getenv("WAVE_TDBG")) fprintf(stderr, "[build] fft\n");
    build_fft   (s_page[UI_PAGE_FFT]);
    if(getenv("WAVE_TDBG")) fprintf(stderr, "[build] lock\n");
    build_lock  (s_page[UI_PAGE_LOCK]);
    if(getenv("WAVE_TDBG")) fprintf(stderr, "[build] ai\n");
    build_ai    (s_page[UI_PAGE_AI]);
    if(getenv("WAVE_TDBG")) fprintf(stderr, "[build] system\n");
    build_system(s_page[UI_PAGE_SYSTEM]);

    if(getenv("WAVE_TDBG")) fprintf(stderr, "[build] controls\n");
    build_controls(root);
    if(getenv("WAVE_TDBG")) fprintf(stderr, "[build] nav\n");
    build_nav(root);
    if(getenv("WAVE_TDBG")) fprintf(stderr, "[build] done\n");
    /* 右下角那行原型注记已按需求删除（见 build_fsnote 的注释） */

    ui_set_page(UI_PAGE_SCOPE);

    /* 接了真实源就立刻按真实帧长把时基写对，
     * 否则没有数据时左下角会一直挂着演示时代的 "2 µs/div"，和实际帧对不上。 */
    if(S.use_real) update_axis_label();

    /* 右上角「重建」牌：只要当前画的是加工过的波形就亮，让人一眼看出 */
    if(s_lbl_recon_badge) {
        const bool bl_on = S.use_real && s_real_valid && S.ai && wf_can_reconstruct();
        lv_obj_set_style_opa(s_lbl_recon_badge,
                             bl_on ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
    }
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

    if(page == UI_PAGE_AI) s_ai_autoframed = false;   /* 进页先允许一次自动框窗 */

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

    if(getenv("WAVE_TDBG")) { fprintf(stderr, "[tick] page=%d\n", (int)S.page); fflush(stderr); }
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
