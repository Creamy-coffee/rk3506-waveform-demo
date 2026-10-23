/*
 * theme.h —— V32 SIGNAL ANALYZER 界面的配色 / 尺寸常量
 *
 * 尺寸全部是从参考稿直接抄下来的，不是"看着差不多"调的：
 *   参考 HTML : v32_linux_touch_ui-1.html   (1024×600 原型)
 *   导出 CSS  : out/ref_style.css（由 tools/grab_ref_css.py 生成，方便逐条核对）
 *
 * 注意：**尺寸沿用参考稿，配色已经不是了**。
 * 按需求从参考稿的深色底改成了白色主题，配色那一段整体重做过，
 * 只在「配色」小节里写了改法和三个必须注意的点（语义色压暗 / 文字反色 / 网格反明暗）。
 * 想改回深色只需要替换那一段，其余代码只引用 V32_C_* 宏，没有硬编码颜色。
 *
 * 抄尺寸时只做两类改动，都写在注释里：
 *   1. CSS 的最小字号是 10~11px，我们只有 12px 字体，统一提到 12px；
 *      .metric b 的 18px 用 16px 字体代替。lv_font_conv 生成 12/16/20 三档。
 *   2. CSS 里 1px 的 border 直接用 LVGL 的 border_width=1。
 *
 * 颜色宏一律带 V32_ 前缀，避免和 LVGL 自己的宏撞名。
 */
#ifndef V32_THEME_H
#define V32_THEME_H

#include "lvgl.h"

/* ==================== 屏幕与栅格 ==================== */
/*
 * .shell{ width:min(1024px,100vw); aspect-ratio:1024/600;
 *         grid-template-rows:48px minmax(0,1fr) 92px 56px;
 *         border:1px solid #344456; background:#0c1420 }
 *
 * ⚠️ 那个 1px 的 .shell 边框必须算进栅格，这是从渲染图上反解出来的：
 *    参考稿 y=0 是 #344456（外框），y=1..48 才是 .top。忽略它会导致
 *    下面每一行都整体偏 1px —— 单看每块都"差一点点"，看多了才发现是
 *    同一个系统性偏移。所以这里先把内区算出来，后面所有行高都以内区为基准。
 *
 *     内区 = 1024-2 × 600-2 = 1022 × 598
 *     1fr（.main 行高） = 598 - 48 - 92 - 56 = 402     （不是 404）
 *     .main 内容高       = 402 - padding(9+8) = 385      （不是 387）
 */
#define V32_SCR_W            1024
#define V32_SCR_H            600
/* .shell 的 1px 外框：位置与内区尺寸 */
#define V32_SHELL_BW         1
#define V32_SHELL_X          V32_SHELL_BW                                   /* 1 */
#define V32_SHELL_Y          V32_SHELL_BW                                   /* 1 */
#define V32_SHELL_W          (V32_SCR_W - 2 * V32_SHELL_BW)                 /* 1022 */
#define V32_SHELL_H          (V32_SCR_H - 2 * V32_SHELL_BW)                 /* 598 */

#define V32_ROW_TOP          48
#define V32_ROW_CTRL         92
#define V32_ROW_NAV          56
#define V32_ROW_MAIN         (V32_SHELL_H - V32_ROW_TOP - V32_ROW_CTRL - V32_ROW_NAV)  /* 402 */

/* .main{ padding:9px 12px 8px; gap:7px } */
#define V32_MAIN_PAD_T       9
#define V32_MAIN_PAD_X       12
#define V32_MAIN_PAD_B       8
#define V32_MAIN_GAP         7
/* .page{ gap:9px }  .twocol/.stack{ gap:9px }  .section{ gap:5px } */
#define V32_PAGE_GAP         9
#define V32_SECTION_GAP      5
/* .right{ width:235px; gap:8px } */
#define V32_RIGHT_W          235
#define V32_RIGHT_GAP        8

/* .panel{ border-radius:8px; padding:10px; border:1px }
 * .plot{ border-radius:7px; border:1px } */
#define V32_PANEL_PAD        10
#define V32_PANEL_BW         1                  /* .panel / .plot 的 1px 边框 */
#define V32_PANEL_RADIUS     8
#define V32_PLOT_RADIUS      7

/* .metric{ padding:7px 0 }  .kv{ padding:6px 0 } */
#define V32_METRIC_PAD_Y     7
#define V32_KV_PAD_Y         6
/* .panel-title{ margin-bottom:8px } */
#define V32_PANEL_TITLE_MB   8

/* .chip{ padding:10px 12px; min-height:37px; border-radius:6px }
 * select{ padding:8px; max-width:140px }
 * ⚠️ 高度取 43 而不是 CSS 的 37：min-height:37px 在参考稿里根本没生效 ——
 *    .chip 的字号是 16px（继承 body），行高 21 撑出 21+20+2=43 > 37。
 *    （反解 ref_scope.png：右栏 CH1 ON 的框是 90×43） */
#define V32_CHIP_PAD_X       7
#define V32_CHIP_PAD_Y       10
#define V32_CHIP_H           34
#define V32_CHIP_RADIUS      6
#define V32_SEL_MAX_W        140

/* .controls{ padding:7px 12px; gap:8px }   .group{ gap:7px } */
#define V32_CTRL_PAD_X       12
#define V32_CTRL_PAD_Y       7
#define V32_CTRL_GAP         8
#define V32_GROUP_GAP        7
#define V32_SLIDER_W         96
/* .nav{ grid-template-columns:repeat(5,1fr); gap:5px; padding:6px 9px } */
#define V32_NAV_COLS         5
#define V32_NAV_GAP          5
#define V32_NAV_PAD_X        9
#define V32_NAV_PAD_Y        6
/* .nav button.active{ border-bottom:2px solid } */
#define V32_NAV_ACTIVE_BW    2

/* .plot-title{ left:12px; top:8px }   .plot-foot{ bottom:7px; left:12px } */
#define V32_PLOT_TITLE_X     12
#define V32_PLOT_TITLE_Y     8
#define V32_PLOT_FOOT_X      12
#define V32_PLOT_FOOT_Y      7

/* 画布网格：grid(ctx,w,h,divx=10,divy=8)；FFT 用 13×8；IQ 用 8×8；AI 用 10×4 */
#define V32_GRID_X           10
#define V32_GRID_Y           8
#define V32_GRID_FFT_X       13
#define V32_GRID_IQ_X        8
#define V32_GRID_AI_Y        4
/* trace() 的采样步长：for(x=0;x<=w;x+=1.5) */
#define V32_TRACE_STEP_X10   15   /* ×10，即 1.5px */

/* ==================== 配色 ==================== */
/*
 * 白色主题（按需求从参考稿的深色底改成浅色/白底）。
 *
 * 换的时候不是"把每个值调亮"就完事，有三类必须跟着改，否则白底上会糊掉：
 *   1. **语义色要压暗**：参考稿的 CH1 亮黄 #F4CE5B、CH2 亮青 #5BDBE9 在深底上
 *      好看，铺到白底上对比度只剩 1.2:1，波形几乎看不见。这里换成同色系的
 *      深色版（琥珀 #C88200 / 深青 #0C7E96），色相不变、明度降下来。
 *   2. **所有"近白"的文字色都要变成"深灰蓝"**：原 #D9E5F1 之类在白底上
 *      等于隐形。文字统一走 #12..#6B 的深灰蓝阶，层级靠明度差拉开。
 *   3. **网格线不能再比底色"亮"**：深色主题里网格是加亮（#23364B 比底 #0A1522 亮），
 *      浅色主题里必须是压暗（#E4EBF3 比底 #FBFDFF 暗），方向反了会变成一团白线。
 *
 * 想改回深色：把这一整块换回去即可，其余代码只引用宏、没有硬编码颜色。
 */
/* 底色 / 外壳 */
#define V32_C_BG             lv_color_hex(0x050805)  /* :root background */
#define V32_C_SHELL          lv_color_hex(0x0A0F0A)  /* .shell background */
#define V32_C_SHELL_BORDER   lv_color_hex(0x1E301C)

/* 顶栏 */
#define V32_C_TOPBAR         lv_color_hex(0x0A0F0A)  /* .top */
#define V32_C_TOPBAR_BORDER  lv_color_hex(0x1A2A18)  /* .top border-bottom */
#define V32_C_BRAND          lv_color_hex(0x3CFF6E)  /* .brand（品牌青，白底压深） */
#define V32_C_BRAND_TEXT     lv_color_hex(0x9FE8B4)  /* .brand span */
#define V32_C_BRAND_SMALL    lv_color_hex(0x4E7A5A)  /* .brand small */
#define V32_C_STATUS         lv_color_hex(0x6FAE84)  /* .statuses */
#define V32_C_LIVE           lv_color_hex(0x35CC45)  /* .live */
#define V32_C_BADGE_BG       lv_color_hex(0x1A2A12)  /* .mode-badge */
#define V32_C_BADGE_FG       lv_color_hex(0xFFB340)

/* 面板 / 绘图区 */
#define V32_C_PANEL          lv_color_hex(0x0A0F0A)  /* .panel background */
#define V32_C_PANEL_BORDER   lv_color_hex(0x1A2A18)
#define V32_C_PLOT           lv_color_hex(0x050805)  /* .plot background */
#define V32_C_PLOT_BORDER    lv_color_hex(0x1F7A2A)
#define V32_C_PLOT_TITLE     lv_color_hex(0x35CC45)  /* .plot-title */
#define V32_C_PLOT_FOOT      lv_color_hex(0x2E8A44)  /* .plot-foot */
#define V32_C_PANEL_TITLE    lv_color_hex(0x35CC45)  /* .panel-title */

/* 指标行 .metric */
#define V32_C_METRIC_LABEL   lv_color_hex(0x35CC45)
#define V32_C_METRIC_VALUE   lv_color_hex(0xE8F2E8)
#define V32_C_METRIC_BORDER  lv_color_hex(0x14210F)

/* 语义色（.yellow/.cyan/.green）—— 深色主题的同色系压暗版，白底可读 */
#define V32_C_CH1            lv_color_hex(0xCFE6F5)  /* .yellow   CH1 / 矢量 */
#define V32_C_CH2            lv_color_hex(0xFFD24A)  /* .cyan     CH2 */
#define V32_C_GREEN          lv_color_hex(0x35CC45)  /* .green    正常/锁定 */
#define V32_C_FFT            lv_color_hex(0x35CC45)  /* FFT 曲线 */
#define V32_C_AI_OUT         lv_color_hex(0x7CFF9A)  /* AI 降噪输出曲线 */

/* 提示文字 */
#define V32_C_TEXT           lv_color_hex(0xCFE0CF)  /* :root color */
#define V32_C_FSNOTE         lv_color_hex(0x4E7A5A)  /* .fullscreen-note */
#define V32_C_NOTE           lv_color_hex(0x6FAE84)  /* .note */
#define V32_C_HINT           lv_color_hex(0x35CC45)  /* .touch-hint */
#define V32_C_MINI           lv_color_hex(0x4E7A5A)  /* .mini */
#define V32_C_LABEL          lv_color_hex(0x35CC45)  /* .group label */
#define V32_C_KV_KEY         lv_color_hex(0x2E8A44)  /* .kv span */
#define V32_C_KV_BORDER      lv_color_hex(0x14210F)
#define V32_C_SUBHEAD        lv_color_hex(0x4E7A5A)  /* .subhead */
#define V32_C_WARN           lv_color_hex(0xFFB340)  /* 模拟件的黄色警示 */

/* 控件条 .controls */
#define V32_C_CTRL_BG        lv_color_hex(0x0A0F0A)
#define V32_C_CTRL_BORDER    lv_color_hex(0x1A2A18)
#define V32_C_SEL_BG         lv_color_hex(0x0A0F0A)  /* select */
#define V32_C_SEL_FG         lv_color_hex(0xCFE0CF)
#define V32_C_SEL_BORDER     lv_color_hex(0x1E301C)
#define V32_C_SEL_BG2        lv_color_hex(0x0A0F0A)  /* dropdown 弹出列表 */

/* 按钮 .chip / .nav button */
#define V32_C_CHIP_BG        lv_color_hex(0x0A0F0A)
#define V32_C_CHIP_BORDER    lv_color_hex(0x22371F)
#define V32_C_CHIP_FG        lv_color_hex(0x6FAE84)
#define V32_C_CHIP_BG_PRESS  lv_color_hex(0x14210F)  /* .chip:active（白底上是压暗） */
#define V32_C_CHIP_ON_BG     lv_color_hex(0x13351B)  /* .chip.selected */
#define V32_C_CHIP_ON_BORDER lv_color_hex(0x35CC45)
#define V32_C_CHIP_ON_FG     lv_color_hex(0x9FE8B4)

/* 底部导航 .nav */
#define V32_C_NAV_BG         lv_color_hex(0x0A0F0A)
#define V32_C_NAV_BORDER     lv_color_hex(0x1A2A18)
#define V32_C_NAV_FG         lv_color_hex(0x6FAE84)
#define V32_C_NAV_ON_BG      lv_color_hex(0x0D3A1A)  /* .nav button.active */
#define V32_C_NAV_ON_FG      lv_color_hex(0x3CFF6E)
#define V32_C_NAV_ON_BAR     lv_color_hex(0x0D3A1A)

/* 画布内部（**黑底**：网格必须比底色**亮**，否则看不见） */
#define V32_C_GRID           lv_color_hex(0x0B3D14)  /* grid() 网格 */
#define V32_C_GRID_CENTER    lv_color_hex(0x1F7A2A)  /* 中心虚线 */
#define V32_C_TRIGGER        lv_color_hex(0xFF45FF)  /* 触发线 + "T" */
#define V32_C_IQ_RING        lv_color_hex(0x22371F)  /* 矢量图同心圆 */

/* ==================== 字体 ==================== */
/* 由 tools/gen_v32_fonts.py 从参考稿 HTML + 本工程源码的字符串字面量里扒字符集
 * 后生成的三档子集。单字体覆盖不全，所以是多合一：
 *   DejaVuSans -> ASCII + °µ·×Δ–—● 和大部分装饰符号
 *   seguisym   -> ⏸(暂停) ⛶(全屏) ⊞(自动) 这些 DejaVu 没有的
 *   simhei     -> 汉字 + 、。，；
 * 第六版界面改中文后一共 322 个字符（汉字 208 个），全部收录，无缺字。
 * ⚠️ 往界面加文案必须重跑那个脚本，否则新字是空白方块。
 *
 * 字号按参考稿 CSS 取，不要随手改：
 *   12px -> .statuses / .metric 标签 / .panel-title / .nav button
 *   16px -> .chip / select / .action  ← 见下面 V32_FONT_CHIP 的说明
 *   18px -> .metric b（右栏读数大字）是 18px，界面上最显眼的一档
 *   20px -> .brand 是 19px，就近取 20
 * （参考稿还有 10/11/13px 三档，2px 以内看不出来，统一就近取档） */
LV_FONT_DECLARE(lv_font_v32_12);
LV_FONT_DECLARE(lv_font_v32_16);
LV_FONT_DECLARE(lv_font_v32_18);
LV_FONT_DECLARE(lv_font_v32_20);
LV_FONT_DECLARE(lv_font_v32_12_bold);
LV_FONT_DECLARE(lv_font_v32_18_bold);
LV_FONT_DECLARE(lv_font_v32_20_bold);

/*
 * 录屏要求：字要大、颜色要深，否则视频里看不清。
 * 原来这一档是 12px（代替 CSS 的 10~12px），录出来太小，整体上调到 16px。
 * 布局上 12→16 只涨 33%，各处都有余量；上板逐页看过没有溢出。
 */
#define V32_FONT_SMALL   (&lv_font_v32_16)   /* 原 12px，录屏看不清，上调一档 */
#define V32_FONT_BASE    (&lv_font_v32_18)   /* 代替 CSS 的 18px（.metric b） */
#define V32_FONT_BIG     (&lv_font_v32_20)   /* 代替 CSS 的 19px（.brand） */

/*
 * .chip / select / .action 的字体 —— **必须是 16px，不能用 12px**。
 *
 * 参考稿这条规则很容易看漏：
 *     .chip,.nav button{ border:...; padding:10px 12px; min-height:37px }   ← 没写 font-size
 *     .nav button{ font-size:12px; font-weight:750 }                        ← 只覆盖了导航
 * 所以 .chip 是继承 <body> 的**浏览器默认 16px**，只有 .nav button 是 12px。
 * 整屏按 12px 做的时候，右栏 "CH1 ON" 量出来是 73×37，而参考稿是 90×43 ——
 * 43 = 16px 字体的行高 21 + 上下 padding 20 + 边框 2。控制条上的
 * ⏸暂停/↺复位/⛶全屏 同理，整体窄了一圈。
 * 我们这档字体的 line_height 是 19，所以高度靠 V32_CHIP_H 显式钉到 43。
 */
#define V32_FONT_CHIP    (&lv_font_v32_12)   /* 照 HTML 13px，我们只有 12 档 */

/*
 * 粗体。参考稿不是"到处都是常规体"：
 *     .brand{font-weight:800}        -> "V32 SIGNAL ANALYZER"
 *     .metric b (18px)  = <b> 700    -> 右栏读数
 *     .kv b      (13px) = <b> 700    -> 系统页键值
 *     .nav button{font-weight:750}   -> 底部导航文字
 *     "ADC <b>65 MSPS</b>"           -> 顶栏只加粗后半段
 * 这几处如果用常规体渲染，整个界面会明显偏"轻"，和参考稿观感差很远。
 * 第六版起粗体子集**也含汉字**：界面改中文后，底部导航 / 品牌标题 / 系统页键值
 * 全在粗体档，汉字进不去就是一片方块（SimHei 没有真粗体，笔画就是常规的，
 * 12px 上这点字重差看不出来）。四个粗体档因此各多几十 KB。
 */
#define V32_FONT_SMALL_B (&lv_font_v32_18_bold)  /* 原 12px 粗体，同上上调 */
#define V32_FONT_BASE_B  (&lv_font_v32_18_bold)
#define V32_FONT_BIG_B   (&lv_font_v32_20_bold)

#endif /* V32_THEME_H */
