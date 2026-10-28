/*
 * plot.c —— 自绘波形控件实现
 *
 * 绘制顺序（和参考稿的 canvas 一致）：
 *   1) 网格（竖线 divx-1 根 + 横线 divy-1 根）
 *   2) 中间那条虚线（dash 4 / gap 6）
 *   3) 曲线（一条一根一段地 lv_draw_line，跟 lv_chart 的做法一样）
 *   4) 触发线 / 峰值箭头 / 同心圆 + 矢量
 *   5) 画布内的坐标文字
 *
 * 关于虚线：LVGL 的软件渲染器只在**水平/垂直**线上实现 dash
 *   （见 lvgl/src/draw/sw/lv_draw_sw_line.c: draw_line_hor / draw_line_ver），
 *   斜线走 draw_line_skew 会忽略 dash_width。我们中间那条线正好是水平的，
 *   所以能正常显示成虚线。
 */
#include "plot.h"
#include "theme.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ==================== 内部状态 ==================== */

typedef struct {
    float      y[PLOT_MAX_PTS];
    int        n;
    lv_color_t color;
    int32_t    width;
    bool       vis;
} trace_t;

typedef struct {
    lv_obj_t   *obj;
    plot_mode_t mode;

    int  divx, divy;
    bool center_dash;
    /* 采样步长 ×10：15 表示每 1.5px 出一个点（照抄参考稿 trace() 的 x+=1.5） */
    int  px_step_x10;
    /* 网格底线样式 */
    lv_color_t grid_color;
    lv_color_t center_color;

    trace_t tr[PLOT_MAX_TRACE];

    bool  trig_vis;
    float trig_x;            /* 0..1 触发位置（水平轴） */
    bool  trig_lev_vis;
    float trig_y;            /* 0..1 触发电平（0=底 1=顶） */

    /* 顶部信息行分段（照 scope_proto：标签绿/数值白/触发洋红） */
    plot_info_seg_t info_seg[12];
    int             info_n;

    bool  peak_vis;
    float peak_x;            /* 0..1 */
    char  peak_txt[32];      /* 必须常驻：lv_draw_label 渲染时才读这段内存 */

    float iq_phase;          /* 度，数学约定：0°=+I 向右，逆时针为正 */
    bool iq_valid;

    bool  cur_vis;           /* 测量光标 */

    int   cur_n;             /* 光标数(补丁后仍被 set/draw_cursors 使用) */

    /* AM 上下包络叠加（金色虚线） */
    float env_u[PLOT_MAX_PTS], env_d[PLOT_MAX_PTS];
    int   env_n;
    bool  env_vis;
    bool  env_fill;
    bool  xy_points,xy_equal;
    float cur_x[4];          /* 竖线位置 0..1 */
    float cur_y[4];          /* 横线位置 0..1 */

    char info[80];           /* 顶部信息行 */
    int  tk_nx, tk_ny;       /* 刻度文字个数（0 = 不画刻度边距） */
    char tk_x[6][12];
    char tk_y[5][12];
    char axis_l[16];
    char axis_r[16];
    char leg0[10];
    char leg1[10];
} plot_t;

#define PLOT_OF(obj)   ((plot_t *)lv_obj_get_user_data(obj))

/* ==================== 生命周期 ==================== */

static void plot_delete_cb(lv_event_t *e)
{
    plot_t *p = lv_event_get_user_data(e);
    if(p) lv_free(p);
}

static void plot_draw_cb(lv_event_t *e);

lv_obj_t *plot_create(lv_obj_t *parent, plot_mode_t mode)
{
    plot_t *p = lv_malloc_zeroed(sizeof(plot_t));
    LV_ASSERT_MALLOC(p);
    if(p == NULL) return NULL;

    lv_obj_t *obj = lv_obj_create(parent);
    if(obj == NULL) { lv_free(p); return NULL; }

    p->obj = obj;
    p->grid_color   = V32_C_GRID;
    p->center_color = V32_C_GRID_CENTER;
    p->trig_y       = 0.5f;

    /* .plot{ background:#0a1522; border:1px solid #2a3c4c; border-radius:7px } */
    lv_obj_remove_style_all(obj);
    lv_obj_set_style_bg_color(obj, V32_C_PLOT, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    /* scope_proto：亮绿外框是**紧贴网格**的 2px 直角框，刻度文字在框外。
     * 对象边框关掉，框在 draw_cb 里绕网格区画（见 plot_draw_cb 末尾）。 */
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_clip_corner(obj, true, LV_PART_MAIN);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_user_data(obj, p);
    lv_obj_add_event_cb(obj, plot_draw_cb, LV_EVENT_DRAW_MAIN, p);
    lv_obj_add_event_cb(obj, plot_delete_cb, LV_EVENT_DELETE, p);

    plot_set_mode(obj, mode);
    return obj;
}

void plot_set_mode(lv_obj_t *plot, plot_mode_t mode)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    if(p->mode==mode && p->divx>0)return;
    p->mode = mode;

    /* 分格数照抄参考稿：
     *   scope/lock grid(10,8)   fft grid(13,8)   iq grid(8,8)   ai grid(10,4)
     * 中间虚线 grid() 里无条件绘，所以四种模式都打开。 */
    switch(mode) {
        case PLOT_FFT:
            p->divx = V32_GRID_FFT_X; p->divy = V32_GRID_Y; p->px_step_x10 = 10;
            break;
        case PLOT_IQ:
            p->divx = V32_GRID_IQ_X;  p->divy = V32_GRID_Y; p->px_step_x10 = 10;
            break;
        case PLOT_AI:
            p->divx = V32_GRID_X;     p->divy = V32_GRID_AI_Y; p->px_step_x10 = V32_TRACE_STEP_X10;
            break;
        case PLOT_SCOPE:
        case PLOT_HIST:
        default:
            p->divx = V32_GRID_X;     p->divy = V32_GRID_Y; p->px_step_x10 = V32_TRACE_STEP_X10;
            break;
    }
    p->center_dash = true;
    lv_obj_invalidate(plot);
}

void plot_set_grid(lv_obj_t *plot, int divx, int divy, bool center_dash)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    if(divx > 1) p->divx = divx;
    if(divy > 1) p->divy = divy;
    p->center_dash = center_dash;
    lv_obj_invalidate(plot);
}

/* ==================== 数据设置 ==================== */

/* 返回 1 = 数据真的变了（已拷贝），0 = 数据相同跳过（没拷） */
static int set_trace(lv_obj_t *plot, int idx, const float *y, int n,
                     lv_color_t color, int32_t width, bool vis)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL || idx < 0 || idx >= PLOT_MAX_TRACE) return 0;

    trace_t *t = &p->tr[idx];
    const int style_changed=!lv_color_eq(t->color,color) || t->width!=width || t->vis!=vis;
    t->color = color;
    t->width = width;
    t->vis   = vis;

    if(y == NULL || n <= 0) { int changed=style_changed || t->n!=0;t->n = 0; return changed; }
    if(n > PLOT_MAX_PTS) n = PLOT_MAX_PTS;
    /* 数据完全一致（暂停/锚定后的稳态帧）就不重绘 —— 高频 set_trace
       的场景下这一下 memcmp 能省掉整块 plot 的软件渲染。 */
    if(t->n == n && memcmp(t->y, y, (size_t)n * sizeof(float)) == 0) return style_changed;
    memcpy(t->y, y, (size_t)n * sizeof(float));
    t->n = n;
    return 1;
}

void plot_set_trace(lv_obj_t *plot, int idx, const float *y, int n,
                    lv_color_t color, int32_t width)
{
    /* 只有数据真的变了才 invalidate —— 上一版在拷贝**之后**比较，
       t->y 已是新数据，memcmp 恒等、永不 invalidate：没有触发线等
       附加重绘源的页面（重建页）整幅冻结，就是它。 */
    if(set_trace(plot, idx, y, n, color, width, true))
        lv_obj_invalidate(plot);
}

void plot_hide_trace(lv_obj_t *plot, int idx)
{
    if(set_trace(plot, idx, NULL, 0, V32_C_TEXT, 1, false))lv_obj_invalidate(plot);
}

void plot_set_trigger(lv_obj_t *plot, bool vis, float x)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    x=(x<0.f)?0.f:(x>1.f?1.f:x);
    if(p->trig_vis==vis && p->trig_x==x)return;
    p->trig_vis = vis;
    p->trig_x = (x < 0.f) ? 0.f : (x > 1.f ? 1.f : x);
    lv_obj_invalidate(plot);
}

void plot_set_trigger_level(lv_obj_t *plot, bool vis, float y01)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    y01=(y01<0.f)?0.f:(y01>1.f?1.f:y01);
    if(p->trig_lev_vis==vis && p->trig_y==y01)return;
    p->trig_lev_vis = vis;
    p->trig_y = (y01 < 0.f) ? 0.f : (y01 > 1.f ? 1.f : y01);
    lv_obj_invalidate(plot);
}

void plot_set_peak(lv_obj_t *plot, bool vis, float x, const char *txt)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    p->peak_vis = vis;
    p->peak_x = (x < 0.f) ? 0.f : (x > 1.f ? 1.f : x);
    if(txt) {
        strncpy(p->peak_txt, txt, sizeof(p->peak_txt) - 1);
        p->peak_txt[sizeof(p->peak_txt) - 1] = '\0';
    }
    else p->peak_txt[0] = '\0';
    lv_obj_invalidate(plot);
}

void plot_set_envelope(lv_obj_t *plot, const float *up, const float *dn,
                       int n, bool show)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    p->env_vis = show && up && dn && n >= 2;
    if(!p->env_vis) { p->env_fill=false; if(p->env_n) { p->env_n = 0; lv_obj_invalidate(plot); } return; }
    if(n > PLOT_MAX_PTS) n = PLOT_MAX_PTS;
    memcpy(p->env_u, up, (size_t)n * sizeof(float));
    memcpy(p->env_d, dn, (size_t)n * sizeof(float));
    p->env_n = n;
    lv_obj_invalidate(plot);
}

void plot_set_envelope_fill(lv_obj_t *plot, bool show)
{
    plot_t *p=PLOT_OF(plot);
    if(p && p->env_fill!=show){p->env_fill=show;lv_obj_invalidate(plot);}
}

void plot_set_xy_style(lv_obj_t *plot, bool points, bool equal_axes)
{
    plot_t *p=PLOT_OF(plot);
    if(p && (p->xy_points!=points || p->xy_equal!=equal_axes)){
        p->xy_points=points;p->xy_equal=equal_axes;lv_obj_invalidate(plot);
    }
}

void plot_set_iq(lv_obj_t *plot, float phase_deg)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    if(p->iq_valid && p->iq_phase==phase_deg)return;
    p->iq_valid=true;
    p->iq_phase = phase_deg;
    lv_obj_invalidate(plot);
}

void plot_hide_iq(lv_obj_t *plot)
{
    plot_t *p=PLOT_OF(plot);
    if(p && p->iq_valid){p->iq_valid=false;lv_obj_invalidate(plot);}
}

void plot_set_ticks(lv_obj_t *plot, const char *const *xl, int nx,
                    const char *const *yl, int ny)
{
    if(plot == NULL) return;
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    int changed = (p->tk_nx != nx || p->tk_ny != ny);
    p->tk_nx = (nx > 6) ? 6 : nx;
    p->tk_ny = (ny > 5) ? 5 : ny;
    for(int i = 0; i < p->tk_nx; i++) { const char *s=xl[i]?xl[i]:"";if(strcmp(p->tk_x[i],s))changed=1;snprintf(p->tk_x[i], sizeof p->tk_x[i], "%s",s); }
    for(int j = 0; j < p->tk_ny; j++) { const char *s=yl[j]?yl[j]:"";if(strcmp(p->tk_y[j],s))changed=1;snprintf(p->tk_y[j], sizeof p->tk_y[j], "%s",s); }
    if(changed) lv_obj_invalidate(plot);
}

void plot_set_info(lv_obj_t *plot, const char *line)
{
    if(plot == NULL) return;
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    if(line == NULL) line = "";
    if(strcmp(p->info, line) == 0) return;      /* 没变就别重绘 */
    snprintf(p->info, sizeof p->info, "%s", line);
    lv_obj_invalidate(plot);
}

void plot_set_info_segs(lv_obj_t *plot, const plot_info_seg_t *segs, int n)
{
    if(plot == NULL) return;
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    if(n < 0) n = 0;
    if(n > 12) n = 12;
    int changed = (p->info_n != n);
    for(int i = 0; i < n; i++) {
        if(strcmp(p->info_seg[i].txt, segs[i].txt) != 0) changed = 1;
        p->info_seg[i].color = segs[i].color;
        snprintf(p->info_seg[i].txt, sizeof p->info_seg[i].txt, "%s",
                 segs[i].txt ? segs[i].txt : "");
    }
    p->info_n = n;
    if(changed) lv_obj_invalidate(plot);
}

void plot_set_cursors(lv_obj_t *plot, int n, const float *xs, const float *ys, bool show)
{
    if(plot == NULL) return;
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    if(n < 0) n = 0;
    if(n > 4) n = 4;
    float nx[4]={0},ny[4]={0};
    for(int i=0;i<n;i++){if(i<2)nx[i]=xs?xs[i]:0.5f;else ny[i-2]=ys?ys[i-2]:0.5f;}
    if(p->cur_n==n && p->cur_vis==(show && n>0) && !memcmp(p->cur_x,nx,sizeof nx) && !memcmp(p->cur_y,ny,sizeof ny))return;
    p->cur_n   = n;
    p->cur_vis = show && n > 0;
    memset(p->cur_x,0,sizeof p->cur_x);memset(p->cur_y,0,sizeof p->cur_y);
    for(int i = 0; i < n; i++) {
        if(i<2)p->cur_x[i]=xs?xs[i]:0.5f;
        else p->cur_y[i-2]=ys?ys[i-2]:0.5f;
    }
    lv_obj_invalidate(plot);
}

void plot_set_axis_labels(lv_obj_t *plot, const char *left, const char *right)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    p->axis_l[0] = '\0';
    p->axis_r[0] = '\0';
    if(left)  { strncpy(p->axis_l, left,  sizeof(p->axis_l) - 1);  p->axis_l[sizeof(p->axis_l) - 1] = '\0'; }
    if(right) { strncpy(p->axis_r, right, sizeof(p->axis_r) - 1); p->axis_r[sizeof(p->axis_r) - 1] = '\0'; }
}

void plot_set_legends(lv_obj_t *plot, const char *l0, const char *l1)
{
    plot_t *p = PLOT_OF(plot);
    if(p == NULL) return;
    p->leg0[0] = '\0';
    p->leg1[0] = '\0';
    if(l0) { strncpy(p->leg0, l0, sizeof(p->leg0) - 1); p->leg0[sizeof(p->leg0) - 1] = '\0'; }
    if(l1) { strncpy(p->leg1, l1, sizeof(p->leg1) - 1); p->leg1[sizeof(p->leg1) - 1] = '\0'; }
}

/* ==================== 绘制小工具 ==================== */

/* 拉一条 1px 直线，省掉反复填 dsc 的样板代码 */
static void line(lv_layer_t *layer, lv_draw_line_dsc_t *ld,
                 int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
    ld->p1.x = x1; ld->p1.y = y1;
    ld->p2.x = x2; ld->p2.y = y2;
    lv_draw_line(layer, ld);
}

/* 在 layer 上写一行小字。text 必须是常驻内存（字符串字面量或结构体里的 char[]），
 * 因为真正的渲染发生在回调返回之后。 */
/* 指定宽度的文字：text() 固定 150px 宽，长串会被折行（信息行当初就折成了三行） */
static void text_w(lv_layer_t *layer, int32_t x, int32_t y, int32_t w,
                   const char *s, lv_color_t color, const lv_font_t *font)
{
    if(s == NULL || s[0] == '\0' || w < 8) return;
    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);
    dsc.color = color;
    dsc.font  = font;
    dsc.text  = s;
    dsc.opa   = LV_OPA_COVER;
    const int32_t lh = font->line_height;
    lv_area_t a = { x, y, x + w, y + lh };
    lv_draw_label(layer, &dsc, &a);
}

static void text(lv_layer_t *layer, int32_t x, int32_t y,
                 const char *s, lv_color_t color, const lv_font_t *font)
{
    if(s == NULL || s[0] == '\0') return;
    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);
    dsc.color = color;
    dsc.font  = font;
    dsc.text  = s;
    dsc.opa   = LV_OPA_COVER;
    const int32_t lh = font->line_height;
    lv_area_t a = { x, y, x + 150, y + lh };
    lv_draw_label(layer, &dsc, &a);
}

/*
 * 同一行字，但先垫一块和画布同色的半透明底板。
 *
 * 为什么需要：画在画布角上的坐标文字（"0 MHz" / "32.5 MHz"）会被曲线
 * 直接穿过去。深色主题时不太明显，换成白底之后谱线是深绿的，从深灰文字上
 * 横穿过去会把字切成几截 —— 实测那两处基本读不出来。
 * 底板只做视觉隔离：位置、颜色、字号都不变，曲线在别处照常可见。
 */
static void text_plated(lv_layer_t *layer, int32_t x, int32_t y,
                        const char *s, lv_color_t color, const lv_font_t *font)
{
    if(s == NULL || s[0] == '\0') return;

    lv_point_t sz;
    lv_text_get_size(&sz, s, font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);

    lv_draw_rect_dsc_t rd;
    lv_draw_rect_dsc_init(&rd);
    rd.bg_color = V32_C_PLOT;       /* 和 .plot 底色一致，看起来像"挖空" */
    rd.bg_opa   = LV_OPA_COVER;
    rd.radius   = 3;
    lv_area_t a = { x - 4, y - 1, x + sz.x + 4, y + font->line_height };
    lv_draw_rect(layer, &rd, &a);

    text(layer, x, y, s, color, font);
}

static void draw_grid(lv_layer_t *layer, plot_t *p,
                      int32_t x0, int32_t y0, int32_t w, int32_t h)
{
    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = p->grid_color;
    ld.width = 1;
    ld.opa = LV_OPA_COVER;

    /*
     * 细网格：每大格再分 5 小格（照 HTML 原型的点阵感）。
     * 先画细的（透明度低），再画大格，层次才对 —— 反过来细线会盖住粗线。
     */
    ld.opa = (lv_opa_t)(LV_OPA_COVER / 3);
    int k;
    for(k = 1; k < p->divx * 5; k++) {
        if(k % 5 == 0) continue;                  /* 大格位置留空 */
        int32_t x = x0 + (int32_t)((int64_t)w * k / (p->divx * 5));
        line(layer, &ld, x, y0, x, y0 + h - 1);
    }
    for(k = 1; k < p->divy * 5; k++) {
        if(k % 5 == 0) continue;
        int32_t y = y0 + (int32_t)((int64_t)h * k / (p->divy * 5));
        line(layer, &ld, x0, y, x0 + w - 1, y);
    }
    ld.opa = LV_OPA_COVER;

    int i;
    for(i = 1; i < p->divx; i++) {
        int32_t x = x0 + (int32_t)((int64_t)w * i / p->divx);
        line(layer, &ld, x, y0, x, y0 + h - 1);
    }
    for(i = 1; i < p->divy; i++) {
        int32_t y = y0 + (int32_t)((int64_t)h * i / p->divy);
        line(layer, &ld, x0, y, x0 + w - 1, y);
    }

    /* 中心十字的短刻度（照 HTML：沿中心线每 8px 一个小刻） */
    ld.color = p->center_color;
    const int32_t yc = y0 + h / 2;
    const int32_t xc = x0 + w / 2;
    for(int32_t x = x0 + 6; x < x0 + w - 6; x += 8)
        line(layer, &ld, x, yc - 3, x, yc + 3);
    for(int32_t y = y0 + 6; y < y0 + h - 6; y += 8)
        line(layer, &ld, xc - 3, y, xc + 3, y);
}

static void draw_one_trace(lv_layer_t *layer, plot_t *p, int ti,
                           int32_t x0, int32_t y0, int32_t w, int32_t h)
{
    trace_t *t = &p->tr[ti];
    if(!t->vis || t->n < 2 || w < 4 || h < 4) return;

    /*
     * 一条曲线最多画多少个点。
     *
     * 参考稿是 canvas 上 x += 1.5 的步长，本来就画不满每个像素列；
     * 但这个"稀疏化"在我们这儿会毁掉示波器的包络：
     *   数据源已经按 min/max 抽成 1024 点（每 2 个点 = 一个像素桶的峰和谷），
     *   再按 1.5px 的步长**跨点取样**（stride=3 时会跳过成对的峰/谷），
     *   4106 样本 @10 MHz 挤进 740 px 的结果就不是"密集成带"，
     *   而是一串假的低频脉冲 —— 实测 10 MHz 被画成约 35 个周期（≈0.55 MHz）。
     * 所以这里给到 2 点/像素：只要 t->n <= 2w 就 stride=1 全画，
     * 每个像素列都能拿到该列的峰和谷，高频信号显示成实心带（示波器就是这样）。
     * 只有 t->n 远大于像素数（>2w）时才退回跨点取样。
     */
    int max_seg = w * 2;
    if(max_seg < 2) max_seg = 2;
    int stride = (t->n + max_seg - 1) / max_seg;
    if(stride < 1) stride = 1;

    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = t->color;
    ld.width = t->width;
    ld.opa   = LV_OPA_COVER;
    ld.round_start = 1;
    ld.round_end   = 1;
    /* 宽度 1 时让渲染器走最省的路径（lv_chart 也这么做） */
    if(t->width == 1) ld.raw_end = 1;

    const int32_t nm1 = t->n - 1;
    const int32_t xspan = w - 1;
    const int32_t yspan = h - 1;

    /*
     * 亚像素步进注意：cx==px 时(多点映射同一像素列)只考察不画,
     * px/py **保持不动** —— 若跟着推进 py, 中间样点的几何丢失,
     * 实线变虚线(603695b2 补丁引入、本版修复)。
     * NaN = 无效样点断口：断口后重锚不画, 不跨断口连线。
     */
    int32_t px=0,py=0;int previous=-1;int anchored=0;
    for(int i=0;;) {
        if(!isfinite(t->y[i])) { previous=-1;anchored=0; }
        else {
            float value=t->y[i];if(value<0.f)value=0.f;if(value>1.f)value=1.f;
            int32_t cx=x0+(int32_t)((int64_t)i*xspan/nm1);
            int32_t cy=y0+yspan-(int32_t)(value*yspan);
            int gap=0;
            if(previous>=0)for(int k=previous+1;k<i;k++)if(!isfinite(t->y[k]))gap=1;
            if(!anchored || gap) { px=cx;py=cy;anchored=1; }
            else if(cx>px) {
                ld.p1.x=px;ld.p1.y=py;ld.p2.x=cx;ld.p2.y=cy;
                lv_draw_line(layer,&ld);
                px=cx;py=cy;
            }
            previous=i;
        }
        if(i==nm1)break;
        i=(i+stride<nm1)?i+stride:nm1;
    }

}

static void draw_iq(lv_layer_t *layer, plot_t *p,
                    int32_t x0, int32_t y0, int32_t w, int32_t h)
{
    const int32_t cx = x0 + w / 2;
    const int32_t cy = y0 + h / 2;
    int32_t r = (w < h ? w : h) * 36 / 100;      /* .36 */
    if(r < 4) return;

    lv_draw_arc_dsc_t ad;
    lv_draw_arc_dsc_init(&ad);
    ad.center.x = cx;
    ad.center.y = cy;
    ad.color    = V32_C_IQ_RING;
    ad.width    = 1;
    ad.opa      = LV_OPA_COVER;
    /* ⚠️ lv_draw_arc 的角度单位是**度**（不是 lv_arc 控件的 0.1 度）：
     *    见 lvgl/src/draw/sw/lv_draw_sw_arc.c 里 `while(start_angle >= 360) start_angle -= 360`
     *    以及 lv_draw_arc_get_area() 里的 `start_quarter = start_angle_int / 90`。 */
    ad.start_angle = 0;
    ad.end_angle   = 360;

    int i;
    for(i = 1; i <= 3; i++) {
        ad.radius = (uint16_t)(r * i / 3);
        lv_draw_arc(layer, &ad);
    }

    if(!p->iq_valid){text(layer,cx-45,cy-10,"NO PHASE",V32_C_NOTE,V32_FONT_SMALL);return;}
    /* 矢量：JS 是 ctx.lineTo(cx + cos(θ)*r*.82, cy - sin(θ)*r*.82) */
    const float rad = p->iq_phase * 3.14159265f / 180.f;
    const int32_t vr = r * 82 / 100;
    const int32_t vx = cx + (int32_t)(cosf(rad) * (float)vr);
    const int32_t vy = cy - (int32_t)(sinf(rad) * (float)vr);

    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = V32_C_CH1;
    ld.width = 3;
    ld.opa   = LV_OPA_COVER;
    ld.round_start = 1;
    ld.round_end   = 1;
    line(layer, &ld, cx, cy, vx, vy);

    /* 轴标注：Q 在上方中间，I 在右侧。参考稿是 ctx.fillText 的 baseline 坐标，
     * 这里换算成左上角坐标，差几像素不影响观感。 */
    text(layer, cx + 8, y0 + 6, "Q", V32_C_CH1, V32_FONT_SMALL);
    text(layer, x0 + w - 20, cy - 16, "I", V32_C_CH1, V32_FONT_SMALL);
}

/* 测量光标：洋红虚线。用短线段拼虚线（这版 LVGL 的 dash 支持不稳）。 */
/* 顶部信息行：照原型的 A1=… A2=… Time/Div=… Trig=… 一行绿字 */
/* 刻度文字：x 在底部留白居中，y 在左侧留白右对齐 */
static void draw_ticks(lv_layer_t *layer, plot_t *p,
                       int32_t ox, int32_t oy, int32_t ow, int32_t oh,
                       int32_t gx, int32_t gy, int32_t gw, int32_t gh)
{
    (void)oy;
    /* scope_proto：刻度字是标签绿（C.label），不是芯片选中色 */
    if(p->tk_nx > 0) {
        for(int i = 0; i < p->tk_nx && i < 6; i++) {
            const int32_t cx = gx + (int32_t)((long)i * 2 * gw / (long)p->divx);
            /* 框下 4px（框本身 2px）居中 */
            lv_point_t sz;
            lv_text_get_size(&sz, p->tk_x[i], V32_FONT_SMALL, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
            int32_t tx=cx-sz.x/2;
            if(tx<ox+4)tx=ox+4;
            if(tx+sz.x>ox+ow-4)tx=ox+ow-4-sz.x;
            text_w(layer, tx, gy + gh + 4, sz.x + 8, p->tk_x[i], V32_C_LABEL, V32_FONT_SMALL);
        }
    }
    if(p->tk_ny > 0) {
        for(int j = 0; j < p->tk_ny && j < 5; j++) {
            const int32_t cy = gy + (int32_t)((long)j * 2 * gh / (long)p->divy);
            /* 右对齐贴在框左侧 8px（scope_proto: textAlign='right', x=PL.x-8） */
            lv_point_t sz;
            lv_text_get_size(&sz, p->tk_y[j], V32_FONT_SMALL, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
            int32_t tx=gx-8-sz.x;if(tx<ox+2)tx=ox+2;
            text_w(layer, tx, cy - sz.y / 2, sz.x + 8, p->tk_y[j], V32_C_LABEL, V32_FONT_SMALL);
        }
    }
    (void)ow;
}

static void draw_info_dummy(lv_layer_t *layer, plot_t *p, int32_t x0, int32_t y0, int32_t w)
{ (void)layer; (void)p; (void)x0; (void)y0; (void)w; }

static void draw_cursors(lv_layer_t *layer, plot_t *p,
                         int32_t x0, int32_t y0, int32_t w, int32_t h)
{
    if(!p->cur_vis) return;

    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = V32_C_TRIGGER;
    ld.width = 2;
    ld.opa   = LV_OPA_COVER;

    int32_t vx[2] = {0, 0}, hy[2] = {0, 0};
    int nv = 0, nh = 0;
    for(int i = 0; i < p->cur_n && i < 4; i++) {
        ld.color=(i%2)?V32_C_CURSOR_B:V32_C_CURSOR_A;
        if(i < 2) {                              /* 竖线：时间光标 */
            const int32_t x = x0 + (int32_t)(p->cur_x[i] * (float)(w - 1));
            for(int32_t y = y0; y < y0 + h - 1; y += 8)
                line(layer, &ld, x, y, x, (y + 4 < y0 + h - 1) ? y + 4 : y0 + h - 1);
            if(nv < 2) vx[nv++] = x;
            text_plated(layer,x+5<x0+w-30?x+5:x-27,y0+16,i?"T2":"T1",ld.color,V32_FONT_SMALL_B);
        } else {                                 /* 横线：电压光标 */
            const int32_t y = y0 + (int32_t)(p->cur_y[i - 2] * (float)(h - 1));   /* i-2：cur_y[] 只有 2 个 */
            for(int32_t x = x0; x < x0 + w - 1; x += 8)
                line(layer, &ld, x, y, (x + 4 < x0 + w - 1) ? x + 4 : x0 + w - 1, y);
            if(nh < 2) hy[nh++] = y;
            text_plated(layer,x0+10,y+4<y0+h-22?y+4:y-22,i==2?"V1":"V2",ld.color,V32_FONT_SMALL_B);
        }
    }
    /* 交点小方块（scope_proto: 4×4 实心方块） */
    lv_draw_rect_dsc_t rd;
    lv_draw_rect_dsc_init(&rd);
    rd.bg_color = V32_C_TRIGGER;
    rd.bg_opa   = LV_OPA_COVER;
    rd.radius   = 1;
    for(int a = 0; a < nv; a++)
        for(int b = 0; b < nh; b++) {
            rd.bg_color=a?V32_C_CURSOR_B:V32_C_CURSOR_A;
            lv_area_t r = { vx[a] - 2, hy[b] - 2, vx[a] + 1, hy[b] + 1 };
            lv_draw_rect(layer, &rd, &r);
        }
}

/* XY 模式（李萨如）：trace0 当 X、trace1 当 Y，逐点连线。
 * 不画时间轴 —— 横轴是 CH1 的电压，纵轴是 CH2 的电压。 */
static void draw_xy(lv_layer_t *layer, plot_t *p,
                    int32_t x0, int32_t y0, int32_t w, int32_t h)
{
    if(!p->tr[0].vis || !p->tr[1].vis) return;

    if(p->xy_equal){int side=w<h?w:h;x0+=(w-side)/2;y0+=(h-side)/2;w=h=side;}

    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = p->tr[0].color;
    ld.width = 2;
    ld.opa   = LV_OPA_COVER;
    ld.round_start=1;ld.round_end=1;

    /* Axes behind samples: a BPSK point on Q=0 must not be split by the axis. */
    ld.color=p->center_color;ld.width=1;
    line(layer,&ld,x0+w/2,y0,x0+w/2,y0+h-1);
    line(layer,&ld,x0,y0+h/2,x0+w-1,y0+h/2);
    ld.color=p->tr[0].color;ld.width=2;

    const int n = (p->tr[0].n < p->tr[1].n) ? p->tr[0].n : p->tr[1].n;
    int32_t px=0,py=0;int have_previous=0;
    for(int i=0;i<n;i++) {
        float x=p->tr[0].y[i],y=p->tr[1].y[i];
        if(!isfinite(x) || !isfinite(y)) { have_previous=0;continue; }
        if(x<0.f)x=0.f;
        if(x>1.f)x=1.f;
        if(y<0.f)y=0.f;
        if(y>1.f)y=1.f;
        int32_t cx=x0+(int32_t)(x*(w-1)),cy=y0+h-1-(int32_t)(y*(h-1));
        if(p->xy_points){
            lv_draw_rect_dsc_t dot;lv_draw_rect_dsc_init(&dot);
            dot.bg_color=ld.color;dot.bg_opa=LV_OPA_COVER;dot.radius=LV_RADIUS_CIRCLE;
            lv_area_t bounds={cx-2,cy-2,cx+2,cy+2};lv_draw_rect(layer,&dot,&bounds);
        }
        else if(have_previous)line(layer,&ld,px,py,cx,cy);
        px=cx;py=cy;have_previous=1;
    }

}

static void draw_trigger(lv_layer_t *layer, plot_t *p,
                         int32_t x0, int32_t y0, int32_t w, int32_t h)
{
    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = V32_C_TRIGGER;
    ld.width = 1;
    ld.opa   = LV_OPA_COVER;

    if(p->mode == PLOT_SCOPE) {
        /* scope_proto 的画法：
         *  - 触发电平 = 贯穿绘图区的洋红细虚线（透明度低），两端 22px 实线头，
         *    左端一个小三角提示"可以拖"
         *  - 触发位置 = 顶部一个小三角（不再画整根竖线，免得糊住波形） */
        if(p->trig_lev_vis) {
            const int32_t y = y0 + (int32_t)((1.f - p->trig_y) * (float)(h - 1));
            if(y >= y0 && y <= y0 + h - 1) {
                ld.opa = (lv_opa_t)(LV_OPA_COVER * 28 / 100);
                for(int32_t x = x0 + 24; x < x0 + w - 24; x += 10)
                    line(layer, &ld, x, y, (x + 4 < x0 + w - 24) ? x + 4 : x0 + w - 24, y);
                ld.opa = LV_OPA_COVER;
                line(layer, &ld, x0, y, x0 + 22, y);
                line(layer, &ld, x0 + w - 23, y, x0 + w - 1, y);
                lv_draw_triangle_dsc_t td;
                lv_draw_triangle_dsc_init(&td);
                td.bg_color = V32_C_TRIGGER;
                td.bg_opa   = LV_OPA_COVER;
                td.p[0].x = x0 + 2;  td.p[0].y = y;
                td.p[1].x = x0 + 11; td.p[1].y = y - 6;
                td.p[2].x = x0 + 11; td.p[2].y = y + 6;
                lv_draw_triangle(layer, &td);
            }
        }
        if(p->trig_vis) {
            const int32_t x = x0 + (int32_t)(p->trig_x * (float)(w - 1));
            lv_draw_triangle_dsc_t td;
            lv_draw_triangle_dsc_init(&td);
            td.bg_color = V32_C_TRIGGER;
            td.bg_opa   = LV_OPA_COVER;
            td.p[0].x = x;     td.p[0].y = y0 + 2;
            td.p[1].x = x - 6; td.p[1].y = y0 + 11;
            td.p[2].x = x + 6; td.p[2].y = y0 + 11;
            lv_draw_triangle(layer, &td);
        }
        return;
    }

    if(!p->trig_vis) return;

    const int32_t x = x0 + (int32_t)(p->trig_x * (float)(w - 1));
    line(layer, &ld, x, y0, x, y0 + h - 1);

    /* JS: ctx.fillText('T', w*.46+4, 28) */
    if(p->mode!=PLOT_FFT)text(layer, x + 4, y0 + 18, "T", V32_C_TRIGGER, V32_FONT_SMALL);
}

/* 包络虚线: 3 段画 2 段跳, 无效点断口 */
static void draw_env_curve(lv_layer_t *layer, const float *arr, int n,
                           int32_t x0, int32_t y0, int32_t w, int32_t h,
                           lv_draw_line_dsc_t *ld)
{
    const int32_t nm1 = n - 1;
    const int32_t xspan = w - 1;
    int run = 0;
    int32_t px = 0, py = 0;
    int have = 0;
    for(int i = 0; i < n; i++) {
        float v = arr[i];
        if(!isfinite(v)) { have = 0; continue; }
        if(v < 0.f) v = 0.f;
        if(v > 1.f) v = 1.f;
        const int32_t cx = x0 + (int32_t)((int64_t)i * xspan / nm1);
        const int32_t cy = y0 + (h - 1) - (int32_t)(v * (float)(h - 1));
        if(have && (run % 5) < 3) {
            ld->p1.x = px; ld->p1.y = py; ld->p2.x = cx; ld->p2.y = cy;
            lv_draw_line(layer, ld);
        }
        px = cx; py = cy; have = 1; run++;
    }
}

/* 找 trace ti 在 peak_x 附近的最高点，用来把 ▼ 贴到曲线的峰上 */
static int32_t peak_curve_y(plot_t *p, int ti, float at_x, int32_t y0, int32_t h)
{
    trace_t *t = &p->tr[ti];
    if(!t->vis || t->n < 2) return -1;

    int32_t best = -1;
    int i;
    for(i = 0; i < t->n; i++) {
        const float u = (float)i / (float)(t->n - 1);
        if(fabsf(u - at_x) > 0.03f) continue;
        const int32_t y = (int32_t)(t->y[i] * (h - 1));
        if(y > best) best = y;          /* y 越大 = 屏幕越靠上 = 值越高 */
    }
    if(best < 0) return -1;
    return y0 + (h - 1) - best;
}

static void draw_peak(lv_layer_t *layer, plot_t *p,
                      int32_t x0, int32_t y0, int32_t w, int32_t h)
{
    if(!p->peak_vis) return;

    const int32_t px = x0 + (int32_t)(p->peak_x * (float)(w - 1));

    /* 曲线上的 ▼ 三角标 */
    const int32_t cy = peak_curve_y(p, 0, p->peak_x, y0, h);
    if(cy > 0) {
        lv_draw_triangle_dsc_t td;
        lv_draw_triangle_dsc_init(&td);
        td.bg_color = V32_C_FFT;
        td.bg_opa   = LV_OPA_COVER;
        td.p[0].x = px - 5; td.p[0].y = cy - 10;
        td.p[1].x = px + 5; td.p[1].y = cy - 10;
        td.p[2].x = px;     td.p[2].y = cy - 2;
        lv_draw_triangle(layer, &td);
    }

    /* 文字放到画面上方（照抄参考稿的 y=32），水平夹在画布内 */
    int32_t tx = px - 30;
    const int32_t tx_min = x0 + 10;
    const int32_t tx_max = x0 + w - 130;
    if(tx < tx_min) tx = tx_min;
    if(tx > tx_max) tx = tx_max;
    if(tx_max < tx_min) tx = tx_min;
    /* Peak frequency already has a dedicated sidebar; avoid a second title-row label. */
    (void)tx;
}

/* ==================== 主绘制回调 ==================== */

bool plot_get_data_area(lv_obj_t *plot,lv_area_t *area)
{
    plot_t *p=PLOT_OF(plot);if(!p || !area)return false;
    lv_obj_get_coords(plot,area);
    int bw=lv_obj_get_style_border_width(plot,LV_PART_MAIN);
    area->x1+=bw;area->x2-=bw;area->y1+=bw;area->y2-=bw;
    if(p->tk_ny>0){area->x1+=80;area->y1+=20;}
    if(p->mode==PLOT_FFT){area->x1+=58;area->y1+=62;area->y2-=30;}
    if(p->tk_nx>0)area->y2-=18;
    return area->x2>area->x1 && area->y2>area->y1;
}

static void plot_draw_cb(lv_event_t *e)
{
    plot_t *p = lv_event_get_user_data(e);
    if(p == NULL || p->obj == NULL) return;

    lv_layer_t *layer = lv_event_get_layer(e);
    if(layer == NULL) return;

    lv_area_t c;
    lv_obj_get_coords(p->obj, &c);

    const int32_t full_w = c.x2 - c.x1 + 1;
    const int32_t full_h = c.y2 - c.y1 + 1;
    if(full_w < 10 || full_h < 10) return;

    /* .plot 的 canvas 在 1px border 里面（box-sizing:border-box） */
    const int32_t bw = (int32_t)lv_obj_get_style_border_width(p->obj, LV_PART_MAIN);
    int32_t x0 = c.x1 + bw;
    int32_t y0 = c.y1 + bw;
    int32_t w  = full_w - bw * 2;
    int32_t h  = full_h - bw * 2;
    if(w < 6 || h < 6) return;

    /* 有刻度文字时，网格往里收：左 42px 给电压刻度，下 18px 给时间刻度 */
    int32_t ox = x0, oy = y0, ow = w, oh = h;
    if(p->tk_ny > 0) { x0 += 80; w -= 80; }
    if(p->tk_nx > 0) { h -= 18; }
    if(p->tk_ny > 0) { y0 += 20; h -= 20; }   /* 顶部信息条 */

    if(p->mode==PLOT_FFT){x0+=58;w-=58;y0+=62;h-=92;}
    draw_grid(layer, p, x0, y0, w, h);
    if(p->mode==PLOT_FFT){
        static const char *db[]={"0","-25","-50","-75","-100"};
        for(int i=0;i<5;i++)text_w(layer,ox+6,y0+i*(h-1)/4-8,48,db[i],V32_C_LABEL,V32_FONT_SMALL);
    }

    if(p->mode == PLOT_IQ) {
        draw_iq(layer, p, x0, y0, w, h);
    }
    else if(p->mode == PLOT_XY) {
        draw_xy(layer, p, x0, y0, w, h);
    }
    else {
        draw_one_trace(layer, p, 0, x0, y0, w, h);
        draw_one_trace(layer, p, 1, x0, y0, w, h);
        /* AM 上下包络(金色虚线), 叠在载波迹线上 */
        if(p->env_vis && p->env_n >= 2 && p->mode == PLOT_SCOPE) {
            lv_draw_line_dsc_t ed;
            lv_draw_line_dsc_init(&ed);
            ed.color = V32_C_WARN;
            ed.width = 1;
            ed.opa   = LV_OPA_COVER;
            if(p->env_fill && w>1){
                ed.opa=LV_OPA_20;
                for(int x=0;x<w;x++){
                    float t=(float)x*(p->env_n-1)/(w-1);
                    int a=(int)t,b=a+1<p->env_n?a+1:a;
                    if(!isfinite(p->env_u[a]) || !isfinite(p->env_d[a]) ||
                       !isfinite(p->env_u[b]) || !isfinite(p->env_d[b]))continue;
                    float f=t-a;
                    float u=p->env_u[a]+f*(p->env_u[b]-p->env_u[a]);
                    float d=p->env_d[a]+f*(p->env_d[b]-p->env_d[a]);
                    u=fminf(1.f,fmaxf(0.f,u));d=fminf(1.f,fmaxf(0.f,d));
                    line(layer,&ed,x0+x,y0+(int)((1-u)*(h-1)),x0+x,y0+(int)((1-d)*(h-1)));
                }
                ed.opa=LV_OPA_COVER;
            }
            draw_env_curve(layer, p->env_u, p->env_n, x0, y0, w, h, &ed);
            draw_env_curve(layer, p->env_d, p->env_n, x0, y0, w, h, &ed);
        }
        /* PLOT_FFT 也画游标线：频谱页要用手指点选频率（cb_fft_cursor） */
        if(p->mode == PLOT_SCOPE || p->mode == PLOT_FFT) draw_trigger(layer, p, x0, y0, w, h);
        if(p->mode == PLOT_SCOPE) draw_cursors(layer, p, x0, y0, w, h);
        /* 信息行改由 ui.c 的标签控件显示（它自带底板、压在波形上也读得清）；
           plot 里这套先关掉，否则同一行字会画两遍。 */
        if(p->mode == PLOT_FFT)   draw_peak(layer, p, x0, y0, w, h);
    }

    if(p->tk_nx > 0 || p->tk_ny > 0)
        draw_ticks(layer, p, ox, oy, ow, oh, x0, y0, w, h);

    /* 顶部信息行（scope_proto 的 A1=… A2=… Time/Div=… Trig=…）：
     * 分段配色画在框上方的留白里（tk_ny>0 时顶部留了 20px）。 */
    if(p->info_n > 0 && p->tk_ny > 0) {
        const int32_t iy = oy + (20 - V32_FONT_INFO->line_height) / 2;
        int32_t ix = x0;
        for(int i = 0; i < p->info_n && i < 12; i++) {
            lv_point_t sz;
            lv_text_get_size(&sz, p->info_seg[i].txt, V32_FONT_INFO, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
            if(sz.x <= 0) continue;
            text_w(layer, ix, iy, sz.x + 8, p->info_seg[i].txt, p->info_seg[i].color, V32_FONT_INFO);
            ix += sz.x;
            if(ix > x0 + w - 40) break;     /* 超出绘图区宽度就截断 */
        }
    }

    /* 角落坐标文字（参考稿 FFT 页画在画布左下角）。
     * 用 text_plated：这两处会被曲线穿过，白底上不垫底板就读不清。 */
    if(p->axis_l[0]) text_plated(layer, x0 + 8, p->mode==PLOT_FFT?y0+h+5:y0+h-20, p->axis_l, V32_C_TEXT, V32_FONT_SMALL);
    if(p->axis_r[0]) {
        lv_point_t size;lv_text_get_size(&size,p->axis_r,V32_FONT_SMALL,0,0,LV_COORD_MAX,LV_TEXT_FLAG_NONE);
        text_plated(layer,x0+w-size.x-8,p->mode==PLOT_FFT?y0+h+5:y0+h-20,p->axis_r,V32_C_TEXT,V32_FONT_SMALL);
    }

    /* 曲线右端的小图例 */
    if(p->leg0[0]) text(layer, x0 + w - 60, y0 + 6,      p->leg0, V32_C_CH1, V32_FONT_SMALL);
    if(p->leg1[0]) text(layer, x0 + w - 60, y0 + 6 + 15, p->leg1, V32_C_CH2, V32_FONT_SMALL);

    /* scope_proto：亮绿 2px 直角外框**紧贴网格区**，刻度文字在框外。
     * 最后画，压住网格和曲线的毛边。 */
    {
        lv_draw_rect_dsc_t fd;
        lv_draw_rect_dsc_init(&fd);
        fd.bg_opa       = LV_OPA_TRANSP;
        fd.border_color = V32_C_PLOT_BORDER;
        fd.border_width = 2;
        fd.border_opa   = LV_OPA_COVER;
        lv_area_t fr = { x0, y0, x0 + w - 1, y0 + h - 1 };
        lv_draw_rect(layer, &fd, &fr);
    }
}

/* ==================== 归一化 ==================== */

int plot_norm(const float *src, int n, float *dst, int dst_n, float margin)
{
    if(src == NULL || dst == NULL || n < 2 || dst_n < 2) return -1;

    float lo = src[0], hi = src[0];
    int i;
    for(i = 1; i < n; i++) {
        if(src[i] < lo) lo = src[i];
        if(src[i] > hi) hi = src[i];
    }
    float span = hi - lo;
    if(span <= 1e-12f) {
        /* 全等于一个值：画成中间一条直线，而不是除零 */
        for(i = 0; i < dst_n; i++) dst[i] = 0.5f;
        return -1;
    }
    if(margin < 0.f) margin = 0.f;
    if(margin > 0.5f) margin = 0.5f;

    /* 用 lo/hi 把数据压到 [margin, 1-margin]，再做线性插值重采样 */
    const float inv = 1.0f / span;
    for(i = 0; i < dst_n; i++) {
        const float u = (dst_n == 1) ? 0.f : (float)i / (float)(dst_n - 1);
        const float fi = u * (float)(n - 1);
        const int   i0 = (int)fi;
        const int   i1 = (i0 + 1 < n) ? i0 + 1 : i0;
        const float fr = fi - (float)i0;
        const float v = src[i0] * (1.f - fr) + src[i1] * fr;

        float yn = (v - lo) * inv;                 /* 0..1 */
        yn = yn * (1.f - 2.f * margin) + margin;   /* 留白 */
        if(yn < 0.f) yn = 0.f;
        if(yn > 1.f) yn = 1.f;
        dst[i] = yn;
    }
    return 0;
}

