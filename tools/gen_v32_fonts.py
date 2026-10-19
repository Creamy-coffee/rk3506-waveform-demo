#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_v32_fonts.py —— 从参考稿 HTML + 本工程源码提取字符集，生成 LVGL 位图字体子集。

背景：参考稿（V32 信号分析仪 UI 原型）底部控制条和系统页有中文
（信号频率 / 噪声 / 时基 / 暂停 / 复位 / 全屏 …），而 LVGL 内置字体只含 ASCII。
不能把整个 10MB 的 SimHei 塞进去，所以按「实际用到的字符」生成子集。

**多字体会合**：单个字体覆盖不全必须分工——
    SimHei  没有 µ (U+00B5) 和 ⌁⏸▥⚙⛶✧↺ 这些装饰符号；
    DejaVuSans 没有汉字，也没有 ⏸ / ⛶；
    Segoe UI Symbol 有全部装饰符号。
所以：
    DejaVuSans  -> 可打印 ASCII + °µ·×Δ–—●      （拉丁和度量符号，字形比 SimHei 好看）
    seguisym    -> ↺⌁⏸▥◎⚙⛶✧                     （界面上的装饰图标）
    simhei      -> 汉字 + 、。，；                （中文文案）

分配是按字体真实 cmap 表算的（fontTools），不是硬编码猜的；
最后会报出「所有字体都没有」的字符，避免静默缺字。

⚠️ 字符集来源有**三处**，缺一处就会静默变成方块（都踩过）：
    1. 参考稿 HTML —— 连 <script> 里的字符串一起扫（有一批文案只在 JS 里）
    2. **本工程自己的源码字符串字面量** —— 我们加的文案（比如系统页那句
       「Render FPS 和 Uptime 是直接读出硬件的真值」里的"硬件"）参考稿里没有，
       只扫 HTML 就会漏，屏幕上就成了 ▯
    3. EXTRA 兜底列表
   来源 2 只取字符串字面量、先剥掉注释 —— 否则随手写的中文注释会把字符集
   撑到几百个字，白烧 flash。

⚠️ Windows 命令行传 UTF-8 汉字很容易被代码页搞坏，所以这里
   把字符集转成纯 ASCII 的 `-r 0x4E0D,0x4E0E,...` 码点列表传给 lv_font_conv。

用法：
    python tools/gen_v32_fonts.py
产物：
    src/fonts/lv_font_v32_12.c / _18.c / _20.c          （常规体）
    src/fonts/lv_font_v32_12_bold.c / _18_bold.c / _20_bold.c （粗体）

字号取值的依据（对着参考稿 CSS 定的，别随手改）：
    12px  —— .statuses / .metric / .panel-title / .plot-title / .nav button
    16px  —— .chip / select / .action；**这一档最容易漏**
             CSS 里 `.chip,.nav button{...}` 只给了边框/底色/内边距，**没给 font-size**，
             而 `.nav button{font-size:12px}` 单独覆盖了导航。也就是说：
                 .chip / select / .action 继承 <body> 的**浏览器默认 16px**
                 .nav button 才是 12px
             一开始整屏按 12px 做，结果右栏 CH1 ON 是 73×37 而参考稿是 90×43
             （16px 的 Segoe UI 行高 21 -> 21+20+2=43），控制条按钮也整体偏窄。
    18px  —— .metric b（右栏读数大字）是 18px，是整个界面最显眼的一档
    20px  —— .brand 是 19px，四舍五入到 20
    （参考稿里还有 10/11/13px 的细档，差异在 2px 内看不出来，统一就近取档）

粗体也不是可有可无的：参考稿 .brand 是 font-weight:800、.metric b / .kv b
是 <b>(700)、.nav button 是 750，这几处**全部是加粗**的。全用常规体渲染出来
观感会明显偏"轻"，和参考稿差得比较远。
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)

REF_HTML = os.environ.get(
    "REF_HTML",
    r"D:\文档\xwechat_files\wxid_gtewgzaz8vtg22_5219\msg\file\2026-10\v32_linux_touch_ui-1.html")
NODE = os.environ.get(
    "NODE", r"C:\Users\21542\.workbuddy\binaries\node\versions\22.22.2-6\node.exe")
LVFC = os.environ.get(
    "LVFC", r"C:\Users\21542\.workbuddy\binaries\node\workspace\node_modules\lv_font_conv\lv_font_conv.js")
PYTHON = os.environ.get(
    "PYTHON", r"C:\Users\21542\.workbuddy\binaries\python\envs\default\Scripts\python.exe")

FONTS_DIR = r"C:\Windows\Fonts"

# 按优先级排列：先来的先挑字符
FONT_PLAN = [
    (os.path.join(FONTS_DIR, "DejaVuSans.ttf"), "拉丁/度量符号"),
    (os.path.join(FONTS_DIR, "seguisym.ttf"),   "装饰图标"),
    (os.path.join(FONTS_DIR, "simhei.ttf"),     "汉字"),
]

# 粗体链。参考稿的字体栈是 Inter,"Microsoft YaHei",system-ui —— 在 Windows 上
# system-ui 落到 Segoe UI，所以粗体用 Segoe UI Bold 最接近（没有 DejaVu 粗体）。
# 图标仍然让 seguisym 出（几何形状，字重看不出差别）。
#
# 汉字为什么要进粗体链（v6 加的）：
#   一开始"参考稿里没有一处中文加粗"，所以粗体只出非 CJK，省几百 KB。
#   但界面改成中文之后，**底部导航、品牌标题、系统页键值全落在粗体档**
#   （V32_FONT_SMALL_B / BIG_B / BASE_B），中文进不去就是一片方块。
#   SimHei 没有真正的粗体字重，lv_font_conv 拿到的是常规笔画 —— 12px 的
#   中文导航按钮上这点字重差看不出来，比"方块"强得多。
#   代价：四个粗体档各多约 20~50KB，总共约 +150KB，板子上完全放得下。
BOLD_FONT_PLAN = [
    (os.path.join(FONTS_DIR, "segoeuib.ttf"), "拉丁（粗）"),
    (os.path.join(FONTS_DIR, "seguisym.ttf"), "装饰图标"),
    (os.path.join(FONTS_DIR, "simhei.ttf"),   "汉字（粗体档兜底）"),
]

OUT_DIR = os.path.join(PROJ, "src", "fonts")
SIZES = [12, 16, 18, 20]

# 非 CJK 的分界线：小于它就当"拉丁/符号"，可以进粗体字体。
# 中文、中文标点（、。是 U+3001/3002，全角逗号分号是 U+FF0C/FF1B）都在这之上。
NON_CJK_LIMIT = 0x2E80

# 除了参考稿和源码里的字符，再补一些兜底（避免加文案后又要重跑）
EXTRA = ("求解时间轴通道模式触发保持自动正常扫描样点窗函数载入保存关闭开启"
         "积分降噪运行离线当前秒未连接真实重启版本编译移植界面"
         "▶▼")   # ▶/▼ 只出现在 JS 字符串里，保险起见显式写上

# 要扫字符串字面量的本工程源码。
#
# ⚠️ 这里**刻意只列会上屏的两个文件**，不要图省事写成 src/*.c：
#    drm_disp.c / port.c / main.c 里有大量中文 printf 日志
#    （"[port] 没有 framebuffer 设备，改用 DRM" 之类），它们只走串口，
#    永远不渲染到屏幕。把它们扫进来会让字符集从 246 暴涨到 449，
#    三档常规字体从 520KB 涨到 1.08MB —— 纯粹白烧 flash。
#    判断标准：这个文件里的中文会不会出现在 LVGL 控件上。
SRC_FILES = ["src/ui.c", "src/plot.c"]


def strip_c_comments(s):
    """剥掉 C 注释。必须先剥再取字符串字面量 ——
    否则我们这些写得密密麻麻的中文注释会把字符集撑到几百字。"""
    s = re.sub(r"/\*.*?\*/", " ", s, flags=re.S)
    s = re.sub(r"//[^\n]*", " ", s)
    return s


def src_string_chars():
    """把本工程源码里**字符串字面量**中的非 ASCII 字符捞出来。

    这一条是补一个真实踩过的坑：系统页那句
        "本页是 UI 移植版，…Render FPS 和 Uptime 是直接读出硬件的真值。"
    里的「硬件」参考稿 HTML 里根本没有，只扫 HTML 时这两个字静默缺失，
    板子上渲染成 ▯。加了这一步以后，往界面加中文文案就不会再漏。
    """
    import glob
    chars = set()
    files = []
    for g in SRC_FILES:
        files += glob.glob(os.path.join(PROJ, g))
    for path in sorted(set(files)):
        try:
            src = strip_c_comments(open(path, encoding="utf-8").read())
        except (OSError, UnicodeDecodeError):
            continue
        for m in re.finditer(r'"((?:[^"\\\n]|\\.)*)"', src):
            for ch in m.group(1):
                if ord(ch) > 0x7F:
                    chars.add(ch)
    print(f"  从 {len(set(files))} 个源码文件里捞到 {len(chars)} 个非 ASCII 字符")
    return chars


def collect_charset():
    """把「参考稿 + 本工程源码 + 兜底表」里出现过的字符全捞出来。

    ⚠️ 扫参考稿时一定要扫**原始文件**，不能先把 <script> 剥掉：
       参考稿有一批文案只在 JS 字符串里，HTML 标记里根本没有——
       比如 '▼ '+freq+' MHz'（FFT 峰值标记）、'▶ 运行'、'窗函数'、'积分时间'、
       'AI 降噪'、'当前为离线 DEMO，未连接真实硬件'。
       按「剥掉 script 再扫」的老写法，这些字会静默缺失，
       板子上就变成一片空白/方块，而且编译期不会有任何提示。
    """
    chars = set(chr(c) for c in range(0x20, 0x7F))       # 可打印 ASCII

    if os.path.isfile(REF_HTML):
        raw = open(REF_HTML, encoding="utf-8").read()

        # 1) HTML 正文（去标签、还原常见实体）——覆盖面最广，正常文案都在这
        body = re.sub(r"<(script|style).*?</\1>", "", raw, flags=re.S)
        body = re.sub(r"<[^>]+>", " ", body)
        body = (body.replace("&nbsp;", " ").replace("&amp;", "&")
                    .replace("&lt;", "<").replace("&gt;", ">"))
        # 2) JS / CSS 里的字符串：直接从原始文本里按非 ASCII 捞，避免解析器
        for ch in raw:
            if ord(ch) > 0x7F:
                chars.add(ch)
        for ch in body:
            if ord(ch) > 0x7F:
                chars.add(ch)
    else:
        print(f"!! 找不到参考稿 {REF_HTML}，只生成 ASCII")

    chars.update(src_string_chars())
    chars.update(EXTRA)
    chars.discard("\x00")
    return chars


def font_cmap(path):
    """读字体真实支持的码点集合。用 fontTools，不用猜。"""
    from fontTools.ttLib import TTFont
    t = TTFont(path, fontNumber=0, lazy=True)
    cmap = set(t.getBestCmap().keys())
    t.close()
    return cmap


def to_ranges(codes):
    """把码点集合压成 '0x20-0x7E,0xB0,...' 这种紧凑的纯 ASCII 参数。"""
    codes = sorted(codes)
    out, i = [], 0
    while i < len(codes):
        j = i
        while j + 1 < len(codes) and codes[j + 1] == codes[j] + 1:
            j += 1
        if j > i + 1:
            out.append("0x%04X-0x%04X" % (codes[i], codes[j]))
        else:
            for k in range(i, j + 1):
                out.append("0x%04X" % codes[k])
        i = j + 1
    return ",".join(out)


def assign_chars(plan, charset):
    """按优先级把字符分给各字体，返回 [(path, desc, codes)]。"""
    remaining = set(ord(c) for c in charset)
    assign = []
    for path, desc in plan:
        cmap = font_cmap(path)
        got = remaining & cmap
        if got:
            assign.append((path, desc, got))
            remaining -= got
        print(f"  {os.path.basename(path):<16} ({desc}) 收录 {len(got):3d} 个，剩 {len(remaining)} 个")
    return assign, remaining


def build_one(size, suffix, assign, label):
    name = f"lv_font_v32_{size}{suffix}"
    out = os.path.join(OUT_DIR, name + ".c")
    cmd = [NODE, LVFC, "--size", str(size), "--bpp", "4",
           "--format", "lvgl", "--lv-font-name", name,
           "--no-compress",     # 压缩要额外解压表，板子上不划算
           "--no-kerning",      # 界面文字等宽排布，去掉省体积
           "-o", out]
    for path, _desc, got in assign:
        cmd += ["--font", path, "-r", to_ranges(got)]

    print(f"==> 生成 {size}px {label} -> {os.path.basename(out)}")
    if os.environ.get("SHOW_CMD"):
        print("    " + " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    txt = (r.stdout or "") + (r.stderr or "")
    for line in txt.splitlines():
        if line.strip():
            print("    " + line.strip()[:170])
    if r.returncode != 0:
        sys.exit(f"!! {size}px {label} 生成失败 (rc={r.returncode})")
    print(f"    OK  {os.path.getsize(out)} 字节")


def main():
    for plan in (FONT_PLAN, BOLD_FONT_PLAN):
        for p, _ in plan:
            if not os.path.isfile(p):
                sys.exit(f"找不到字体: {p}")
    if not os.path.isfile(LVFC):
        sys.exit(f"找不到 lv_font_conv: {LVFC}\n先跑: npm install lv_font_conv")

    charset = collect_charset()
    n_cjk = sum(1 for c in charset if ord(c) >= NON_CJK_LIMIT)
    print(f"需要 {len(charset)} 个字符（非 ASCII {len(charset) - 95} 个，其中汉字/中文标点 {n_cjk} 个）")

    os.makedirs(OUT_DIR, exist_ok=True)

    # ---- 常规体：全字符集 ----
    print("\n[常规体]")
    assign, remaining = assign_chars(FONT_PLAN, charset)
    if remaining:
        miss = "".join(chr(c) for c in sorted(remaining))
        print(f"\n!! 以下 {len(remaining)} 个字符所有候选字体都没有，会被跳过：\n   {miss}")
        print("   （要么换字体，要么在 UI 里避开这些字符）\n")

    # ---- 粗体：全字符集（v6 起包含汉字，因为导航/品牌/键值都是粗体档）----
    print("\n[粗体]")
    bold_charset = set(charset)
    print(f"  全字符集 {len(bold_charset)} 个字符（含汉字）")
    bold_assign, bold_remaining = assign_chars(BOLD_FONT_PLAN, bold_charset)
    if bold_remaining:
        miss = "".join(chr(c) for c in sorted(bold_remaining))
        print(f"\n!! 粗体有 {len(bold_remaining)} 个字符没字体能出：\n   {miss}")
        print("   （这些字符如果出现在加粗的文案里，板上会是方块）\n")

    print()
    for size in SIZES:
        build_one(size, "", assign, "常规")
    for size in SIZES:
        build_one(size, "_bold", bold_assign, "粗体")

    print("\n全部完成。记得把 src/fonts/*.c 加进 CMakeLists 的源文件列表。")


def reexec_with_fonttools():
    """如果当前解释器没有 fontTools，就用 PYTHON 指的那个解释器重新跑自己。

    踩过的坑：这个脚本容易被 Git-bash 自带的 python(3.13) 跑起来，
    而 fontTools 是装在另一个 venv 里的，于是 import fontTools 直接炸。
    与其让调用方记住用哪个 python，不如脚本自己纠正。
    """
    try:
        import fontTools  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("_V32_FONT_REEXEC"):        # 防无限递归
        sys.exit("!! 已经用 %s 重试过，仍然没有 fontTools。\n"
                 "   装一下: %s -m pip install fonttools" % (sys.executable, PYTHON))
    if not os.path.isfile(PYTHON):
        sys.exit("!! 当前解释器没有 fontTools，且找不到备用解释器 %s\n"
                 "   装一下: %s -m pip install fonttools" % (PYTHON, sys.executable))
    env = dict(os.environ, _V32_FONT_REEXEC="1")
    print("[gen_v32_fonts] 当前解释器没有 fontTools，改用 %s" % PYTHON)
    rc = subprocess.call([PYTHON, os.path.abspath(__file__)] + sys.argv[1:], env=env)
    sys.exit(rc)


if __name__ == "__main__":
    reexec_with_fonttools()
    main()
