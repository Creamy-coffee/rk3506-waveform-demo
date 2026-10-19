#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cmp_shot.py —— 把「参考稿渲染图」和「板子截图」裁同一块区域叠起来比

为什么要有它：
    1024x600 整图并排看，细节全糊在一起，判断不了「像不像」。
    裁到具体区域（比如顶栏、波形绘图区、右栏 MEASUREMENTS）上下叠放并放大，
    才能一眼看出字号/间距/颜色/波形形状差在哪。

用法：
    python tools/cmp_shot.py scope                 # 用内置默认区域
    python tools/cmp_shot.py scope topbar          # 指定命名区域
    python tools/cmp_shot.py scope 0 0 1024 60     # 手动给 x y w h

约定：
    板子侧截图取自 out/board_<page>.png（由 tools/board_shot.py 生成）
    参考稿取自   out/ref/ref_<page>.png（由 tools/render_ref.py 生成）
    产物         out/cmp/cmp_<page>_<region>.png
"""
import os
import sys

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REF_DIR = os.path.join(ROOT, "out", "ref")
BOARD_DIR = os.path.join(ROOT, "out")
CMP_DIR = os.path.join(ROOT, "out", "cmp")

W, H = 1024, 600

# 命名区域 —— 直接照着**修正后**的栅格算（含 .shell 的 1px 外框）：
#   外框 0 / 顶栏 1..48 / main 行 49..450（内容 58..442）/ controls 451..542 / nav 543..598
#   左绘图区 x 13..766 / 右栏 x 776..1010（宽 235）
REGIONS = {
    "all":     (0, 0, 1024, 600),
    "topbar":  (0, 0, 1024, 49),
    "main":    (1, 49, 1022, 402),
    "plot":    (13, 58, 754, 385),
    "right":   (776, 58, 235, 385),
    "metrics": (787, 69, 213, 218),
    "chips":   (787, 317, 213, 115),
    "ctrl":    (1, 451, 1022, 92),
    "nav":     (1, 543, 1022, 56),
    # LOCK 页是左右两栏，右栏起点一样
    "left2":   (13, 58, 754, 385),
    # 各页右栏 / 绘图区的常用放大块
    "frame":   (0, 0, 1024, 4),
}


def load(path):
    if not os.path.exists(path):
        sys.exit("缺少 %s" % path)
    return Image.open(path).convert("RGB")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    page = sys.argv[1].lower()

    if len(sys.argv) == 2:
        rname, box = "metrics", REGIONS["metrics"]
    elif len(sys.argv) == 3:
        rname = sys.argv[2]
        if rname not in REGIONS:
            sys.exit("未知区域 %r，可选: %s" % (rname, "/".join(REGIONS)))
        box = REGIONS[rname]
    elif len(sys.argv) == 6:
        # 手动区域要带上坐标命名，否则连续裁好几块会互相覆盖（踩过）
        rname = "x%d_y%d_w%d_h%d" % tuple(int(v) for v in sys.argv[2:6])
        box = tuple(int(v) for v in sys.argv[2:6])
    else:
        sys.exit(__doc__)

    x, y, w, h = box
    ref = load(os.path.join(REF_DIR, "ref_%s.png" % page)).crop((x, y, x + w, y + h))
    brd = load(os.path.join(BOARD_DIR, "board_%s.png" % page)).crop((x, y, x + w, y + h))

    # 上下叠放：参考稿在上，板子在下，中间一条标记线
    pad, bar = 6, 16
    canvas = Image.new("RGB", (w, h * 2 + bar + pad * 2), (24, 24, 28))
    canvas.paste(ref, (0, pad))
    canvas.paste(brd, (0, pad + h + bar))

    d = ImageDraw.Draw(canvas)
    d.text((4, h + pad + 1), "▲ ref (HTML proto)   ▼ board (LVGL)", fill=(255, 255, 255))

    # 小区域放大看清楚
    if w <= 520 and h <= 200:
        s = 2
        canvas = canvas.resize((canvas.width * s, canvas.height * s), Image.NEAREST)

    os.makedirs(CMP_DIR, exist_ok=True)
    out = os.path.join(CMP_DIR, "cmp_%s_%s.png" % (page, rname))
    canvas.save(out)
    print("%s  <%s %d,%d %d x %d>  %s" % (out, rname, x, y, w, h, canvas.size))


if __name__ == "__main__":
    main()
