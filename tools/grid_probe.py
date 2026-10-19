#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""grid_probe.py —— 从截图里"反解"出栅格：横向/纵向结构线的位置与颜色

为什么要反解而不是照 CSS 推：
    CSS 里的 48px / 92px / 56px 是**行高**，落到屏幕上是「内容 + 边框」的像素分布，
    哪一行是边框、哪一行是内边距，光看 CSS 会算错一格。
    实测反解出来的线位才是唯一可信的基准 —— 参考稿和板子各解一遍，
    逐条对齐，差几像素一目了然。

算法：
    对每一行 y 取全宽中位数颜色（中位数天然忽略文字和波形），
    相邻两行颜色差的 L1 范数就是"这一行是不是分界线"的强度。
    取强度曲线的峰，输出 y、颜色、以及相对上一峰的高度。
    纵向同理。
    中位数而不是均值，是为了不被绘图区里的亮波形带偏。

用法：
    python tools/grid_probe.py ref scope      # 参考稿
    python tools/grid_probe.py board scope    # 板子
    python tools/grid_probe.py both scope     # 两条并排 + 差异
"""
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REF_DIR = os.path.join(ROOT, "out", "ref")
OUT_DIR = os.path.join(ROOT, "out")


def load_img(which, page):
    if which == "ref":
        p = os.path.join(REF_DIR, "ref_%s.png" % page)
    else:
        p = os.path.join(OUT_DIR, "board_%s.png" % page)
    if not os.path.exists(p):
        sys.exit("缺少 %s" % p)
    return np.asarray(Image.open(p).convert("RGB"), dtype=np.float32)


def row_medians(a):
    return np.median(a, axis=1)          # (H,3)


def col_medians(a):
    return np.median(a, axis=0)          # (W,3)


def profile_lines(med, thresh):
    """med: (N,3) 逐行/逐列中位色。返回 [(idx, strength, rgb_below)]，idx 是分界线所在行"""
    d = np.abs(np.diff(med, axis=0)).sum(axis=1)   # 长度 N-1
    out = []
    for i in range(len(d)):
        if d[i] >= thresh:
            # 只保留局部极大，避免一条粗线报成好几条
            lo = max(0, i - 1)
            hi = min(len(d), i + 2)
            if d[i] == d[lo:hi].max():
                out.append((i, float(d[i]), med[i + 1].astype(int).tolist()))
    return out


def hexs(rgb):
    return "#%02x%02x%02x" % tuple(int(v) for v in rgb)


def report(which, page, thresh=18):
    a = load_img(which, page)
    rm = row_medians(a)
    cm = col_medians(a)

    print("-" * 70)
    print("%s_%s.png   尺寸 %dx%d" % (which, page, a.shape[1], a.shape[0]))
    print("  起始行底色 %s" % hexs(rm[0]))

    rows = profile_lines(rm, thresh)
    print("  -- 横向结构线（y: 颜色, 与上一线距）--")
    prev = None
    print("     y=  0  %s  <- 顶" % hexs(rm[0]))
    for (i, s, c) in rows:
        y = i + 1                       # 新色从 y=i+1 开始，分界线在 y=i
        gap = "" if prev is None else "  Δ%-4d" % (y - prev)
        print("     y=%3d  %s  s=%4.0f%s" % (y, hexs(c), s, gap))
        prev = y

    cols = profile_lines(cm, thresh)
    print("  -- 纵向结构线（x）--")
    print("     x=  0  %s" % hexs(cm[0]))
    prev = None
    for (i, s, c) in cols:
        x = i + 1
        gap = "" if prev is None else "  Δ%-4d" % (x - prev)
        print("     x=%3d  %s  s=%4.0f%s" % (x, hexs(c), s, gap))
        prev = x
    return rows, cols


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    which, page = sys.argv[1].lower(), sys.argv[2].lower()

    if which == "both":
        print("=" * 70)
        print("参考稿 (HTML proto)")
        rr, rc = report("ref", page)
        print()
        print("板子 (LVGL)")
        br, bc = report("board", page)
        print()
        print("=" * 70)
        print("横向线对位：ref vs board")
        rf = [i + 1 for i, _, _ in rr]
        bf = [i + 1 for i, _, _ in br]
        n = max(len(rf), len(bf))
        print("  %-6s %-6s %-6s" % ("#", "ref", "board"))
        for i in range(n):
            a = rf[i] if i < len(rf) else None
            b = bf[i] if i < len(bf) else None
            d = "" if (a is None or b is None) else ("  Δ%+d" % (b - a))
            print("  %-6d %-6s %-6s%s" % (i, a, b, d))
    else:
        report(which, page)


if __name__ == "__main__":
    main()
