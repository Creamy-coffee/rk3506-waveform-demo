#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diff_shot.py —— 参考稿渲染图 vs 板子截图 的**数值**差异定位

为什么要有它（而不是只看 cmp_shot.py 的上下叠图）：
    叠图靠眼睛看，只能判出"大概齐"，判不出"这行偏了 2px""这块整体暗了 8 个灰阶"。
    1024x600 一共 61 万个像素，肉眼最多覆盖几十个特征点。
    这个脚本把差异量化：整体 RMSE、分区 RMSE、以及**逐行/逐列**的差异曲线，
    用来回答"下一处该改哪里"，而不是"看起来还行"。

关键实现细节：
    1) 先做一次**全局亮度/对比度归一**再比。
       板子是 MIPI 屏 + DRM，参考稿是浏览器渲染，两者灰阶映射不可能完全一致
       （实测板子整体偏暗一档）。如果不归一，所有区域 RMSE 都会被这个整体偏移
       主导，局部结构差异就被淹没了。
       归一方式：按 ref 的 per-channel 均值/标准差线性映射 board，
       只保留"结构性"差异。同时**也输出未归一的原始 RMSE**，两个都看。
    2) 逐行差异用「每行平均绝对差」，然后找连续高于阈值的行段 —— 直接给出
       "y=130..134 这一条有明显差异"这种可操作的定位。
    3) 忽略绘图区**波形本身**。波形是实时动画，相位对不齐会贡献巨大差异，
       但那不是 UI 缺陷。所以分区统计里把 plot 内部单独标出来，并且
       提供 --mask-plot 把它排除掉（默认开）。

用法：
    python tools/diff_shot.py scope            # 全图 + 分区报告
    python tools/diff_shot.py scope --no-mask  # 不排除波形区
    python tools/diff_shot.py all              # 五页一起跑摘要
"""
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REF_DIR = os.path.join(ROOT, "out", "ref")
OUT_DIR = os.path.join(ROOT, "out")
DIFF_DIR = os.path.join(OUT_DIR, "diff")

PAGES = ["scope", "fft", "lock", "ai", "system"]

# 分区定义 —— 与 cmp_shot.py 对齐，另外拆出波形区用于掩蔽
# 分区定义 —— 坐标按**修正后的栅格**给（theme.h 里那套，含 .shell 1px 外框）：
#   外框 0 / 顶栏 1..48 / main 行 49..450（内容 58..442） / controls 451..542 / nav 543..598 / 外框 599
#   绘图区 x 13..766 / 右栏 x 776..1010
# 之前这套框是按"没有外框"的旧栅格算的，整体差 1px 且右栏右侧多出 14px，
# 会让分区 RMSE 凭空偏高（右栏那几个区尤其明显）。
REGIONS = {
    "topbar":  (0, 0, 1024, 49),
    "main":    (1, 49, 1022, 402),
    "plot":    (13, 58, 754, 385),
    "right":   (776, 58, 235, 385),
    "metrics": (787, 69, 213, 218),
    "chips":   (787, 317, 213, 115),
    "ctrl":    (1, 451, 1022, 92),
    "nav":     (1, 543, 1022, 56),
}
# 波形绘制区的"内容"范围（不含外框与标题/脚注），相位不同必差，默认掩掉
PLOT_INNER = (20, 85, 745, 428)

ROW_TOL = 6.0     # 每行平均绝对差超过这个值算"可疑行"
COL_TOL = 6.0


def load(p):
    if not os.path.exists(p):
        sys.exit("缺少 %s" % p)
    return np.asarray(Image.open(p).convert("RGB"), dtype=np.float32)


def normalise(ref, board):
    """把 board 线性映射到 ref 的均值/方差，返回 (board_norm, 参数描述)"""
    desc = []
    out = board.copy()
    for c in range(3):
        rm, rs = ref[..., c].mean(), ref[..., c].std() + 1e-6
        bm, bs = board[..., c].mean(), board[..., c].std() + 1e-6
        scale = rs / bs
        out[..., c] = (board[..., c] - bm) * scale + rm
        desc.append("ch%d x%.3f %+.1f" % (c, scale, rm - bm))
    return np.clip(out, 0, 255), "  ".join(desc)


def rmse(a, b, mask=None):
    d = a - b
    if mask is not None:
        d = d[mask]
        if d.size == 0:
            return float("nan")
    return float(np.sqrt((d ** 2).mean()))


def row_profile(a, b):
    return np.abs(a - b).mean(axis=(1, 2))       # 长度 H


def col_profile(a, b):
    return np.abs(a - b).mean(axis=(0, 2))       # 长度 W


def find_runs(profile, tol, min_len=3):
    """返回连续超过 tol 的区间 [(start, end, peak)]（闭区间下标）"""
    runs = []
    i, n = 0, len(profile)
    while i < n:
        if profile[i] > tol:
            j = i
            while j + 1 < n and profile[j + 1] > tol:
                j += 1
            if j - i + 1 >= min_len:
                runs.append((i, j, float(profile[i:j + 1].max())))
            i = j + 1
        else:
            i += 1
    runs.sort(key=lambda r: -r[2])
    return runs


def mask_from_boxes(shape, boxes):
    m = np.ones(shape[:2], dtype=bool)
    for (x, y, w, h) in boxes:
        m[y:y + h, x:x + w] = False
    return m


def analyse(page, use_mask=True, verbose=True):
    ref = load(os.path.join(REF_DIR, "ref_%s.png" % page))
    brd = load(os.path.join(OUT_DIR, "board_%s.png" % page))
    if ref.shape != brd.shape:
        sys.exit("尺寸不一致 ref %s vs board %s" % (ref.shape, brd.shape))

    brd_n, ndesc = normalise(ref, brd)

    mask = mask_from_boxes(ref.shape, [PLOT_INNER]) if use_mask else None

    raw_all = rmse(ref, brd)
    nrm_all = rmse(ref, brd_n, mask) if mask is not None else rmse(ref, brd_n)

    if verbose:
        print("=" * 78)
        print("PAGE %-8s   归一: %s" % (page, ndesc))
        print("  全图原始 RMSE      : %7.2f   (整体灰阶偏移，不必追)" % raw_all)
        print("  归一后 RMSE%-10s: %7.2f   <== 只看这个"
              % ("(掩波形)" if use_mask else "(含波形)", nrm_all))

        print("  -- 分区（归一后）--")
        bad = []
        for name, box in REGIONS.items():
            x, y, w, h = box
            sub_r = ref[y:y + h, x:x + w]
            sub_b = brd_n[y:y + h, x:x + w]
            if name == "plot" and use_mask:
                sub_m = mask_from_boxes(sub_r.shape,
                                        [(PLOT_INNER[0] - x, PLOT_INNER[1] - y,
                                          PLOT_INNER[2] - PLOT_INNER[0],
                                          PLOT_INNER[3] - PLOT_INNER[1])])
                v = rmse(sub_r, sub_b, sub_m)
            else:
                v = rmse(sub_r, sub_b)
            flag = ""
            if v > 12:
                flag = "  <<< 检查"
                bad.append((name, v))
            elif v > 6:
                flag = "  < 关注"
            print("    %-9s %-22s RMSE %6.2f%s"
                  % (name, "(%d,%d %dx%d)" % (x, y, w, h), v, flag))

        # 逐行定位（排除波形区行）
        if mask is not None:
            m3 = np.repeat(mask[..., None], 3, axis=2)
            rp = np.abs(ref - brd_n).mean(axis=2)
            rp = np.where(mask, rp, np.nan)
            rprof = np.nanmean(rp, axis=1)
            cprof = np.nanmean(rp, axis=0)
        else:
            rprof = row_profile(ref, brd_n)
            cprof = col_profile(ref, brd_n)

        rruns = find_runs(np.nan_to_num(rprof), ROW_TOL)
        cruns = find_runs(np.nan_to_num(cprof), COL_TOL)
        print("  -- 可疑行段 (y, 平均绝对差>%.0f) --" % ROW_TOL)
        if not rruns:
            print("     (无)")
        for (a, b, p) in rruns[:8]:
            print("     y %3d..%-3d  (h=%2d)  peak %.1f" % (a, b, b - a + 1, p))
        print("  -- 可疑列段 (x, 平均绝对差>%.0f) --" % COL_TOL)
        if not cruns:
            print("     (无)")
        for (a, b, p) in cruns[:8]:
            print("     x %3d..%-3d  (w=%2d)  peak %.1f" % (a, b, b - a + 1, p))

    # 存一张放大差异热图，便于肉眼定位
    os.makedirs(DIFF_DIR, exist_ok=True)
    d = np.abs(ref - brd_n).mean(axis=2)
    heat = np.zeros(ref.shape, dtype=np.uint8)
    heat[..., 0] = np.clip(d * 4, 0, 255)          # 差异 -> 红
    heat[..., 1] = np.clip(255 - d * 2, 0, 255)    # 背景 -> 青白
    heat[..., 2] = np.clip(255 - d * 2, 0, 255)
    Image.fromarray(heat).save(os.path.join(DIFF_DIR, "diff_%s.png" % page))

    return nrm_all, raw_all


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    use_mask = "--no-mask" not in sys.argv

    if not args or args[0] == "all":
        print("%-8s %10s %10s" % ("page", "raw", "norm"))
        rows = []
        for p in PAGES:
            n, r = analyse(p, use_mask=use_mask, verbose=False)
            rows.append((p, r, n))
            print("%-8s %10.2f %10.2f" % (p, r, n))
        print("\n按归一后 RMSE 排序（越小越像）:")
        for p, r, n in sorted(rows, key=lambda t: t[2]):
            print("   %-8s %6.2f" % (p, n))
    else:
        analyse(args[0].lower(), use_mask=use_mask)


if __name__ == "__main__":
    main()
