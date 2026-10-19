#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_frame.py —— 校验一个（或多个拼接的）V32CAP3 帧是否符合协议。

零依赖，Python 3.6+。用法：

    python check_frame.py sim_frame.bin            # 仿真 dump / 串口抓包
    python check_frame.py capture.bin --dump 32    # 同时打印前 32 组样点

退出码：0 全部合格，1 有问题。

它会逐帧检查：魔数、8 个头部字段的固定值、CRC、样点是否越界、
以及多帧时的 frame_id 递增与 start_index 窗口是否重叠。
"""
import struct
import sys
import zlib

PKT = 16428
HDR = 40
PTS = 4096
MAGIC = b"V32CAP3!"


def check_one(d, idx):
    """检查从 d[idx] 开始的一帧，返回 (ok, 描述, 下一帧偏移)。"""
    p = d[idx:idx + PKT]
    if len(p) < PKT:
        return False, "剩余 %d 字节，不足一帧" % len(p), len(d)

    if p[:8] != MAGIC:
        return False, "魔数错: %s（应为 %s）" % (p[:8].hex(" "), MAGIC.hex(" ")), idx + 1

    version, frame_id, fs, dec, pts, flags, lo, hi = struct.unpack("<8I", p[8:40])

    errs = []
    if version != 3:
        errs.append("version=%d(应为3)" % version)
    if fs != 65000000:
        errs.append("adc_fs_hz=%d(应为65000000)" % fs)
    if dec != 1:
        errs.append("decimation=%d(应为1)" % dec)
    if pts != PTS:
        errs.append("points_per_channel=%d(应为4096)" % pts)
    if flags != 0:
        errs.append("flags=%d(应为0)" % flags)

    crc_want = struct.unpack("<I", p[PKT - 4:])[0]
    crc_calc = zlib.crc32(p[:PKT - 4])
    if crc_want != crc_calc:
        errs.append("CRC 不符: 帧内=0x%08X 实算=0x%08X" % (crc_want, crc_calc))

    # 样点越界检查（能抓到"12 位零扩展"这个经典错误）
    bad = []
    for i in range(PTS):
        a, b = struct.unpack("<hh", p[HDR + 4 * i: HDR + 4 * i + 4])
        if not (-2048 <= a <= 2047):
            bad.append("CH0[%d]=%d" % (i, a))
        if not (-2048 <= b <= 2047):
            bad.append("CH1[%d]=%d" % (i, b))
        if len(bad) >= 3:
            break
    if bad:
        errs.append("样点越界(疑似零扩展): " + ", ".join(bad))

    desc = "frame_id=%-6d start_index=%-12d crc=0x%08X" % (frame_id, (hi << 32) | lo, crc_want)
    if errs:
        return False, desc + "\n    !! " + "\n    !! ".join(errs), idx + PKT
    return True, desc, idx + PKT


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    path = sys.argv[1]
    dump_n = 0
    if "--dump" in sys.argv:
        dump_n = int(sys.argv[sys.argv.index("--dump") + 1])

    d = open(path, "rb").read()
    print("文件 %s，共 %d 字节（%.2f 帧）" % (path, len(d), len(d) / PKT))

    off, n_ok, n_bad = 0, 0, 0
    last_id = last_index = None
    while off < len(d):
        ok, desc, nxt = check_one(d, off)
        if ok:
            n_ok += 1
            fid = struct.unpack("<I", d[off + 12:off + 16])[0]
            hi, lo = struct.unpack("<II", d[off + 32:off + 40])
            si = (hi << 32) | lo
            print("  [OK ] 偏移 %-7d %s" % (off, desc))
            if last_id is not None:
                if fid <= last_id:
                    print("       !! frame_id 未递增（%d -> %d），接收端会丢弃" % (last_id, fid))
                    n_ok -= 1
                    n_bad += 1
                elif si < last_index + PTS:
                    print("       !! start_index 窗口重叠（%d < %d+4096），接收端会丢弃"
                          % (si, last_index))
                    n_ok -= 1
                    n_bad += 1
            last_id, last_index = fid, si
            if dump_n:
                pairs = [struct.unpack("<hh", d[off + HDR + 4 * i: off + HDR + 4 * i + 4])
                         for i in range(min(dump_n, PTS))]
                print("       前 %d 组: %s" % (len(pairs), pairs))
        else:
            n_bad += 1
            print("  [BAD] 偏移 %-7d %s" % (off, desc))
        if nxt <= off:
            nxt = off + 1
        off = nxt

    print("\n结果：合格 %d 帧，问题 %d 处" % (n_ok, n_bad))
    if n_ok and not n_bad:
        print("成功：帧格式完全符合 V32CAP3")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
