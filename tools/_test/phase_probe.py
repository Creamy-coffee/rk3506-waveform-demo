#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""phase_probe.py —— 发一段「已知相位差」的帧，用来验证板端 PHASE Δ 算得对不对。

CH0 = A*sin(2πf n/fs)，CH1 = A*sin(2πf n/fs + φ)，φ 由命令行给。
板端读数应当≈ φ（CH1 相对 CH0 超前为正）。

用法：
    python tools/_test/phase_probe.py COM29 90 40
"""
import math
import os
import struct
import sys
import time
import zlib

import serial

FS = 65000000
POINTS = 4096
HDR = 40
PKT = 16428
STATE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".last_frame_id")


def frame(fid, start, freq, phi_deg, amp=800):
    buf = bytearray(PKT)
    buf[0:8] = b"V32CAP3!"
    struct.pack_into("<I", buf, 8, 3)
    struct.pack_into("<I", buf, 12, fid & 0xFFFFFFFF)
    struct.pack_into("<I", buf, 16, FS)
    struct.pack_into("<I", buf, 20, 1)
    struct.pack_into("<I", buf, 24, POINTS)
    struct.pack_into("<I", buf, 28, 0)
    struct.pack_into("<I", buf, 32, start & 0xFFFFFFFF)
    struct.pack_into("<I", buf, 36, (start >> 32) & 0xFFFFFFFF)
    w = 2 * math.pi * freq / FS
    ph = math.radians(phi_deg)
    off = HDR
    for i in range(POINTS):
        a = int(round(amp * math.sin(w * i)))
        b = int(round(amp * math.sin(w * i + ph)))
        struct.pack_into("<hh", buf, off, a, b)
        off += 4
    struct.pack_into("<I", buf, PKT - 4, zlib.crc32(bytes(buf[:-4])) & 0xFFFFFFFF)
    return bytes(buf)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    port = sys.argv[1]
    phi = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 40

    try:
        fid = int(open(STATE).read().strip())
    except Exception:
        fid = 1000

    ser = serial.Serial(port, 921600, timeout=2)
    t0 = time.time()
    for i in range(n):
        # start_index 由帧号推出来，保证跨多次运行也单调递增：
        # 板端要求 start_index >= 上一帧 + 4096，写死一个常量第二批就会被整批拒掉。
        start = 0x100000000 + (fid + i) * POINTS
        ser.write(frame(fid + i, start, 1000000.0, phi))
        ser.flush()
    ser.close()
    open(STATE, "w").write(str(fid + n))
    print("已发 %d 帧，相位差设定 %.1f°，用时 %.2f s" % (n, phi, time.time() - t0))


if __name__ == "__main__":
    main()
