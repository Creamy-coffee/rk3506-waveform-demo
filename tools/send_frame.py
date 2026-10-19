#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""send_frame.py —— 往 PC 的某个 COM 口发 V32CAP3 帧，用来验证板子 RX 链路。

用法：
    python tools/send_frame.py COM29 [帧数] [波特率]

做什么：
  以 921600/8N1（或命令行给的第 3 个参数）把 tools/sample_frame.bin 的内容发出去。原始样例帧只有一帧，
  直接重复发会被板子上的解析器按「重复帧号」拒掉（协议要求拒绝重复/重叠帧），
  所以这里会重新生成：
      - frame_id    逐帧 +1
      - start_index 逐帧 +4096（保证样点窗口不重叠）
      - 样点        循环左移 i*7 个采样点（让波形滚动，一眼能看出是实时刷新）
      - CRC32       按新内容重算
"""
import struct
import sys
import time
import zlib

import serial

SRC = "tools/sample_frame.bin"

# 帧号必须**单调递增**跨多次运行 —— 协议要求拒绝重复帧和倒序帧。
# 每次从 124 重新开始的话，第二批的前几帧会被板子正确拒掉，
# 界面上看起来像"丢包"，其实是框架的问题。
STATE = "tools/.last_frame_id"


def make_frame(raw, frame_id, start_index, shift_pairs):
    hdr = bytearray(raw[:40])
    struct.pack_into("<I", hdr, 12, frame_id & 0xFFFFFFFF)
    struct.pack_into("<I", hdr, 32, start_index & 0xFFFFFFFF)
    struct.pack_into("<I", hdr, 36, (start_index >> 32) & 0xFFFFFFFF)

    body = raw[40 : 40 + 16384]
    pairs = [body[i : i + 4] for i in range(0, 16384, 4)]
    k = shift_pairs % len(pairs)
    pairs = pairs[k:] + pairs[:k]

    pkt = bytes(hdr) + b"".join(pairs)
    return pkt + struct.pack("<I", zlib.crc32(pkt) & 0xFFFFFFFF)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM29"
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 921600

    raw = open(SRC, "rb").read()
    if len(raw) != 16428 or raw[:8] != b"V32CAP3!":
        sys.exit("样例帧不对：%d 字节，魔数 %r" % (len(raw), raw[:8]))

    base_id = struct.unpack_from("<I", raw, 12)[0]      # 124
    lo = struct.unpack_from("<I", raw, 32)[0]
    hi = struct.unpack_from("<I", raw, 36)[0]
    base_idx = (hi << 32) | lo                          # 1417389169

    # 接着上一次的帧号往下发，保证单调递增
    try:
        last = int(open(STATE).read().strip())
    except Exception:
        last = base_id - 1
    start_id = max(base_id, last + 1)
    start_idx = base_idx + (start_id - base_id) * 4096

    with serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1,
                       timeout=1, write_timeout=5) as sp:
        sp.reset_input_buffer()
        t0 = time.time()
        for i in range(count):
            fid = start_id + i
            idx = start_idx + i * 4096
            pkt = make_frame(raw, fid, idx, i * 7)
            sp.write(pkt)
            sp.flush()
            print("  帧 %d: id=%d start=%d crc=%08x  (%d 字节)"
                  % (i, fid, idx,
                     struct.unpack_from("<I", pkt, 16424)[0], len(pkt)))
            time.sleep(0.04)
        dt = time.time() - t0
    open(STATE, "w").write(str(start_id + count - 1))
    print("发送完成：%d 帧 / %.1f 字节 / %.2f 秒 @ %d Bd（理论下限 %.2f 秒）"
          % (count, count * 16428, dt, baud, count * 16428 * 10 / baud))


if __name__ == "__main__":
    main()
