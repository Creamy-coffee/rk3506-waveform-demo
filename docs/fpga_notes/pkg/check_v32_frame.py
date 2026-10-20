#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_v32_frame.py —— 校验 V32CAP3 抓包

用法:
    python check_v32_frame.py 抓包.bin
    python check_v32_frame.py 抓包.bin --max 10

会打印每帧的字段、抽取倍数、一帧覆盖的时间、CRC 是否正确，
以及两个通道的样点范围。用于确认 FPGA 侧发出来的数据符合协议。
"""
import sys, struct, zlib

MAGIC = b'V32CAP3!'
HDR   = 40
POINTS_DEF = 4096
FS_DEF = 65000000

def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    path = sys.argv[1]
    limit = 10
    if '--max' in sys.argv:
        limit = int(sys.argv[sys.argv.index('--max') + 1])

    data = open(path, 'rb').read()
    print('文件: %s  (%d 字节)' % (path, len(data)))

    pos = []
    p = data.find(MAGIC)
    while p >= 0:
        pos.append(p)
        p = data.find(MAGIC, p + 1)
    print('找到 %d 个帧头\n' % len(pos))
    if not pos:
        print('没有找到 V32CAP3! 帧头 —— 抓的字节不是这个协议，或者数据没对齐')
        return 1

    ok = bad = 0
    for n, off in enumerate(pos[:limit]):
        if off + HDR + 4 > len(data):
            print('帧 #%d 在偏移 %d，但数据不够解析' % (n, off)); continue
        ver, fid, fs, dec, pts, flags, lo, hi = struct.unpack('<8I', data[off+8:off+40])
        start = (hi << 32) | lo
        pkt = HDR + pts * 4 + 4
        line = '帧 #%-3d fid=%-6d ver=%d Fs=%d dec=%d(→%.3f MSPS) pts=%d start=%d' % (
            n, fid, ver, fs, dec, fs / max(dec,1) / 1e6, pts, start)
        print(line)
        span_us = pts * dec / fs * 1e6 if fs else 0
        print('       一帧覆盖 %.1f µs' % span_us)

        if dec < 1 or dec > 256 or (dec & (dec - 1)):
            print('       ⚠ decimation 非法（应为 1..256 的 2 的幂）')
        if fs != FS_DEF:
            print('       ⚠ Fs 应为 %d（ADC 原始采样率，不是抽取后的）' % FS_DEF)
        # 点数：256..4096 的 2 的幂（与主机固件一致，2026-10-20 起支持可变点数）
        if pts < 256 or pts > 4096 or (pts & (pts - 1)):
            print('       ⚠ points 非法：%d（应为 256..4096 的 2 的幂）' % pts)
        if ver != 3:
            print('       ⚠ version 应为 3')

        if off + pkt > len(data):
            print('       （数据不够一整帧，无法校验 CRC）'); continue
        crc = struct.unpack('<I', data[off + pkt - 4: off + pkt])[0]
        calc = zlib.crc32(data[off: off + pkt - 4]) & 0xFFFFFFFF
        if crc == calc:
            ok += 1
            rng = ''
            if pts > 0:
                import array
                a = array.array('h'); a.frombytes(data[off+HDR: off+HDR+pts*4])
                if sys.byteorder != 'little': a.byteswap()
                rng = '   ch0 %d..%d  ch1 %d..%d' % (
                    min(a[0::2]), max(a[0::2]), min(a[1::2]), max(a[1::2]))
            print('       CRC OK' + rng)
        else:
            bad += 1
            print('       CRC 错误：帧内 %08x，计算 %08x' % (crc, calc))

    print('\nCRC 通过 %d 帧，失败 %d 帧' % (ok, bad))
    return 0 if bad == 0 else 2

if __name__ == '__main__':
    sys.exit(main())
