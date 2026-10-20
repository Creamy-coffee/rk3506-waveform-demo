#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_v32_frame.py —— 校验 AB5 三帧（波形 / 锁相 / 频谱）

用法:
    python check_v32_frame.py 抓包.bin

会按顺序识别并逐帧校验：
    V32CAP3!  波形帧（动态长度 = 40 + points*4 + 4）
    V32MEAS!  锁相帧（128 B）
    V32SPEC!  频谱帧（动态长度 = 44 + bins + 4）
每帧都验 CRC32（等价 zlib.crc32）。
"""
import sys, struct, zlib

WAVE = b'V32CAP3!'
MEAS = b'V32MEAS!'
SPEC = b'V32SPEC!'
WIN = {0: 'Hann', 1: 'Blackman', 2: 'FlatTop'}


def u32(b, o):
    return struct.unpack_from('<I', b, o)[0]


def check_wave(d, off):
    pts = u32(d, off + 24)
    pkt = 44 + pts * 4
    if off + pkt > len(d):
        print('  [V32CAP3!] 数据不够一整帧'); return None
    fid, fs, dec = u32(d, off+12), u32(d, off+16), u32(d, off+20)
    ok = (zlib.crc32(d[off:off+pkt-4]) & 0xFFFFFFFF) == u32(d, off+pkt-4)
    print('  [V32CAP3!] fid=%-6d pts=%-5d dec=%-3d fs=%d  %s'
          % (fid, pts, dec, fs, 'CRC OK' if ok else '**CRC 错**'))
    return pkt


def check_meas(d, off):
    if off + 128 > len(d):
        print('  [V32MEAS!] 数据不够 128 字节'); return None
    ok = (zlib.crc32(d[off:off+124]) & 0xFFFFFFFF) == u32(d, off+124)
    wid, st, qual = u32(d, off+16), u32(d, off+64), d[off+62]
    pdi = struct.unpack_from('<h', d, off+60)[0] * 360.0 / 65536
    a1 = u32(d, off+48) / 4096.0
    usable = ((st & 0x1F) == 0x1F) and ((st & 0xF20) == 0) and qual == 3 and d[off+68] == 0
    print('  [V32MEAS!] wid=%-6d usable=%d amp1=%.1f counts pdiff=%.2f° qual=%d status=0x%X  %s'
          % (wid, 1 if usable else 0, a1, pdi, qual, st, 'CRC OK' if ok else '**CRC 错**'))
    return 128


def check_spec(d, off):
    if off + 44 > len(d):
        print('  [V32SPEC!] 数据不够头'); return None
    length = u32(d, off+12)
    if off + length > len(d):
        print('  [V32SPEC!] 数据不够一整帧（length=%d）' % length); return None
    ok = (zlib.crc32(d[off:off+length-4]) & 0xFFFFFFFF) == u32(d, off+length-4)
    wid  = u32(d, off+16)
    nfft = u32(d, off+24)
    bins = u32(d, off+28)
    fs   = u32(d, off+32)
    win  = u32(d, off+36)
    scal = u32(d, off+40)
    expect = 44 + bins + 4
    tag = ''
    if length != expect:
        tag += '  ⚠ length 应为 %d' % expect
    if bins <= 0 or 44 + bins + 4 > len(d):
        print('  [V32SPEC!] bins 非法：%d' % bins); return length
    sp = d[off+44: off+44+bins]
    pk = max(range(bins), key=lambda k: sp[k])
    fpk = pk * fs / float(nfft) if nfft else 0
    pk_db = sp[pk] / 255.0 * scal - scal
    print('  [V32SPEC!] wid=%-6d nfft=%-5d bins=%-5d win=%s 峰值 bin %d = %.3f MHz (%.1f dBFS)  %s%s'
          % (wid, nfft, bins, WIN.get(win, str(win)), pk, fpk/1e6, pk_db,
             'CRC OK' if ok else '**CRC 错**', tag))
    return length


def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    d = open(sys.argv[1], 'rb').read()
    print('文件: %s  (%d 字节)\n' % (sys.argv[1], len(d)))

    off, n = 0, {'wave': 0, 'meas': 0, 'spec': 0}
    while off < len(d) - 8:
        if d[off:off+8] == WAVE:
            r = check_wave(d, off); n['wave'] += 1
        elif d[off:off+8] == MEAS:
            r = check_meas(d, off); n['meas'] += 1
        elif d[off:off+8] == SPEC:
            r = check_spec(d, off); n['spec'] += 1
        else:
            off += 1; continue
        if not r:
            break
        off += r

    print('\n统计: 波形 %d 帧，锁相 %d 帧，频谱 %d 帧' % (n['wave'], n['meas'], n['spec']))
    print('典型一组应为 波形×1 + 锁相×1 + 频谱×1')
    return 0


if __name__ == '__main__':
    sys.exit(main())
