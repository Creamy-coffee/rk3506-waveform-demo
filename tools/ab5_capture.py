#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AB5 独占接收抓包（无第三方依赖，板端可直接跑）

用途：在没有 pyserial 的目标板上，按 FPGA 侧要求做
      「单读者 + 带时间戳」的 30 秒连续记录。

输出（--out 目录）：
  raw.bin        原始字节（含无法解析的部分）
  health.jsonl   约每秒一行：字节增量、有效帧数、CRC/头错误
  frames.jsonl   每个 CRC 正确帧：类型、偏移、到达时间、fid、pts/dec、间隔
  summary.json   总体统计

用法：
  python3 ab5_capture.py --port /dev/ttyS1 --baud 2000000 --seconds 30 --out /data/live01
"""
import argparse, json, os, struct, sys, time, zlib

MAGICS = [(b'V32CAP3!', 'wave'), (b'V32MEAS!', 'meas'),
          (b'V32SPEC!', 'spec'), (b'V32IQ01!', 'iq')]


def crc32(b):
    return zlib.crc32(b) & 0xFFFFFFFF


def frame_len(kind, buf, off):
    """返回该帧总长；信息不足或不合法返回 None"""
    if kind == 'wave':
        if len(buf) - off < 44:
            return None
        pts = struct.unpack_from('<I', buf, off + 24)[0]
        if pts == 0 or pts > 4096 or (pts & (pts - 1)):
            return None
        return 44 + 4 * pts
    if kind == 'meas':
        return 128
    if kind == 'spec':
        return 2096
    if kind == 'iq':
        return 1068
    return None


def describe(kind, buf, off):
    """按协议解出关键字段"""
    d = {}
    if kind == 'wave':
        d['version'] = struct.unpack_from('<I', buf, off + 8)[0]
        d['frame_id'] = struct.unpack_from('<I', buf, off + 12)[0]
        d['fs_hz'] = struct.unpack_from('<I', buf, off + 16)[0]
        d['decimation'] = struct.unpack_from('<I', buf, off + 20)[0]
        d['points'] = struct.unpack_from('<I', buf, off + 24)[0]
        st = struct.unpack_from('<Q', buf, off + 32)[0]
        d['start_tick'] = st
    elif kind == 'meas':
        d['wave_frame_id'] = struct.unpack_from('<I', buf, off + 16)[0]
        d['result_id'] = struct.unpack_from('<I', buf, off + 20)[0]
        d['snapshot_tick'] = struct.unpack_from('<Q', buf, off + 24)[0]
        d['fs_hz'] = struct.unpack_from('<I', buf, off + 32)[0]
        d['result_age_ticks'] = struct.unpack_from('<I', buf, off + 36)[0]
        d['active_fcw'] = struct.unpack_from('<I', buf, off + 40)[0]
        d['amp0_q12'] = struct.unpack_from('<I', buf, off + 44)[0]
        d['amp1_q12'] = struct.unpack_from('<I', buf, off + 48)[0]
        d['pdiff_millideg'] = int(struct.unpack_from('<h', buf, off + 60)[0] * 360000 / 65536)
        d['qualified'] = buf[off + 62]
        d['detected'] = buf[off + 63]
        d['status'] = struct.unpack_from('<I', buf, off + 64)[0]
        d['flags'] = buf[off + 68]
        d['afc_state'] = buf[off + 69]
        d['afc_reason'] = buf[off + 70]
        d['guard0'] = buf[off + 72]
        d['guard1'] = buf[off + 73]
        d['usable'] = d['status'] & 1
    elif kind == 'spec':
        d['nfft'] = struct.unpack_from('<I', buf, off + 24)[0]
        d['bins'] = struct.unpack_from('<I', buf, off + 28)[0]
        d['fs_eff_hz'] = struct.unpack_from('<I', buf, off + 32)[0]
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, default=2000000)
    ap.add_argument('--seconds', type=float, default=30)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    # stty 必须在打开前设好；板端用 ioctl 太啰嗦，直接调用 stty
    os.system('stty -F %s %d raw -echo -crtscts' % (a.port, a.baud))

    fd = os.open(a.port, os.O_RDONLY | os.O_NONBLOCK)
    f_raw = open(os.path.join(a.out, 'raw.bin'), 'wb')
    f_frm = open(os.path.join(a.out, 'frames.jsonl'), 'w')
    f_hlt = open(os.path.join(a.out, 'health.jsonl'), 'w')

    buf = bytearray()
    t0 = time.time()
    total = 0
    last_sec = 0
    sec_bytes = 0
    stat = {'bytes': 0, 'frames': {'wave': 0, 'meas': 0, 'spec': 0, 'iq': 0},
            'crc_err': 0, 'hdr_err': 0, 'discarded': 0}
    last_wave = None
    t_start_iso = time.strftime('%Y-%m-%dT%H:%M:%S')

    import select
    while time.time() - t0 < a.seconds:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                chunk = b''
            if chunk:
                buf.extend(chunk)
                f_raw.write(chunk)
                total += len(chunk)
                sec_bytes += len(chunk)

        # 逐帧切分
        while True:
            best = None
            for m, k in MAGICS:
                i = buf.find(m)
                if i >= 0 and (best is None or i < best[0]):
                    best = (i, m, k)
            if best is None:
                if len(buf) > 8:                    # 留住可能的半个魔数
                    stat['discarded'] += len(buf) - 8
                    del buf[:len(buf) - 8]
                break
            off, m, kind = best
            if off > 0:
                stat['discarded'] += off
                del buf[:off]
            ln = frame_len(kind, buf, 0)
            if ln is None or len(buf) < ln:
                break                                # 等信息更全
            body, want = bytes(buf[:ln - 4]), struct.unpack_from('<I', buf, ln - 4)[0]
            ok = crc32(body) == want
            now = time.time() - t0
            rec = {'type': kind, 't': round(now, 6), 'crc': bool(ok)}
            rec.update(describe(kind, buf, 0))
            if ok:
                stat['frames'][kind] += 1
                if kind == 'wave':
                    rec['interval_s'] = None if last_wave is None else round(now - last_wave, 6)
                    last_wave = now
            else:
                stat['crc_err'] += 1
            f_frm.write(json.dumps(rec, ensure_ascii=False) + '\n')
            del buf[:ln]

        sec = int(time.time() - t0)
        if sec != last_sec:
            f_hlt.write(json.dumps({'t': sec, 'bytes_delta': sec_bytes,
                                    'frames': dict(stat['frames']),
                                    'crc_err': stat['crc_err'],
                                    'discarded': stat['discarded']}) + '\n')
            f_hlt.flush()
            last_sec = sec
            sec_bytes = 0

    os.close(fd)
    f_raw.close(); f_frm.close(); f_hlt.close()

    summary = {
        'port': a.port, 'baud': a.baud,
        'started': t_start_iso, 'seconds': a.seconds,
        'total_bytes': total, 'avg_bytes_per_s': round(total / max(a.seconds, 1e-9), 1),
        'frames': stat['frames'], 'crc_err': stat['crc_err'],
        'discarded_bytes': stat['discarded'], 'pending_bytes': len(buf),
    }
    open(os.path.join(a.out, 'summary.json'), 'w').write(json.dumps(summary, indent=2, ensure_ascii=False))
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
