"""Exercise packet scanning/CRC and the screen's actual OOK core together."""
import json
import math
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path


def packets(count=40):
    frames = []
    filtered = 0.0
    for fid in range(1, count + 1):
        words = []
        for i in range(256):
            t = ((fid - 1) * 256 + i) * 4096 / 65000000
            level = 100 if math.floor(t * 1000 + 0.23) % 2 else 0
            filtered += 0.65 * (level - filtered)
            words.append(round(filtered * 256))
        header = b'V32ANA1!' + struct.pack('<10I', 1, fid, 65000000, 4096,
                                           256, 0, 7, 1 | (5 << 16), 66076420, 0)
        frame = header + struct.pack('<256I', *words)
        frames.append(frame + struct.pack('<I', zlib.crc32(frame)))
    return frames


with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp) / 'capture.bin'
    frames = packets()
    path.write_bytes(b'noise' + b''.join(frames))

    def decode():
        return json.loads(subprocess.check_output([sys.argv[1], str(path), '1000', '1']))

    clean = decode()
    assert clean['CRC_good_OOK_frames'] == 40, clean
    assert clean['CRC_bad_ANA_candidates'] == clean['frame_or_config_gaps'] == 0, clean
    assert clean['locked'] and clean['last_segment_bits'] > 600, clean
    assert clean['erasures'] == clean['alternating_errors'] == 0, clean
    assert all(a != b for a, b in zip(clean['recent'], clean['recent'][1:])), clean
    corrupt = bytearray(frames[10])
    corrupt[100] ^= 1
    path.write_bytes(b''.join(frames[:10]) + corrupt + b''.join(frames[11:]))
    damaged = decode()
    assert damaged['CRC_good_OOK_frames'] == 39, damaged
    assert damaged['CRC_bad_ANA_candidates'] == damaged['frame_or_config_gaps'] == 1, damaged
    assert 400 < damaged['last_segment_bits'] < clean['last_segment_bits'], damaged
    assert damaged['alternating_errors'] == 0, damaged
    print('OOK_CAPTURE_PASS', json.dumps({'clean': clean, 'corrupted': damaged}))
