#!/usr/bin/env python3
"""Generate a real image payload and a Manchester arbitrary-modulation envelope.

No instrument output is changed. CSV is NOT renamed to proprietary .bsv.
"""
from pathlib import Path
import argparse,hashlib,json,struct,zlib

def demo_image(w=32,h=32):
    rows=bytearray(((w+7)//8)*h)
    for y in range(h):
        for x in range(w):
            on=x in (0,w-1) or y in (0,h-1) or (4<=x<28 and 4<=y<28 and
                 (abs(x-y)<=1 or abs((w-1-x)-y)<=1 or (x//4+y//4)%2==0 and 11<=y<=20))
            if on:rows[y*((w+7)//8)+x//8]|=1<<(7-x%8)
    return bytes(rows),w,h

def load_pbm(path):
    # P4, one raw byte-aligned row per line; only two header dimensions.
    data=Path(path).read_bytes();at=0;tokens=[]
    while len(tokens)<3:
        while at<len(data) and data[at] in b' \t\r\n':at+=1
        if data[at:at+1]==b'#':
            at=data.index(b'\n',at)+1;continue
        end=at
        while end<len(data) and data[end] not in b' \t\r\n':end+=1
        tokens.append(data[at:end]);at=end
    assert tokens[0]==b'P4','Use binary PBM/P4 (or omit input for the built-in test image)'
    w,h=map(int,tokens[1:]);assert 1<=w<=128 and 1<=h<=128
    assert data[at:at+1] in (b'\n',b' ','\t',b'\r')
    at+=2 if data[at:at+2]==b'\r\n' else 1
    pixels=data[at:];assert len(pixels)==((w+7)//8)*h
    return pixels,w,h

def make_packets(pixels,w,h,image_id):
    crc=zlib.crc32(pixels);out=[]
    for i,at in enumerate(range(0,len(pixels),32)):
        payload=pixels[at:at+32]
        header=struct.pack('<8sBBHHHIIIHH',b'V32IMG1!',1,1,w,h,32,image_id,crc,len(pixels),i,len(payload))
        assert len(header)==32
        packet=header+payload.ljust(32,b'\0')
        out.append(packet+struct.pack('<I',zlib.crc32(packet)))
    return out

def symbols_for(packets):
    symbols=bytearray()
    for packet in packets:
        for byte in b'\x55'*32+packet:
            for i in range(7,-1,-1):
                bit=(byte>>i)&1;symbols.extend((bit,1-bit))
    return bytes(symbols)

def generate(out,pbm=None,baud=1000):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    pixels,w,h=load_pbm(pbm) if pbm else demo_image()
    packets=make_packets(pixels,w,h,0x20261027)
    symbols=symbols_for(packets)
    (out/'image.pbm').write_bytes(f'P4\n{w} {h}\n'.encode()+pixels)
    (out/'image.raw').write_bytes(pixels)
    (out/'packets.bin').write_bytes(b''.join(packets))
    (out/'symbols.bin').write_bytes(symbols)
    # Eight held samples per symbol. Vendor conversion/import must preserve
    # the full sample count and this full-loop frequency, not use baud as Fmod.
    with (out/'modulation.csv').open('w',newline='') as f:
        for bit in symbols:
            for _ in range(8):f.write('1\n' if bit else '-1\n')
    record=dict(width=w,height=h,bytes=len(pixels),packets=len(packets),symbols=len(symbols),
                line_baud=baud,net_bit_rate=baud/2,loop_seconds=len(symbols)/baud,
                modulation_loop_hz=baud/len(symbols),arb_points=len(symbols)*8,
                image_crc32=f'{zlib.crc32(pixels):08x}',sha256=hashlib.sha256(pixels).hexdigest(),
                source_changed=False,physical_tested=False,
                bsv_status='Not generated: requires verified vendor BSV conversion/import')
    (out/'manifest.json').write_text(json.dumps(record,indent=2)+'\n')
    return record

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--pbm',type=Path)
    p.add_argument('--baud',type=int,default=1000);a=p.parse_args();assert a.baud>0
    print(json.dumps(generate(a.out,a.pbm,a.baud),indent=2))
