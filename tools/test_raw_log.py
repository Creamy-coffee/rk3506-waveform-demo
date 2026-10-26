#!/usr/bin/env python3
"""Real UART reader via host PTY: log validated 1024 points, not capacity 4096."""
import os,pty,subprocess,struct,zlib,time,json,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
root=Path(__file__).resolve().parents[1]
c=out/'test_raw_log.c';exe=out/'test_raw_log'
c.write_text('''#include "v32cap3.h"
#include "wave_source.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
wave_source_t wave_source_v32={.frame_len=4096,.sample_rate=65000000};
int main(int n,char **v){setbuf(stdout,NULL);setenv("WAVE_MEASDBG","1",1);
if(n!=2||v32_init_uart(v[1],2000000))return 2;puts("READY");sleep(3);v32_shutdown();return 0;}
''')
subprocess.run(['gcc','-std=gnu99','-O1','-I'+str(root/'src'),str(c),str(root/'src/v32cap3.c'),'-lpthread','-lm','-o',str(exe)],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
master,slave=pty.openpty()
with (out/'raw_log_reader.log').open('w') as log:
    proc=subprocess.Popen([str(exe),os.ttyname(slave)],stdout=subprocess.PIPE,stderr=log,text=True)
    assert proc.stdout.readline().strip()=='READY'
    for fid in range(1,27):
        frame=bytearray(4140);frame[:8]=b'V32CAP3!'
        for offset,value in [(8,3),(12,fid),(16,65000000),(20,1),(24,1024),(32,fid*1024)]:struct.pack_into('<I',frame,offset,value)
        struct.pack_into('<I',frame,4136,zlib.crc32(frame[:4136]));at=0
        while at<len(frame):at+=os.write(master,frame[at:])
        time.sleep(.1)
    assert proc.wait(timeout=5)==0
os.close(master);os.close(slave)
text=(out/'raw_log_reader.log').read_text();assert 'fn=1024' in text and 'fn=4096' not in text,text
assert 'CRC错=0' in text,text
(out/'raw_log_validation.json').write_text(json.dumps({'pass':True,'capacity':4096,'actual_points':1024,'transport':'host PTY, actual UART reader thread','crc_errors':0},indent=2))
print('RAW_LOG_TEST_PASS capacity4096 actual_frame1024 crc0')
