#!/usr/bin/env python3
from pathlib import Path
import argparse,hashlib,json,os,subprocess
from make_image_demo import generate
p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
root=Path(__file__).resolve().parents[1];build=a.build.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
generate(out)
archives=list(build.rglob('libliquid.a'));assert len(archives)==1,archives
inputs=[root/'src'/s for s in ['image_transport.h','decoded_bit_sink.h','ook_decoder.h','bpsk_decoder.h','bpsk_decoder.c']]
inputs += [root/'tools/test_image_transport.c',root/'tools/make_image_demo.py']
def hashes():return {str(x.relative_to(root)):hashlib.sha256(x.read_bytes()).hexdigest() for x in inputs}
before=hashes()
cmd=['gcc','-std=gnu99','-O1','-g','-fno-omit-frame-pointer','-fsanitize=address,undefined',
     '-Wno-deprecated-declarations','-I'+str(root/'src'),'-I'+str(build/'third_party/liquid-dsp-1.7.0/include'),
     str(root/'tools/test_image_transport.c'),str(root/'src/bpsk_decoder.c'),str(archives[0]),'-lm','-lpthread','-o',str(out/'test_image_transport')]
with (out/'compile.log').open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
assert r.returncode==0,(out/'compile.log').read_text()[-2500:]
env=os.environ.copy();env['ASAN_OPTIONS']='detect_leaks=1';env['UBSAN_OPTIONS']='halt_on_error=1'
with (out/'run.log').open('w') as f:r=subprocess.run([str(out/'test_image_transport'),str(out)],stdout=f,stderr=subprocess.STDOUT,env=env)
log=(out/'run.log').read_text();assert r.returncode==0 and 'IMAGE_TRANSPORT_PASS' in log,log[-3000:]
assert hashes()==before
(out/'validation.json').write_text(json.dumps({'pass':True,'inputs':before,'scope':'Synthetic symbols/baseband through production OOK/2FSK/BPSK DSP; not physical RF acceptance'},indent=2)+'\n')
print(log)
