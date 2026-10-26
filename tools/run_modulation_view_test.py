#!/usr/bin/env python3
"""Run the real LVGL modulation page on a Linux host, without a screen/UART.

First configure/build a native CMake tree with LVGL 9.1, then run:
python3 tools/run_modulation_view_test.py --build build_host --lvgl ../lvgl-release-v9.1
This does not build or deploy an ARM binary.
"""
from pathlib import Path
import argparse,subprocess,os,hashlib,json
p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--lvgl',type=Path,required=True);p.add_argument('--out',type=Path,default=Path('build_modulation_test'));a=p.parse_args()
root=Path(__file__).resolve().parents[1];build=a.build.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
archives=list(build.rglob('liblvgl.a'));assert len(archives)==1,archives
skip={'main.c.o','ui.c.o','plot.c.o','v32cap3.c.o','wave_source_v32.c.o'}
objects=[x for x in (build/'CMakeFiles/waveform_demo.dir/src').rglob('*.o') if x.name not in skip]
assert len(objects)>=14
sources=[root/'src/ui.c',root/'src/modulation_view.h',root/'src/plot.c',root/'src/v32cap3.c',root/'src/v32cap3.h',root/'src/wave_source_v32.c',root/'src/wave_source.h',root/'tools/test_modulation_view.c']
before={str(x.relative_to(root)):hashlib.sha256(x.read_bytes()).hexdigest() for x in sources}
cmd=['gcc','-std=gnu99','-O1','-g','-fno-omit-frame-pointer','-fsanitize=address,undefined,float-cast-overflow','-DLV_CONF_INCLUDE_SIMPLE','-DLV_CONF_PATH='+str(root/'lv_conf.h'),'-DLV_LVGL_H_INCLUDE_SIMPLE','-I'+str(root/'src'),'-I'+str(a.lvgl.resolve()),'-I'+str(root),str(root/'tools/test_modulation_view.c'),str(root/'src/plot.c'),str(root/'src/v32cap3.c'),str(root/'src/wave_source_v32.c'),*[str(x) for x in objects],str(archives[0]),'-lm','-lpthread','-o',str(out/'test_modulation_view')]
with (out/'compile.log').open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
if r.returncode:raise SystemExit((out/'compile.log').read_text()[-3000:])
env=os.environ.copy();env['ASAN_OPTIONS']='detect_leaks=0';env['UBSAN_OPTIONS']='halt_on_error=1'
with (out/'run.log').open('w') as f:r=subprocess.run([str(out/'test_modulation_view'),str(out)],stdout=f,stderr=subprocess.STDOUT,env=env)
log=(out/'run.log').read_text();assert r.returncode==0 and 'MODULATION_VIEW_PASS' in log and 'runtime error:' not in log and 'ERROR: AddressSanitizer' not in log,log[-3500:]
assert before=={str(x.relative_to(root)):hashlib.sha256(x.read_bytes()).hexdigest() for x in sources}
(out/'validation.json').write_text(json.dumps({'pass':True,'sources':before,'limits':['Native host LVGL synthetic inputs, not RK3506 deployment or physical UART acceptance.']},indent=2))
print(log)
