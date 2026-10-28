"""Native ASan/UBSan checks for the single UART dispatcher and queued touch input."""
from pathlib import Path
import argparse,subprocess,os,json,hashlib
p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--lvgl',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
r=Path(__file__).resolve().parents[1];out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
archives=list(a.build.resolve().rglob('liblvgl.a'));assert len(archives)==1
files=['src/port.c','src/v32cap3.c','src/v32cap3.h','src/harmonic_receiver.c','src/harmonic_receiver.h','tools/test_touch_reports.c','tools/test_harmonic_dispatch.c']
def hashes():return {f:hashlib.sha256((r/f).read_bytes()).hexdigest() for f in files}
before=hashes();env=os.environ.copy();env.update(ASAN_OPTIONS='detect_leaks=0',UBSAN_OPTIONS='halt_on_error=1')
results={}
for name in ('harmonic_dispatch','touch_reports'):
    cmd=['gcc','-std=gnu99','-O1','-g','-fno-omit-frame-pointer','-ffunction-sections','-fdata-sections','-fsanitize=address,undefined','-I'+str(r/'src')]
    if name=='touch_reports':cmd+=['-DLV_CONF_INCLUDE_SIMPLE','-DLV_CONF_PATH='+str(r/'lv_conf.h'),'-I'+str(a.lvgl.resolve()),str(r/'tools/test_touch_reports.c'),str(archives[0])]
    else:cmd += [str(r/'tools/test_harmonic_dispatch.c'),str(r/'src/harmonic_receiver.c')]
    cmd+=['-Wl,--gc-sections','-lpthread','-lm','-o',str(out/name)]
    with (out/(name+'_compile.log')).open('w') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
    result=subprocess.run([str(out/name)],env=env,capture_output=True,text=True)
    log=result.stdout+result.stderr;(out/(name+'.log')).write_text(log)
    assert result.returncode==0 and '_PASS' in log and 'runtime error:' not in log and 'ERROR: AddressSanitizer' not in log,log
    results[name]=True;print(log.strip())
assert hashes()==before
(out/'validation.json').write_text(json.dumps({'pass':True,'tests':results,'sources':before,'limits':'Native input replay, not physical harmonic accuracy.'},indent=2))
