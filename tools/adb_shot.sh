#!/bin/bash
# adb_shot.sh —— 通过 adb 给板子拍一张当前屏幕（SIGUSR1 -> /tmp/wave_frame.ppm -> PNG）
#
# 为什么不用 screencap：板子的 Buildroot 里没有这个命令，也没有 surfaceflinger。
# drm_disp.c 里留了 SIGUSR1 处理，收到就把当前 framebuffer 写成 P6 PPM。
#
# 用法：  ./tools/adb_shot.sh out/shot.png
set -e
cd "$(dirname "$0")/.."

OUT=${1:-out/adb_shot.png}
PPM=out/_adb_shot.ppm
PY=C:/Users/21542/.workbuddy/binaries/python/envs/default/Scripts/python.exe

export MSYS_NO_PATHCONV=1

# ⚠️ 反引号/$( ) 要留在**远程**执行，所以整句用单引号包住
adb shell 'kill -USR1 $(pidof waveform_demo)' >/dev/null
# 写文件 + 转码都在下一帧之后，等一拍
sleep 1
adb pull /tmp/wave_frame.ppm "$PPM" >/dev/null
"$PY" - "$PPM" "$OUT" <<'EOF'
import sys
from PIL import Image
src, dst = sys.argv[1], sys.argv[2]
Image.open(src).save(dst)
print("saved", dst, Image.open(dst).size)
EOF
