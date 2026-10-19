#!/bin/bash
# deploy.sh —— 通过 ADB 把编译好的二进制推到板子上运行
#
# 前提：板子已用 USB 连到电脑，并且 `adb devices` 能看到设备。
# 用法：
#   ./deploy.sh              # 正常跑（有触摸）
#   AUTO=--autoswitch ./deploy.sh   # 强制定时轮播波形/FFT
set -e

ADB=${ADB:-adb}
BIN=${BIN:-out/waveform_demo}
TARGET=${TARGET:-/tmp/waveform_demo}
FB=${FB:-/dev/fb0}
AUTO=${AUTO:-}

if [ ! -f "$BIN" ]; then
    echo "找不到二进制: $BIN"
    echo "先在 Ubuntu 上跑 build.sh，把产物拷到 out/ 下"
    exit 1
fi

echo "==> 等待设备"
"$ADB" wait-for-device
"$ADB" devices -l

echo
echo "==> 停掉官方 lv_demo（它占着 framebuffer 和输入设备）"
"$ADB" shell 'for p in lv_demo rk_demo; do killall $p 2>/dev/null; done; /etc/init.d/S10lv_demo stop 2>/dev/null; true'

echo
echo "==> 探测板子环境"
"$ADB" shell 'echo -n "arch   : "; uname -m; echo -n "kernel : "; uname -r; echo "--- fb ---"; ls -l /dev/fb* 2>/dev/null; cat /proc/fb 2>/dev/null; echo "--- input ---"; ls /dev/input/ 2>/dev/null; echo "--- libc ---"; ls -l /lib/libc.so* /lib/ld-* 2>/dev/null'

echo
echo "==> 推送 $BIN -> $TARGET"
"$ADB" push "$BIN" "$TARGET"
"$ADB" shell "chmod +x $TARGET"

echo
echo "==> 运行（Ctrl+C 结束）"
if [ -n "$AUTO" ]; then
    "$ADB" shell "$TARGET $FB $AUTO"
else
    "$ADB" shell "$TARGET $FB"
fi
