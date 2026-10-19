#!/bin/sh
#
# board_tap_test.sh —— 在板子上验证「触摸点击真的能切视图」
#
# 背景：这块板子没有可以自动化的手指，所以要靠 demo 自带的 --tap 往
#       /dev/input/event0 注入一次「按下-保持-抬起」，再用 SIGUSR1 前后各截一张图，
#       比对两张图是否变了。
#
# 前置：板子上已有一个以 --snap-file 启动的 waveform_demo 实例，例如
#       nohup /tmp/waveform_demo --view wave --snap-file /tmp/tap_after.ppm &
#
# 用法：把本文件放到板子上执行，  sh /tmp/board_tap_test.sh [X] [Y]
#
set -u

X=${1:-234}     # 默认点在 FFT 按钮中心
Y=${2:-554}

SNAP=/tmp/tap_after.ppm

# ⚠️ 必须用 pidof，不能用 `ps | grep waveform_demo`：
#    执行本脚本的 sh 进程，它自己的 argv 里就带着 "waveform_demo" 这个字符串，
#    grep 会把它一起匹配上，于是 PID 变成多个值，kill -USR1 会把脚本自己杀掉
#    （表现为脚本中途 exit 127 / 被信号终止）。
PID=$(pidof waveform_demo)
echo "波形应用 PID = ${PID:-未找到}"
[ -n "$PID" ] || { echo "应用没在跑，先启动它"; exit 1; }

rm -f /tmp/before.ppm /tmp/after.ppm "$SNAP"

echo "--- 1) 点击前截图 ---"
kill -USR1 "$PID"
sleep 1.5
[ -f "$SNAP" ] || { echo "点击前截图没生成"; exit 1; }
cp -f "$SNAP" /tmp/before.ppm
ls -l /tmp/before.ppm

echo "--- 2) 注入点击 ($X,$Y) ---"
/tmp/waveform_demo --tap "$X" "$Y" 200
sleep 1.5

echo "--- 3) 点击后截图 ---"
kill -USR1 "$PID"
sleep 1.5
[ -f "$SNAP" ] || { echo "点击后截图没生成"; exit 1; }
cp -f "$SNAP" /tmp/after.ppm
ls -l /tmp/after.ppm

echo "--- 两张图 md5 ---"
if md5sum /tmp/before.ppm /tmp/after.ppm | awk '{print $1}' | uniq | wc -l | grep -q '^1$'; then
    echo "!! 两张图完全一样 —— 点击没有产生任何变化（按钮没被触发）"
    exit 2
else
    echo "OK 两张图不同 —— 点击确实改变了界面"
fi
