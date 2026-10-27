#!/bin/bash
# push_build.sh —— 一键「推源码到构建机 -> 交叉编译 -> 把产物拉回 out/」
#
# 为什么需要它：
#   本机（Windows）没有交叉工具链，也刻意不放 LVGL 源码（10MB+），
#   编译全在 VMware 里的 Ubuntu（192.168.8.130）上做。
#   手工三件事（sftp_put / ssh build / sftp_get）很容易漏掉某一步，
#   导致「改了代码但跑的还是旧二进制」——所以固化成脚本。
#
# 用法（waveform_demo/ 目录下）：
#   ./push_build.sh            增量编译
#   CLEAN=1 ./push_build.sh    全量重编（LVGL 从头编，约 1 分钟）
#
# 产物：out/waveform_demo
set -e
set -o pipefail   # 管道里 build.sh 失败也要让整个脚本失败，别再把旧产物当成功拉回来
cd "$(dirname "$0")"

BUILD_HOST=${BUILD_HOST:-192.168.8.130}
BUILD_USER=${BUILD_USER:-lqnm}
BUILD_PASS=${BUILD_PASS:-q369258147}
REMOTE_DIR=${REMOTE_DIR:-/home/lqnm/waveform_demo/project}

PYBIN=${PYBIN:-C:/Users/21542/.workbuddy/binaries/python/envs/default/Scripts/python.exe}
[ -x "$PYBIN" ] || PYBIN=python

export SSH_HOST="$BUILD_HOST" SSH_USER="$BUILD_USER" SSH_PASS="$BUILD_PASS"
# ⚠️ 远程路径以 / 开头，不关掉 MSYS 路径转换会被改写成
#    "E:/Program Files/.../home/lqnm/..." 这种鬼东西（踩过）
export MSYS_NO_PATHCONV=1

echo "==> 1/3 推源码 -> $BUILD_USER@$BUILD_HOST:$REMOTE_DIR"
# 本地 tools/ 里的生成脚本和字体也一起推，保证两边可复现
"$PYBIN" ../scripts/sftp_put.py . "$REMOTE_DIR" | tail -3

echo
echo "==> 2/3 交叉编译"
CLEAN_ENV=""
[ "x$CLEAN" = "x1" ] && CLEAN_ENV="CLEAN=1"
printf -v REMOTE_DIR_Q '%q' "$REMOTE_DIR"
printf -v BUILD_DIR_Q '%q' "${BUILD_DIR:-build}"
REMOTE_LVGL_ENV=""
if [ -n "${LVGL_DIR:-}" ]; then
    printf -v LVGL_DIR_Q '%q' "$LVGL_DIR"
    REMOTE_LVGL_ENV="LVGL_DIR=$LVGL_DIR_Q"
fi
"$PYBIN" ../scripts/ssh_run.py - 900 <<EOF
bash -s <<'V32_REMOTE_BUILD'
set -e
set -o pipefail
cd $REMOTE_DIR_Q
$CLEAN_ENV BUILD_DIR=$BUILD_DIR_Q $REMOTE_LVGL_ENV bash ./build.sh 2>&1 | tail -40
V32_REMOTE_BUILD
EOF

echo
echo "==> 3/3 拉回产物"
REMOTE_PROJECT=${REMOTE_DIR%/}
REMOTE_OUT=${REMOTE_OUT:-${REMOTE_PROJECT%/*}/out}
mkdir -p out
for artifact in waveform_demo ook_capture_decode fsk_capture_decode bpsk_capture_decode; do
    "$PYBIN" ../scripts/sftp_get.py "$REMOTE_OUT/$artifact" "out/$artifact"
done

echo
echo "================ 本地产物 ================"
ls -la out/waveform_demo
md5sum out/waveform_demo
