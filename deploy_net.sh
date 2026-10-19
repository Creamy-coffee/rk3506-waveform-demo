#!/bin/bash
# deploy_net.sh —— 走**网口**把二进制推到板子上（不碰 USB adb）
#
# 为什么弃用 adb push：
#   这块板子的 USB gadget 会反复重新枚举，adb 经常报
#   「no devices/emulators found」；更糟的时候 Windows 侧连 USB 设备都枚举不到
#   （pnputil 里看不到任何 USB 复合设备），这时 adb kill-server 也救不回来，
#   必须拔插线。而板子网口上的 sshd 一直是稳的，所以部署改走 SSH/SFTP。
#
# 网络拓扑（板子网口直连 PC 网口）：
#   PC   192.168.1.1 /24
#   板子 192.168.1.10 /24   （板子的默认路由网关就是 PC）
#
# 板子 root 密码不是猜的，来自 SDK 的 buildroot defconfig：
#   buildroot/configs/rockchip_hd_rk3506g_evb_nand_defconfig
#   BR2_TARGET_GENERIC_ROOT_PASSWD="root"
#   （用 openssl passwd -1 -salt <shadow里的盐> root 验证过，和板子 /etc/shadow 一致）
#
# 用法（waveform_demo/ 目录下）：
#   ./deploy_net.sh                 # 推 out/waveform_demo 到板子并重启 demo
#   NO_RUN=1 ./deploy_net.sh        # 只推，不启动
#   BOARD_PASS=xxx ./deploy_net.sh  # 换密码
set -e
cd "$(dirname "$0")"

# ⚠️ 必须在调用任何 python 脚本**之前**设好。
#    Git-bash/MSYS 会把参数里以 / 开头的路径「自动翻译」成 Windows 路径：
#      /tmp/waveform_demo  ->  E:/Program Files/WorkBuddy/resources/vendor/PortableGit/tmp/waveform_demo
#    结果 sftp 去写一个根本不存在的目录，报 FileNotFoundError: [Errno 2]，
#    看起来像「板子 /tmp 没了」，其实是本地参数被改了。
#    （同一个坑踩过两次：一次是 sftp_put.py，一次是 board_push.py）
export MSYS_NO_PATHCONV=1

BIN=${BIN:-out/waveform_demo}
DEST=${DEST:-/tmp/waveform_demo}
VIEW=${VIEW:-wave}
BOARD_HOST=${BOARD_HOST:-192.168.1.10}
BOARD_USER=${BOARD_USER:-root}
BOARD_PASS=${BOARD_PASS:-root}
# board_push.py 认 BOARD_*，ssh_run.py 认 SSH_*，两个都导出免得混
export BOARD_HOST BOARD_USER BOARD_PASS
export SSH_HOST="$BOARD_HOST" SSH_USER="$BOARD_USER" SSH_PASS="$BOARD_PASS"

# 需要带 paramiko 的解释器（Windows 侧那个 venv 里有）
PYBIN=${PYBIN:-C:/Users/21542/.workbuddy/binaries/python/envs/default/Scripts/python.exe}
[ -x "$PYBIN" ] || PYBIN=python

[ -f "$BIN" ] || { echo "找不到 $BIN —— 先在编译机上跑 build.sh，把产物拷到 out/"; exit 1; }

echo "==> 推送 $BIN -> $BOARD_USER@$BOARD_HOST:$DEST"
"$PYBIN" ../scripts/board_push.py "$BIN" "$DEST"

if [ "x$NO_RUN" = "x1" ]; then
    echo "==> NO_RUN=1，只推送不启动"
    exit 0
fi

echo "==> 启动 demo（视图=$VIEW）"
# ⚠️ 结尾必须有 </dev/null：后台进程如果继承着 SSH 通道的 stdin，
#    通道不会关闭，ssh_run.py 会在最后读 stderr 时一直卡到超时。
"$PYBIN" ../scripts/ssh_run.py - 60 <<EOF
cd /tmp && nohup ./waveform_demo --view $VIEW > /tmp/app.log 2>&1 </dev/null &
sleep 3
echo "进程: \$(pidof waveform_demo)"
cat /tmp/app.log
EOF

echo
echo "==> 完成，屏幕上是 $VIEW 视图"
echo "    测帧率 : cd ../scripts && BOARD_PASS=$BOARD_PASS $PYBIN ssh_run.py - 120 <<< 'cd /tmp && ./waveform_demo --fps 5'"
