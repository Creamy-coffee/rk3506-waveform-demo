#!/bin/sh
#
# 波形显示 demo —— 开机自启动服务
#
#   程序位置：/usr/bin/waveform_demo
#   本脚本  ：/etc/init.d/S90waveform_demo
#   日志    ：/tmp/waveform_demo.log   （tmpfs，不磨损 flash）
#
# 为什么排在 S90：
#   1) 必须晚于 S10udev —— 程序启动时要打开 /dev/input/event0（goodix 触摸），
#      这个节点是 udev 建出来的。放在 pre_init 里会拿不到触摸设备。
#   2) 命名不能和 S99lvgl-test 冲突 —— 那个脚本只在插 U 盘时才 killall，
#      而且 kill 的是 lv_demo / rk_demo / lv_rk3506_evb_test，不含本程序。
#
# 为什么必须停掉厂商的 lv_demo：
#   /etc/init.d/pre_init/S10lv_demo 里是 `lv_demo &`，它在 rcS.early 阶段先跑，
#   会占住 /dev/dri/card0 的 DRM master。后启动的进程拿不到 master 就画不上屏
#   （本机没有 fbdev，只能走 DRM）。
#   已把该脚本改名为 pre_init/lv_demo.disabled。
#   ⚠️ 不能命名成 S10lv_demo.bak —— rcS.early 的 glob 是 `S??*`，
#      只要还是「S + 两位 + 任意」就照样会被执行。
#
# 卸载方法：
#   rm /etc/init.d/S90waveform_demo
#   mv /etc/init.d/pre_init/lv_demo.disabled /etc/init.d/pre_init/S10lv_demo   # 恢复厂商例程
#   rm /usr/bin/waveform_demo

DAEMON=/usr/bin/waveform_demo

# 串口波特率。改这一行 + 重启服务即可，FPGA 那边必须改成同一个值。
#   921600  一帧 16428 B 要 178 ms -> 约 5.6 帧/秒
#   2000000 一帧 16428 B 要  82 ms -> 约 12 帧/秒
# 开机时如果读不到这个文件就按 921600 走。
BAUD_FILE=/etc/waveform_demo.baud
BAUD=$(cat "$BAUD_FILE" 2>/dev/null)
[ -n "$BAUD" ] || BAUD=921600

# --source uart：走 V32CAP3 真实帧（/dev/ttyS1 = JP4 第 10 脚 UART1_RX）。
# 串口打不开时界面就是一条零线（不会退回演示数据）。
ARGS="--source uart --view scope --baud $BAUD"
PIDFILE=/tmp/waveform_demo.pid
LOGFILE=/tmp/waveform_demo.log

is_running() {
    [ -f "$PIDFILE" ] || return 1
    _pid=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$_pid" ] || return 1
    kill -0 "$_pid" 2>/dev/null
}

start() {
    if is_running; then
        echo "waveform_demo 已在运行 (pid $(cat $PIDFILE))"
        return 0
    fi

    if [ ! -x "$DAEMON" ]; then
        echo "错误: $DAEMON 不存在或不可执行"
        return 1
    fi

    # 清掉所有可能抢 DRM master 的实例（包括厂商例程和上一轮的残留）
    killall -9 waveform_demo lv_demo lv_rk3506_evb_test 2>/dev/null
    sleep 1

    echo "启动 waveform_demo ..."
    # 外层 while 负责崩溃后自动拉起；日志写 /tmp（tmpfs），不磨损 flash
    #
    # ⚠️ 末尾那组 </dev/null >>"$LOGFILE" 2>&1 **不能省**。
    #    只给 $DAEMON 单条命令重定向是不够的 —— 后台子 shell 本身仍然继承着
    #    调用者的 stdin/stdout。开机时调用者是 rcS，它是**串行 wait** 的：
    #    只要这个子 shell 还攥着 rcS 的 stdout 管道没关，rcS 就永远等不到 EOF，
    #    整台机器卡在 S90waveform_demo 这一步，后面的 S99* 全都不跑了。
    #    （用 adb shell 手动 start 时现象一样：命令挂住，最后被 SIGTERM 杀掉。）
    (
        while true; do
            $DAEMON $ARGS >>"$LOGFILE" 2>&1
            echo "[S90waveform_demo] 程序退出 rc=$? ，3 秒后自动重启" >>"$LOGFILE"
            sleep 3
        done
    ) </dev/null >>"$LOGFILE" 2>&1 &
    echo $! > "$PIDFILE"
    echo "已启动 (watchdog pid $(cat $PIDFILE))"
    return 0
}

stop() {
    if [ -f "$PIDFILE" ]; then
        _pid=$(cat "$PIDFILE" 2>/dev/null)
        [ -n "$_pid" ] && kill "$_pid" 2>/dev/null
        rm -f "$PIDFILE"
    fi
    # watchdog 死了，子进程可能还活着，单独清一遍
    killall -9 waveform_demo 2>/dev/null

    # ⚠️ 还要清掉**历史遗留的 watchdog**。
    #    多次 start（中间没 stop）会攒下好几个 watchdog —— pidfile 只记最后一个，
    #    上面那句 kill 只干掉一个，剩下的会在 3 秒后把实例拉起来。
    #    两个实例同时抢 /dev/ttyS1，各拿一半字节，谁都拼不出 16428 字节的完整帧，
    #    界面就变成一条零线（看起来像"波形没了"）。
    #    这里按命令行匹配清，跳过自己（$$ 就是本次调用 stop 的那个 shell）。
    for _p in $(ps | grep S90waveform_demo | grep -v grep | awk '{print $1}'); do
        [ "$_p" = "$$" ] && continue
        kill -9 "$_p" 2>/dev/null
    done
    echo "已停止 waveform_demo"
    return 0
}

case "$1" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    restart)
        stop
        sleep 1
        start
        ;;
    status)
        if is_running; then
            echo "running (watchdog pid $(cat $PIDFILE))"
        else
            echo "stopped"
        fi
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac

exit $?
