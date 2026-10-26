# 已知符号率 OOK bit 判决（第一版）

本功能在Linux屏端处理现有`V32ANA1!` mode5/kind1包络，不改FPGA协议、不需更换bit。保留原始ADC波形和包络观察。2FSK见[fsk-bit-decoder-20261026.md](fsk-bit-decoder-20261026.md)，后续BPSK见[bpsk-bit-decoder-20261026.md](bpsk-bit-decoder-20261026.md)。QPSK仍不输出bit，不能把单个制式进展推广为所有数字制式已解码。

## 页面与参数

选OOK后出现三个本地控件：`解码 开/关`、`1000Bd`（循环100/250/500/1000/2000）、`0101 开/关`。前两项默认开、1000Bd；0101测试默认关。Bd是**人工指定的符号率**，不是估计值，不会设置发生器或修改FPGA抽取率。三个控件只改本地处理，不触发FPGA重配。

右侧增加符号率、同步候选状态、幅度门限、最近40个bit（每行20个）、当前连续片段的有效bit计数和`?`不确定项。映射固定为高幅度=1、低幅度=0。`0101 E=错误/检查次数`。

启用既有`WAVE_MEASDBG=1`后，运行中的GUI每秒向stderr打印一行`[ook]`：含cfg/fid/ch/fcw/R/baud/lock/n/erasures/alt_test/errors/checks/low/high/thr/recent（最近64bit）。读取正在运行实例的日志，不要另启dump进程冒充当前UI开关/计数；日志不发FPGA命令，也不占用FPGA UART带宽。

`0101`只允许在源确定输出重复交替码时开启：统计相邻有效bit不交替的次数，显示errors/checks。它**不是通用BER、协议校验或已知帧头同步**，不能用于PN7或任意数据。不能从0个相邻错误推出没有丢失偶数个bit；还要核对ANA帧号、有效性和符号计数/时长。

## 算法和边界

- 用有效幅度的10/90分位数学习off/on电平；需要足够分离的开关电平，否则不获取。合法0幅度保留。
- 40%/60%幅度滞回判边沿；用人工符号率检查边沿间距是否接近整数个符号，至少3个相符间隔后建立时钟候选。
- 在符号中心附近的窗口积累幅度，再判0/1；不明确时记`?`，不强制猜bit。
- 每符号至少6个ANA样点。高Rs相对当前Fs/R太高时拒绝获取并提示；这不保证滤波带宽支持所有列出的速率。
- 连续超过32个符号没有边沿则重新获取，因此不是任意长同码串的通用接收机。没有自动识别制式或符号率；错误的整数倍速率也可能满足边沿约束，必须人工给对参数，并通过已知码型验证。
- CRC失败帧不会送入；帧号缺口、cfg/通道/FCW/R/符号率变化会清空解码片段。无效样点中断定时并可记`?`；重复帧不重复产生bit。
- 页面`n`、errors/checks只针对当前连续解码片段，重新配置、丢帧或重启后会清零，不是整场测试累计统计。`L/H`为低/高电平，`thr`为滞回门限，`?`为不确定判决数。关闭解码清空状态；暂停冻结显示，恢复后依帧连续性重新处理。
- 同步指符号时钟候选，不代表协议帧同步。无FEC、协议帧解析、包校验、PSK载波恢复或通用BER功能。

## 编译、部署和离线复核

按交付消息中的精确提交更新`fpga-modulation-ui-20261026`；保存本地改动，有分叉不得强制覆盖。例：

```bash
git fetch origin fpga-modulation-ui-20261026
git switch test/fpga-mod-ui
git merge --ff-only origin/fpga-modulation-ui-20261026
LVGL_DIR=/现有LVGL9.1绝对路径 BUILD_DIR=build_ook_bits bash build.sh
file ../out/waveform_demo build_ook_bits/ook_capture_decode
sha256sum ../out/waveform_demo
```

备份现有屏上程序与`../out/waveform_demo`，确认ARMv7产物后按原流程部署。CMake同时构建`ook_capture_decode`离线工具；`build.sh`仅自动复制GUI到`../out`，工具仍在构建目录，需另行传到屏端（或在Linux主机原生编译）。

```bash
# 使用与页面完全同一src/ook_decoder.h，不靠另一套算法生成理想bit。
./ook_capture_decode capture.bin 1000 1
# 第3个参数1只适用于此次已知0101输入；其他数据必须用0。
```

输出含CRC正确OOK帧、CRC错误ANA候选、帧/配置缺口、最后片段有效bit数、`?`、交替错误/检查次数及最近bit。不要将最后片段计数误写成跨缺口累计BER。工具只读文件，不操作串口或FPGA。

## 验证与实机步骤

主机单元测试已知滤波/噪声/5种起相的0101和合成PN7，比较判决bit与独立真值；同时检查常量CW、错误非整数倍Rs、重复帧、缺口、无效样点、flags和过高速率。GUI需跑真实解析器/LVGL回放、ASan/UBSan和渲染检查；合成通过不等于实机验收。

部署后先保持源原AM，回复`OOK_DECODE_READY`，本端才设：发生器CH2的1MHz载波、500Hz方波100%AM实现等效OOK（每半周期1ms，对应1000bit/s），幅度减半避免提高峰值。屏端选OOK/CH1/R4096/1000Bd/解码开/0101开。

本端每阶段最长保持10分钟，收到完成通知后恢复AM，不自动轮转其他制式。请回原阶段编号含`OOK_BITS_DONE`并附：真实ANA原始流、屏幕图、门限/同步/bit片段/有效bit/不确定项/交替错误检查数、cfg/fid/flags/nvalid、RAW和ANA各自统计。用同一核心离线重放原始流，比对屏端结果；另核对包络时长与约1000bit/s的数量关系。

抓原始流若必须停止GUI，应先直接发OOK配置并核实mode5/kind1/CH0/R4096，避免启动时重配旧模式；记录新cfg。离线与GUI可能是不同片段，计数不能逐字比较，但可在各自片段内对已知交替码检验。最后恢复GUI及原AM显示。
