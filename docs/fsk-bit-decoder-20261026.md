# 已知符号率2FSK bit判决与实机验收

本版在Linux处理现有V32ANA1! mode3/kind2有符号频偏Hz，FPGA协议/波特率/点数不变，不需为本功能更换bit。保留原始ADC和频偏波形。与OOK共用定时/滞回/中心积分核心，避免复制两套同步逻辑；模式变化会清状态，OOK仍仅接受mode5/kind1。

## 页面

2FSK新增与OOK相同的解码开关、人工符号率100/250/500/1000/2000Bd、0101测试开关，都是本地操作，不重配FPGA。两个制式共用这些本地设置；切换制式清解码状态，不能把一种制式的bit累到另一种。

映射为**低频=0、高频=1**，频偏可能同为正或同为负，不能用频偏正负号直接判bit。L/H和thr在FSK下单位Hz，均相对DDC中心；实际两频率约为DDC中心加L/H，并非独立精密频率测量。每行20bit显示最近40bit，n/?与0101 E=errors/checks含义沿用OOK。人工符号率必须已知，不自动识别。

WAVE_MEASDBG=1时运行实例每秒stderr打印[fsk]，字段与[ook]一致；切回OOK仍打印[ook]。不得另起dump进程取默认本地状态代替运行实例。请只保留一个UART读者，先停S90及守护，再启动唯一GUI并在串口打开后检查FD。

## 算法与限制

- 使用有效频偏样点10/90分位获得低/高频候选。候选间隔至少40Hz，且两个半区内部的分位宽度必须小于间隔的15%，用于拒绝单音噪声/平滑FM；不是自动调制识别或可靠性证明。
- 40%/60%频偏滞回检测边沿，按人工Rs检查整数符号间隔并至少3个合格间隔建立时钟候选，在符号中心附近积分判低/高频。每符号至少6样点；32符号无边沿需重新获取。滤波带宽仍需满足，列表中的Rs不保证所有硬件档位都适用。
- 保留signed24→Hz和bit31无效标记，绝不对signed频偏截成非负。cfg/mode/channel/FCW/R/baud变化或帧号缺口清片段；重复帧不重复计数，flags/无效点中断处理。
- 已知重复0101时才开0101测试，统计相邻bit交替一致性，不是通用BER/协议解码/校验。错误整数倍Rs可能伪同步；固定频率不一定能获取电平；任意长同码、未知Rs、PSK载波恢复不支持。
- 不能根据fsk字样或lock=1直接宣称正确，需要已知源、完整原始包和同核心回放。后续BPSK见[bpsk-bit-decoder-20261026.md](bpsk-bit-decoder-20261026.md)，QPSK仍未输出bit。

## 构建与回放

在现有SDK/LVGL9.1使用新BUILD_DIR构建，备份GUI，确认ARMv7校验后部署。CMake新增fsk_capture_decode；build.sh仍只自动复制GUI，工具须另行从构建目录部署并核对。

```sh
LVGL_DIR=/现有LVGL9.1路径 BUILD_DIR=build_fsk_bits bash build.sh
file ../out/waveform_demo build_fsk_bits/fsk_capture_decode
# 或独立编译，板子部署使用SDK交叉编译器替代gcc
gcc -std=gnu99 -O2 -DFSK_CAPTURE=1 -I src tools/decode_ook_capture.c -lm -o fsk_capture_decode
./fsk_capture_decode capture.bin 1000 1
```

本工具与GUI使用同一判决核心，扫描真实CRC包，输出有效FSK帧数、坏ANA候选、缺口、最后片段bit/?/交替错误检查数、频偏电平、最近64bit。第3参数1仅适用已知交替码，其他数据用0。OOK工具仍默认编译为OOK，保持原调用方式。

## 测试与部署握手

主机需验证：signed两频档偏离零点、滤波/噪声/五起相的0101与合成PN7独立真值逐bit比对，CW/FM/间隔过小拒绝，重复/缺口/无效点/flags；实际CRC→GUI回放、长bit串布局、OOK回归；CLI含坏CRC注入。合成不等于实机通过。

屏端部署后先保持原AM，回复FSK_BITS_READY、准确HEAD/GUI及工具校验、唯一串口读者、后台点击/自动应答停止状态。源窗口另通知后才切2FSK/屏CH1(ADC CH0)/R4096/1000Bd/解码开/0101开，读当前实例[fsk] alt_test=1确认。

拟测源：CH2两频999300与1000700Hz、内部跳频率500Hz（完整交替周期2ms，对应1000Bd）、正弦、原幅度/0偏置。最终以VISA读回与原始流频偏电平/翻转率共同核实，不仅凭设置判定源正确。DDC中心固定约1MHz，记录精确FCW，不能把偏中心自动估计当故障。

静置至少10秒无点击，回传完整[fsk]/[ana]/[rx]日志及当前真实截图（先核对mode3/cfg/mtime，勿旧图）；然后先停GUI/守护独占抓6秒raw.bin，直接配置并核实mode3/kind2/ch0/R4096/中心1MHz，上传未经修改原始流与fsk_capture_decode JSON。RAW和ANA CRC/点数分开，记录帧/有效点和参数。cfg可能与GUI段不同，离线计数不能冒充GUI段。

附件齐后回复阶段号+FSK_BITS_DONE并附文件；最长保持10分钟，收到完成或到期恢复原AM并核验，屏端随通知恢复GUI/AM，回GUI_RUNNING。不要自动轮转其他制式。
