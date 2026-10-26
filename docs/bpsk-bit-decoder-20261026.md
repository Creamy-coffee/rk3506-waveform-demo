# 已知符号率BPSK恢复与验收

Linux处理现有V32ANA1! mode6/kind0的signed I/Q，不修改FPGA协议或bit。kind0高16位为有符号Q，bit31是Q符号位，**不是无效标记**。QPSK(mode4)仍只观察原始I/Q，不输出bit。

## 实现

1. 检查CRC后帧、flags、完整256点、有效能量、配置和帧连续性；同帧不重复处理。
2. 用首1024个I/Q样点，沿用项目的Hann窗M次方4096点FFT估计BPSK粗频偏与相位；初始搜索范围约|CFO|<=min(Rs,0.225Fs_IQ)，不保证边界内所有信号可获取。
3. 相位连续粗频偏补偿，liquid-dsp 1.7.0的msresamp重采样到4Rs，再由symtrack做AGC、符号定时、载波相频跟踪及均衡。固定库源码随仓库提供，静态链接，不需要额外动态库。
4. 至少256个恢复符号预热；最近64个候选符号的判决参考EVM<0.25且正负符号/转换足够时报告同步候选，排除恒定单音获取。弱/不明确符号记?，不强迫判bit。EVM是用于本算法资格的归一化参考距离，不是通信标准校准EVM仪表。
5. 正实部=1、负实部=0，INV可全局取反。BPSK固有180度相位歧义，未提供已知参考时不能证明绝对0/1极性或协议起点。

每符号6..256个输入样点；人工指定Rs（100/250/500/1000/2000Bd），没有自动识别制式/Rs。cfg/channel/FCW/R/Rs/INV/PN开关变化、缺帧、flags/无效点会重置相应接收片段；缺口不能拼成连续bit。长期同码/CW不提供足够定时信息，可能失去资格，不是任意码流/通用接收机保证。

n统计当前配置片段中可靠判决的bit；?含不可靠符号或失去同步的事件，并不等于失锁期间丢失的真实bit数量。失锁会中断PN7参考并重新获取，含?的记录不能当完整连续码流或用来算BER。

## 页面与保留功能

BPSK显示解码开关、人工Bd、PN7开关。上图仍保留真实I/Q时间波形，可用原有波形按钮切到原始ADC。右侧分析帧标题行增加：
- `INV 0/1`：人工极性选择，改变后重新获取，不能消除没有参考时的物理歧义。
- `SYM / RAW IQ`：下图切恢复符号点/原始基带样点，等比例坐标。SYM未获资格时清图，绝不拿原始环形散点冒充恢复星座。

右栏有粗CFO估计、EVM、n/?、最近40bit。CFO~是初始粗估计，不是最终残余频偏/精密频率测量。SYM点为AGC归一化符号，与原始DDC码RMS不是同一单位。

WAVE_MEASDBG=1每秒输出[bpsk]：cfg/fid/ch/fcw/R/baud/lock/n/erasures/CFO_coarse/evm/invert/pn_test/pn_locked/pn_errors/pn_checks/pn_dir/pn_invert/recent。读当前运行实例，不另启dump获取默认开关状态；只保留一个UART读者，避免守护与手动进程争读。

## PN7参考检查，不是BER

仅当发生器明确输出PN7时才开启PN7。参考为PRBS7 x7+x6+1族，允许循环起相、正/反序及全局取反；至少256个已判符号获取参考，获取过程不计入随后独立的pn_checks。pn_locked=1后比较后续符号，错误/检查次数不能叫通用BER，不能验证协议起点。

pn_dir、pn_invert明确报告所选假设，不会偷偷改变显示的原始判决bit。INV是人工选择，PN7检查允许整体反相；未提供参考不宣称绝对极性。PN7 acquiring和checks0不表示零误码。任意数据要关PN7，仍可观察判决和不确定项。

## 构建和离线工具

固定liquid源码约1.3MB在third_party，配置时核验SHA并离线解包；不联网取依赖。用原SDK/LVGL9.1的新构建目录，旧65e113c GUI备份后再部署。不能把主机x86工具推成ARM程序。源码与库许可证保留，精度不为节省资源降低。

```sh
LVGL_DIR=/现有LVGL9.1路径 BUILD_DIR=build_bpsk_bits bash build.sh
file ../out/waveform_demo build_bpsk_bits/bpsk_capture_decode
# 工具留在构建目录，build.sh仅自动复制GUI，需另外部署并校验
./bpsk_capture_decode raw.bin 1000 1 0 recovered_bits.txt
# 参数为文件、已知Rs、PN7开关、人工INV、可选全bit导出路径
```

缺口在导出文件里用# segment分段，?是不可判项，不跨缺口伪造连续序列。JSON记录当前最后连续片段的bit/PN7检查/参考方向极性及EVM/粗CFO。主机同核心CLI可处理原始bin，不强制占板上存储空间。

## 验证与实机约定

先验证频偏/载波初相/分数样点与滤波噪声、PN7获取后的独立检查、重复/缺口/flags/CW拒绝；另用独立递推+127周期+127个不同7bit状态审核导出串，不能仅信内部PN7模板。再跑实际CRC解析→队列→GUI/ASan/UBSan，保留OOK/FSK/QPSK回归。历史抓包回放通过不等于新一轮实屏部署通过。

屏端部署后回复BPSK_BITS_READY，附准确HEAD、GUI和bpsk_capture_decode校验、唯一串口读者及后台自动点击停的证据。先保持原AM，收到本端源窗口再切BPSK/屏CH1(ADC CH0)/R4096/1000Bd/解码开/PN7开，INV0，SYM视图。用当前[bpsk]确认pn_test1、pn_locked1后至少10秒不操作，回完整日志和真实新截图。

拟用CH2=1MHz/BPSK/内部PN7/1000bit/s/相位0与180度/原幅度/0偏置，最终以VISA读回和实际包共同确认。之后先停GUI与守护，独占抓6秒raw.bin，直接配置核实mode6/kind0/ch0/R4096/DDC1MHz；运行同核心工具，上传原始流、JSON及bit导出串供本端独立审核。GUI与raw的cfg/计数分开。

全部附件齐才回阶段号+BPSK_BITS_DONE并附文件，最长保持10分钟，不自动切其他制式；结束恢复原AM并核验，屏端恢复唯一GUI/AM后回GUI_RUNNING。无新FPGA bit要求，若发现硬件数据不连续另用证据定位。
