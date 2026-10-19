#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 FPGA 端交接材料打成一个 zip。

用法（waveform_demo/ 目录下）：
    python docs/tools/make_handoff_zip.py
产物：waveform_demo/V32CAP3_FPGA_TX_handoff.zip
"""
import os
import shutil
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))          # docs/tools
DOCS = os.path.dirname(HERE)                               # docs
PROJ = os.path.dirname(DOCS)                               # waveform_demo
SRC_PKG = os.path.join(PROJ, "..", "fpga_v32cap3", "v32_linux_cap3")

STAGE = os.path.join(DOCS, "_stage")
ROOT = os.path.join(STAGE, "V32CAP3_FPGA_TX_handoff")
OUT = os.path.join(PROJ, "V32CAP3_FPGA_TX_handoff.zip")

PLAN = [
    (os.path.join(DOCS, "V32CAP3_FPGA_TX_SPEC.md"), "V32CAP3_FPGA_TX_SPEC.md"),
    (os.path.join(DOCS, "rtl", "v32cap3_tx.v"), "rtl/v32cap3_tx.v"),
    (os.path.join(DOCS, "rtl", "uart_tx.v"), "rtl/uart_tx.v"),
    (os.path.join(HERE, "check_frame.py"), "tools/check_frame.py"),
    (os.path.join(SRC_PKG, "samples", "real_frame_000001.bin"), "samples/real_frame_000001.bin"),
    (os.path.join(SRC_PKG, "samples", "expected.json"), "samples/expected.json"),
    (os.path.join(SRC_PKG, "references", "raw_snapshot_uart.v"), "references/raw_snapshot_uart.v"),
]

README = """V32CAP3 数据帧 —— FPGA 发送端交接包
=====================================

先读这个，再动手：
  V32CAP3_FPGA_TX_SPEC.md          规格书（第 0 节有可直接给 AI 的提示词）

参考实现：
  rtl/v32cap3_tx.v                 组帧状态机（CRC 模式）
  rtl/uart_tx.v                    8N1 发送器，分频四舍五入
  references/raw_snapshot_uart.v   厂商原始 RTL（真实归档用的就是这份，
                                   另含 VOFA 浮点模式，本协议只用 CRC 模式）

对照数据：
  samples/real_frame_000001.bin    真实采集帧，16428 字节，可逐字节比对
  samples/expected.json            该帧的 frame_id / start_index / CRC / 前 16 组样点

自检：
  python tools/check_frame.py <你的输出.bin>
  合格时打印"成功：帧格式完全符合 V32CAP3"，退出码 0。

  先跑这条确认脚本本身正常：
  python tools/check_frame.py samples/real_frame_000001.bin --dump 8

最容易写错的四处（详见规格书 §10）：
  1. 12 位样点必须符号扩展（-13 要发 F3 FF，不是 F3 0F）
  2. CRC 只覆盖前 16424 字节，尾部 4 字节是 ~crc
  3. points_per_channel = 4096，不是 8192
  4. 帧长恒为 16428，多一字节会让下一帧的魔数错位
"""


def main():
    if os.path.isdir(STAGE):
        shutil.rmtree(STAGE)
    os.makedirs(ROOT)

    for src, dst in PLAN:
        dp = os.path.join(ROOT, dst)
        d = os.path.dirname(dp)
        if d:
            os.makedirs(d, exist_ok=True)
        shutil.copyfile(src, dp)
        print("  %-42s %8d B" % (dst, os.path.getsize(dp)))

    with open(os.path.join(ROOT, "README.txt"), "w", encoding="utf-8", newline="\r\n") as f:
        f.write(README)

    if os.path.exists(OUT):
        os.remove(OUT)
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
        for dirpath, _, files in os.walk(STAGE):
            for fn in sorted(files):
                full = os.path.join(dirpath, fn)
                arc = os.path.relpath(full, STAGE).replace("\\", "/")
                z.write(full, arc)

    shutil.rmtree(STAGE)

    print("\n打包完成: %s  (%d B)" % (OUT, os.path.getsize(OUT)))
    with zipfile.ZipFile(OUT) as z:
        for i in z.infolist():
            print("  %-46s %8d B" % (i.filename, i.file_size))


if __name__ == "__main__":
    main()
