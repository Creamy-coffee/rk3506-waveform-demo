#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 serial_frame_sender.html 里的组帧逻辑抽出来，用 node 跑一帧落盘。

目的是离线验证：网页生成的帧能否通过 check_frame.py 的协议校验。
用法：
    python tools/test_sender_html.py
产物：
    tools/_test/frame_js.bin   网页逻辑生成的一帧
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HTML = os.path.join(HERE, "serial_frame_sender.html")
OUTDIR = os.path.join(HERE, "_test")
NODE = r"C:\Users\21542\.workbuddy\binaries\node\versions\22.22.2-6\node.exe"

html = open(HTML, encoding="utf-8").read()
m = re.search(r"<script>(.*)</script>", html, re.S)
if not m:
    sys.exit("没找到 <script> 块")
js = m.group(1)

os.makedirs(OUTDIR, exist_ok=True)
harness = os.path.join(OUTDIR, "harness.js")
runner = os.path.join(OUTDIR, "run.js")
open(harness, "w", encoding="utf-8").write(js)

open(runner, "w", encoding="utf-8").write(
    'const fs = require("fs");\n'
    'global.window = global;\n'
    'require("./harness.js");\n'
    'const v = global.__v32;\n'
    'if (!v) { console.error("未导出 __v32"); process.exit(1); }\n'
    'const st = v.st;\n'
    'const opts = { freqHz: 1000000, amp0: 300, amp1: 60, dc: 0,\n'
    '               sigma: 8, spikes: 0, noiseType: "gauss",\n'
    '               frameId: 1234, startIndex: 0x100001234 };\n'
    'const s = v.buildFrame(opts, st);\n'
    'fs.writeFileSync("' + os.path.join(OUTDIR, "frame_js.bin").replace("\\", "/") + '",\n'
    '                 Buffer.from(st.buf));\n'
    'console.log("长度", st.buf.length, "| CH0 Vpp", s.vpp0, "CH1 Vpp", s.vpp1,\n'
    '            "| 饱和", s.sat);\n'
)

r = subprocess.run([NODE, runner], cwd=OUTDIR, capture_output=True, text=True, encoding="utf-8")
sys.stdout.write(r.stdout)
if r.returncode != 0:
    sys.stderr.write(r.stderr)
    sys.exit(r.returncode)

# 再用协议校验器验一遍
chk = os.path.join(HERE, "..", "docs", "tools", "check_frame.py")
pypath = r"C:\Users\21542\.workbuddy\binaries\python\envs\default\Scripts\python.exe"
if os.path.exists(chk):
    r2 = subprocess.run([pypath, chk, os.path.join(OUTDIR, "frame_js.bin")],
                        capture_output=True, text=True, encoding="utf-8")
    sys.stdout.write("\n" + r2.stdout)
    sys.exit(r2.returncode)
