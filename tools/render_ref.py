#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""render_ref.py —— 把参考稿 HTML 渲染成 1024x600 的 PNG（ground truth）

为什么需要它：
    光靠肉眼看板子截图，判断不了「像不像参考稿」。必须先把参考稿本身
    在 1024x600 下渲染成图，再拿板子截图去叠，才能定量说哪里偏了。

关键手法 —— 冻结动画：
    参考稿里 `tick` 每帧 +1，波形相位 `t = tick*.02`。不冻结的话每次
    截图相位都不一样，两张图没法直接叠（正弦会横向错开）。
    这里在页面尾部再插一段 script 把 running 置 false、tick 定死，
    于是 render() 每帧画的内容完全一致。

    之所以能这么干：`tick` / `running` 是经典 <script> 顶层用 let 声明的，
    位于全局词法环境，同一文档里**后面的** <script> 可以直接读写。
    （不是 window 属性，所以不能写 window.tick。）

用法：
    python tools/render_ref.py                # 渲染全部 5 页
    python tools/render_ref.py scope fft      # 只渲染指定页
产物：
    out/ref/ref_<page>.png
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REF_HTML = os.path.join(ROOT, "out", "ref", "v32_ref.html")
OUT_DIR = os.path.join(ROOT, "out", "ref")

W, H = 1024, 600
PAGES = ["scope", "fft", "lock", "ai", "system"]

# 冻结用的 tick。板子侧要冻结成同一个值（main.c 的 --freeze），否则正弦会错相。
FREEZE_TICK = 200
CLOCK_TEXT = "00:00:07"     # 和板子那次截图对齐，方便比顶栏
FPS_TEXT = "10.0 FPS"

EDGE_CANDIDATES = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
]

INJECT = """
<script>
/* ==== render_ref.py 注入：冻结动画，保证截图可复现 ==== */
(function () {
    /* 顶栏时钟/uptime/帧率是 setInterval 在刷，清掉它们才能定死 */
    for (var i = 1; i < 2000; i++) { try { clearInterval(i); } catch (e) {} }

    running = false;                 /* render() 里不再 tick++ */
    tick = %d;                       /* 相位定死 */
    setTab('%s');

    function setText(id, v) { var e = document.getElementById(id); if (e) e.textContent = v; }
    setText('runStatus', 'RUNNING');
    setText('clock', '%s');
    setText('uptime', '00:00');
    setText('fps', '%s');

    /* 把 render 的 rAF 循环踢一脚，让上面这些文字进到画布之后的 DOM 里 */
    render();
})();
</script>
</body>"""


def find_browser():
    for p in EDGE_CANDIDATES:
        if os.path.isfile(p):
            return p
    sys.exit("找不到 Edge/Chrome，候选：\n  " + "\n  ".join(EDGE_CANDIDATES))


def render(browser, page):
    src = open(REF_HTML, encoding="utf-8").read()
    if "</body>" not in src:
        sys.exit("参考稿里没有 </body>，注入点找不到")

    patched = src.replace(
        "</body>",
        INJECT % (FREEZE_TICK, page, CLOCK_TEXT, FPS_TEXT),
        1,
    )
    tmp_html = os.path.join(OUT_DIR, "_render_%s.html" % page)
    with open(tmp_html, "w", encoding="utf-8") as f:
        f.write(patched)

    out_png = os.path.join(OUT_DIR, "ref_%s.png" % page)
    if os.path.exists(out_png):
        os.remove(out_png)

    profile = os.path.join(OUT_DIR, "_chromeprofile")
    url = "file:///" + tmp_html.replace("\\", "/")
    cmd = [
        browser,
        "--headless=new",
        "--disable-gpu",
        "--no-first-run",
        "--no-default-browser-check",
        "--hide-scrollbars",
        "--force-device-scale-factor=1",
        "--user-data-dir=" + profile,
        "--window-size=%d,%d" % (W, H),
        "--virtual-time-budget=1200",
        "--screenshot=" + out_png,
        url,
    ]
    # ⚠️ Windows 上 msedge.exe 只是个「启动器」：它把命令行转交给真正的浏览器进程后
    #    立刻以 rc=0 退出，截图是**异步**落盘的。所以 subprocess.run() 一返回就去
    #    检查文件必然扑空（踩过：三种 --headless=* 全是 rc=0 且没有 png）。
    #    正确做法：起进程后轮询产物。
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    ok = False
    for _ in range(60):                       # 最多等 30s
        if os.path.exists(out_png) and os.path.getsize(out_png) > 0:
            ok = True
            break
        time.sleep(0.5)
    try:
        proc.kill()
    except OSError:
        pass

    if not ok:
        try:
            o, e = proc.communicate(timeout=5)
        except Exception:
            o = e = b""
        print(o.decode("utf-8", "replace")[-2000:])
        print(e.decode("utf-8", "replace")[-2000:])
        sys.exit("渲染 %s 失败，没生成 %s" % (page, out_png))

    try:
        from PIL import Image
        im = Image.open(out_png)
        print("  ref_%-6s %s" % (page + ".png", im.size))
        if im.size != (W, H):
            print("     ⚠ 尺寸不是 %dx%d —— --window-size 没生效？" % (W, H))
    except ImportError:
        print("  ref_%s.png (没装 Pillow，未查尺寸)" % page)


def main():
    pages = sys.argv[1:] or PAGES
    for p in pages:
        if p not in PAGES:
            sys.exit("未知页面 %r，可选 %s" % (p, "/".join(PAGES)))
    if not os.path.exists(REF_HTML):
        sys.exit("找不到 %s" % REF_HTML)
    os.makedirs(OUT_DIR, exist_ok=True)
    b = find_browser()
    print("浏览器:", b)
    for p in pages:
        render(b, p)
    print("产物目录:", OUT_DIR)


if __name__ == "__main__":
    main()
