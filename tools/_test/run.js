const fs = require("fs");
global.window = global;
require("./harness.js");
const v = global.__v32;
if (!v) { console.error("未导出 __v32"); process.exit(1); }
const st = v.st;
const opts = { freqHz: 1000000, amp0: 300, amp1: 60, dc: 0,
               sigma: 8, spikes: 0, noiseType: "gauss",
               frameId: 1234, startIndex: 0x100001234 };
const s = v.buildFrame(opts, st);
fs.writeFileSync("G:/Desktop/Project/GC9503CV/waveform_demo/tools/_test/frame_js.bin",
                 Buffer.from(st.buf));
console.log("长度", st.buf.length, "| CH0 Vpp", s.vpp0, "CH1 Vpp", s.vpp1,
            "| 饱和", s.sat);
