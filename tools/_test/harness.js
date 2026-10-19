
(function () {
'use strict';

/* ============================ 协议常量 ============================ */
const POINTS = 4096;
const HDR    = 40;
const PKT    = 16428;
const FS     = 65000000;
const FFT_N  = 8192;                 // 补零到 8192，bin = 7.935 kHz
const MAGIC  = [0x56,0x33,0x32,0x43,0x41,0x50,0x33,0x21]; // "V32CAP3!"

/* CRC32：反射多项式 0xEDB88320，初值/最终异或均为 0xFFFFFFFF */
const CRC_TABLE = (function () {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
    t[n] = c >>> 0;
  }
  return t;
})();

function crc32(buf, len) {
  let c = 0xFFFFFFFF;
  for (let i = 0; i < len; i++) c = CRC_TABLE[(c ^ buf[i]) & 0xFF] ^ (c >>> 8);
  return (c ^ 0xFFFFFFFF) >>> 0;
}

/* ============================ 噪声源 ============================ */
let gSpare = null;
function gauss() {
  if (gSpare !== null) { const v = gSpare; gSpare = null; return v; }
  let u, v, s;
  do { u = Math.random() * 2 - 1; v = Math.random() * 2 - 1; s = u * u + v * v; } while (s >= 1 || s === 0);
  const m = Math.sqrt(-2 * Math.log(s) / s);
  gSpare = v * m;
  return u * m;
}

/* ============================ 组帧 ============================ */
/**
 * 纯函数：把一帧写进 st.buf，返回统计量。不碰 DOM，便于单独测试。
 * opts: {freqHz, amp0, amp1, dc, sigma, spikes, noiseType, frameId, startIndex}
 */
function buildFrame(opts, st) {
  const buf = st.buf, dv = st.dv;
  const N = POINTS;

  /* ---- 头部 40 字节 ---- */
  for (let i = 0; i < 8; i++) buf[i] = MAGIC[i];
  dv.setUint32(8,  3,         true);   // version
  dv.setUint32(12, opts.frameId >>> 0, true);
  dv.setUint32(16, FS,        true);   // adc_fs_hz
  dv.setUint32(20, 1,         true);   // decimation
  dv.setUint32(24, POINTS,    true);   // points_per_channel
  dv.setUint32(28, 0,         true);   // flags
  dv.setUint32(32, opts.startIndex % 4294967296 >>> 0, true);
  dv.setUint32(36, Math.floor(opts.startIndex / 4294967296) >>> 0, true);

  /* ---- 样点：12 位有符号，符号扩展到 int16 ---- */
  const w    = 2 * Math.PI * opts.freqHz / FS;
  const sig  = opts.noiseType === 'gauss';
  const dc   = opts.dc | 0;
  const a0   = opts.amp0, a1 = opts.amp1, sg = opts.sigma;
  const ph1  = Math.PI * 0.5;              // CH1 相对 CH0 滞后 90°

  let min0 = 32767, max0 = -32768, min1 = 32767, max1 = -32768;
  let sat = 0, sum0 = 0, sum1 = 0;
  let off = HDR;

  for (let i = 0; i < N; i++) {
    const ph = w * i;

    let v0 = dc + a0 * Math.sin(ph);
    let v1 = dc + a1 * Math.sin(ph + ph1);

    if (sg > 0) {
      if (sig) {
        v0 += gauss() * sg;
        v1 += gauss() * sg;
      } else {
        v0 += (Math.random() * 2 - 1) * sg * 1.732;   // 均匀分布，σ 对齐
        v1 += (Math.random() * 2 - 1) * sg * 1.732;
      }
    }

    let s0 = Math.round(v0), s1 = Math.round(v1);

    if (s0 < -2048) { s0 = -2048; sat++; } else if (s0 > 2047) { s0 = 2047; sat++; }
    if (s1 < -2048) { s1 = -2048; sat++; } else if (s1 > 2047) { s1 = 2047; sat++; }

    if (s0 < min0) min0 = s0; if (s0 > max0) max0 = s0;
    if (s1 < min1) min1 = s1; if (s1 > max1) max1 = s1;
    sum0 += s0 * s0; sum1 += s1 * s1;

    dv.setInt16(off, s0, true);
    dv.setInt16(off + 2, s1, true);
    off += 4;

    st.ch0[i] = s0;
    st.ch1[i] = s1;
  }

  /* ---- 脉冲干扰：随机撒几根，模拟偶发毛刺 ---- */
  if (opts.spikes > 0) {
    for (let k = 0; k < opts.spikes; k++) {
      const i = (Math.random() * N) | 0;
      const mag = (Math.random() < 0.5 ? -1 : 1) * (300 + Math.random() * 900);
      let s0 = Math.max(-2048, Math.min(2047, Math.round(st.ch0[i] + mag)));
      let s1 = Math.max(-2048, Math.min(2047, Math.round(st.ch1[i] + mag * 0.5)));
      st.ch0[i] = s0; st.ch1[i] = s1;
      dv.setInt16(HDR + 4 * i, s0, true);
      dv.setInt16(HDR + 4 * i + 2, s1, true);
    }
  }

  /* ---- CRC，覆盖 [0, 16424) ---- */
  dv.setUint32(PKT - 4, crc32(buf, PKT - 4), true);

  return {
    min0, max0, min1, max1,
    vpp0: max0 - min0, vpp1: max1 - min1,
    rms0: Math.sqrt(sum0 / N), rms1: Math.sqrt(sum1 / N),
    sat
  };
}

/* ============================ FFT（实信号，结果放一半） ============================ */
const re = new Float64Array(FFT_N);
const im = new Float64Array(FFT_N);

function fftInPlace(re, im) {
  const n = re.length;
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      let t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = -2 * Math.PI / len;
    const wr = Math.cos(ang), wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1, ci = 0;
      for (let k = 0; k < len / 2; k++) {
        const ur = re[i + k], ui = im[i + k];
        const vr = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci;
        const vi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr;
        re[i + k] = ur + vr; im[i + k] = ui + vi;
        re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi;
        const ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr; cr = ncr;
      }
    }
  }
}

const mag = new Float64Array(FFT_N / 2);

function spectrum(src) {
  const nReal = POINTS;
  let wsum = 0;
  for (let i = 0; i < nReal; i++) {
    const win = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / (nReal - 1));
    wsum += win;
    re[i] = src[i] * win;
    im[i] = 0;
  }
  for (let i = nReal; i < FFT_N; i++) { re[i] = 0; im[i] = 0; }

  fftInPlace(re, im);

  for (let k = 0; k < FFT_N / 2; k++) {
    mag[k] = 2 * Math.sqrt(re[k] * re[k] + im[k] * im[k]) / wsum;
  }
  return mag;
}

/* ============================ 状态 ============================ */
const st = {
  buf: new Uint8Array(PKT),
  dv:  null,
  ch0: new Int16Array(POINTS),
  ch1: new Int16Array(POINTS),
  stats: null,
  hasFrame: false,     // 没发过帧就一直按 0 画
  frameId: 1000,
  startIndex: 0x100001234,
  lastBuildMs: 0,
  fftOn: 0,            // 0=CH0 1=CH1
  peakHz: 0, peakDb: -999, floorDb: -999
};
st.dv = new DataView(st.buf.buffer);

/* Web Serial 运行时 */
const rt = {
  port: null, writer: null, connected: false, connecting: false, baud: 0,
  sentFrames: 0, sentBytes: 0, t0: 0, stop: false, rate: 0, bps: 0, lastErr: ''
};

/* ============================ 控件定义 ============================ */
const CONTROLS = [
  { id:'freq',  host:'ctlSig',   label:'信号频率', min:10000, max:20000000, step:1000, val:1000000, fmt:v=>fmtHz(v) },
  { id:'amp0',  host:'ctlSig',   label:'CH0 幅度', min:0, max:2047, step:1, val:300, fmt:v=>v+' LSB' },
  { id:'amp1',  host:'ctlSig',   label:'CH1 幅度', min:0, max:2047, step:1, val:60,  fmt:v=>v+' LSB' },
  { id:'dc',    host:'ctlSig',   label:'直流偏置', min:-1024, max:1024, step:1, val:0, fmt:v=>v+' LSB' },
  { id:'sigma', host:'ctlNoise', label:'噪声 σ',   min:0, max:300, step:1, val:8, fmt:v=>v+' LSB' },
  { id:'spikes',host:'ctlNoise', label:'脉冲干扰', min:0, max:20, step:1, val:0, fmt:v=>v+' 个/帧' },
  { id:'view',  host:'ctlView',  label:'绘图点数', min:256, max:4096, step:256, val:512,
    fmt:v => v + ' 点 · ' + (v / POINTS * 63.0154).toFixed(2) + ' µs' },
  { id:'period',host:'ctlTx',    label:'帧周期',   min:180, max:2000, step:10, val:200, fmt:v=>v+' ms' }
];
const els = {};

function fmtHz(v) {
  if (v >= 1e6) return (v / 1e6).toFixed(3) + ' MHz';
  return (v / 1e3).toFixed(1) + ' kHz';
}

function buildControls() {
  for (const c of CONTROLS) {
    const wrap = document.createElement('div');
    wrap.className = 'ctl';
    wrap.innerHTML =
      '<div class="row"><label for="' + c.id + '">' + c.label + '</label>' +
      '<span class="val" id="v_' + c.id + '">' + c.fmt(c.val) + '</span></div>' +
      '<input type="range" id="' + c.id + '" min="' + c.min + '" max="' + c.max +
      '" step="' + c.step + '" value="' + c.val + '">';
    document.getElementById(c.host).appendChild(wrap);
    const range = wrap.querySelector('input');
    const out = wrap.querySelector('.val');
    els[c.id] = range;
    range.addEventListener('input', () => { out.textContent = c.fmt(+range.value); });
  }
}

function readOpts() {
  const g = id => +els[id].value;
  return {
    freqHz: g('freq'), amp0: g('amp0'), amp1: g('amp1'), dc: g('dc'),
    sigma: g('sigma'), spikes: g('spikes'),
    noiseType: document.getElementById('noiseType').value,
    frameId: st.frameId, startIndex: st.startIndex
  };
}

/* ============================ 画布 ============================ */
function fitCanvas(cv) {
  const dpr = window.devicePixelRatio || 1;
  const r = cv.getBoundingClientRect();
  const w = Math.max(1, Math.round(r.width * dpr));
  const h = Math.max(1, Math.round(r.height * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: r.width, h: r.height };
}

function grid(ctx, w, h, nx, ny) {
  ctx.strokeStyle = '#16202c';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let i = 1; i < nx; i++) { const x = Math.round(i * w / nx) + .5; ctx.moveTo(x, 0); ctx.lineTo(x, h); }
  for (let i = 1; i < ny; i++) { const y = Math.round(i * h / ny) + .5; ctx.moveTo(0, y); ctx.lineTo(w, y); }
  ctx.stroke();
}

function drawScope() {
  const cv = document.getElementById('scope');
  const { ctx, w, h } = fitCanvas(cv);
  ctx.clearRect(0, 0, w, h);
  grid(ctx, w, h, 10, 8);

  const N = Math.max(2, Math.min(POINTS, +(els.view ? els.view.value : POINTS)));

  const s = st.stats;
  const lo = Math.min(s.min0, s.min1), hi = Math.max(s.max0, s.max1);
  const flat = (hi - lo) < 1;                 // 没有数据：整幅就是一条 0 线
  const span = Math.max(8, hi - lo);
  const mid = (lo + hi) / 2;
  const y0 = mid + span * 0.62, y1 = mid - span * 0.62;
  const toY = v => (y0 - v) / (y0 - y1) * h;

  // 中轴
  ctx.strokeStyle = '#243040'; ctx.beginPath();
  ctx.moveTo(0, Math.round(toY(mid)) + .5); ctx.lineTo(w, Math.round(toY(mid)) + .5); ctx.stroke();

  const stepX = w / (N - 1);
  for (const [arr, color] of [[st.ch0, '--ch0'], [st.ch1, '--ch1']]) {
    ctx.strokeStyle = getComputedStyle(document.body).getPropertyValue(color);
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    for (let i = 0; i < N; i++) {
      const x = i * stepX, y = toY(arr[i]);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }

  // 顶端标注幅度（带底衬，避免压在波形上）
  ctx.font = '11px ui-monospace,Consolas,monospace';
  const tag = (txt, x, y, align) => {
    const tw = ctx.measureText(txt).width;
    const lx = align === 'right' ? x - tw : x;
    ctx.fillStyle = 'rgba(10,14,20,.85)';
    ctx.fillRect(lx - 3, y - 11, tw + 6, 14);
    ctx.fillStyle = '#5d7085';
    ctx.textAlign = 'left';
    ctx.fillText(txt, lx, y);
  };
  tag(flat ? '0 LSB' : y1.toFixed(0) + ' LSB', 6, 13);
  tag(flat ? '0 LSB' : y0.toFixed(0) + ' LSB', 6, h - 5);
  tag((N / POINTS * 63.0154).toFixed(2) + ' µs', w - 6, h - 5, 'right');
}

function drawFft() {
  const cv = document.getElementById('fft');
  const { ctx, w, h } = fitCanvas(cv);
  ctx.clearRect(0, 0, w, h);
  grid(ctx, w, h, 10, 6);

  const half = FFT_N / 2;
  const dbMin = -140, dbMax = 0;
  const toY = db => h - (Math.max(dbMin, Math.min(dbMax, db)) - dbMin) / (dbMax - dbMin) * h;

  ctx.beginPath();
  ctx.moveTo(0, h);
  for (let k = 1; k < half; k++) {
    const db = 20 * Math.log10(Math.max(mag[k], 1e-9) / 2048);
    ctx.lineTo(k / (half - 1) * w, toY(db));
  }
  ctx.lineTo(w, h);
  ctx.closePath();
  ctx.fillStyle = 'rgba(78,168,255,.15)';
  ctx.fill();
  ctx.strokeStyle = '#4ea8ff';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let k = 1; k < half; k++) {
    const db = 20 * Math.log10(Math.max(mag[k], 1e-9) / 2048);
    const x = k / (half - 1) * w, y = toY(db);
    if (k === 1) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();

  // 峰值标记
  if (st.peakHz > 0) {
    const x = st.peakHz / (FS / 2) * w;
    ctx.strokeStyle = 'rgba(255,179,71,.55)';
    ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    ctx.setLineDash([]);
  }

  ctx.fillStyle = '#5d7085';
  ctx.font = '11px ui-monospace,Consolas,monospace';
  ctx.fillText('0 dBFS', 4, 12);
  ctx.fillText('-140', 4, h - 4);
  ctx.fillText('0', w - 52, h - 4);
  ctx.fillText((FS / 2 / 1e6).toFixed(1) + ' MHz', w - 62, 12);
}

/* ============================ 统计计算 ============================ */
function analyse(chIdx) {
  const src = chIdx === 0 ? st.ch0 : st.ch1;
  spectrum(src);

  let peak = 3, peakMag = 0;
  const half = FFT_N / 2;
  for (let k = 3; k < half; k++) {
    if (mag[k] > peakMag) { peakMag = mag[k]; peak = k; }
  }
  st.peakHz = peak * FS / FFT_N;
  st.peakDb = 20 * Math.log10(Math.max(peakMag, 1e-9) / 2048);

  // 噪声底：排除峰值附近 ±8 bin，取中位数
  const tmp = [];
  for (let k = 20; k < half; k++) {
    if (Math.abs(k - peak) <= 8) continue;
    tmp.push(mag[k]);
  }
  tmp.sort((a, b) => a - b);
  const med = tmp.length ? tmp[tmp.length >> 1] : 1e-9;
  st.floorDb = 20 * Math.log10(Math.max(med, 1e-9) / 2048);
}

/* ============================ 主循环 ============================ */
let loopRunning = false;

async function loop() {
  if (loopRunning) return;
  loopRunning = true;

  let nextDue = performance.now();
  while (!rt.stop && rt.connected) {
    const now = performance.now();

    // start_index 按真实流逝时间推进（1 ms = 65000 个采样时钟）
    if (st.lastBuildMs) {
      const dtms = now - st.lastBuildMs;
      st.startIndex += Math.max(POINTS, Math.round(dtms * 65000));
    }
    st.lastBuildMs = now;

    const opts = readOpts();
    st.stats = buildFrame(opts, st);
    st.frameId++;
    st.hasFrame = true;

    const ch = st.stats.vpp0 >= st.stats.vpp1 ? 0 : 1;
    st.fftOn = ch;
    analyse(ch);

    if (rt.connected && rt.writer) {
      try {
        await rt.writer.ready;            // 背压：等发送缓冲有空位
        await rt.writer.write(st.buf);
        rt.sentFrames++;
        rt.sentBytes += PKT;
      } catch (e) {
        setErr('写入失败：' + e.message);
        await disconnect();
      }
    }

    render();

    const period = +els.period.value;
    nextDue += period;
    const wait = nextDue - performance.now();
    if (wait < 0) nextDue = performance.now();
    await sleep(Math.max(0, wait));
  }
  loopRunning = false;
}

function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

/* ============================ 渲染 ============================ */
/* 没有在发帧（未连接 / 刚断开）时一律按 0 画：平直零线，全部统计归零，
   不保留上一帧的残留，和板端行为保持一致。 */
function zeroStats() {
  return { min0: 0, max0: 0, min1: 0, max1: 0, vpp0: 0, vpp1: 0, rms0: 0, rms1: 0, sat: 0 };
}

function render() {
  const live = rt.connected && st.hasFrame;
  if (!live) {
    st.ch0.fill(0); st.ch1.fill(0);
    mag.fill(0);
    st.stats = zeroStats();
    st.peakHz = 0; st.peakDb = -999; st.floorDb = -999;
    st.fftOn = 0;
  }

  drawScope();
  drawFft();

  const s = st.stats;
  const nv = +(els.view ? els.view.value : POINTS);
  document.getElementById('scopeInfo').textContent =
    '前 ' + nv + ' 点 · ' + (nv / POINTS * 63.0154).toFixed(2) + ' µs' +
    (nv < POINTS ? '（整帧 4096 点 / 63.015 µs）' : '');

  const dbTxt = v => live ? v.toFixed(1) + ' dBFS' : '— dBFS';

  document.getElementById('statsTime').innerHTML =
    kv('CH0 Vpp', s.vpp0 + ' LSB') +
    kv('CH1 Vpp', s.vpp1 + ' LSB') +
    kv('CH0 RMS', s.rms0.toFixed(1)) +
    kv('CH1 RMS', s.rms1.toFixed(1)) +
    kv('饱和', s.sat + ' 点', s.sat > 0) +
    kv('帧号', live ? (st.frameId - 1) : '—');

  document.getElementById('statsFft').innerHTML =
    kv('峰值频率', live ? (st.peakHz / 1e6).toFixed(3) + ' MHz' : '0.000 MHz') +
    kv('峰值电平', dbTxt(st.peakDb)) +
    kv('噪声底', dbTxt(st.floorDb)) +
    kv('bin 宽', (FS / FFT_N / 1e3).toFixed(3) + ' kHz');
  document.getElementById('fftSrc').textContent = live
    ? '取自 CH' + st.fftOn + '（Vpp 较大的一路，与板端一致）'
    : '—';

  // 帧头 hex：没在发帧时显示 40 个 00，不留上一帧的残留
  let hex = '';
  for (let i = 0; i < HDR; i++) {
    if (i === 8 || i === 12 || i === 16 || i === 20 || i === 24 || i === 28 || i === 32 || i === 36)
      hex += '<b>| </b>';
    hex += (live ? st.buf[i] : 0).toString(16).padStart(2, '0').toUpperCase() + ' ';
  }
  document.getElementById('hex').innerHTML = hex;

  updateStatus();
}

function kv(k, v, warn) {
  return '<span class="stat' + (warn ? ' warn' : '') + '">' + k + '<b>' + v + '</b></span>';
}

/* 右下角不再挂小字，速率直接写在右上角状态里 */
function updateStatus() {
  const now = performance.now();
  if (rt.connected) {
    if (!rt.t0) { rt.t0 = now; rt.sentFrames = 0; rt.sentBytes = 0; }
    const dt = (now - rt.t0) / 1000;
    if (dt > 1) {
      rt.rate = rt.sentFrames / dt;
      rt.bps  = rt.sentBytes / dt;
      rt.t0 = now; rt.sentFrames = 0; rt.sentBytes = 0;
    }
    setState('已连接 · 发送中 · ' + rt.rate.toFixed(2) + ' 帧/s', 'on');
  } else {
    rt.rate = 0; rt.bps = 0;
    setState('未连接', '');
  }
}

/* ============================ Web Serial ============================ */
/*
 * 波特率可选：一帧 16428 字节 8N1 = 164280 bit，所以
 *   921600  -> 178.3 ms/帧，帧周期再低就会堆积 -> 实际约 5.6 帧/秒
 *   2000000 ->  82.1 ms/帧                          -> 实际约 12 帧/秒
 * 帧周期滑块的上下限跟着波特率走，不然选 2 M 时还卡在 190 ms 就白改了。
 * 板端要同时改：echo 2000000 > /etc/waveform_demo.baud 再重启服务，
 * FPGA 端也必须跟着改，三方不一致时收到的是乱码而不是帧。
 */
const BAUD0 = 921600;

function curBaud() {
  const el = document.getElementById('baud');
  return (el && +el.value) || BAUD0;
}

/* 一帧在线上占用的毫秒 */
function wireMs(baud) { return PKT * 10 / baud; }

/* 把波特率相关的界面（页脚提示、帧周期下限）刷一遍 */
function applyBaud() {
  const b = curBaud();
  const wm = wireMs(b);
  const minMs = Math.ceil((wm + 10) / 10) * 10;   // 留 10 ms 余量，向上取整到 10

  if (els.period) {
    els.period.min = minMs;
    if (+els.period.value < minMs) els.period.value = minMs;
    const out = document.getElementById('v_period');
    if (out) out.textContent = els.period.value + ' ms';
  }

  const h = document.getElementById('hdrBaud');
  if (h) h.textContent = 'Web Serial · ' + b + ' 8N1';
  const f = document.getElementById('footWire');
  if (f) f.textContent = b + ' 下一帧约 ' + Math.round(wm) + ' ms，帧周期低于 ' +
                         minMs + ' ms 会堆积（上限约 ' + (1000 / wm).toFixed(1) + ' 帧/秒）';
}

function setState(txt, cls) {
  document.getElementById('stateTxt').textContent = txt;
  const dot = document.getElementById('dot');
  dot.className = 'dot' + (cls ? ' ' + cls : '');
}

function banner(msg, isErr) {
  const b = document.getElementById('banner');
  if (!msg) { b.className = 'banner'; return; }
  b.textContent = msg;
  b.className = 'banner show' + (isErr ? ' err' : '');
}

async function refreshPorts() {
  const sel = document.getElementById('ports');
  const keep = sel.value;
  sel.innerHTML = '<option value="">— 选择串口 —</option>';
  if (!('serial' in navigator)) return;
  const ports = await navigator.serial.getPorts();
  ports.forEach((p, i) => {
    const info = p.getInfo();
    const name = info.usbVendorId
      ? `USB ${info.usbVendorId.toString(16).padStart(4,'0')}:${(info.usbProductId||0).toString(16).padStart(4,'0')}`
      : `串口 ${i + 1}`;
    const o = document.createElement('option');
    o.value = String(i);
    o.textContent = name + (info.serialNumber ? ' · ' + info.serialNumber : '');
    sel.appendChild(o);
  });
  if (keep) sel.value = keep;
  document.getElementById('btnConn').disabled = ports.length === 0;
  document.getElementById('btnConn').disabled = !sel.value;
}

async function pickPort() {
  try {
    await navigator.serial.requestPort();
    await refreshPorts();
    const sel = document.getElementById('ports');
    if (sel.options.length > 1) sel.selectedIndex = sel.options.length - 1;
    document.getElementById('btnConn').disabled = false;
  } catch (e) {
    if (e.name !== 'NotFoundError') banner('选择串口失败：' + e.message, true);
  }
}

async function connect() {
  if (rt.connected || rt.connecting) return;
  const sel = document.getElementById('ports');
  const ports = await navigator.serial.getPorts();
  const idx = parseInt(sel.value, 10);
  if (isNaN(idx) || !ports[idx]) { banner('请先选择串口', true); return; }

  const baud = curBaud();
  rt.connecting = true;
  setState('正在打开…', '');
  try {
    const port = ports[idx];
    await port.open({ baudRate: baud, dataBits: 8, stopBits: 1, parity: 'none', flowControl: 'none' });
    rt.port = port;
    rt.baud = baud;
    rt.writer = port.writable.getWriter();
    rt.connected = true;
    rt.stop = false;
    rt.t0 = 0; rt.rate = 0; rt.bps = 0; rt.lastErr = '';
    banner('');
    document.getElementById('btnConn').disabled = true;
    document.getElementById('btnDisc').disabled = false;
    document.getElementById('btnPick').disabled = true;
    document.getElementById('ports').disabled = true;
    document.getElementById('baud').disabled = true;   // 串口开着的时候不能改速率

    // 帧号与起始计数重新从当前时刻起算，保持单调
    st.lastBuildMs = 0;

    if (!loopRunning) loop();          // 只有连上才开始生成/发送
  } catch (e) {
    banner('打开串口失败：' + e.message, true);
    setState('未连接', 'err');
  } finally {
    rt.connecting = false;
  }
}

async function disconnect() {
  rt.stop = true;
  rt.connected = false;
  st.hasFrame = false;
  try { if (rt.writer) { rt.writer.releaseLock(); rt.writer = null; } } catch (e) {}
  try { if (rt.port) { await rt.port.close(); rt.port = null; } } catch (e) {}
  document.getElementById('btnConn').disabled = false;
  document.getElementById('btnDisc').disabled = true;
  document.getElementById('btnPick').disabled = false;
  document.getElementById('ports').disabled = false;
  document.getElementById('baud').disabled = false;
  rt.stop = false;
  render();                            // 断开后立刻回到平直零线
}

function setErr(msg) {
  rt.lastErr = msg;
  banner(msg, true);
  setState('错误', 'err');
}

/* ============================ 启动 ============================ */
function init() {
  buildControls();

  if (!('serial' in navigator)) {
    banner('这个浏览器没有 Web Serial。请用 Chrome / Edge 89+ 打开本页。' +
           '如果是从本地文件直接双击打开的，建议改用本地服务器：' +
           'python -m http.server 8000 然后访问 http://localhost:8000/。', true);
  } else if (!window.isSecureContext) {
    banner('当前不是安全上下文，Web Serial 不可用。请用 http://localhost 或 https 打开。', true);
  } else if (location.protocol === 'file:') {
    banner('提示：正以 file:// 打开。部分 Chrome / Edge 版本会拒绝 file:// 的串口授权，' +
           '点「请求串口」若无反应，请在本目录执行 python -m http.server 8000，' +
           '再访问 http://localhost:8000/serial_frame_sender.html（预览功能不受影响）。');
  }

  document.getElementById('btnPick').addEventListener('click', pickPort);
  document.getElementById('btnConn').addEventListener('click', connect);
  document.getElementById('btnDisc').addEventListener('click', disconnect);
  document.getElementById('baud').addEventListener('change', applyBaud);
  document.getElementById('ports').addEventListener('change', e => {
    document.getElementById('btnConn').disabled = !e.target.value || rt.connected;
  });
  applyBaud();          // 页脚与帧周期下限先按默认 921600 摆好

  document.getElementById('btnSave').addEventListener('click', () => {
    if (!st.hasFrame) { banner('还没发过帧，没有可保存的数据', true); return; }
    const blob = new Blob([st.buf], { type: 'application/octet-stream' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'frame_' + (st.frameId - 1) + '.bin';
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  });

  if ('serial' in navigator) {
    refreshPorts();
    navigator.serial.addEventListener('disconnect', () => {
      if (rt.connected) { banner('串口已断开', true); disconnect(); }
    });
  }

  window.addEventListener('resize', () => { render(); });

  // 上电/打开页面时没有数据：直接画平直零线，点「连接」后才开始生成与发送
  render();
}

/* 供 node 端单测 / 截图核对使用：不依赖 DOM 的部分 */
if (typeof window !== 'undefined') {
  window.__v32 = { buildFrame, crc32, st, rt, render, analyse, PKT, POINTS, FS };
}

if (typeof document !== 'undefined' && document.addEventListener) {
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
}

})();
