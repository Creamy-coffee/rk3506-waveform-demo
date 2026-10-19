// 用 CDP 给无头 Edge 截图：node shot.mjs <url> <out.png> [width] [height] [waitMs]
// 之所以不用 --screenshot：本机 msedge.exe 启动后立刻脱离父进程，命令行截图拿不到文件。
import fs from 'node:fs';

const [url, out, w = '1600', h = '1000', waitMs = '1200', evalJs = ''] = process.argv.slice(2);
const PORT = process.env.CDP_PORT || '9333';

const list = await (await fetch(`http://127.0.0.1:${PORT}/json/list`)).json();
const page = list.find(t => t.type === 'page');
if (!page) throw new Error('没有 page target');

const ws = new WebSocket(page.webSocketDebuggerUrl);
let id = 0;
const pending = new Map();

function call(method, params = {}) {
  return new Promise((resolve, reject) => {
    const mid = ++id;
    pending.set(mid, { resolve, reject });
    ws.send(JSON.stringify({ id: mid, method, params }));
  });
}

ws.addEventListener('message', ev => {
  const msg = JSON.parse(ev.data);
  if (msg.id && pending.has(msg.id)) {
    const { resolve, reject } = pending.get(msg.id);
    pending.delete(msg.id);
    msg.error ? reject(new Error(msg.method + ' ' + JSON.stringify(msg.error))) : resolve(msg.result);
  }
});

await new Promise(r => ws.addEventListener('open', r, { once: true }));

await call('Page.enable');
await call('Emulation.setDeviceMetricsOverride', {
  width: +w, height: +h, deviceScaleFactor: 1, mobile: false
});
await call('Page.navigate', { url });
await new Promise(r => setTimeout(r, +waitMs));

if (evalJs) {
  const r = await call('Runtime.evaluate', { expression: evalJs, awaitPromise: true, returnByValue: true });
  if (r.exceptionDetails) throw new Error('注入脚本报错：' + JSON.stringify(r.exceptionDetails));
  console.log('注入结果', JSON.stringify(r.result && r.result.value));
  await new Promise(r => setTimeout(r, 400));
}

const { data } = await call('Page.captureScreenshot', { format: 'png', captureBeyondViewport: false });
fs.writeFileSync(out, Buffer.from(data, 'base64'));
console.log('已写入', out, fs.statSync(out).size, '字节');
ws.close();
process.exit(0);
