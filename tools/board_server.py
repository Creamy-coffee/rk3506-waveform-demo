#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AB5 联调留言板 —— 屏端 AI 与 FPGA 端 AI 的共享黑板

零依赖（Python 3.8+ 标准库），单文件，数据落 board.jsonl（只追加，不覆盖）。

启动：
    python3 board_server.py --port 8765
    python3 board_server.py --port 8765 --bind 0.0.0.0

接口（JSON，UTF-8）：
    GET  /api/list?since=<id>          取 id > since 的消息；不带 since 取全部
    POST /api/post                     发一条，body:
                                        {"from":"屏端|FPGA端","kind":"error|finding|question|answer","text":"...","ref":"可选"}
    GET  /api/stats                    计数
    GET  /                             网页版（自动刷新）

curl 示例：
    curl -s http://127.0.0.1:8765/api/list
    curl -s -X POST http://127.0.0.1:8765/api/post \
         -H 'Content-Type: application/json' \
         -d '{"from":"屏端","kind":"finding","text":"单读者 30 秒 25 组/秒，CRC 0 错"}'
"""
import argparse, json, mimetypes, os, re, threading, time
from urllib.parse import unquote
import socketserver
from http.server import BaseHTTPRequestHandler, HTTPServer
try:                                        # Python 3.7+
    from http.server import ThreadingHTTPServer
except ImportError:                         # Python 3.6（阿里云 Anolis 默认就是）
    class ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

HERE = os.path.dirname(os.path.abspath(__file__))
STORE = os.path.join(HERE, 'board.jsonl')

# 图片附件：存 uploads/ 下，只允许常见位图，单张上限 8 MB。
# 不做 multipart —— 前端直接把 File 当 body 发、文件名放 X-Filename 头。
# 好处：Python 3.13 删掉了 cgi 模块，multipart 解析在 3.6/3.13 上行为还不一致。
UPLOAD_DIR = os.path.join(HERE, 'uploads')
MAX_UPLOAD = 32 * 1024 * 1024
# 图片：内嵌预览；其余：一律按附件下载（Content-Disposition: attachment）
IMAGE_EXT = {'.png', '.jpg', '.jpeg', '.gif', '.webp', '.bmp'}
# 明确拒绝的类型：能在服务器或浏览器里当代码跑的东西，别往上传目录里放
BLOCK_EXT = {'.php', '.phtml', '.php3', '.jsp', '.asp', '.aspx', '.cgi',
             '.exe', '.dll', '.bat', '.cmd', '.com', '.scr', '.msi', '.sh'}
try:
    os.makedirs(UPLOAD_DIR)
except OSError:
    pass
LOCK = threading.Lock()
STATE = {'id': 0}

KIND_CN = {'error': '问题', 'finding': '结论', 'question': '提问', 'answer': '回复'}


def load_state():
    """重启后从文件恢复最大 id"""
    if not os.path.exists(STORE):
        return
    with open(STORE, 'rb') as f:
        for line in f:
            try:
                m = json.loads(line.decode('utf-8'))
                if m.get('id', 0) > STATE['id']:
                    STATE['id'] = m['id']
            except Exception:
                pass


def append_msg(msg):
    with LOCK:
        STATE['id'] += 1
        msg['id'] = STATE['id']
        msg['ts'] = time.strftime('%Y-%m-%d %H:%M:%S')
        with open(STORE, 'a', encoding='utf-8') as f:
            f.write(json.dumps(msg, ensure_ascii=False) + '\n')
            f.flush()
            os.fsync(f.fileno())
        return msg


def read_msgs(since=0):
    out = []
    if not os.path.exists(STORE):
        return out
    with open(STORE, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                m = json.loads(line)
            except Exception:
                continue
            if m.get('id', 0) > since:
                out.append(m)
    return out


PAGE = """<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>AB5 联调留言板</title>
<style>
:root{--bg:#050805;--pnl:#0b120b;--line:#1b2e19;--grn:#35cc45;--txt:#cfe0cf;--dim:#4e7a5a;
      --mag:#ff45ff;--amb:#ffb340;--cyan:#5ad8e0}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--txt);font:14px/1.6 "Consolas","Microsoft YaHei",monospace}
header{padding:10px 16px;border-bottom:1px solid var(--line);display:flex;gap:14px;align-items:center}
h1{font-size:15px;margin:0;color:var(--grn);letter-spacing:.1em}
.tag{font-size:12px;color:var(--dim)}
main{padding:14px 16px;max-width:1100px}
#list{display:flex;flex-direction:column;gap:8px}
.msg{background:var(--pnl);border:1px solid var(--line);border-left:3px solid var(--grn);
     border-radius:6px;padding:8px 12px}
.msg.fpga{border-left-color:var(--amb)}
.msg.head{display:flex;gap:10px;font-size:12px;color:var(--dim);margin-bottom:4px;flex-wrap:wrap}
.from{color:var(--grn);font-weight:700} .msg.fpga .from{color:var(--amb)}
.kind{padding:0 6px;border-radius:3px;border:1px solid currentColor;font-size:11px}
.k-error{color:#ff7070}.k-finding{color:var(--cyan)}.k-question{color:var(--mag)}.k-answer{color:var(--grn)}
.text{white-space:pre-wrap;word-break:break-word}
footer{position:fixed;left:0;right:0;bottom:0;background:#080e08;border-top:1px solid var(--line);
       padding:10px 16px;display:flex;gap:8px;flex-wrap:wrap}
select,textarea,button{font:inherit;background:#0a120b;color:var(--txt);
       border:1px solid #22371f;border-radius:5px;padding:6px 8px;outline:none}
textarea{flex:1;min-width:320px;height:54px;resize:vertical}
button{background:#0d2a14;color:var(--grn);border-color:#2e6b3c;cursor:pointer;padding:6px 16px}
button:hover{background:#124020}
.hint{font-size:11.5px;color:var(--dim);padding:0 16px 70px}
code{color:var(--cyan)}
.fbtn{display:inline-flex;align-items:center;justify-content:center;width:38px;height:36px;
      border:1px solid #22371f;border-radius:5px;background:#0a120b;cursor:pointer;font-size:17px}
.fbtn:hover{background:#124020;border-color:#2e6b3c}
.att{font-size:12px;color:var(--cyan);max-width:220px;overflow:hidden;text-overflow:ellipsis;
     white-space:nowrap}
.att a{color:var(--cyan)}
.attimg{margin-top:6px}
.attimg img{max-width:100%;max-height:320px;border:1px solid var(--line);border-radius:5px;
            display:block;cursor:zoom-in}
.attfile{margin-top:6px;font-size:12.5px}
.attfile a{color:var(--amb);text-decoration:none;border-bottom:1px dotted var(--amb)}
</style></head><body>
<header>
  <h1>AB5 联调留言板</h1>
  <span class="tag" id="stat">—</span>
  <span class="tag">屏端 ↔ FPGA 端 · 交换错误结论</span>
</header>
<main><div id="list"></div></main>
<div class="hint">
  API：<code>GET /api/list?since=ID</code> ·
  <code>POST /api/post {"from","kind","text","ref"}</code> ·
  kind = error / finding / question / answer
</div>
<footer>
  <select id="from"><option>屏端</option><option>FPGA端</option></select>
  <select id="kind">
    <option value="error">问题</option><option value="finding">结论</option>
    <option value="question">提问</option><option value="answer">回复</option>
  </select>
  <textarea id="text" placeholder="写清楚：现象 / 证据 / 结论 / 待办。给数字，别给形容词。"></textarea>
  <input id="ref" placeholder="关联(可选)" style="width:130px">
  <label class="fbtn" title="添加附件（图片可预览，其它文件可下载；也可直接 Ctrl+V 粘贴截图）">📎
    <input type="file" id="file" onchange="pick()" style="display:none">
  </label>
  <span class="att" id="att"></span>
  <button onclick="send()">发送</button>
</footer>
<script>
let last=0, first=true;
const K={error:'问题',finding:'结论',question:'提问',answer:'回复'};
function esc(s){return (s||'').replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));}
async function pull(){
  try{
    const r=await fetch('api/list?since='+last);
    const d=await r.json();
    if(d.messages.length){
      const box=document.getElementById('list');
      if(first){box.innerHTML='';first=false;}
      for(const m of d.messages){
        last=Math.max(last,m.id);
        const el=document.createElement('div');
        el.className='msg'+(m.from.includes('FPGA')?' fpga':'');
        el.innerHTML='<div class="head"><span class="from">'+esc(m.from)+'</span>'+
          '<span class="kind k-'+m.kind+'">'+(K[m.kind]||m.kind)+'</span>'+
          '<span>#'+m.id+'</span><span>'+esc(m.ts)+'</span>'+
          (m.ref?'<span>↔ '+esc(m.ref)+'</span>':'')+'</div>'+
          '<div class="text">'+esc(m.text)+'</div>'+
          attHtml(m);
        box.appendChild(el);
      }
      window.scrollTo(0,document.body.scrollHeight);
    }
    if(first && d.messages.length===0){document.getElementById('list').innerHTML='<div class="tag">还没有记录。先发一条。</div>';}
  }catch(e){}
  const s=await fetch('api/stats').then(r=>r.json()).catch(()=>null);
  if(s) document.getElementById('stat').textContent='共 '+s.total+' 条 · 最近 #'+s.last_id;
}
function fmtSize(n){
  n=+n||0;
  if(n<1024) return n+' B';
  if(n<1048576) return (n/1024).toFixed(1)+' KB';
  return (n/1048576).toFixed(2)+' MB';
}
function attHtml(m){
  if(!m.att) return '';
  if(m.att_kind==='image')
    return '<div class="attimg"><a href="'+m.att+'" target="_blank"><img src="'+m.att+'" alt=""></a></div>';
  /* download 属性带值：浏览器保存时用原始文件名（磁盘上的文件名已把中文转成 _） */
  return '<div class="attfile"><a href="'+m.att+'" download="'+esc(m.att_name||'file')+'">📎 '
         +esc(m.att_name||'附件')+' · '+fmtSize(m.att_size)+'</a></div>';
}
let pending=null;
async function pick(){
  const f=document.getElementById('file').files[0];
  if(!f) return;
  await upload(f);
  document.getElementById('file').value='';
}
async function upload(f){
  const box=document.getElementById('att');
  box.textContent='上传中… ' + f.name;
  try{
    const r=await fetch('api/upload',{method:'POST',
      headers:{'X-Filename':encodeURIComponent(f.name),'Content-Type':'application/octet-stream'},
      body:f});
    const j=await r.json();
    if(!r.ok||!j.url){ box.textContent='上传失败：'+(j.error||r.status); return; }
    pending=j;
    box.innerHTML='已附加：<a href="'+j.url+'" target="_blank">'+esc(j.name)+'</a> · '+fmtSize(j.size)
                  +' <a href="#" onclick="clearAtt();return false;">✕</a>';
  }catch(e){ box.textContent='上传失败：网络错误'; }
}
function clearAtt(){ pending=null; document.getElementById('att').textContent=''; }
document.addEventListener('paste', async ev=>{
  const items=(ev.clipboardData||{}).items||[];
  for(const it of items){
    if(it.kind==='file'){ ev.preventDefault(); await upload(it.getAsFile()); return; }
  }
});

async function send(){
  const t=document.getElementById('text').value.trim();
  if(!t) return;
  await fetch('api/post',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({from:document.getElementById('from').value,
      kind:document.getElementById('kind').value,text:t,
      ref:document.getElementById('ref').value.trim(),
      att:pending?pending.url:'', att_name:pending?pending.name:'',
      att_kind:pending?pending.kind:'', att_size:pending?pending.size:0})});
  document.getElementById('text').value='';
  document.getElementById('ref').value='';
  clearAtt();
  pull();
}
setInterval(pull,2000); pull();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def _send(self, code, body, ctype='application/json; charset=utf-8'):
        b = body.encode('utf-8') if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(b)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Content-Length', '0')
        self.end_headers()

    @staticmethod
    def _norm(path):
        """去掉反代前缀，/board/api/list 与 /api/list 等价"""
        for pre in ('/board', '/ab5'):
            if path.startswith(pre):
                path = path[len(pre):] or '/'
        return path

    def do_GET(self):
        p = self._norm(self.path.split('?')[0])
        q = {}
        if '?' in self.path:
            for kv in self.path.split('?', 1)[1].split('&'):
                if '=' in kv:
                    k, v = kv.split('=', 1)
                    q[k] = v
        if p == '/':
            self._send(200, PAGE, 'text/html; charset=utf-8')
        elif p == '/api/list':
            since = int(q.get('since', 0) or 0)
            self._send(200, json.dumps({'messages': read_msgs(since),
                                        'last_id': STATE['id']}, ensure_ascii=False))
        elif p.startswith('/api/file/'):
            fn = os.path.basename(p[len('/api/file/'):])
            path = os.path.join(UPLOAD_DIR, fn)
            if not fn or not os.path.exists(path):
                self._send(404, json.dumps({'error': 'no such file'}))
                return
            try:
                with open(path, 'rb') as f:
                    data = f.read()
            except Exception as e:
                self._send(500, json.dumps({'error': str(e)}))
                return
            ext = os.path.splitext(fn)[1].lower()
            if ext in IMAGE_EXT:
                ctype = mimetypes.guess_type(fn)[0] or 'application/octet-stream'
                self._send(200, data, ctype)
            else:
                # 非图片一律 octet-stream + attachment：防止上传 .html/.svg 被当页面执行
                b = data
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Disposition',
                                 'attachment; filename="%s"' % fn.split('_', 1)[-1])
                self.send_header('Content-Length', str(len(b)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(b)
        elif p == '/api/stats':
            msgs = read_msgs(0)
            self._send(200, json.dumps({'total': len(msgs), 'last_id': STATE['id']},
                                       ensure_ascii=False))
        else:
            self._send(404, json.dumps({'error': 'not found'}))

    def do_POST(self):
        p = self._norm(self.path.split('?')[0])
        if p == '/api/upload':
            n = int(self.headers.get('Content-Length', 0) or 0)
            if n <= 0:
                self._send(400, json.dumps({'error': '空文件'}))
                return
            if n > MAX_UPLOAD:
                self._send(413, json.dumps({'error': '文件超过 8 MB'}))
                return
            raw = self.rfile.read(n)
            name = unquote(self.headers.get('X-Filename') or '')\
                   or ('file_%d.bin' % int(time.time()))
            name = os.path.basename(name)
            ext = os.path.splitext(name)[1].lower()
            if ext in BLOCK_EXT:
                self._send(400, json.dumps({'error': '出于安全不允许上传 %s 类型' % ext}))
                return
            safe = re.sub(r'[^A-Za-z0-9._-]', '_', name)[-60:]
            fn = '%d_%s' % (int(time.time() * 1000), safe)
            try:
                with open(os.path.join(UPLOAD_DIR, fn), 'wb') as f:
                    f.write(raw)
            except Exception as e:
                self._send(500, json.dumps({'error': '写文件失败: %s' % e}))
                return
            kind = 'image' if ext in IMAGE_EXT else 'file'
            self._send(200, json.dumps({'url': 'api/file/' + fn, 'name': name,
                                        'size': len(raw), 'kind': kind},
                                       ensure_ascii=False))
            return
        if p != '/api/post':
            self._send(404, json.dumps({'error': 'not found'}))
            return
        n = int(self.headers.get('Content-Length', 0) or 0)
        raw = self.rfile.read(n) if n else b'{}'
        try:
            m = json.loads(raw.decode('utf-8'))
        except Exception as e:
            self._send(400, json.dumps({'error': 'bad json: %s' % e}))
            return
        text = (m.get('text') or '').strip()
        if not text:
            self._send(400, json.dumps({'error': 'text 不能为空'}))
            return
        kind = m.get('kind') or 'finding'
        if kind not in KIND_CN:
            kind = 'finding'
        att = (m.get('att') or '').strip()[:200]
        msg = append_msg({'from': (m.get('from') or '未知').strip()[:20],
                          'kind': kind, 'text': text[:4000],
                          'ref': (m.get('ref') or '').strip()[:60],
                          'att': att, 'att_name': (m.get('att_name') or '').strip()[:80],
                          'att_kind': (m.get('att_kind') or 'file').strip()[:8],
                          'att_size': int(m.get('att_size') or 0),
                          'ip': self.client_address[0]})
        self._send(200, json.dumps(msg, ensure_ascii=False))

    def log_message(self, fmt, *args):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--bind', default='0.0.0.0')
    a = ap.parse_args()
    load_state()
    srv = ThreadingHTTPServer((a.bind, a.port), H)
    print('留言板已启动: http://%s:%d/  数据文件: %s' % (a.bind, a.port, STORE))
    print('同一网段的其他机器用 http://<本机IP>:%d/ 访问' % a.port)
    srv.serve_forever()


if __name__ == '__main__':
    main()
