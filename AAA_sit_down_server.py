# -*- coding: utf-8 -*-
"""
趴下窗口可视化 - 本地上传服务器
用法: python AAA_sit_down_server.py [端口, 默认 8306]
启动后自动打开浏览器 http://127.0.0.1:端口/
在页面上传 charge_manager 日志和 reflective_column_node 日志,
自动生成可视化页面; 同时提供 txt 文本报告下载
"""
import sys, os, re, io, json, threading, tempfile, subprocess, webbrowser, traceback
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sitdown_web import parse_logs, build_html

# 多线程版: 上传大文件/解析期间不阻塞 /status 轮询; 且禁止端口复用,
# 避免旧进程残留时新进程绑定同一端口导致请求路由到旧代码(Windows 幽灵复用)
class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8306
state = {'status': 'idle', 'html': None, 'txt': None, 'err': '', 'info': ''}

INDEX = '''<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8"><title>趴下窗口分析 - 上传日志</title>
<style>
body { font: 14px/1.7 "Segoe UI", "Microsoft YaHei", sans-serif; background: #f4f6f8; }
.card { max-width: 640px; margin: 60px auto; background: #fff; border: 1px solid #dde;
        border-radius: 8px; padding: 30px 36px; box-shadow: 0 2px 8px rgba(0,0,0,.06); }
h1 { font-size: 19px; margin-bottom: 4px; }
.tip { color: #778; font-size: 13px; margin-bottom: 22px; }
.row { margin: 14px 0; }
.row label { display: block; font-weight: 600; margin-bottom: 4px; }
.row .hint { color: #99a; font-weight: normal; font-size: 12px; margin-left: 6px; }
input[type=file] { width: 100%; padding: 7px; border: 1px solid #ccd; border-radius: 4px; background: #fafbfd; }
button { margin-top: 18px; padding: 9px 30px; background: #2a7; color: #fff; border: none;
         border-radius: 4px; font-size: 14px; cursor: pointer; }
button:hover { background: #289; }
#bar { display: none; margin-top: 16px; color: #556; }
</style></head><body>
<div class="card">
  <h1>趴下窗口可视化分析</h1>
  <div class="tip">上传两份日志后自动生成: 左右柱坐标 + 机器人定位轨迹图、失败窗口列表、逐秒明细表。日志仅在本机解析, 不会上传到任何外部服务器。</div>
  <form action="/upload" method="post" enctype="multipart/form-data" target="_blank" onsubmit="document.getElementById('bar').style.display='block'">
    <div class="row">
      <label>charge_manager 日志<span class="hint">如 charge_manager.2026_0905.log 或 charge_manager_v5.0.22.2026_0907.log</span></label>
      <input type="file" name="charge" required>
    </div>
    <div class="row">
      <label>reflective_column_node 日志<span class="hint">如 reflective_column_node.2026_0905.log 或 reflective_column_node_v5.0.22.2026_0907.log</span></label>
      <input type="file" name="loc" required>
    </div>
    <button type="submit">开始分析</button>
  </form>
  <div id="bar">解析中… 反光柱日志较大时约需 1~2 分钟, 请耐心等待新标签页打开。</div>
</div>
</body></html>'''

WAITING = '''<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8"><title>解析中</title>
<style>body { font: 14px "Microsoft YaHei"; color: #445; margin-top: 120px; text-align: center; }
.dot { animation: blink 1.2s infinite; } @keyframes blink { 50% { opacity: .2; } }</style>
<script>
let n = 0;
function poll() {
  fetch('/status?' + Date.now()).then(r => r.json()).then(s => {
    if (s.status === 'done') { location.href = '/result'; return; }
    if (s.status === 'error') { document.body.innerHTML = '<pre style="color:#c33">' + s.err + '</pre>'; return; }
    n++; if (n % 6 === 0) document.getElementById('msg').textContent += ' .';
    setTimeout(poll, 1000);
  }).catch(() => setTimeout(poll, 2000));
}
setTimeout(poll, 800);
</script></head>
<body><div id="msg" class="dot">正在解析日志(约 1~2 分钟)</div></body></html>'''

class Handler(BaseHTTPRequestHandler):
    def _send(self, code, ctype, body, disp=None):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        if disp:
            self.send_header('Content-Disposition', disp)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/':
            self._send(200, 'text/html; charset=utf-8', INDEX.encode('utf-8'))
        elif self.path == '/result':
            if state['html']:
                self._send(200, 'text/html; charset=utf-8', state['html'].encode('utf-8'))
            else:
                self._send(200, 'text/html; charset=utf-8', b'<meta http-equiv="refresh" content="2">')
        elif self.path.startswith('/txt'):
            if state['txt'] and os.path.exists(state['txt']):
                with io.open(state['txt'], 'rb') as f:
                    self._send(200, 'text/plain; charset=utf-8', f.read(), 'attachment; filename="sitdown_windows.txt"')
            else:
                self._send(404, 'text/plain', b'no txt')
        elif self.path.startswith('/status'):
            self._send(200, 'application/json',
                       json.dumps({'status': state['status'], 'err': state['err']}).encode('utf-8'))
        else:
            self._send(404, 'text/plain', b'not found')

    def do_POST(self):
        if self.path != '/upload':
            self._send(404, 'text/plain', b'not found'); return
        if state['status'] == 'busy':
            self._send(409, 'text/html; charset=utf-8',
                       '<meta charset="utf-8">已有解析任务在进行中, 请等它完成后再上传。'.encode('utf-8')); return
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            files = parse_multipart(self.headers.get('Content-Type', ''), body)
            charge, loc = files.get('charge'), files.get('loc')
            if not charge or not loc:
                self._send(400, 'text/html; charset=utf-8', '两份日志都要选择'.encode('utf-8')); return
            self._send(200, 'text/html; charset=utf-8', WAITING.encode('utf-8'))
            threading.Thread(target=work, args=(charge, loc), daemon=True).start()
        except Exception as e:
            state['status'] = 'error'; state['err'] = f'{e}\n{traceback.format_exc()}'

    def log_message(self, fmt, *args):
        pass  # 静默访问日志

def parse_multipart(ctype, body):
    """极简 multipart/form-data 解析, 返回 {字段名: (文件名, bytes)}"""
    m = re.search(r'boundary=([^;]+)', ctype)
    if not m:
        return {}
    boundary = m.group(1).strip().strip('"').encode()
    files = {}
    for seg in body.split(b'--' + boundary):
        seg = seg.strip(b'\r\n')
        if not seg or seg == b'--' or b'\r\n\r\n' not in seg:
            continue
        head, data = seg.split(b'\r\n\r\n', 1)
        head_text = head.decode('utf-8', errors='replace')
        mn = re.search(r'name="([^"]+)"', head_text)
        fn = re.search(r'filename="([^"]+)"', head_text)
        if mn and mn.group(1):
            files[mn.group(1)] = (fn.group(1) if fn else '', data)
    return files

def work(charge, loc):
    state['status'] = 'busy'; state['err'] = ''
    tmpdir = tempfile.mkdtemp(prefix='sitdown_')
    try:
        cp = os.path.join(tmpdir, 'charge.log')
        lp = os.path.join(tmpdir, 'loc.log')
        with io.open(cp, 'wb') as f: f.write(charge[1])
        with io.open(lp, 'wb') as f: f.write(loc[1])
        data = parse_logs(cp, lp)
        data['charge'] = charge[0] or data['charge']
        data['loc'] = loc[0] or data['loc']
        state['html'] = build_html(data)
        state['info'] = f"窗口 {data['nWin']}, 成功 {data['nOk']}, 无电流 {data['nNc']}"
        # 同步生成 txt 报告
        txt = os.path.join(tmpdir, 'sitdown_windows.txt')
        try:
            here = os.path.dirname(os.path.abspath(__file__))
            subprocess.run([sys.executable, os.path.join(here, 'sitdown_windows.py'), cp, lp, txt], check=True)
            state['txt'] = txt
        except Exception:
            state['txt'] = None
        state['status'] = 'done'
    except Exception as e:
        state['status'] = 'error'
        state['err'] = f'{e}\n{traceback.format_exc()}'

def main():
    try:
        srv = Server(('127.0.0.1', PORT), Handler)
    except OSError as e:
        print(f'端口 {PORT} 被占用: {e}\n可能有旧的服务器进程残留, 请先关闭旧的命令行窗口(或任务管理器结束 python 进程)再启动。')
        sys.exit(1)
    url = f'http://127.0.0.1:{PORT}/'
    print(f'上传服务器已启动: {url}  (Ctrl+C 退出)')
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n已退出')

if __name__ == '__main__':
    main()
