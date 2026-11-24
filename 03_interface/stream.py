import threading
import time
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
import config as C
_latest = {"jpg": None, "seq": 0}
_cmd = {"action": "stop", "at": 0.0}
_lock = threading.Lock()


def get_cmd(ttl=0.6):
    with _lock:
        if time.time() - _cmd["at"] > ttl:
            return "stop"
        return _cmd["action"]
PAGE = b"""<html><head><meta name=viewport content="width=device-width,initial-scale=1">
<style>
 body{margin:0;background:#111;color:#ddd;font:14px system-ui,sans-serif;text-align:center}
 img{width:100%;max-width:860px;display:block;margin:0 auto}
 #pad{display:grid;grid-template-columns:repeat(3,86px);gap:8px;justify-content:center;margin:14px auto}
 button{font-size:20px;padding:16px 0;border:1px solid #444;border-radius:8px;
        background:#222;color:#eee;touch-action:none;user-select:none;cursor:pointer}
 button:active{background:#d8541c;border-color:#d8541c}
 .wide{grid-column:1/4}
 #auto{background:#1b5e3a;border-color:#1b5e3a}
 #halt{background:#7a1f1f;border-color:#7a1f1f}
 small{color:#888}
</style></head><body>
<img src="/stream.mjpg">
<div id=pad>
  <div></div><button data-a=fwd>&#9650;</button><div></div>
  <button data-a=left>&#9664;</button>
  <button data-a=stop>&#9632;</button>
  <button data-a=right>&#9654;</button>
  <div></div><button data-a=back>&#9660;</button><div></div>
  <button class=wide id=auto data-a=auto>AUTO: drive to goal</button>
  <button class=wide id=halt data-a=halt>STOP AUTO</button>
</div>
<small>hold a button to drive &middot; release to stop</small>
<pre id=log style="text-align:left;max-width:860px;margin:12px auto;padding:10px;
 background:#000;color:#6f6;font:11px/1.45 ui-monospace,monospace;border-radius:8px;
 height:260px;overflow:auto;white-space:pre-wrap">waiting for log...</pre>
<script>
let timer=null;
const send=a=>fetch('/cmd?a='+a).catch(()=>{});
document.querySelectorAll('button').forEach(b=>{
  const a=b.dataset.a;
  if(a==='auto'||a==='halt'||a==='stop'){ b.onclick=()=>send(a); return; }
  const start=e=>{e.preventDefault(); send(a); clearInterval(timer); timer=setInterval(()=>send(a),150);};
  const end=e=>{e.preventDefault(); clearInterval(timer); send('stop');};
  b.addEventListener('pointerdown',start); b.addEventListener('pointerup',end);
  b.addEventListener('pointerleave',end); b.addEventListener('pointercancel',end);
});
const logEl=document.getElementById('log');
setInterval(()=>fetch('/log').then(r=>r.text()).then(t=>{
  const atEnd = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 20;
  logEl.textContent = t;
  if (atEnd) logEl.scrollTop = logEl.scrollHeight;
}).catch(()=>{}), 400);
addEventListener('keydown',e=>{const m={ArrowUp:'fwd',ArrowDown:'back',ArrowLeft:'left',ArrowRight:'right'};
  if(m[e.key]){e.preventDefault(); send(m[e.key]);}});
addEventListener('keyup',e=>{if(e.key.startsWith('Arrow')) send('stop');});
</script></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _authed(self):
        if not C.WEB_PASS:
            return True
        want = "Basic " + base64.b64encode(
            f"{C.WEB_USER}:{C.WEB_PASS}".encode()).decode()
        if self.headers.get("Authorization") == want:
            return True
        self.send_response(401)
        self.send_header("WWW-Authenticate", 'Basic realm="rover"')
        self.send_header("Content-Length", "0")
        self.end_headers()
        return False

    def do_GET(self):
        if not self._authed():
            return
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(PAGE)
            return
        if self.path == "/snap":
            jpg = _latest["jpg"]
            if jpg is None:
                self.send_error(503)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpg)
            return
        if self.path == "/log":
            try:
                with open("/tmp/service.log", "rb") as fh:
                    fh.seek(0, 2)
                    fh.seek(max(0, fh.tell() - 6000))
                    body = b"\n".join(fh.read().split(b"\n")[-28:])
            except OSError:
                body = b"(no log yet)"
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/cmd"):
            from urllib.parse import urlparse, parse_qs
            a = parse_qs(urlparse(self.path).query).get("a", ["stop"])[0]
            with _lock:
                _cmd["action"] = a
                _cmd["at"] = time.time()
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"ok")
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
        self.end_headers()
        try:
            last = -1
            while True:
                with _lock:
                    jpg, seq = _latest["jpg"], _latest["seq"]
                if jpg is None or seq == last:
                    time.sleep(0.02)
                    continue
                last = seq
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(C.STREAM_MIN_INTERVAL)
        except Exception:
            pass


def publish(frame):
    if C.STREAM_SCALE != 1.0:
        frame = cv2.resize(frame, None, fx=C.STREAM_SCALE, fy=C.STREAM_SCALE,
                           interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, C.STREAM_QUALITY])
    if ok:
        with _lock:
            _latest["jpg"] = buf.tobytes()
            _latest["seq"] += 1


def start(port):
    srv = ThreadingHTTPServer(("0.0.0.0", port), _Handler)
    srv.daemon_threads = True
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    return srv
