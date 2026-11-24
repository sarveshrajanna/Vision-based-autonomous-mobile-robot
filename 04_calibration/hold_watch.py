import json
import subprocess
import sys
import time
import urllib.request
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0


def data():
    with urllib.request.urlopen("http://127.0.0.1:8008/data", timeout=1) as f:
        return json.loads(f.read())


def throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"],
                             capture_output=True, text=True, timeout=2)
        return out.stdout.strip()
    except Exception:
        return "?"
t0 = time.time()
last = None
last_thr = throttled()
print(f"watching {dur:.0f}s - start holding a button", flush=True)
print(f"  t=0.00s  {last_thr}", flush=True)
while time.time() - t0 < dur:
    try:
        d = data()
        dr = d["drive"]
        cur = (dr["held"], round(dr["l"], 2), round(dr["r"], 2))
        if cur != last:
            t = time.time() - t0
            print(f"  t={t:6.2f}s  held={str(dr['held']):5s} "
                  f"l={dr['l']:+.2f} r={dr['r']:+.2f} duty={dr['duty']:.2f} "
                  f"imu_drops={d['dropouts']} err={dr['err'] or '-'}", flush=True)
            last = cur
        thr = throttled()
        if thr != last_thr:
            print(f"  t={time.time()-t0:6.2f}s  THROTTLE CHANGED: {thr}", flush=True)
            last_thr = thr
    except Exception as e:
        print(f"  t={time.time()-t0:6.2f}s  sample error: {e}", flush=True)
    time.sleep(0.08)
print(f"done. final {throttled()}", flush=True)
