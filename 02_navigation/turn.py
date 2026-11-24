import json
import sys
import time
import urllib.request
import config as C
target = float(sys.argv[1]) if len(sys.argv) > 1 else 360.0
C.MAX_DUTY = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
LEAD = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
from drive import Drive
TIMEOUT = max(8.0, abs(target) / 20.0 + 6.0)


def imu():
    try:
        with urllib.request.urlopen("http://127.0.0.1:8008/data", timeout=1.0) as f:
            d = json.loads(f.read())
        return d["heading"] if d.get("live") and d.get("state") == "running" else None
    except Exception:
        return None


def step(prev, cur):
    return (cur - prev + 180.0) % 360.0 - 180.0
h = imu()
if h is None:
    sys.exit("IMU not available - is imu_web.py running on port 8008?")
if target >= 0:
    l, r, side = 0.0, 1.0, "right"
else:
    l, r, side = 1.0, 0.0, "left"
print(f"TURN {target:+.0f} deg  pivot on {side.upper()} pair  "
      f"cap={C.MAX_DUTY:.2f}  lead={LEAD:.0f} deg", flush=True)
d = Drive()
total = 0.0
last = h
t0 = time.time()
stop_at = abs(target) - LEAD
reason = "reached target"
try:
    d.tank(l, r, smooth=False)
    while True:
        time.sleep(0.02)
        cur = imu()
        if cur is not None:
            total += step(last, cur)
            last = cur
        if abs(total) >= stop_at:
            break
        if time.time() - t0 > TIMEOUT:
            reason = f"TIMED OUT after {TIMEOUT:.0f}s"
            break
finally:
    d.stop()
    elapsed = time.time() - t0
    at_cut = total
for _ in range(25):
    time.sleep(0.04)
    cur = imu()
    if cur is not None:
        total += step(last, cur)
        last = cur
print(f"  {reason} in {elapsed:.2f}s", flush=True)
print(f"  at power cut   {at_cut:+.1f} deg", flush=True)
print(f"  after coast    {total:+.1f} deg   (coasted {abs(total - at_cut):.1f} deg)",
      flush=True)
print(f"  error vs {target:+.0f}   {total - target:+.1f} deg", flush=True)
print(f"  mean rate      {abs(at_cut)/max(1e-6, elapsed):.1f} deg/s", flush=True)
print(f"  suggested LEAD {abs(total - at_cut):.0f}", flush=True)
