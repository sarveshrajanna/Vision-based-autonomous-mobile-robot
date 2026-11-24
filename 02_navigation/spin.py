import json
import sys
import time
import urllib.request
import config as C
which = (sys.argv[1] if len(sys.argv) > 1 else "left").lower()
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
C.MAX_DUTY = float(sys.argv[3]) if len(sys.argv) > 3 else 0.95
from drive import Drive
if which not in ("left", "right"):
    sys.exit("first arg must be 'left' or 'right'")
direction = 1 if which == "left" else -1


def imu():
    try:
        with urllib.request.urlopen("http://127.0.0.1:8008/data", timeout=1.0) as r:
            d = json.loads(r.read())
        return d["heading"] if d.get("live") and d.get("state") == "running" else None
    except Exception:
        return None


def unwrap(prev, cur):
    return (cur - prev + 180.0) % 360.0 - 180.0
h = imu()
print(f"{which.upper()} spin {secs}s at SPIN_DUTY={C.SPIN_DUTY} cap={C.MAX_DUTY:.2f}"
      f"   IMU: {'yes' if h is not None else 'NOT AVAILABLE'}", flush=True)
d = Drive()
total = 0.0
t0 = time.time()
try:
    d.spin(direction)
    last = h
    while time.time() - t0 < secs:
        time.sleep(0.05)
        cur = imu()
        if cur is not None and last is not None:
            total += unwrap(last, cur)
        if cur is not None:
            last = cur
finally:
    d.stop()
    elapsed = time.time() - t0
time.sleep(0.6)
after = imu()
if after is not None and last is not None:
    total += unwrap(last, after)
print(f"stopped after {elapsed:.2f}s", flush=True)
if h is None or after is None:
    print("no IMU reading - spin not measured", flush=True)
else:
    print(f"  turned      {total:+.1f} deg", flush=True)
    print(f"  rate        {abs(total)/elapsed:.1f} deg/s", flush=True)
    print(f"  for 360 deg you would need {360.0*elapsed/max(1e-6,abs(total)):.2f} s",
          flush=True)
