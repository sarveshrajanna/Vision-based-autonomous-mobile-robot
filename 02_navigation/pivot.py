import json
import sys
import time
import urllib.request
import config as C
which = (sys.argv[1] if len(sys.argv) > 1 else "left").lower()
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
C.MAX_DUTY = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
sense = -1.0 if (len(sys.argv) > 4 and sys.argv[4].lower().startswith("b")) else 1.0
from drive import Drive
if which not in ("left", "right"):
    sys.exit("first arg must be 'left' or 'right'")
l, r = (sense, 0.0) if which == "left" else (0.0, sense)


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
print(f"PIVOT on {which.upper()} pair {'FORWARD' if sense > 0 else 'REVERSE'} "
      f"{secs}s at cap {C.MAX_DUTY:.2f} (~{C.MAX_DUTY*17.5:.1f} V)   "
      f"IMU: {'yes' if h is not None else 'NOT AVAILABLE'}", flush=True)
d = Drive()
total = 0.0
t0 = time.time()
try:
    d.tank(l, r, smooth=False)
    last = h
    while time.time() - t0 < secs:
        time.sleep(0.05)
        cur = imu()
        if cur is not None and last is not None:
            total += step(last, cur)
        if cur is not None:
            last = cur
finally:
    d.stop()
    elapsed = time.time() - t0
time.sleep(0.6)
after = imu()
if after is not None and last is not None:
    total += step(last, after)
print(f"stopped after {elapsed:.2f}s", flush=True)
if h is None or after is None:
    print("  no IMU reading - not measured", flush=True)
else:
    print(f"  turned      {total:+.1f} deg   ({'left/CCW' if total > 0 else 'right/CW'})",
          flush=True)
    print(f"  rate        {abs(total)/elapsed:.1f} deg/s", flush=True)
    if abs(total) > 2:
        print(f"  360 deg would take {360.0*elapsed/abs(total):.1f} s", flush=True)
