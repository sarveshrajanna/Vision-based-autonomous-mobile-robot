import subprocess
import sys
import time
import config as C
which = (sys.argv[1] if len(sys.argv) > 1 else "left").lower()
C.MAX_DUTY = float(sys.argv[2]) if len(sys.argv) > 2 else 0.45
from drive import Drive
if which not in ("left", "right"):
    sys.exit("first arg must be 'left' or 'right'")
l, r = (1.0, 0.0) if which == "left" else (0.0, 1.0)
L = (C.LEFT_EN, C.LEFT_IN1, C.LEFT_IN2)
R = (C.RIGHT_EN, C.RIGHT_IN1, C.RIGHT_IN2)
allpins = ",".join(str(p) for p in L + R)


def read_pins():
    try:
        out = subprocess.run(["/usr/bin/pinctrl", "get", allpins],
                             capture_output=True, text=True, timeout=3)
        return out.stdout.strip()
    except Exception as e:
        return f"pinctrl failed: {e}"
print(f"left  EN,IN1,IN2 = GPIO {L[0]},{L[1]},{L[2]}")
print(f"right EN,IN1,IN2 = GPIO {R[0]},{R[1]},{R[2]}")
print("\n--- idle (nothing commanded) ---")
d = Drive()
print(read_pins())
print(f"\n--- {which.upper()} pair commanded forward at cap {C.MAX_DUTY:.2f} ---")
try:
    d.tank(l, r, smooth=False)
    for i in range(3):
        time.sleep(0.7)
        print(f"[sample {i + 1}]")
        print(read_pins())
finally:
    d.stop()
    print("\nstopped")
