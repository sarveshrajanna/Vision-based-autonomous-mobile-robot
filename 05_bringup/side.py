import sys
import time
import config as C
which = (sys.argv[1] if len(sys.argv) > 1 else "left").lower()
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5
C.MAX_DUTY = float(sys.argv[3]) if len(sys.argv) > 3 else 0.45
from drive import Drive
if which not in ("left", "right"):
    sys.exit("first arg must be 'left' or 'right'")
l, r = (1.0, 0.0) if which == "left" else (0.0, 1.0)
pins = (C.LEFT_EN, C.LEFT_IN1, C.LEFT_IN2) if which == "left" else (C.RIGHT_EN, C.RIGHT_IN1, C.RIGHT_IN2)
print(f"{which.upper()} pair FORWARD for {secs}s at cap {C.MAX_DUTY:.2f} "
      f"(~{C.MAX_DUTY*17.5:.1f} V)  [EN,IN1,IN2 = GPIO {pins[0]},{pins[1]},{pins[2]}]",
      flush=True)
d = Drive()
try:
    d.tank(l, r, smooth=False)
    time.sleep(secs)
finally:
    d.stop()
    print("stopped", flush=True)
