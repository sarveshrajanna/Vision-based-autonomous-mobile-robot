import sys
import time
import config as C
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
C.MAX_DUTY = float(sys.argv[2]) if len(sys.argv) > 2 else 0.70
from drive import Drive
d = Drive()
print(f"FORWARD {secs:.1f}s at cap {C.MAX_DUTY:.2f} (~{C.MAX_DUTY*17.5:.1f} V)", flush=True)
try:
    d.tank(1.0, 1.0, smooth=False)
    time.sleep(secs)
finally:
    d.stop()
    d.close()
    print("stopped", flush=True)
