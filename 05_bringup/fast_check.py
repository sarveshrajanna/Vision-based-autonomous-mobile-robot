import time
import config as C
C.MAX_DUTY = 0.70
from drive import Drive
STEPS = [
    ("LEFT side  FORWARD",  ( 1.0,  0.0)),
    ("LEFT side  BACKWARD", (-1.0,  0.0)),
    ("RIGHT side FORWARD",  ( 0.0,  1.0)),
    ("RIGHT side BACKWARD", ( 0.0, -1.0)),
    ("BOTH forward",        ( 1.0,  1.0)),
    ("BOTH backward",       (-1.0, -1.0)),
    ("SPIN left",           (-1.0,  1.0)),
    ("SPIN right",          ( 1.0, -1.0)),
]
d = Drive()
print(f"duty cap {C.MAX_DUTY:.2f} -> ~{C.MAX_DUTY*17.5:.1f} V at the motors", flush=True)
try:
    for label, (l, r) in STEPS:
        print(f">>> {label}", flush=True)
        d.tank(l, r, smooth=False)
        time.sleep(1.2)
        d.stop()
        time.sleep(0.5)
    print(">>> done", flush=True)
finally:
    d.close()
