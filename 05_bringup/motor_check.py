import time
import config as C
from drive import Drive
STEPS = [
    ("LEFT wheel  FORWARD",  ( C.CRUISE, 0.0)),
    ("LEFT wheel  BACKWARD", (-C.CRUISE, 0.0)),
    ("RIGHT wheel FORWARD",  (0.0,  C.CRUISE)),
    ("RIGHT wheel BACKWARD", (0.0, -C.CRUISE)),
    ("BOTH forward (straight)",   ( C.CRUISE,  C.CRUISE)),
    ("BOTH backward",             (-C.CRUISE, -C.CRUISE)),
    ("SPIN left  (L back, R fwd)",(-C.SPIN_DUTY, C.SPIN_DUTY)),
    ("SPIN right (L fwd, R back)",( C.SPIN_DUTY,-C.SPIN_DUTY)),
]
d = Drive()
try:
    for label, (l, r) in STEPS:
        print(f">>> {label}", flush=True)
        d.tank(l, r, smooth=False)
        time.sleep(1.5)
        d.stop()
        time.sleep(0.8)
    print(">>> smooth LEFT arc", flush=True)
    d.arc(C.CRUISE, +1.2)
    time.sleep(2.0)
    print(">>> smooth RIGHT arc", flush=True)
    d.arc(C.CRUISE, -1.2)
    time.sleep(2.0)
    print(">>> done", flush=True)
finally:
    d.close()
