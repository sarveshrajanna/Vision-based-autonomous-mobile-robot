import sys
import time
import config as C
MAX_SECONDS = 25.0
MAX_DUTY_CAP = 0.70
LOST_STOP = 12
marker = int(sys.argv[1]) if len(sys.argv) > 1 else 0
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
cap = float(sys.argv[3]) if len(sys.argv) > 3 else 0.55
if secs > MAX_SECONDS:
    sys.exit(f"{secs}s exceeds the {MAX_SECONDS}s cap - refusing")
if cap > MAX_DUTY_CAP:
    sys.exit(f"duty {cap} exceeds the {MAX_DUTY_CAP} cap - refusing")
C.GOAL_MARKER_ID = marker
C.MAX_DUTY = cap
from cam import Cam
from drive import Drive
from follow import Follower
from vision import Vision
print(f"GOTO marker {marker}: max {secs}s, duty cap {cap:.2f} "
      f"(~{cap*17.5/2:.1f} V per motor in series)", flush=True)
print(f"  stops at GOAL_STOP_CM = {C.GOAL_STOP_CM} cm "
      f"(assumes MARKER_WIDTH_CM = {C.MARKER_WIDTH_CM})", flush=True)
cam = Cam()
vis = Vision()
drv = Drive()
fol = Follower()
t0 = time.time()
lost = 0
why = "time limit"
try:
    while time.time() - t0 < secs:
        frame = cam.frame()
        out = vis.process(frame)
        goal = out.get("goal")
        if goal is None:
            lost += 1
            if lost >= LOST_STOP:
                why = f"marker not seen for {LOST_STOP} frames"
                break
            drv.stop()
            time.sleep(1.0 / C.HZ)
            continue
        lost = 0
        v, kappa, note, pivot = fol.step(goal, vis.marker_obstacles)
        if fol.arrived:
            why = "ARRIVED"
            break
        if pivot:
            drv.tank(-pivot * C.PIVOT_DUTY, pivot * C.PIVOT_DUTY, smooth=False)
        else:
            drv.arc(v, kappa)
        print(f"  t={time.time()-t0:5.1f}s  bearing {goal['bearing']:+6.1f}  "
              f"dist {goal['dist']:6.1f}cm  v={v:.2f} k={kappa:+.2f}  {note}",
              flush=True)
        time.sleep(1.0 / C.HZ)
finally:
    drv.stop()
    drv.close()
    cam.close()
print(f"\nstopped: {why}  after {time.time()-t0:.1f}s", flush=True)
