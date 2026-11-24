import math
import sys
import time
import cv2
import numpy as np
import config as C
from cam import Cam
from vision import Vision
d_cm = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
cam = Cam()
vis = Vision()
for _ in range(15):
    frame = cam.frame()
    out = vis.process(frame)
    time.sleep(0.08)
cam.close()
all_rows = vis.scan_rows(vis.mask)
ranges = out["ranges"]
best = None
for i, (r, d) in enumerate(zip(all_rows, ranges)):
    if r is not None and (best is None or d < ranges[best]):
        best = i
rows = [all_rows[best]] if best is not None else []
if best is not None:
    print(f"  using column {best} (the nearest detection, reported {ranges[best]:.0f} cm)")
cv2.imwrite("/tmp/tilt.jpg", vis.annotate(frame, out, f"object at {d_cm:.0f} cm"))
if not rows:
    print("No object detected dead ahead. Move it into the centre of the view.")
    sys.exit(1)
row = sum(rows) / len(rows)
true_depression = math.degrees(math.atan(C.CAM_HEIGHT_CM / d_cm))
off = ((row - C.CAM_H / 2.0) / (C.CAM_H / 2.0)) * (C.FOV_V_DEG / 2.0)
tilt = true_depression - off
print(f"  contact pixel row      {row:.0f}  (of {C.CAM_H})")
print(f"  true depression angle  {true_depression:.1f} deg   (atan {C.CAM_HEIGHT_CM}/{d_cm})")
print(f"  row offset from centre {off:+.1f} deg")
print(f"\n  CAM_TILT_DEG  = {tilt:.1f}")
if not 15 <= tilt <= 40:
    print("  ! outside the sensible 15-40 deg range - check the height and distance")
