import math
import sys
import time
from smbus2 import SMBus
import config as C
MAX_SECONDS = 60.0
MAX_DUTY_CAP = 0.65
TURN_BURST = 0.18
DRIVE_BURST = 0.40
CENTRE_TOL = 7.0
SEARCH_BURST = 0.30
STOP_FRAC = 0.34
SETTLE = 0.25
marker = int(sys.argv[1]) if len(sys.argv) > 1 else 0
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
turn_duty = float(sys.argv[3]) if len(sys.argv) > 3 else 0.55
drive_duty = float(sys.argv[4]) if len(sys.argv) > 4 else 0.55
if secs > MAX_SECONDS:
    sys.exit(f"{secs}s exceeds the {MAX_SECONDS}s cap - refusing")
if max(turn_duty, drive_duty) > MAX_DUTY_CAP:
    sys.exit(f"duty exceeds the {MAX_DUTY_CAP} cap - refusing")
C.GOAL_MARKER_ID = marker
C.MAX_DUTY = max(turn_duty, drive_duty)
from cam import Cam
from drive import Drive
from vision import Vision
ADDR = 0x68
ACCEL_LSB, GYRO_LSB = 16384.0, 16.4
bus = SMBus(1)


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


def imu_setup():
    bus.write_byte_data(ADDR, 0x6B, 0x00)
    time.sleep(0.05)
    bus.write_byte_data(ADDR, 0x1A, 0x03)
    bus.write_byte_data(ADDR, 0x1B, 0x18)
    bus.write_byte_data(ADDR, 0x1C, 0x00)
    time.sleep(0.05)


def imu_read():
    try:
        d = bus.read_i2c_block_data(ADDR, 0x3B, 14)
    except OSError:
        return None
    return (s16(d[0], d[1]) / ACCEL_LSB, s16(d[2], d[3]) / ACCEL_LSB,
            s16(d[4], d[5]) / ACCEL_LSB, s16(d[8], d[9]) / GYRO_LSB,
            s16(d[10], d[11]) / GYRO_LSB, s16(d[12], d[13]) / GYRO_LSB)
have_imu = True
try:
    for _ in range(15):
        try:
            imu_setup()
            break
        except OSError:
            time.sleep(0.15)
    else:
        raise OSError("no answer")
    cal = []
    while len(cal) < 90:
        s = imu_read()
        if s:
            cal.append(s)
        time.sleep(0.008)
    n = float(len(cal))
    bias = [sum(c[3 + k] for c in cal) / n for k in range(3)]
    mx, my, mz = (sum(c[k] for c in cal) / n for k in range(3))
    m = math.sqrt(mx * mx + my * my + mz * mz) or 1.0
    up = [mx / m, my / m, mz / m]
except Exception as e:
    print(f"  IMU unavailable ({e}) - search will not know when it has"
          f" swept a full turn", flush=True)
    have_imu = False
    bias, up = [0, 0, 0], [0, 0, 1]
swept = 0.0
_last_t = None


def sweep_update():
    global swept, _last_t
    if not have_imu:
        return
    s = imu_read()
    now = time.time()
    if s is None:
        return
    cx, cy, cz = s[3] - bias[0], s[4] - bias[1], s[5] - bias[2]
    rate = cx * up[0] + cy * up[1] + cz * up[2]
    if _last_t is not None:
        swept += rate * min(now - _last_t, 0.5)
    _last_t = now
cam = Cam()
vis = Vision()
drv = Drive()
turn_sign = 1
search_dir = 1
state = "SEARCH"
last_err = None
why = "time limit"
t0 = time.time()
print(f"SEEK marker {marker}: {secs}s max, turn duty {turn_duty:.2f}, "
      f"drive duty {drive_duty:.2f}", flush=True)
print(f"  arrives when the marker spans {STOP_FRAC*100:.0f}% of the frame "
      f"({int(STOP_FRAC*C.CAM_W)} px of {C.CAM_W})", flush=True)


def pivot(direction, seconds):
    C.MAX_DUTY = turn_duty
    l, r = (1.0, 0.0) if direction > 0 else (0.0, 1.0)
    drv.tank(l, r, smooth=False)
    t = time.time()
    while time.time() - t < seconds:
        sweep_update()
        time.sleep(0.01)
    drv.stop()
    time.sleep(SETTLE)


def advance(seconds):
    C.MAX_DUTY = drive_duty
    drv.tank(1.0, 1.0, smooth=False)
    t = time.time()
    while time.time() - t < seconds:
        sweep_update()
        time.sleep(0.01)
    drv.stop()
    time.sleep(SETTLE)
try:
    while time.time() - t0 < secs:
        frame = cam.frame()
        out = vis.process(frame)
        goal = out.get("goal")
        el = time.time() - t0
        if goal is None:
            if state != "SEARCH":
                print(f"  t={el:5.1f}s  lost the marker -> SEARCH", flush=True)
                state = "SEARCH"
                swept = 0.0
            if have_imu and abs(swept) > 380:
                why = "swept a full turn without finding it"
                break
            print(f"  t={el:5.1f}s  SEARCH   swept {swept:+6.0f} deg", flush=True)
            pivot(search_dir, SEARCH_BURST)
            continue
        bearing = float(goal["bearing"])
        width = float(goal["box"][2])
        frac = width / float(C.CAM_W)
        dist = float(goal["dist"])
        if frac >= STOP_FRAC:
            why = "ARRIVED"
            print(f"  t={el:5.1f}s  ARRIVED  bearing {bearing:+.1f}  "
                  f"width {width:.0f}px ({frac*100:.0f}%)  reported {dist:.0f}cm",
                  flush=True)
            break
        if abs(bearing) > CENTRE_TOL:
            dur = min(TURN_BURST * max(1.0, abs(bearing) / 12.0), 0.5)
            print(f"  t={el:5.1f}s  CENTRE   bearing {bearing:+6.1f}  "
                  f"width {width:3.0f}px  burst {dur:.2f}s", flush=True)
            state = "CENTRE"
            before = abs(bearing)
            pivot(turn_sign * (1 if bearing > 0 else -1), dur)

            f2 = cam.frame()
            g2 = vis.process(f2).get("goal")
            if g2 is not None and last_err is None:
                if abs(float(g2["bearing"])) > before + 2.0:
                    turn_sign = -turn_sign
                    print(f"           turn direction was backwards -> "
                          f"flipped (sign now {turn_sign:+d})", flush=True)
                last_err = True
            search_dir = turn_sign * (1 if bearing > 0 else -1)
            continue
        state = "ADVANCE"
        print(f"  t={el:5.1f}s  ADVANCE  bearing {bearing:+6.1f}  "
              f"width {width:3.0f}px ({frac*100:.0f}%)  reported {dist:.0f}cm",
              flush=True)
        advance(DRIVE_BURST)
finally:
    drv.stop()
    drv.close()
    cam.close()
print(f"\nstopped: {why}   after {time.time()-t0:.1f}s", flush=True)
