import math
import os
import signal
import time
import config as C
import cv2
import numpy as np
import follow
import stream
import vision as V
from cam import Cam
from drive import Drive
GO = "/tmp/rover_go"
running = True


def bye(*_):
    global running
    running = False
signal.signal(signal.SIGINT, bye)
signal.signal(signal.SIGTERM, bye)
cam, vis, drv = Cam(), V.Vision(), Drive()
stream.start(C.STREAM_PORT)
dt = 1.0 / C.HZ
mid = C.N_COLS // 2
fol = follow.Follower()
_prev_gray = None
_win = None
_FX = (C.CAM_W / 2.0) / math.tan(math.radians(C.FOV_H_DEG / 2.0))


def measure_turn(frame):
    global _prev_gray, _win
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    g = cv2.resize(g, (C.CAM_W // 2, C.CAM_H // 2), interpolation=cv2.INTER_AREA)
    g = g.astype(np.float32)
    if _prev_gray is None or _prev_gray.shape != g.shape:
        _prev_gray, _win = g, cv2.createHanningWindow((g.shape[1], g.shape[0]), cv2.CV_32F)
        return 0.0, 0.0
    try:
        (dx, _), conf = cv2.phaseCorrelate(_prev_gray * _win, g * _win)
    except cv2.error:
        dx, conf = 0.0, 0.0
    _prev_gray = g
    if conf < C.TURN_CONF_MIN:
        return 0.0, conf

    return math.degrees(math.atan2(dx * 2.0, _FX)), conf
driving = False
turn_ticks = 0
man_cmd, man_on, man_off = None, 0, 0
t_run = 0.0
print(f"service up, streaming on :{C.STREAM_PORT}. touch {GO} to drive.", flush=True)
try:
    while running:
        t0 = time.time()
        frame = cam.frame()
        out = vis.process(frame)
        ranges = out["ranges"]
        goal = out.get("goal")
        cmd = stream.get_cmd()
        if cmd == "auto":
            open(GO, "w").close()
        elif cmd == "halt":
            if os.path.exists(GO):
                os.remove(GO)
        manual = cmd in ("fwd", "back", "left", "right")
        want = os.path.exists(GO) and not manual
        if want and not driving:
            fol = follow.Follower()
            turn_ticks, t_run = 0, time.time()
            print("--- GO ---", flush=True)
        if driving and not want:
            drv.stop()
            print("--- STOP ---", flush=True)
        driving = want
        front = min(ranges[mid - 1:mid + 2])
        turned_deg, _ = measure_turn(frame)
        fol.observe_turn(turned_deg)


        if C.AVOID and C.OBSTACLE_MODE == "bright":
            vis.last_obstacles = vis.find_bright_obstacles(frame, goal)
        else:
            vis.last_obstacles = []
        if manual or man_on or man_off:
            if manual and not man_on and not man_off:
                man_cmd, man_on = cmd, C.MANUAL_ON
            if man_on:
                d, t = C.MANUAL_DUTY, C.MANUAL_TURN
                if man_cmd == "fwd":
                    drv.tank(d, d, smooth=False)
                elif man_cmd == "back":
                    drv.tank(-d, -d, smooth=False)
                elif man_cmd == "left":
                    drv.tank(-t, t, smooth=False)
                else:
                    drv.tank(t, -t, smooth=False)
                man_on -= 1
                if not man_on:
                    man_off = C.MANUAL_OFF
                note = f"MANUAL {man_cmd} step ({man_on} left)"
            else:
                drv.stop()
                man_off -= 1
                note = f"MANUAL settle ({man_off})"
        elif not driving:
            drv.stop()
            note = f"IDLE  front={front:.0f}cm   (waiting)"
        else:
            v, kappa, why, pivot = fol.step(goal, vis.last_obstacles if C.AVOID else [])


            if goal is None and (abs(kappa) > 0.15 or pivot):
                turn_ticks += 1
            else:
                turn_ticks = 0
            if turn_ticks > C.TURN_CLAMP_TICKS:
                kappa, pivot = 0.0, 0
                why = "TURN CLAMPED (" + why + ")"
            if pivot:
                drv.tank(-pivot * C.PIVOT_DUTY, pivot * C.PIVOT_DUTY, smooth=False)
            elif v <= 0:
                drv.stop()
            else:
                drv.arc(v, kappa)
            note = f"TRACK {why}"
        stream.publish(vis.annotate(frame, out, note + f"  |turned {fol.blind_deg:+.0f}deg"))
        if driving:
            print(f"{time.time() - t_run:5.1f}s [{1.0 / max(time.time() - t0, 1e-3):4.1f}Hz]"
                  f" turn{fol.blind_deg:+5.0f}d  {note}  {[round(r) for r in ranges]}",
                  flush=True)
        sleep = dt - (time.time() - t0)
        if sleep > 0:
            time.sleep(sleep)
finally:
    drv.close()
    cam.close()
    print("service stopped, motors off")
