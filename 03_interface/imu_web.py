import base64
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from smbus2 import SMBus
ADDR = 0x68
WHO_AM_I = 0x75
PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
GYRO_CFG = 0x1B
ACCEL_CFG = 0x1C
ACCEL_XOUT = 0x3B
ACCEL_LSB = 16384.0
GYRO_LSB = 16.4
PORT = 8008
RATE = 50.0
CAL_SAMPLES = 150
STILL_DPS = 8.0
MAX_DT = 1.0


_lock = threading.Lock()
S = {
    "live": False,
    "state": "waiting",
    "who": None,
    "heading": 0.0,
    "yaw_rate": 0.0,
    "acc": [0.0, 0.0, 0.0],
    "gyro": [0.0, 0.0, 0.0],
    "amag": 0.0,
    "tilt": 0.0,
    "up": [0.0, 0.0, 1.0],
    "bias": [0.0, 0.0, 0.0],
    "ok": 0,
    "bad": 0,
    "streak_bad": 0,
    "dropouts": 0,
    "last_err": "",
    "cal_progress": 0.0,
    "hz": 0.0,
    "drift_dps": 0.0,
    "trail": [],
    "total": 0.0,


    "_recal": False,
}


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


def configure(bus):
    who = bus.read_byte_data(ADDR, WHO_AM_I)
    bus.write_byte_data(ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.05)
    bus.write_byte_data(ADDR, CONFIG, 0x03)
    bus.write_byte_data(ADDR, GYRO_CFG, 0x18)
    bus.write_byte_data(ADDR, ACCEL_CFG, 0x00)
    time.sleep(0.05)
    return who


def reader():
    bus = SMBus(1)
    inited = False
    cal = []
    bias = [0.0, 0.0, 0.0]
    up = [0.0, 0.0, 1.0]
    heading = 0.0
    total = 0.0
    last_t = None
    ticks = 0
    t_hz = time.time()
    still_since = None
    still_start_heading = 0.0
    while True:
        try:
            d = bus.read_i2c_block_data(ADDR, ACCEL_XOUT, 14)
            if not inited:
                raise OSError("not configured")
            now = time.time()
            with _lock:
                if S["_recal"]:
                    S["_recal"] = False
                    cal = []
                    last_t = None
            ax, ay, az = (s16(d[i], d[i + 1]) / ACCEL_LSB for i in (0, 2, 4))
            gx, gy, gz = (s16(d[i], d[i + 1]) / GYRO_LSB for i in (8, 10, 12))
            amag = math.sqrt(ax * ax + ay * ay + az * az)
            if len(cal) < CAL_SAMPLES:
                cal.append((ax, ay, az, gx, gy, gz))
                with _lock:
                    S["live"] = True
                    S["state"] = "calibrating"
                    S["cal_progress"] = len(cal) / CAL_SAMPLES
                    S["acc"] = [ax, ay, az]
                    S["gyro"] = [gx, gy, gz]
                    S["amag"] = amag
                if len(cal) == CAL_SAMPLES:
                    n = float(len(cal))
                    bias = [sum(c[3 + k] for c in cal) / n for k in range(3)]
                    mx, my, mz = (sum(c[k] for c in cal) / n for k in range(3))
                    m = math.sqrt(mx * mx + my * my + mz * mz) or 1.0
                    up = [mx / m, my / m, mz / m]
                    spread = max(
                        max(abs(c[3 + k] - bias[k]) for c in cal) for k in range(3)
                    )
                    heading = 0.0
                    total = 0.0
                    last_t = None
                    still_since = now
                    still_start_heading = 0.0
                    with _lock:
                        S["bias"] = list(bias)
                        S["up"] = list(up)
                        S["tilt"] = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
                        S["state"] = "running"
                        S["last_err"] = (
                            "" if spread < STILL_DPS
                            else f"moved during calibration (spread {spread:.1f} dps) - hit Recalibrate while holding still"
                        )
                continue

            cx, cy, cz = gx - bias[0], gy - bias[1], gz - bias[2]
            yaw_rate = cx * up[0] + cy * up[1] + cz * up[2]
            if last_t is not None:
                step_deg = yaw_rate * min(now - last_t, MAX_DT)
                heading += step_deg
                heading = (heading + 180.0) % 360.0 - 180.0
                total += step_deg
            last_t = now

            moving = abs(yaw_rate) > 2.0 or abs(amag - 1.0) > 0.15
            if moving:
                still_since = None
            elif still_since is None:
                still_since = now
                still_start_heading = heading
            drift = 0.0
            if still_since is not None and now - still_since > 3.0:
                drift = (heading - still_start_heading) / (now - still_since)
            ticks += 1
            with _lock:
                S["live"] = True
                S["ok"] += 1
                S["streak_bad"] = 0
                S["heading"] = heading
                S["total"] = total
                S["yaw_rate"] = yaw_rate
                S["acc"] = [ax, ay, az]
                S["gyro"] = [gx, gy, gz]
                S["amag"] = amag
                S["drift_dps"] = drift
                S["trail"] = (S["trail"] + [yaw_rate])[-180:]
                if now - t_hz >= 1.0:
                    S["hz"] = ticks / (now - t_hz)
                    ticks = 0
                    t_hz = now
        except OSError as e:
            with _lock:
                if S["live"]:
                    S["dropouts"] += 1
                S["live"] = False
                S["bad"] += 1
                S["streak_bad"] += 1
                S["state"] = "waiting"
                S["last_err"] = f"{e}"
            inited = False


            time.sleep(0.2)
            try:
                who = configure(bus)
                inited = True
                with _lock:
                    S["who"] = f"0x{who:02x}"
            except OSError:
                pass
            continue
        time.sleep(1.0 / RATE)
DEADMAN_S = 0.45
_dlock = threading.Lock()
D = {
    "drv": None,
    "l": 0.0,
    "r": 0.0,
    "duty": 0.6,
    "until": 0.0,
    "err": "",
    "held": False,
}


def _get_drive():
    if D["drv"] is None:
        try:
            from drive import Drive
            D["drv"] = Drive()
            D["err"] = ""
        except Exception as e:
            D["err"] = f"{type(e).__name__}: {e}"[:140]
            return None, D["err"]
    return D["drv"], ""


def drive_cmd(l, r, duty):
    import config as CFG
    with _dlock:
        d, err = _get_drive()
        if d is None:
            return err
        CFG.MAX_DUTY = max(0.15, min(1.0, duty))
        D["l"], D["r"], D["duty"] = l, r, duty
        D["until"] = time.time() + DEADMAN_S
        D["held"] = True
        try:
            d.tank(l, r, smooth=False)
        except Exception as e:
            D["err"] = f"{type(e).__name__}: {e}"[:140]
            return D["err"]
        return ""


def drive_stop():
    with _dlock:
        D["l"] = D["r"] = 0.0
        D["until"] = 0.0
        D["held"] = False
        if D["drv"] is not None:
            try:
                D["drv"].stop()
            except Exception:
                pass


def drive_release():
    with _dlock:
        if D["drv"] is not None:
            try:
                D["drv"].close()
            except Exception:
                pass
            D["drv"] = None
        D["l"] = D["r"] = 0.0
        D["until"] = 0.0
        D["held"] = False
        D["err"] = ""


def watchdog():
    while True:
        time.sleep(0.05)
        with _dlock:
            if D["held"] and time.time() > D["until"]:
                D["l"] = D["r"] = 0.0
                D["held"] = False
                if D["drv"] is not None:
                    try:
                        D["drv"].stop()
                    except Exception:
                        pass
CAM_HZ = 15.0
CAM_MIN_SLEEP = 0.012
VISION_EVERY = 3
JPEG_QUALITY = 68
_clock = threading.Lock()
CAMS = {
    "jpeg": None,
    "seq": 0,
    "ids": [],
    "goal": None,
    "ranges": [],
    "err": "",
    "fps": 0.0,
    "markers": [],
    "obs": [],
    "polarity": "",
}
import config as C_OBS


def camera():
    try:
        import cv2
        from cam import Cam
        from vision import Vision
    except Exception as e:
        with _clock:
            CAMS["err"] = f"import failed: {type(e).__name__}: {e}"[:160]
        return
    cam = vis = None
    ticks = 0
    vticks = 0
    last_out = None
    last_obs = []
    t_fps = time.time()
    now = time.time()
    while True:
        if cam is None:
            try:
                cam = Cam()
                vis = Vision()
                with _clock:
                    CAMS["err"] = ""
            except Exception as e:
                with _clock:
                    CAMS["err"] = f"{type(e).__name__}: {e}"[:160]
                time.sleep(2.0)
                continue
        try:
            frame = cam.frame()
            vticks += 1
            if vticks % VISION_EVERY == 0 or last_out is None:
                last_out = vis.process(frame)
            out = last_out
            ids = out.get("found_ids") or []
            goal = out.get("goal")


            try:
                with _seek_lock:
                    seeking = SK["active"]
                if seeking or vticks % VISION_EVERY != 0:
                    obs = last_obs
                else:
                    obs = vis.find_bright_obstacles(frame, goal)
                    last_obs = obs
            except Exception:
                obs = []
            vis.last_obstacles = obs
            label = f"ids {sorted(ids)}" if ids else "no marker"
            if obs:
                label += f"  obs x{len(obs)}"
            shown = vis.annotate(frame, out, label)


            for m in getattr(vis, "all_markers", []):
                x, y, w, h = m["box"]
                is_goal = int(m["id"]) == int(C_OBS.GOAL_MARKER_ID)
                colour = (0, 255, 0) if is_goal else (255, 170, 0)
                cv2.rectangle(shown, (x, y), (x + w, y + h), colour, 2)
                tag = f"{m['id']}" + (" GOAL" if is_goal else "")
                cv2.putText(shown, tag, (x, max(14, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            ok, buf = cv2.imencode(".jpg", shown,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            ticks += 1
            now = time.time()
            with _clock:
                if ok:
                    CAMS["jpeg"] = buf.tobytes()
                    CAMS["seq"] += 1
                CAMS["ids"] = sorted(int(i) for i in ids)

                CAMS["goal"] = ({"bearing": round(float(goal["bearing"]), 1),
                                 "dist": round(float(goal["dist"]), 1),
                                 "width": int(goal["box"][2]),
                                 "frac": round(goal["box"][2] / float(C_OBS.CAM_W), 3)}
                                if isinstance(goal, dict) else None)


                for mid in CAMS["ids"]:
                    SEEN[int(mid)] = now
                CAMS["markers"] = [
                    {"id": int(m["id"]),
                     "bearing": round(float(m["bearing"]), 1),
                     "frac": round(m["box"][2] / float(C_OBS.CAM_W), 3)}
                    for m in getattr(vis, "all_markers", [])]
                CAMS["ranges"] = [None if r is None else round(float(r))
                                  for r in out.get("ranges", [])]
                CAMS["obs"] = [{"bearing": round(float(o["bearing"]), 1),
                                "dist": round(float(o["dist"]), 1)}
                               for o in obs]
                CAMS["polarity"] = C_OBS.OBS_POLARITY
                CAMS["err"] = ""
                if now - t_fps >= 1.0:
                    CAMS["fps"] = ticks / (now - t_fps)
                    ticks = 0
                    t_fps = now
        except Exception as e:
            with _clock:
                CAMS["err"] = f"{type(e).__name__}: {e}"[:160]
            try:
                cam.close()
            except Exception:
                pass
            cam = None
            last_out = None
            time.sleep(1.0)
            continue


        time.sleep(max(CAM_MIN_SLEEP, 1.0 / CAM_HZ - (time.time() - now)))
SEEN = {}
TURN_RATE_DPS = 31.6
TURN_COAST_DEG = 4.6
MIN_BURST = 0.06
MAX_BURST = 0.45
CENTRE_TOL = 5.0
EDGE_ABORT = 12.0


DRIVE_BURST = 0.18
SEARCH_BURST = 0.20
SETTLE = 0.40
STOP_FRAC = 0.34
SEEK_TURN_DUTY = 0.48
SEEK_DRIVE_DUTY = 0.42
SEEK_MAX_S = 150.0
_seek_lock = threading.Lock()
SK = {
    "active": False,
    "target": None,
    "state": "idle",
    "note": "",
    "sign": 1,
    "probed": False,
    "swept": 0.0,
    "started": 0.0,
}
_seek_cancel = threading.Event()


def _burst(l, r, duty, seconds, watch_bearing=False, cancel=None):
    if cancel is None:
        cancel = _seek_cancel
    t = time.time()
    while time.time() - t < seconds:
        if cancel.is_set():
            break
        drive_cmd(l, r, duty)
        if watch_bearing:
            with _clock:
                g = CAMS["goal"]
            if g is not None and abs(float(g["bearing"])) > EDGE_ABORT:
                break
        time.sleep(0.05)
    drive_stop()
    time.sleep(SETTLE)


def _set(**kw):
    with _seek_lock:
        SK.update(kw)


def seek_worker(target):
    import config as CFG
    _set(active=True, target=target, state="SEARCH", note="switching target",
         swept=0.0, started=time.time())


    with _clock:
        seq0 = CAMS["seq"]
    CFG.GOAL_MARKER_ID = target
    t_wait = time.time()
    while time.time() - t_wait < 3.0:
        with _clock:
            if CAMS["seq"] - seq0 >= 2:
                break
        if _seek_cancel.is_set():
            _set(active=False, state="stopped", note="cancelled")
            return
        time.sleep(0.05)
    _set(state="SEARCH", note="")


    with _seek_lock:
        sign = SK["sign"]
        probed = SK["probed"]
    search_dir = 1
    h0 = None


    last_bearing = None
    prev_bearing = None
    try:
        while not _seek_cancel.is_set():
            if time.time() - SK["started"] > SEEK_MAX_S:
                _set(state="stopped", note="time limit")
                return
            with _clock:
                goal = CAMS["goal"]
            with _lock:
                heading = S["total"]
            if goal is None:
                if h0 is None:
                    h0 = heading


                    if last_bearing is not None:
                        if abs(last_bearing) > 2.0:
                            towards = 1 if last_bearing > 0 else -1
                        elif prev_bearing is not None:
                            drift = last_bearing - prev_bearing
                            towards = 1 if drift > 0 else -1
                        else:
                            towards = 1
                        search_dir = sign * towards
                        _set(note=f"lost at {last_bearing:+.0f} deg, "
                                  f"sweeping back toward it")
                swept = heading - h0
                _set(state="SEARCH", note=f"swept {swept:+.0f} deg", swept=swept)
                if abs(swept) > 380:
                    _set(state="stopped", note="full turn, marker not found")
                    return
                _burst(1.0 if search_dir > 0 else 0.0,
                       0.0 if search_dir > 0 else 1.0,
                       SEEK_TURN_DUTY, SEARCH_BURST)
                continue
            h0 = None
            bearing = float(goal["bearing"])
            frac = float(goal["frac"])
            prev_bearing, last_bearing = last_bearing, bearing
            if frac >= STOP_FRAC:
                _set(state="ARRIVED",
                     note=f"{goal['width']}px, {frac*100:.0f}% of frame")
                return
            if abs(bearing) > CENTRE_TOL:
                before = abs(bearing)
                d = sign * (1 if bearing > 0 else -1)

                powered = max(0.0, before - TURN_COAST_DEG)
                dur = max(MIN_BURST, min(powered / TURN_RATE_DPS, MAX_BURST))
                _set(state="CENTRE",
                     note=f"bearing {bearing:+.1f}, turning {powered:.0f}deg")
                _burst(1.0 if d > 0 else 0.0, 0.0 if d > 0 else 1.0,
                       SEEK_TURN_DUTY, dur)
                if not probed:
                    time.sleep(0.2)
                    with _clock:
                        g2 = CAMS["goal"]
                    if g2 is not None:
                        if abs(float(g2["bearing"])) > before + 2.0:
                            sign = -sign
                            _set(note="turn sign was backwards, flipped")
                        probed = True
                        _set(sign=sign, probed=True)
                search_dir = sign * (1 if bearing > 0 else -1)
                continue
            _set(state="ADVANCE", note=f"bearing {bearing:+.1f}, "
                                       f"{frac*100:.0f}% of frame")
            _burst(1.0, 1.0, SEEK_DRIVE_DUTY, DRIVE_BURST, watch_bearing=True)
    finally:
        drive_stop()
        _set(active=False)


def seek_start(target):
    with _seek_lock:
        if SK["active"]:
            return "already running"
    _seek_cancel.clear()
    threading.Thread(target=seek_worker, args=(target,), daemon=True).start()
    return ""


def seek_stop():
    _seek_cancel.set()
    drive_stop()
    _set(active=False, state="stopped", note="stopped by user")
SCAN_BURST = 0.16
SCAN_SETTLE = 0.45
SCAN_DUTY = 0.48
SCAN_MAX_S = 120.0
DEG_PER_BURST = 9.7
MAX_BURSTS = 60
_scan_lock = threading.Lock()
SC = {
    "active": False,
    "state": "idle",
    "swept": 0.0,
    "found": {},
    "note": "",
}
_scan_cancel = threading.Event()


def _scan_set(**kw):
    with _scan_lock:
        SC.update(kw)


def scan_worker():
    _scan_cancel.clear()
    _scan_set(active=True, state="scanning", swept=0.0, found={}, note="")
    with _lock:
        h0 = S["total"]
        imu_ok_at_start = S["live"]
    found = {}
    t0 = time.time()
    direction = 1
    bursts = 0
    imu_failed = False
    try:
        while not _scan_cancel.is_set():
            with _lock:
                heading = S["total"]
                imu_live = S["live"] and S["state"] == "running"


            if imu_live:
                swept = heading - h0
            else:
                imu_failed = True
                swept = bursts * DEG_PER_BURST * direction
            if abs(swept) >= 360.0:
                _scan_set(state="done", swept=round(swept, 1),
                          note=(f"{len(found)} marker(s) mapped"
                                + (" - GYRO DIED, headings are estimated"
                                   if imu_failed else "")))
                return
            if bursts >= MAX_BURSTS:
                _scan_set(state="done", swept=round(swept, 1),
                          note=f"stopped after {bursts} steps - "
                               f"{len(found)} marker(s), gyro unreliable")
                return
            if time.time() - t0 > SCAN_MAX_S:
                _scan_set(state="done", swept=round(swept, 1),
                          note=f"timed out - {len(found)} marker(s)")
                return
            with _clock:
                markers = list(CAMS["markers"])
            for m in markers:
                mid = int(m["id"])
                absolute = (heading if imu_live else swept) + float(m["bearing"])
                prev = found.get(mid)


                if prev is None or m["frac"] > prev["frac"]:
                    found[mid] = {"heading": round(absolute, 1),
                                  "bearing": m["bearing"],
                                  "frac": m["frac"],
                                  "swept": round(swept, 1),
                                  "imu": bool(imu_live)}
            _scan_set(swept=round(swept, 1), found=dict(found),
                      note=f"{len(found)} found so far")
            time.sleep(0.02)
            _burst(1.0 if direction > 0 else 0.0,
                   0.0 if direction > 0 else 1.0,
                   SCAN_DUTY, SCAN_BURST, cancel=_scan_cancel)
            bursts += 1
            time.sleep(SCAN_SETTLE - SETTLE if SCAN_SETTLE > SETTLE else 0)
        _scan_set(state="stopped", note="cancelled")
    finally:
        drive_stop()
        _scan_set(active=False)


def scan_start():
    with _scan_lock:
        if SC["active"]:
            return "already scanning"
    with _seek_lock:
        if SK["active"]:
            return "a seek is running"
    _scan_cancel.clear()
    threading.Thread(target=scan_worker, daemon=True).start()
    return ""


def scan_stop():
    _scan_cancel.set()
    drive_stop()
    _scan_set(active=False, state="stopped", note="stopped by user")
PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Rover</title><style>
*{box-sizing:border-box}
html,body{height:100%;margin:0}
body{background:#0f1115;color:#e7e9ee;
  font:16px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace;
  display:flex;flex-direction:column;overflow:hidden}
.strip{display:flex;gap:12px;align-items:center;flex-wrap:wrap;
  padding:6px max(10px,env(safe-area-inset-left,0px));
  background:#171a21;border-bottom:1px solid #242934;font-size:14px;color:#8b93a7}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:5px}
.up{background:#3ddc84}.down{background:#ff5f56}
.val{color:#e7e9ee;font-variant-numeric:tabular-nums}
.err{color:#ff5f56}.warn{color:#ffb454}.ok{color:#3ddc84}
main{flex:1 1 auto;display:grid;min-height:0;gap:10px;padding:10px;
  padding-left:max(10px,env(safe-area-inset-left,0px));
  padding-right:max(10px,env(safe-area-inset-right,0px));
  grid-template-columns:minmax(0,1fr) minmax(400px,0.85fr);
  grid-template-rows:minmax(0,1fr);overflow:hidden}
.box{background:#171a21;border:1px solid #242934;border-radius:9px;padding:10px;
  min-height:0;min-width:0;display:flex;flex-direction:column}
.lbl{font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:#6d7488;
  margin-bottom:6px;flex:0 0 auto}

/* camera: the left half, as big as the space allows */
#cambox{grid-column:1;grid-row:1}
#feed{flex:1 1 auto;min-height:0;width:100%;object-fit:contain;background:#000;
  border-radius:6px;display:block}

/* right column: compass on top, destinations in the middle, drive at the end */
.side{grid-column:2;grid-row:1;display:flex;flex-direction:column;gap:10px;
  min-height:0;overflow:hidden}

#imubox{flex:0 0 auto}
.imuwrap{display:flex;gap:12px;align-items:center}
#rose{width:200px;height:200px;flex:0 0 auto;display:block}
.stats{flex:1 1 auto;min-width:0}
.kv{display:flex;justify-content:space-between;font-size:15px;padding:3px 0}
.kv span:first-child{color:#6d7488}
#spark{width:100%;height:56px;display:block;margin-top:8px}

#destbox{flex:1 1 auto;min-height:0}
.dests{display:grid;grid-template-columns:repeat(auto-fill,minmax(130px,1fr));
  gap:8px;overflow-y:auto;align-content:start;flex:1 1 auto;min-height:0}
.dests button{font:inherit;font-size:30px;font-weight:700;padding:20px 6px;
  background:#242934;color:#e7e9ee;border:1px solid #333a49;border-radius:9px;
  cursor:pointer;line-height:1.1}
.dests button .sub{display:block;font-size:13px;font-weight:400;color:#8b93a7;
  margin-top:5px;letter-spacing:.03em}
.dests button:active,.dests button.on{background:#1d4e7a;border-color:#4da3ff}
.dests button.live{border-color:#3ddc84}
#seekstate{font-size:16px;color:#ffb454;margin-bottom:8px;flex:0 0 auto;
  min-height:20px}

#drivebox{flex:0 0 auto}
.pad{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
.pad button{font:inherit;font-size:14px;background:#242934;color:#e7e9ee;
  border:1px solid #333a49;border-radius:7px;padding:15px 2px;cursor:pointer}
.pad button:active,.pad button.on{background:#1d4e7a;border-color:#2f7fc4}
.stopb{background:#5a1f1f;border-color:#8a3030;font-weight:700;width:100%;
  padding:13px;margin-top:8px;font:inherit;font-size:15px;color:#e7e9ee;
  border-radius:7px;border:1px solid #8a3030;cursor:pointer;letter-spacing:.08em}
.row{display:flex;align-items:center;gap:10px;font-size:14px;margin-top:8px}
input[type=range]{flex:1;min-width:0}
.hint{font-size:13px;color:#6d7488;margin-top:6px}

@media(max-width:900px){
  main{grid-template-columns:1fr;grid-template-rows:auto auto;overflow-y:auto}
  #cambox{grid-column:1;grid-row:1}
  #feed{max-height:42vh}
  .side{grid-column:1;grid-row:2;overflow:visible}
  #rose{width:130px;height:130px}
}
</style></head><body>

<div class="strip">
  <span><span class="dot" id="light"></span><span id="imu">--</span></span>
  <span>drops <span class="val" id="drops">0</span></span>
  <span>cam <span class="val" id="cfps">--</span></span>
  <span>in view <span class="val" id="ids">--</span></span>
  <span id="cerr"></span>
  <span style="flex:1"></span>
  <button onclick="zero()" style="font:inherit;font-size:10px;padding:4px 9px;
    background:#242934;color:#e7e9ee;border:1px solid #333a49;border-radius:6px;
    cursor:pointer;font-size:13px">zero imu</button>
  <button onclick="release()" style="font:inherit;font-size:10px;padding:4px 9px;
    background:#242934;color:#e7e9ee;border:1px solid #333a49;border-radius:6px;
    cursor:pointer;font-size:13px">release gpio</button>
</div>

<main>
  <div class="box" id="cambox">
    <div class="lbl">Camera</div>
    <img id="feed" alt="camera">
  </div>

  <div class="side">
    <div class="box" id="imubox">
      <div class="lbl">Heading</div>
      <div class="imuwrap">
        <canvas id="rose" width="420" height="420"></canvas>
        <div class="stats">
          <div class="kv"><span>heading</span><span class="val" id="hdg">--</span></div>
          <div class="kv"><span>turned</span><span class="val" id="total">--</span></div>
          <div class="kv"><span>rate</span><span class="val" id="rate">--</span></div>
          <div class="kv"><span>tilt</span><span class="val" id="tilt">--</span></div>
          <div class="kv"><span>drift</span><span class="val" id="drift">--</span></div>
        </div>
      </div>
      <canvas id="spark" width="600" height="84"></canvas>
    </div>

    <div class="box" id="destbox">
      <div class="lbl">Go to marker</div>
      <button id="scanbtn" onclick="scanToggle()"
              style="width:100%;font:inherit;font-size:15px;font-weight:700;
                     padding:13px;margin-bottom:9px;background:#1d4e7a;
                     color:#e7e9ee;border:1px solid #4da3ff;border-radius:7px;
                     cursor:pointer;letter-spacing:.06em">SCAN FOR MARKERS</button>
      <div id="scanstate" class="hint" style="margin-top:0;margin-bottom:7px"></div>
      <div id="seekstate">idle</div>
      <div class="dests" id="dests"></div>
      <button class="stopb" onclick="seekStop()">CANCEL SEEK</button>
    </div>

    <div class="box" id="drivebox">
      <div class="lbl">Manual drive</div>
      <div class="pad">
        <button data-l="1" data-r="1">FWD</button>
        <button data-l="-1" data-r="-1">BACK</button>
        <button data-l="1" data-r="0">PIV L</button>
        <button data-l="0" data-r="1">PIV R</button>
      </div>
      <div class="row"><span>duty</span>
        <input id="duty" type="range" min="0.2" max="1" step="0.05" value="0.55">
        <span class="val" id="dutyv">0.55</span></div>
      <button class="stopb" onclick="stopAll()">STOP</button>
      <div class="hint" id="dstat">idle</div>
    </div>
  </div>
</main>

<script>
var $=function(i){return document.getElementById(i);};
var holdTimer=null, lastSeq=-1, scanFound={};
function duty(){return parseFloat($('duty').value);}
$('duty').addEventListener('input',function(){$('dutyv').textContent=duty().toFixed(2);});
async function send(l,r){try{await fetch('/drive?l='+l+'&r='+r+'&duty='+duty(),{method:'POST'});}catch(e){}}
async function stopAll(){
  if(holdTimer){clearInterval(holdTimer);holdTimer=null;}
  Array.prototype.forEach.call(document.querySelectorAll('.pad button'),
    function(b){b.classList.remove('on');});
  try{await fetch('/stop',{method:'POST'});}catch(e){}
}
function startHold(l,r,btn){
  if(holdTimer)clearInterval(holdTimer);
  if(btn)btn.classList.add('on');
  send(l,r); holdTimer=setInterval(function(){send(l,r);},150);
}
async function release(){await stopAll();try{await fetch('/release',{method:'POST'});}catch(e){}}
async function zero(){try{await fetch('/zero',{method:'POST'});}catch(e){}}
var scanning=false;
async function scanToggle(){
  try{ await fetch(scanning? '/scan_stop':'/scan',{method:'POST'}); }catch(e){}
}
async function seekGo(id){try{await fetch('/seek?id='+id,{method:'POST'});}catch(e){}}
async function seekStop(){try{await fetch('/seek_stop',{method:'POST'});}catch(e){}}

Array.prototype.forEach.call(document.querySelectorAll('.pad button'),function(b){
  var l=parseFloat(b.dataset.l), r=parseFloat(b.dataset.r);
  var go=function(e){e.preventDefault();startHold(l,r,b);};
  b.addEventListener('mousedown',go);
  b.addEventListener('touchstart',go,{passive:false});
  ['mouseup','mouseleave','touchend','touchcancel'].forEach(function(ev){
    b.addEventListener(ev,stopAll);});
});
var KEYS={w:[1,1],s:[-1,-1],q:[1,0],e:[0,1]}, keyDown=null;
addEventListener('keydown',function(ev){
  var k=ev.key.toLowerCase();
  if(KEYS[k]&&keyDown!==k){keyDown=k;startHold(KEYS[k][0],KEYS[k][1],null);}
});
addEventListener('keyup',function(ev){
  if(ev.key.toLowerCase()===keyDown){keyDown=null;stopAll();}
});
addEventListener('blur',stopAll);

function rose(h, markers){
  var c=$('rose'),x=c.getContext('2d'),W=c.width,R=W/2-16;
  x.clearRect(0,0,W,W); x.save(); x.translate(W/2,W/2);
  x.strokeStyle='#2b3140'; x.lineWidth=2;
  x.beginPath(); x.arc(0,0,R,0,7); x.stroke();
  for(var a=0;a<360;a+=15){
    var r=a%45===0?R-20:R-10, t=(a-90)*Math.PI/180;
    x.strokeStyle=a%45===0?'#3b4354':'#262c38'; x.lineWidth=a%45===0?2:1;
    x.beginPath(); x.moveTo(Math.cos(t)*R,Math.sin(t)*R);
    x.lineTo(Math.cos(t)*r,Math.sin(t)*r); x.stroke();
  }
  x.fillStyle='#6d7488'; x.font='600 17px ui-monospace,monospace';
  x.textAlign='center'; x.textBaseline='middle';
  [['FWD',0],['R',90],['BACK',180],['L',270]].forEach(function(p){
    var t=(p[1]-90)*Math.PI/180;
    x.fillText(p[0],Math.cos(t)*(R-40),Math.sin(t)*(R-40));
  });
  // markers the scan mapped, at their absolute heading round the room
  Object.keys(scanFound).forEach(function(k){
    var t=(scanFound[k].heading-90)*Math.PI/180;
    var px=Math.cos(t)*(R-26), py=Math.sin(t)*(R-26);
    x.fillStyle='#4a5468'; x.beginPath(); x.arc(px,py,9,0,7); x.fill();
    x.fillStyle='#c9cedb'; x.font='700 12px ui-monospace,monospace';
    x.fillText(k,px,py+1);
  });
  // where each marker sits right now, relative to straight ahead
  (markers||[]).forEach(function(m){
    var t=(m.bearing-90)*Math.PI/180;
    var px=Math.cos(t)*(R-62), py=Math.sin(t)*(R-62);
    x.fillStyle='#ffaa00'; x.beginPath(); x.arc(px,py,11,0,7); x.fill();
    x.fillStyle='#0f1115'; x.font='700 15px ui-monospace,monospace';
    x.fillText(String(m.id),px,py+1);
  });
  if(h!==null){
    var t2=(h-90)*Math.PI/180;
    x.strokeStyle='#4da3ff'; x.lineWidth=5; x.lineCap='round';
    x.beginPath(); x.moveTo(0,0);
    x.lineTo(Math.cos(t2)*(R-28),Math.sin(t2)*(R-28)); x.stroke();
    x.fillStyle='#4da3ff'; x.beginPath(); x.arc(0,0,7,0,7); x.fill();
  }
  x.restore();
}
function spark(a){
  var c=$('spark'),x=c.getContext('2d'),W=c.width,H=c.height;
  x.clearRect(0,0,W,H);
  x.strokeStyle='#262c38'; x.lineWidth=1;
  x.beginPath(); x.moveTo(0,H/2); x.lineTo(W,H/2); x.stroke();
  if(!a||!a.length)return;
  var m=Math.max(30,Math.max.apply(null,a.map(Math.abs)));
  x.strokeStyle='#4da3ff'; x.lineWidth=2; x.beginPath();
  a.forEach(function(v,i){
    var px=i/Math.max(1,a.length-1)*W, py=H/2-v/m*(H/2-8);
    i?x.lineTo(px,py):x.moveTo(px,py);
  });
  x.stroke();
  x.fillStyle='#6d7488'; x.font='13px ui-monospace,monospace'; x.textAlign='left';
  x.fillText('+'+m.toFixed(0)+' dps',4,12);
}

function renderDests(seen,sk,inview){
  var box=$('dests');
  var live={}; (inview||[]).forEach(function(m){live[m.id]=m;});
  var want=(seen||[]).map(function(m){return m.id;}).join(',');
  if(box.dataset.want!==want){
    box.dataset.want=want; box.innerHTML='';
    (seen||[]).forEach(function(m){
      var b=document.createElement('button');
      b.innerHTML=m.id+'<span class="sub" id="ds'+m.id+'"></span>';
      b.onclick=function(){seekGo(m.id);};
      b.dataset.mid=m.id;
      box.appendChild(b);
    });
    if(!(seen||[]).length){
      var d=document.createElement('div');
      d.className='hint';
      d.textContent='no markers seen yet - point the camera at one';
      box.appendChild(d);
    }
  }
  Array.prototype.forEach.call(box.querySelectorAll('button'),function(b){
    var id=parseInt(b.dataset.mid,10);
    var m=(seen||[]).filter(function(x){return x.id===id;})[0];
    b.classList.toggle('on', !!(sk&&sk.active&&sk.target===id));
    b.classList.toggle('live', !!live[id]);
    var sub=b.querySelector('.sub');
    if(sub){
      sub.textContent = live[id]
        ? ((live[id].bearing>=0?'+':'')+live[id].bearing+'° in view')
        : (m? 'seen '+Math.round(m.ago)+'s ago':'');
    }
  });
}

async function tick(){
  try{
    var s=await (await fetch('/data',{cache:'no-store'})).json();
    $('light').className='dot '+(s.live?'up':'down');
    $('imu').textContent = s.state==='calibrating'
      ? 'cal '+Math.round(s.cal_progress*100)+'%'
      : (s.live? s.hz.toFixed(0)+'Hz':'NO IMU');
    $('drops').textContent=s.dropouts;
    $('drops').className = s.dropouts? 'val err':'val';
    $('hdg').textContent=s.heading.toFixed(1)+'°';
    $('total').textContent=s.total.toFixed(0)+'°';
    $('rate').textContent=(s.yaw_rate>=0?'+':'')+s.yaw_rate.toFixed(1)+' dps';
    $('tilt').textContent=s.tilt.toFixed(1)+'°';
    $('drift').textContent=s.drift_dps.toFixed(2)+' dps';
    spark(s.trail);

    var inview=[];
    if(s.cam){
      var c=s.cam;
      inview=c.markers||[];
      // the feed is an MJPEG stream set once at load - no per-frame fetch
      $('cfps').textContent=c.fps? c.fps.toFixed(1)+'fps':'--';
      $('ids').textContent=c.ids.length? c.ids.join(', '):'none';
      $('ids').className=c.ids.length?'val ok':'val warn';
      $('cerr').textContent=c.err||'';
      $('cerr').className=c.err?'err':'';
    }
    rose(s.live? s.heading:null, inview);

    if(s.scan){
      var sc=s.scan;
      scanning=sc.active;
      var btn=$('scanbtn');
      btn.textContent = sc.active? 'STOP SCANNING':'SCAN FOR MARKERS';
      btn.style.background = sc.active? '#5a1f1f':'#1d4e7a';
      btn.style.borderColor = sc.active? '#8a3030':'#4da3ff';
      var ids=Object.keys(sc.found||{});
      $('scanstate').textContent = sc.active
        ? ('swept '+Math.round(sc.swept)+'\u00b0 of 360 \u2014 '+sc.note)
        : (ids.length? ('mapped: '+ids.map(function(k){
              return k+' at '+Math.round(sc.found[k].heading)+'\u00b0';
            }).join(',  ')) : '');
      scanFound = sc.found||{};
    }
    if(s.seek){
      var k=s.seek;
      $('seekstate').textContent = k.active
        ? (k.state+' → marker '+k.target+(k.note?'  ('+k.note+')':''))
        : (k.state==='idle'?'idle':k.state+(k.note?'  ('+k.note+')':''));
      renderDests(s.seen,k,inview);
    }
    if(s.drive){
      var d=s.drive;
      $('dstat').textContent = d.err? ('ERR '+d.err)
        : (d.held? 'L'+d.l.toFixed(1)+' R'+d.r.toFixed(1)+' @ '+d.duty.toFixed(2)
                 : 'idle');
      $('dstat').className = d.err? 'hint err':'hint';
    }
  }catch(e){
    $('imu').textContent='lost the Pi'; $('light').className='dot down';
  }
}
$('feed').src='/mjpeg';
// If the stream drops (wifi blip, server restart) the img goes blank and
// never recovers on its own, so re-point it when the browser reports an error.
$('feed').addEventListener('error',function(){
  setTimeout(function(){ $('feed').src='/mjpeg?r='+Date.now(); },1500);
});
rose(null,[]); setInterval(tick,200); tick();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _authed(self):
        import config as CA
        if not getattr(CA, "WEB_PASS", ""):
            return True
        want = "Basic " + base64.b64encode(
            f"{CA.WEB_USER}:{CA.WEB_PASS}".encode()).decode()
        if self.headers.get("Authorization") == want:
            return True
        self.send_response(401)
        self.send_header("WWW-Authenticate", 'Basic realm="rover"')
        self.send_header("Content-Length", "0")
        self.end_headers()
        return False

    def do_GET(self):
        if not self._authed():
            return
        if self.path.startswith("/mjpeg"):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=rovframe")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            last = -1
            try:
                while True:
                    with _clock:
                        seq, buf = CAMS["seq"], CAMS["jpeg"]
                    if buf and seq != last:
                        last = seq
                        self.wfile.write(b"--rovframe\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            ("Content-Length: %d\r\n\r\n" % len(buf)).encode())
                        self.wfile.write(buf)
                        self.wfile.write(b"\r\n")
                    else:
                        time.sleep(0.01)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            return
        if self.path.startswith("/frame.jpg"):
            with _clock:
                body = CAMS["jpeg"]
            if not body:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/data"):
            with _lock:
                payload = dict(S)
            with _clock:
                payload["cam"] = {
                    "seq": CAMS["seq"], "ids": list(CAMS["ids"]),
                    "goal": CAMS["goal"], "ranges": list(CAMS["ranges"]),
                    "err": CAMS["err"], "fps": CAMS["fps"],
                    "obs": list(CAMS["obs"]), "polarity": CAMS["polarity"],
                    "markers": list(CAMS["markers"]),
                }
            now_t = time.time()
            with _clock:
                payload["seen"] = sorted(
                    [{"id": k, "ago": round(now_t - v, 1)} for k, v in SEEN.items()],
                    key=lambda d: d["id"])
            with _seek_lock:
                payload["seek"] = dict(SK)
            with _scan_lock:
                payload["scan"] = {
                    "active": SC["active"], "state": SC["state"],
                    "swept": SC["swept"], "note": SC["note"],
                    "found": {str(k): v for k, v in SC["found"].items()},
                }
            with _dlock:
                payload["drive"] = {
                    "l": D["l"], "r": D["r"], "duty": D["duty"],
                    "held": D["held"], "err": D["err"],
                    "attached": D["drv"] is not None,
                }
            body = json.dumps(payload).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        body = PAGE.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _q(self):
        from urllib.parse import parse_qs, urlparse
        return parse_qs(urlparse(self.path).query)

    def do_POST(self):
        if not self._authed():
            return
        if self.path.startswith("/drive"):
            q = self._q()
            try:
                l = float(q.get("l", ["0"])[0])
                r = float(q.get("r", ["0"])[0])
                du = float(q.get("duty", ["0.6"])[0])
            except ValueError:
                self.send_response(400)
                self.end_headers()
                return
            err = drive_cmd(max(-1.0, min(1.0, l)), max(-1.0, min(1.0, r)), du)
            self.send_response(500 if err else 204)
            self.end_headers()
            if err:
                self.wfile.write(err.encode())
            return
        if self.path.startswith("/scan_stop"):
            scan_stop()
            self.send_response(204)
            self.end_headers()
            return
        if self.path.startswith("/scan"):
            err = scan_start()
            self.send_response(409 if err else 204)
            self.end_headers()
            return
        if self.path.startswith("/seek_stop"):
            seek_stop()
            self.send_response(204)
            self.end_headers()
            return
        if self.path.startswith("/seek"):
            q = self._q()
            try:
                tid = int(q.get("id", ["0"])[0])
            except ValueError:
                self.send_response(400)
                self.end_headers()
                return
            err = seek_start(tid)
            self.send_response(409 if err else 204)
            self.end_headers()
            return
        if self.path.startswith("/stop"):
            seek_stop()
            scan_stop()
            drive_stop()
            self.send_response(204)
            self.end_headers()
            return
        if self.path.startswith("/release"):
            drive_release()
            self.send_response(204)
            self.end_headers()
            return
        if self.path.startswith("/zero"):
            with _lock:
                S["_recal"] = True
            self.send_response(204)
            self.end_headers()
            return
        self.send_response(404)
        self.end_headers()
if __name__ == "__main__":
    threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=watchdog, daemon=True).start()
    threading.Thread(target=camera, daemon=True).start()
    print(f"IMU view on port {PORT} - open http://<pi-ip>:{PORT}/", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
