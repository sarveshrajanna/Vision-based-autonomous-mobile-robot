import json
import math
import os
import cv2
import numpy as np
import config as C


def _load(path, default=None):
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return default


class Vision:
    def __init__(self):
        self.mean = None
        self.std = None
        self.table = _load(C.DIST_TABLE)
        self.goal_hsv = _load(C.GOAL_HSV_FILE)
        self.mask = None
        self.qr = cv2.QRCodeDetector()
        self.found_ids = []
        self.all_markers = []


        self.aruco = [cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name)),
            cv2.aruco.DetectorParameters()) for name in C.ARUCO_DICTS]
        self._hist = []
        self.marker_obstacles = []

    def row_to_cm(self, row):
        if self.table:
            rows = np.array(self.table["rows"], dtype=float)
            cms = np.array(self.table["cms"], dtype=float)
            order = np.argsort(rows)
            return float(np.interp(row, rows[order], cms[order]))

        half_v = C.FOV_V_DEG / 2.0
        off = ((row - C.CAM_H / 2.0) / (C.CAM_H / 2.0)) * half_v
        depression = C.CAM_TILT_DEG + off
        if depression <= 1.0:
            return C.MAX_RANGE_CM
        d = C.CAM_HEIGHT_CM / math.tan(math.radians(depression))
        return float(min(d, C.MAX_RANGE_CM))

    def top_row(self):
        dep = math.degrees(math.atan(C.CAM_HEIGHT_CM / C.MAX_RANGE_CM))
        off = dep - C.CAM_TILT_DEG
        row = (C.CAM_H / 2.0) * (1.0 + off / (C.FOV_V_DEG / 2.0))
        return int(max(row, C.SCAN_TOP * C.CAM_H, 0))

    def col_to_bearing(self, x):
        return -((x - C.CAM_W / 2.0) / (C.CAM_W / 2.0)) * (C.FOV_H_DEG / 2.0)

    def _seed_box(self):
        x0, x1, y0, y1 = C.SEED_BOX
        return (int(x0 * C.CAM_W), int(x1 * C.CAM_W),
                int(y0 * C.CAM_H), int(y1 * C.CAM_H))

    def _learn_floor(self, hsv):
        x0, x1, y0, y1 = self._seed_box()
        patch = hsv[y0:y1, x0:x1].reshape(-1, 3).astype(np.float32)
        mean = np.median(patch, axis=0)
        std = np.maximum(patch.std(axis=0), [4.0, 12.0, 18.0])


        ang = patch[:, 0] * (2 * np.pi / 180.0)
        cs, sn = np.cos(ang).mean(), np.sin(ang).mean()
        mean[0] = (math.degrees(math.atan2(sn, cs)) % 360.0) / 2.0
        R = min(math.hypot(cs, sn), 0.999999)
        circ = math.degrees(math.sqrt(-2.0 * math.log(R))) / 2.0
        std[0] = max(circ, C.HUE_MIN_SPREAD)
        if self.mean is None:
            self.mean, self.std = mean, std
        else:
            self.mean = 0.85 * self.mean + 0.15 * mean
            self.std = 0.85 * self.std + 0.15 * std

    def _obstacle_mask(self, hsv):
        h = hsv[:, :, 0].astype(np.float32)
        s = hsv[:, :, 1].astype(np.float32)
        v = hsv[:, :, 2].astype(np.float32)
        dh = np.abs(h - self.mean[0])
        dh = np.minimum(dh, 180.0 - dh)
        ds = np.abs(s - self.mean[1])
        dv = np.abs(v - self.mean[2])

        dv = np.where(v > self.mean[2], dv / C.V_BRIGHT_TOL, dv)
        score = np.sqrt((dh / self.std[0]) ** 2
                        + (ds / self.std[1]) ** 2
                        + (dv / self.std[2]) ** 2)
        m = (score > C.COLOR_K).astype(np.uint8)
        m[: self.top_row(), :] = 0
        if C.SCAN_BOTTOM_PX:
            m[C.CAM_H - C.SCAN_BOTTOM_PX:, :] = 0
        if C.SCAN_SIDE_PX:
            m[:, :C.SCAN_SIDE_PX] = 0
            m[:, C.CAM_W - C.SCAN_SIDE_PX:] = 0


        k = np.ones((C.OPEN_PX, C.OPEN_PX), np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
        return self._drop_small(m)

    @staticmethod
    def _drop_small(m):
        if C.MIN_BLOB_PX <= 0:
            return m
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        if n <= 1:
            return m
        keep = np.zeros(n, np.uint8)
        for i in range(1, n):
            if (stats[i, cv2.CC_STAT_AREA] >= C.MIN_BLOB_PX
                    and stats[i, cv2.CC_STAT_HEIGHT] >= C.MIN_BLOB_H):
                keep[i] = 1
        return keep[lab]

    def _edge_mask(self, bgr):
        g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        g = cv2.GaussianBlur(g, (5, 5), 0)
        gy = np.abs(cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))
        m = (gy > C.EDGE_THRESH).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 7), np.uint8))
        m[: self.top_row(), :] = 0
        if C.SCAN_BOTTOM_PX:
            m[C.CAM_H - C.SCAN_BOTTOM_PX:, :] = 0
        if C.SCAN_SIDE_PX:
            m[:, :C.SCAN_SIDE_PX] = 0
            m[:, C.CAM_W - C.SCAN_SIDE_PX:] = 0
        return m

    def scan_rows(self, mask):
        rows = []
        step = C.CAM_W // C.N_COLS
        top = self.top_row()
        for i in range(C.N_COLS):
            strip = mask[:, i * step:(i + 1) * step]
            prof = (strip.mean(axis=1) > C.COL_FILL)
            hit = None
            run = 0
            for y in range(C.CAM_H - 1 - C.SCAN_BOTTOM_PX, top - 1, -1):
                if prof[y]:
                    run += 1
                    if run >= C.RUN_PX:
                        hit = y + C.RUN_PX - 1
                        break
                else:
                    run = 0
            rows.append(hit)
        return rows

    def _scan(self, mask):
        return [self.row_to_cm(r) if r is not None else C.MAX_RANGE_CM
                for r in self.scan_rows(mask)]

    def _find_goal_aruco(self, bgr):
        corners, found_ids = [], []
        for det in self.aruco:
            c, ids, _ = det.detectMarkers(bgr)
            if ids is not None and len(ids):
                corners, found_ids = list(c), [int(i) for i in ids.flatten()]
                break


        self.found_ids = sorted(set(found_ids))
        if not corners:
            self.marker_obstacles = []
            self.all_markers = []
            return None

        def describe(p):
            cx = float(p[:, 0].mean())
            base = float(p[:, 1].max())
            x0, y0 = float(p[:, 0].min()), float(p[:, 1].min())
            return {"bearing": self.col_to_bearing(cx),
                    "dist": self.row_to_cm(min(base, C.CAM_H - 1)),
                    "box": (int(x0), int(y0), int(p[:, 0].max() - x0),
                            int(p[:, 1].max() - y0))}


        self.marker_obstacles = [describe(c.reshape(-1, 2))
                                 for c, i in zip(corners, found_ids)
                                 if i != C.GOAL_MARKER_ID]


        self.all_markers = []
        for c, i in zip(corners, found_ids):
            d = describe(c.reshape(-1, 2))
            d["id"] = int(i)
            self.all_markers.append(d)
        goal_c = [c for c, i in zip(corners, found_ids) if i == C.GOAL_MARKER_ID]
        if not goal_c:
            return None
        corners = goal_c
        best, area = None, 0
        for c in corners:
            p = c.reshape(-1, 2)
            a = cv2.contourArea(p.astype(np.float32))
            if a > area:
                area, best = a, p
        if best is None or area < 300:
            return None
        cx = float(best[:, 0].mean())
        base = float(best[:, 1].max())
        x0, y0 = float(best[:, 0].min()), float(best[:, 1].min())
        wpx = float(best[:, 0].max() - x0)


        fx = (C.CAM_W / 2.0) / math.tan(math.radians(C.FOV_H_DEG / 2.0))
        dist = (fx * C.MARKER_WIDTH_CM / wpx) if wpx > 4 else C.MAX_RANGE_CM
        return {"bearing": self.col_to_bearing(cx),
                "dist": min(dist, C.MAX_RANGE_CM),
                "box": (int(x0), int(y0),
                        int(best[:, 0].max() - x0), int(best[:, 1].max() - y0))}

    def _find_goal_qr(self, bgr):
        try:
            ok, pts = self.qr.detect(bgr)
        except cv2.error:
            return None
        if not ok or pts is None or len(pts) == 0:
            return None
        p = pts.reshape(-1, 2)
        cx = float(p[:, 0].mean())
        base = float(p[:, 1].max())
        w = float(p[:, 0].max() - p[:, 0].min())
        h = float(p[:, 1].max() - p[:, 1].min())
        if w < 18 or h < 18:
            return None
        return {"bearing": self.col_to_bearing(cx),
                "dist": self.row_to_cm(min(base, C.CAM_H - 1)),
                "box": (int(p[:, 0].min()), int(p[:, 1].min()), int(w), int(h))}

    def _find_goal(self, hsv):
        if not self.goal_hsv:
            return None
        g = self.goal_hsv
        lo = np.array([g["h_lo"], g["s_lo"], g["v_lo"]], np.uint8)
        hi = np.array([g["h_hi"], g["s_hi"], g["v_hi"]], np.uint8)
        if g["h_lo"] > g["h_hi"]:
            m = cv2.inRange(hsv, np.array([0, g["s_lo"], g["v_lo"]], np.uint8),
                            np.array([g["h_hi"], g["s_hi"], g["v_hi"]], np.uint8))
            m |= cv2.inRange(hsv, np.array([g["h_lo"], g["s_lo"], g["v_lo"]], np.uint8),
                             np.array([179, g["s_hi"], g["v_hi"]], np.uint8))
        else:
            m = cv2.inRange(hsv, lo, hi)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < C.GOAL_MIN_AREA:
            return None
        x, y, w, h = cv2.boundingRect(c)
        cx = x + w / 2.0
        base = y + h
        return {"bearing": self.col_to_bearing(cx),
                "dist": self.row_to_cm(min(base, C.CAM_H - 1)),
                "box": (x, y, w, h)}

    @staticmethod
    def _flatten(hsv):
        v = hsv[:, :, 2].astype(np.float32)


        small = cv2.resize(v, (C.CAM_W // 8, C.CAM_H // 8), interpolation=cv2.INTER_AREA)
        small = cv2.GaussianBlur(small, (0, 0), sigmaX=7.5, sigmaY=7.5)
        field = cv2.resize(small, (C.CAM_W, C.CAM_H), interpolation=cv2.INTER_LINEAR)
        v = np.clip(v * (float(field.mean()) / np.maximum(field, 1.0)), 0, 255)
        out = hsv.copy()
        out[:, :, 2] = v.astype(np.uint8)
        return out

    def process(self, bgr):
        sharp = bgr
        if C.CONTROLLER == "track" and (not C.AVOID or C.OBSTACLE_MODE == "bright"):
            self.mask = np.zeros((C.CAM_H, C.CAM_W), np.uint8)
            self.colour_ranges = self.edge_ranges = None
            far = [C.MAX_RANGE_CM] * C.N_COLS
            return {"ranges": far, "raw": far,
                    "goal": self._goal(sharp, None),
                    "found_ids": list(self.found_ids)}
        if C.PRE_BLUR > 1:
            bgr = cv2.GaussianBlur(bgr, (0, 0), sigmaX=C.PRE_BLUR, sigmaY=C.PRE_BLUR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        if C.FLAT_FIELD:
            hsv = self._flatten(hsv)
        self._learn_floor(hsv)
        colour = self._obstacle_mask(hsv)
        if C.DETECTOR == "colour":
            self.mask = colour
            raw = self._scan(colour)
        else:
            edge = self._edge_mask(bgr)
            self.edge_mask = edge
            r_col, r_edg = self._scan(colour), self._scan(edge)
            if C.DETECTOR == "edge":
                self.mask, raw = edge, r_edg
            elif C.DETECTOR == "safest":
                self.mask = np.maximum(colour, edge)
                raw = [min(a, b) for a, b in zip(r_col, r_edg)]
            elif C.DETECTOR == "agree":
                self.mask = (colour & edge)
                raw = [max(a, b) for a, b in zip(r_col, r_edg)]
            else:
                raw = [min(a, b) if min(a, b) < C.CLOSE_TRUST_CM else max(a, b)
                       for a, b in zip(r_col, r_edg)]
                self.mask = np.maximum(colour, edge)
        self.colour_ranges, self.edge_ranges = None, None
        if C.DETECTOR not in ("colour", "edge"):
            self.colour_ranges, self.edge_ranges = r_col, r_edg


        self._hist.append(raw)
        if len(self._hist) > C.SMOOTH_FRAMES:
            self._hist.pop(0)
        ranges = np.median(np.array(self._hist), axis=0).tolist()
        return {"ranges": ranges, "raw": raw,
                "goal": self._goal(sharp, hsv),
                "found_ids": list(self.found_ids)}

    def find_bright_obstacles(self, bgr, goal=None):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        s, v = hsv[:, :, 1], hsv[:, :, 2]
        if C.OBS_POLARITY == "dark":
            m = (v < C.OBS_V_MAX).astype(np.uint8)
        else:
            m = ((v > C.OBS_V_MIN) & (s < C.OBS_S_MAX)).astype(np.uint8)
        m[: self.top_row(), :] = 0
        if C.SCAN_BOTTOM_PX:
            m[C.CAM_H - C.SCAN_BOTTOM_PX:, :] = 0
        if C.SCAN_SIDE_PX:
            m[:, :C.SCAN_SIDE_PX] = 0
            m[:, C.CAM_W - C.SCAN_SIDE_PX:] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        self.obs_mask = m
        n, lab, stats, cent = cv2.connectedComponentsWithStats(m, 8)
        gx, gy, gw, gh = goal["box"] if goal else (0, 0, 0, 0)
        out = []
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < C.OBS_MIN_PX:
                continue
            x, y = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
            w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
            if goal and not (x > gx + gw or x + w < gx or y > gy + gh or y + h < gy):
                continue

            if goal:
                ob = self.col_to_bearing(float(cent[i][0]))
                od = self.row_to_cm(min(y + h, C.CAM_H - 1))
                if abs(ob - goal["bearing"]) < C.GOAL_GUARD_DEG:
                    continue
            out.append({"bearing": self.col_to_bearing(float(cent[i][0])),
                        "dist": self.row_to_cm(min(y + h, C.CAM_H - 1)),
                        "box": (int(x), int(y), int(w), int(h))})
        return out

    def _goal(self, sharp, hsv):
        if C.GOAL_MODE == "aruco":
            return self._find_goal_aruco(sharp)
        if C.GOAL_MODE == "qr":
            return self._find_goal_qr(sharp)
        return self._find_goal(hsv)

    def annotate(self, bgr, out, note=""):
        img = bgr.copy()
        if self.mask is not None:
            red = np.zeros_like(img)
            red[:, :, 2] = 255
            img = np.where(self.mask[:, :, None].astype(bool),
                           (0.6 * img + 0.4 * red).astype(np.uint8), img)
        x0, x1, y0, y1 = self._seed_box()
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 255), 1)
        step = C.CAM_W // C.N_COLS
        for i, r in enumerate(out["ranges"]):
            cx = i * step + step // 2
            cv2.putText(img, f"{r:.0f}", (cx - 16, C.CAM_H - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 0, 255) if r < 60 else (0, 255, 0), 1)
            cv2.line(img, (i * step, C.CAM_H - 24), (i * step, C.CAM_H), (80, 80, 80), 1)
        g = out.get("goal")
        if g:
            x, y, w, h = g["box"]
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.putText(img, f"GOAL {g['dist']:.0f}cm {g['bearing']:+.0f}deg",
                        (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 0, 255), 1)
        for o in getattr(self, "last_obstacles", []):
            x, y, w, h = o["box"]
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 200, 255), 2)
            cv2.putText(img, f"OBST {o['dist']:.0f}cm", (x, max(y - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        if note:
            cv2.putText(img, note, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 1)
        self._radar(img, out["ranges"])
        return img

    def _radar(self, img, ranges):
        PW, PH, PAD = 190, 140, 8
        x0, y0 = C.CAM_W - PW - PAD, PAD
        panel = img[y0:y0 + PH, x0:x0 + PW]
        cv2.rectangle(img, (x0, y0), (x0 + PW, y0 + PH), (40, 40, 40), -1)
        cv2.addWeighted(panel, 0.25, img[y0:y0 + PH, x0:x0 + PW], 0.75, 0,
                        img[y0:y0 + PH, x0:x0 + PW])
        cv2.rectangle(img, (x0, y0), (x0 + PW, y0 + PH), (160, 160, 160), 1)
        ox, oy = x0 + PW // 2, y0 + PH - 12
        top = 100.0
        scale = (PH - 24) / top
        for r_cm in (25, 50, 75, 100):
            cv2.ellipse(img, (ox, oy), (int(r_cm * scale), int(r_cm * scale)),
                        0, 200, 340, (90, 90, 90), 1)
            cv2.putText(img, f"{r_cm}", (ox + 3, oy - int(r_cm * scale) + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (140, 140, 140), 1)
        for b, r in zip(col_bearings(), ranges):
            a = math.radians(90 + b)
            d = min(r, top) * scale
            ex, ey = int(ox + d * math.cos(a)), int(oy - d * math.sin(a))
            hit = r < C.MAX_RANGE_CM - 1
            cv2.line(img, (ox, oy), (ex, ey),
                     (80, 80, 255) if hit else (90, 170, 90), 1)
            if hit:
                cv2.circle(img, (ex, ey), 3, (60, 60, 255), -1)
        cv2.circle(img, (ox, oy), 4, (255, 255, 255), -1)
        cv2.putText(img, "top-down, cm ahead", (x0 + 6, y0 + 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (200, 200, 200), 1)


def col_bearings():
    step = C.CAM_W // C.N_COLS
    out = []
    for i in range(C.N_COLS):
        px = i * step + step / 2.0
        out.append(-((px - C.CAM_W / 2.0) / (C.CAM_W / 2.0)) * (C.FOV_H_DEG / 2.0))
    return out
