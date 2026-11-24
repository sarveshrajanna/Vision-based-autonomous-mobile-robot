import math
import config as C


class Follower:
    def __init__(self):
        self.kappa = 0.0
        self.bearing = None
        self.lost = 0
        self.seen = False
        self.blind_deg = 0.0
        self.recentre_deg = 0.0
        self.phase = None
        self.phase_ticks = 0
        self.side = 1
        self.solid = 0
        self.arrived = False
        self.ptick = 0

    def _settling(self):
        if not C.PULSE_MODE:
            return False
        self.ptick = (self.ptick + 1) % (C.PULSE_ON + C.PULSE_OFF)
        return self.ptick >= C.PULSE_ON

    def observe_turn(self, deg):
        if self.lost > 0:
            self.blind_deg += deg
        self.recentre_deg += deg

    def step(self, goal, obstacles):
        if self.arrived:
            return 0.0, 0.0, "ARRIVED - holding position", 0
        obstacles = [o for o in (obstacles or []) if o["dist"] >= C.OBS_MIN_CM]
        if goal is None:
            return self._blind()

        self.lost = 0
        self.seen = True
        self.solid += 1
        if self.solid >= C.SOLID_FRAMES:
            self.blind_deg = 0.0
        self.bearing = goal["bearing"]
        b, dist = goal["bearing"], goal["dist"]
        if dist <= C.GOAL_STOP_CM:
            self.kappa, self.phase = 0.0, None
            self.arrived = True
            return 0.0, 0.0, f"ARRIVED at {dist:.0f}cm", 0
        if self.phase:
            if self._settling():
                return 0.0, self.kappa, f"AVOID {self.phase}: settling", 0
            return self._sidestep(b, obstacles)
        blocking = [o for o in obstacles
                    if o["dist"] < C.AVOID_CM and abs(o["bearing"]) < C.BLOCK_DEG]
        if blocking:
            near = min(blocking, key=lambda o: o["dist"])
            self.side = 1 if near["bearing"] >= 0 else -1
            self.phase, self.phase_ticks = "OUT", int(C.OUT_MAX_S * C.HZ)
            return self._sidestep(b, obstacles, start=near)
        if abs(b) <= C.CENTRED_DEG:
            self.recentre_deg = 0.0
        if abs(b) > C.RECENTER_DEG and abs(self.recentre_deg) < C.MAX_RECENTRE_DEG:
            self.kappa = 0.0
            if self._settling():
                return 0.0, 0.0, f"recentring: settling (marker {b:+.0f}deg)", 0
            return (0.0, 0.0,
                    f"recentring, marker {b:+.0f}deg "
                    f"(turned {self.recentre_deg:+.0f}/{C.MAX_RECENTRE_DEG:.0f})",
                    (1 if b > 0 else -1))
        if self._settling():
            return 0.0, self.kappa, f"bearing {b:+5.1f}deg  dist {dist:4.0f}cm  [settle]", 0
        return self._pursue(b, dist)

    def _pursue(self, b, dist):
        d_m = max(dist / 100.0, 0.15)


        if abs(b) < C.STRAIGHT_DEG:
            self.kappa += max(-C.KAPPA_RATE, min(C.KAPPA_RATE, -self.kappa))
            v = max(C.CRUISE * max(0.35, min(1.0, (dist - C.GOAL_STOP_CM) / 60.0)),
                    C.MIN_DRIVE_V)
            return v, self.kappa, f"bearing {b:+5.1f}deg  dist {dist:4.0f}cm  straight", 0
        k = 2.0 * math.sin(math.radians(b)) / d_m


        cap = C.KAPPA_MAX_NEAR if dist <= C.NEAR_CM else C.KAPPA_MAX
        k = max(-cap, min(cap, k))
        self.kappa += max(-C.KAPPA_RATE, min(C.KAPPA_RATE, k - self.kappa))
        close = max(0.35, min(1.0, (dist - C.GOAL_STOP_CM) / 60.0))
        straight = 1.0 - 0.4 * min(1.0, abs(self.kappa) / cap)
        v = max(C.CRUISE * close * straight, C.MIN_DRIVE_V)
        return v, self.kappa, f"bearing {b:+5.1f}deg  dist {dist:4.0f}cm  k={self.kappa:+.2f}", 0

    def _sidestep(self, b, obstacles, start=None):
        if start is not None:
            return (C.CRUISE * 0.6, -self.side * C.AVOID_KAPPA,
                    f"AVOID start: obstacle {start['dist']:.0f}cm "
                    f"at {start['bearing']:+.0f}deg, going "
                    f"{'right' if self.side > 0 else 'left'}", 0)
        self.phase_ticks -= 1
        blocking = [o for o in obstacles if o["dist"] < C.AVOID_CM
                    and abs(o["bearing"]) < C.BLOCK_DEG]
        if self.phase_ticks <= 0:
            self.phase = None
        elif self.phase == "OUT":
            if abs(b) >= C.MAX_OFFSET_DEG:
                self.phase, self.phase_ticks = "PASS", int(C.PASS_MAX_S * C.HZ)
            else:
                return (C.CRUISE * 0.6, -self.side * C.AVOID_KAPPA,
                        f"AVOID out: marker {b:+.0f}deg (limit {C.MAX_OFFSET_DEG:.0f})", 0)
        elif self.phase == "PASS":
            if not blocking:
                self.phase, self.phase_ticks = "BACK", int(C.BACK_MAX_S * C.HZ)
            else:
                return C.CRUISE * 0.6, 0.0, f"AVOID pass: marker {b:+.0f}deg", 0
        elif self.phase == "BACK":
            if abs(b) <= C.CENTRED_DEG:
                self.phase = None
            else:
                return (C.CRUISE * 0.6, self.side * C.AVOID_KAPPA,
                        f"AVOID back: marker {b:+.0f}deg", 0)
        return self._pursue(b, max(C.GOAL_STOP_CM + 1, 60.0))

    def _blind(self):
        self.phase = None
        if not self.seen:
            return 0.0, 0.0, "waiting for marker", 0
        self.lost += 1
        self.solid = 0


        if (self.bearing is not None and abs(self.bearing) >= C.EDGE_DEG
                and abs(self.blind_deg) < C.MAX_BLIND_DEG):
            side = 1 if self.bearing > 0 else -1
            return (0.0, 0.0,
                    f"EDGE RECOVERY: was {self.bearing:+.0f}deg, turned "
                    f"{self.blind_deg:+.0f} of {C.MAX_BLIND_DEG:.0f}deg", side)
        if self.lost <= C.LOST_GRACE:
            return 0.0, 0.0, f"waiting for marker ({self.lost})", 0
        return 0.0, 0.0, "marker lost - STOPPED", 0
