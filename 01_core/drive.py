import atexit
from gpiozero import DigitalOutputDevice, PWMOutputDevice
import config as C


def _shape(duty):
    if abs(duty) < 0.02:
        return 0.0
    s = 1.0 if duty > 0 else -1.0
    m = min(abs(duty), 1.0)
    return s * (C.MIN_DUTY + (C.MAX_DUTY - C.MIN_DUTY) * m)


class Drive:
    def __init__(self):
        self.l_in1 = DigitalOutputDevice(C.LEFT_IN1, initial_value=False)
        self.l_in2 = DigitalOutputDevice(C.LEFT_IN2, initial_value=False)
        self.r_in1 = DigitalOutputDevice(C.RIGHT_IN1, initial_value=False)
        self.r_in2 = DigitalOutputDevice(C.RIGHT_IN2, initial_value=False)


        self.en_l = PWMOutputDevice(C.LEFT_EN, frequency=C.PWM_HZ,
                                    initial_value=0.0)
        self.en_r = PWMOutputDevice(C.RIGHT_EN, frequency=C.PWM_HZ,
                                    initial_value=0.0)
        self._l = 0.0
        self._r = 0.0
        atexit.register(self.stop)

    def _slew(self, cur, target):
        if target > cur + C.SLEW:
            return cur + C.SLEW
        if target < cur - C.SLEW:
            return cur - C.SLEW
        return target

    def tank(self, l, r, smooth=True):
        if smooth:
            l = self._slew(self._l, l)
            r = self._slew(self._r, r)
        self._l, self._r = l, r


        for in1, in2, en, val, trim in (
                (self.l_in1, self.l_in2, self.en_l, l, C.LEFT_TRIM),
                (self.r_in1, self.r_in2, self.en_r, r, C.RIGHT_TRIM)):
            d = _shape(val) * trim
            if d == 0:
                en.value = 0.0
                in1.off()
                in2.off()
            elif d > 0:
                in1.on()
                in2.off()
                en.value = min(d, 1.0)
            else:
                in1.off()
                in2.on()
                en.value = min(-d, 1.0)

    def arc(self, v, kappa):
        mix = kappa * C.HALF_TRACK_M * C.STEER_GAIN
        l = v * (1.0 - mix)
        r = v * (1.0 + mix)
        peak = max(abs(l), abs(r), 1.0)
        self.tank(l / peak, r / peak)

    def spin(self, direction):
        self.tank(-direction * C.SPIN_DUTY, direction * C.SPIN_DUTY,
                  smooth=False)

    def stop(self):
        self._l = self._r = 0.0
        for en in (self.en_l, self.en_r):
            try:
                en.value = 0.0
            except Exception:
                pass
        for p in (self.l_in1, self.l_in2, self.r_in1, self.r_in2):
            try:
                p.off()
            except Exception:
                pass

    def close(self):
        self.stop()
        for d in (self.l_in1, self.l_in2, self.r_in1, self.r_in2,
                  self.en_l, self.en_r):
            try:
                d.close()
            except Exception:
                pass
