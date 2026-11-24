import subprocess
import sys
import time
import config as C
which = (sys.argv[1] if len(sys.argv) > 1 else "right").lower()
cap = float(sys.argv[2]) if len(sys.argv) > 2 else 0.6
if cap > 0.75:
    sys.exit("refusing above 0.75 - this is a measurement, not a power test")
C.MAX_DUTY = cap
from drive import Drive
if which not in ("left", "right"):
    sys.exit("first arg must be 'left' or 'right'")
if which == "left":
    l, r = 1.0, 0.0
    pwm_pin, dir_pin = C.LEFT_IN1, C.LEFT_IN2
    en_pin = C.LEFT_EN
else:
    l, r = 0.0, 1.0
    pwm_pin, dir_pin = C.RIGHT_IN1, C.RIGHT_IN2
    en_pin = C.RIGHT_EN
pins = f"{en_pin},{pwm_pin},{dir_pin}"
N = 150


def sample():
    out = subprocess.run(["/usr/bin/pinctrl", "get", pins],
                         capture_output=True, text=True, timeout=3)
    st = {}
    for line in out.stdout.strip().splitlines():
        try:
            gp = int(line.split(":")[0].strip())
            st[gp] = 1 if "| hi" in line else 0
        except (ValueError, IndexError):
            pass
    return st
expect = C.MIN_DUTY + (C.MAX_DUTY - C.MIN_DUTY) * 1.0
print(f"{which.upper()} pair, cap={cap:.2f}")
print(f"  MIN_DUTY={C.MIN_DUTY}  MAX_DUTY={C.MAX_DUTY}")
print(f"  _shape(1.0) = {expect:.3f}  -> expect ~{expect*100:.0f}% high on GPIO {pwm_pin}")
print(f"  (that is ~{expect*17.5:.1f} V average into the motor)")
print(f"  EN={en_pin}  PWM={pwm_pin}  DIR={dir_pin}\n")
d = Drive()
counts = {en_pin: 0, pwm_pin: 0, dir_pin: 0}
n = 0
try:
    d.tank(l, r, smooth=False)
    time.sleep(0.2)
    for _ in range(N):
        st = sample()
        if len(st) == 3:
            n += 1
            for gp in counts:
                counts[gp] += st.get(gp, 0)
finally:
    d.stop()
if n == 0:
    sys.exit("no samples read")
print(f"measured over {n} samples:")
print(f"  EN  GPIO{en_pin:<3} high {100.0*counts[en_pin]/n:5.1f}%   (want 100%)")
print(f"  PWM GPIO{pwm_pin:<3} high {100.0*counts[pwm_pin]/n:5.1f}%   "
      f"(want ~{expect*100:.0f}%)")
print(f"  DIR GPIO{dir_pin:<3} high {100.0*counts[dir_pin]/n:5.1f}%   (want 0%)")
meas = counts[pwm_pin] / n
if counts[en_pin] / n < 0.95:
    print("\n  ** ENABLE is not held high - that channel is gated off **")
elif abs(meas - expect) > 0.15:
    print(f"\n  ** measured {meas*100:.0f}% vs expected {expect*100:.0f}% - "
          f"duty is NOT reaching the pin **")
else:
    print("\n  OK - commanded duty is reaching the pin")
