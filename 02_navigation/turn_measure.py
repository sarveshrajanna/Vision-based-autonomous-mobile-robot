import math
import sys
import time
from smbus2 import SMBus
import config as C
ADDR = 0x68
PWR_MGMT_1 = 0x6B
CONFIG_REG = 0x1A
GYRO_CFG = 0x1B
ACCEL_CFG = 0x1C
ACCEL_XOUT = 0x3B
ACCEL_LSB = 16384.0
GYRO_LSB = 16.4
CAL_N = 120
which = (sys.argv[1] if len(sys.argv) > 1 else "left").lower()
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
C.MAX_DUTY = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
from drive import Drive
if which not in ("left", "right"):
    sys.exit("first arg must be 'left' or 'right'")
l, r = (1.0, 0.0) if which == "left" else (0.0, 1.0)
bus = SMBus(1)


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


def setup():
    bus.write_byte_data(ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.05)
    bus.write_byte_data(ADDR, CONFIG_REG, 0x03)
    bus.write_byte_data(ADDR, GYRO_CFG, 0x18)
    bus.write_byte_data(ADDR, ACCEL_CFG, 0x00)
    time.sleep(0.05)


def read():
    try:
        d = bus.read_i2c_block_data(ADDR, ACCEL_XOUT, 14)
    except OSError:
        return None
    return (s16(d[0], d[1]) / ACCEL_LSB, s16(d[2], d[3]) / ACCEL_LSB,
            s16(d[4], d[5]) / ACCEL_LSB, s16(d[8], d[9]) / GYRO_LSB,
            s16(d[10], d[11]) / GYRO_LSB, s16(d[12], d[13]) / GYRO_LSB)
for attempt in range(20):
    try:
        setup()
        break
    except OSError:
        time.sleep(0.2)
else:
    sys.exit("IMU never answered")
print("calibrating - hold still", flush=True)
samples = []
while len(samples) < CAL_N:
    s = read()
    if s:
        samples.append(s)
    time.sleep(0.01)
n = float(len(samples))
bias = [sum(s[3 + k] for s in samples) / n for k in range(3)]
mx, my, mz = (sum(s[k] for s in samples) / n for k in range(3))
m = math.sqrt(mx * mx + my * my + mz * mz) or 1.0
up = [mx / m, my / m, mz / m]
print(f"  bias {[round(b,2) for b in bias]}  up {[round(u,2) for u in up]}", flush=True)
print(f"\nPIVOT on {which.upper()} pair, {secs}s at cap {C.MAX_DUTY:.2f} "
      f"(~{C.MAX_DUTY*17.5:.1f} V)", flush=True)
d = Drive()
total = 0.0
ok = miss = 0
t0 = time.time()
last_t = None
try:
    d.tank(l, r, smooth=False)
    while time.time() - t0 < secs:
        s = read()
        now = time.time()
        if s is None:
            miss += 1
        else:
            ok += 1
            cx, cy, cz = s[3] - bias[0], s[4] - bias[1], s[5] - bias[2]
            rate = cx * up[0] + cy * up[1] + cz * up[2]
            if last_t is not None:
                total += rate * (now - last_t)
            last_t = now
        time.sleep(0.005)
finally:
    d.stop()
    elapsed = time.time() - t0
for _ in range(40):
    s = read()
    now = time.time()
    if s:
        cx, cy, cz = s[3] - bias[0], s[4] - bias[1], s[5] - bias[2]
        rate = cx * up[0] + cy * up[1] + cz * up[2]
        if last_t is not None:
            total += rate * (now - last_t)
        last_t = now
    time.sleep(0.01)
print(f"\n  turned     {total:+.1f} deg   ({'CCW/left' if total > 0 else 'CW/right'})")
print(f"  that is    {abs(total)/360.0:.2f} full turns")
print(f"  rate       {abs(total)/elapsed:.1f} deg/s")
print(f"  samples    {ok} good, {miss} missed "
      f"({100.0*miss/max(1,ok+miss):.1f}% lost to bus noise)")
if miss:
    print("  NOTE: missed samples mean the real rotation is larger than measured")
if abs(total) > 5:
    print(f"  360 deg would take {360.0*elapsed/abs(total):.2f} s")
