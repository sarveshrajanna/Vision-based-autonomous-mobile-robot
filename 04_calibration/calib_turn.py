import math
import sys
import time
from smbus2 import SMBus
import config as C
ADDR = 0x68
PWR_MGMT_1, CONFIG_REG, GYRO_CFG, ACCEL_CFG = 0x6B, 0x1A, 0x1B, 0x1C
ACCEL_XOUT = 0x3B
ACCEL_LSB = 16384.0
GYRO_LSB = 16.4
CAL_N = 120
MAX_DT = 0.5
COOL_S = 3.0
BURST_CAP = 1.5
burst = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
duties = [float(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2    else [0.40, 0.50, 0.60, 0.70]
if burst > BURST_CAP:
    sys.exit(f"burst {burst}s exceeds the {BURST_CAP}s cap - refusing")
if max(duties) > 0.85:
    sys.exit("duty above 0.85 - refusing, that is heat territory")
C.MAX_DUTY = max(duties)
from drive import Drive
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
for _ in range(20):
    try:
        setup()
        break
    except OSError:
        time.sleep(0.2)
else:
    sys.exit("IMU never answered")
print("calibrating bias - hold still", flush=True)
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
print(f"  bias {[round(b, 2) for b in bias]}\n", flush=True)
d = Drive()
rows = []
try:
    for duty in duties:
        C.MAX_DUTY = duty
        acc, prev, ok, miss = 0.0, None, 0, 0
        t0 = time.time()
        d.tank(1.0, 0.0, smooth=False)
        while time.time() - t0 < burst:
            s = read()
            now = time.time()
            if s is None:
                miss += 1
            else:
                ok += 1
                cx, cy, cz = s[3] - bias[0], s[4] - bias[1], s[5] - bias[2]
                rate = cx * up[0] + cy * up[1] + cz * up[2]
                if prev is not None:
                    acc += rate * min(now - prev, MAX_DT)
                prev = now
            time.sleep(0.004)
        d.stop()
        powered = time.time() - t0
        at_cut = acc
        for _ in range(60):
            s = read()
            now = time.time()
            if s:
                cx, cy, cz = s[3] - bias[0], s[4] - bias[1], s[5] - bias[2]
                rate = cx * up[0] + cy * up[1] + cz * up[2]
                if prev is not None:
                    acc += rate * min(now - prev, MAX_DT)
                prev = now
            time.sleep(0.01)
        coast = acc - at_cut
        rate = abs(at_cut) / powered
        rows.append((duty, at_cut, coast, rate, miss, ok))
        print(f"  duty {duty:.2f} (~{duty*17.5/2:.1f} V/motor): "
              f"turned {at_cut:+7.1f} deg in {powered:.2f}s = {rate:6.1f} deg/s, "
              f"coast {coast:+.1f} deg, {miss} samples lost", flush=True)
        time.sleep(COOL_S)
finally:
    d.stop()
    d.close()
print("\n  duty   deg/s   coast   s per 90deg   s per 360deg")
for duty, cut, coast, rate, miss, ok in rows:
    if rate > 1:
        print(f"  {duty:.2f}  {rate:6.1f}  {coast:+6.1f}      "
              f"{90.0/rate:6.2f}        {360.0/rate:6.2f}")
    else:
        print(f"  {duty:.2f}  {rate:6.1f}   (did not turn)")
print("\n  'coast' is how far it keeps turning after power is cut - a closed-loop")
print("  turn must cut power that many degrees early to land on target.")
