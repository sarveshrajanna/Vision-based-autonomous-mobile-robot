import sys
import time
from smbus2 import SMBus
ADDR = 0x68
PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
GYRO_CFG = 0x1B
ACCEL_CFG = 0x1C
ACCEL_XOUT = 0x3B
ACCEL_LSB = 16384.0
GYRO_LSB = 131.0
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v
bus = SMBus(1)


def init():
    who = bus.read_byte_data(ADDR, 0x75)
    bus.write_byte_data(ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.05)
    bus.write_byte_data(ADDR, CONFIG, 0x03)
    bus.write_byte_data(ADDR, GYRO_CFG, 0x00)
    bus.write_byte_data(ADDR, ACCEL_CFG, 0x00)
    time.sleep(0.05)
    return who
who = None
for attempt in range(40):
    try:
        who = init()
        break
    except OSError as e:
        print(f"init attempt {attempt+1} failed: {e}", flush=True)
        time.sleep(0.25)
if who is None:
    print("GAVE UP - device never answered in 10 s", flush=True)
    sys.exit(1)
print(f"WHO_AM_I = 0x{who:02x}", flush=True)
t0 = time.time()
nxt = t0 + 1.0
ok = bad = sec_ok = sec_bad = 0
last = None
while time.time() - t0 < dur:
    try:
        d = bus.read_i2c_block_data(ADDR, ACCEL_XOUT, 14)
        ax, ay, az = (s16(d[i], d[i + 1]) / ACCEL_LSB for i in (0, 2, 4))
        gx, gy, gz = (s16(d[i], d[i + 1]) / GYRO_LSB for i in (8, 10, 12))
        last = (ax, ay, az, gx, gy, gz)
        ok += 1
        sec_ok += 1
    except Exception as e:
        bad += 1
        sec_bad += 1
        last_err = repr(e)[:60]
    now = time.time()
    if now >= nxt:
        t = int(now - t0)
        if last and sec_bad == 0:
            ax, ay, az, gx, gy, gz = last
            mag = (ax * ax + ay * ay + az * az) ** 0.5
            print(f"t={t:3d}s  ok={sec_ok:3d} bad={sec_bad:3d}  "
                  f"acc=({ax:+.2f},{ay:+.2f},{az:+.2f}) |a|={mag:.2f}g  "
                  f"gyro=({gx:+7.1f},{gy:+7.1f},{gz:+7.1f})dps", flush=True)
        else:
            print(f"t={t:3d}s  ok={sec_ok:3d} bad={sec_bad:3d}  <<< DROPOUT", flush=True)
        sec_ok = sec_bad = 0
        nxt += 1.0
    time.sleep(0.02)
print(f"TOTAL good={ok} bad={bad}  rate={100.0*bad/max(1,ok+bad):.2f}% failed", flush=True)
