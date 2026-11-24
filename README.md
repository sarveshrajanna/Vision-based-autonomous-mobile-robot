# Vision-based-autonomous-mobile-robot
Vision based Autonomous mobile robo with FIDUCIAL-GUIDED Navigation and Intelligent obstacle Avoidance.

# Results & Discussion — Camera-Only Autonomous Rover

Platform: Raspberry Pi 4B · OV5647 camera (640×480) · MPU-6050 IMU · L298N driver ·
four TT gearmotors · 20 V tool-pack supply. No LiDAR, no ultrasonics, no encoders,
no ROS, no network dependency.

---

## 1. System performance

| Parameter | Value |
|---|---|
| Vision pipeline throughput | **8.2 fps** |
| Marker detection rate | 1 frame in 3 (~2.7 Hz) |
| Inertial sampling rate | **42 Hz** |
| Gyroscope full scale | ±2000 °/s |
| Control command latency | **2 ms** |
| I²C bus error rate (motors idle) | **1 in 7 197 reads** |
| Live telemetry bandwidth | 340 KB/s |
| Approximate platform cost | **≈ ₹5 000** |

---

## 2. Rotational characterisation

**→ `fig1_pivot_characterisation.png`**

Four duty levels were commanded as fixed-duration pivots, with the resulting
rotation measured by the onboard gyroscope.

| Duty | Pivot rate | Overshoot after power cut |
|---|---|---|
| 0.40 | 1.7 °/s | — (does not break static friction) |
| 0.50 | 31.6 °/s | 4.6° |
| 0.60 | 49.5 °/s | 15.4° |
| 0.70 | 86.5 °/s | 30.2° |

**Finding.** Across the usable range, angular speed increases **2.7×** while
overshoot increases **6.6×**. Precision therefore degrades faster than speed
improves, and accurate angular positioning requires the *lowest* duty that
reliably overcomes static friction — not the highest available.

This result is used directly in the controller: a correcting turn subtracts the
characterised coast from the commanded displacement and powers only the
remainder, so momentum completes the manoeuvre rather than spoiling it.

---

## 3. Drivetrain constraint: skid-steer scrub

| Rotation strategy | Measured rate |
|---|---|
| Counter-rotation (all four wheels) | **0.2 °/s** |
| Single-pair pivot | **31.6 °/s** |

A four-wheel differential platform must drag all four tyres laterally to rotate
about its centre. The available torque proved insufficient to overcome this
scrub, reducing counter-rotation to a stall. Driving **one pair only** — so the
chassis pivots about the stationary pair — restored full rotational authority at
identical drive power, a **150× improvement** with no hardware change.

---

## 4. Driver current limit

**→ `fig2_series_vs_parallel.png`**

Motor winding resistance was measured at **1.1 Ω**. With two motors per H-bridge
channel, the configuration determines current demand:

| Configuration | Per-channel stall demand @ 17.5 V | Outcome |
|---|---|---|
| Parallel pair | **≈ 31 A** | Output collapsed to 1.8 V; thermal fold-back |
| Series pair | **≈ 8 A** | Full output retained |

The L298N is rated at 2 A per channel. In parallel the demand exceeded this by
an order of magnitude; the bridge entered current limiting and the output
voltage collapsed under load — measured at **1.8 V against a 17.5 V supply**,
with the driver overheating within seconds.

Reconfiguring each pair in **series** divides the supply across two windings and
quarters the current demand for equivalent total torque. Both failure modes were
eliminated.

---

## 5. Monocular metric ranging

**→ `fig3_monocular_ranging.png`**

Range is recovered from a single image using the pinhole projection relation:

```
d = f · W / w_px        f = (W_img / 2) / tan(FOV_h / 2) = 635 px
```

where `W` is the known physical marker width and `w_px` its apparent width in
frame. No stereo baseline, no time-of-flight, no structured light.

Working envelope: markers are acquired at roughly 60–80 px apparent width
(~85 cm) and arrival is declared at ~30% of frame width (~28 cm). Beyond about
2.5 m the marker subtends too few pixels for reliable decoding; closer than
~25 cm its border leaves the field of view and detection ceases — a limit the
controller handles by latching arrival rather than treating it as loss of target.

---

## 6. Compute budget

**→ `fig4_compute_budget.png`**

The full perception pipeline costs ~163 ms per frame on the Pi 4B without any
accelerator, dominated by fiducial detection. Running every stage on every frame
would cap the system at ~6 fps and starve the control loop, since the Python
interpreter serialises the work.

**Mitigation.** Video capture and detection were decoupled: every frame is
captured and streamed, while detection runs on one frame in three. This raised
throughput to **8.2 fps** and reduced processor occupancy from **100% to 36%**
of a core, restoring command latency to **2 ms**.

---

## 7. Sensor robustness

The IMU is near-perfect at rest — **1 failed read in 7 197** — but degrades
sharply under motor load, with brush noise coupling into the I²C bus and
producing dozens of dropouts per run.

Three design responses, all of which remain valid under degraded sensing:

1. **Bus speed reduced** to 50 kHz, doubling bit duration and noise immunity.
2. **Dead-reckoning fallback** — if the gyroscope fails mid-survey, the sweep is
   estimated from a characterised degrees-per-burst constant, so the manoeuvre
   terminates rather than running indefinitely.
3. **Angle-free obstacle avoidance** — the avoidance routine turns until the path
   is visually clear and reverses by recorded direction, never by measured angle.
   It is therefore unaffected by inertial dropout entirely.

---

## 8. Summary

The system demonstrates that vision-based indoor navigation is achievable
**without LiDAR, depth sensing, wheel odometry, ROS, or network connectivity**,
on a single-board computer costing a fraction of conventional platforms.

Key engineering results:

- Metric range from one camera via projective geometry — **no depth sensor**
- Single-pair pivoting overcomes 4WD scrub — **150× rotation improvement**
- Series motor pairing resolves H-bridge current limiting — **4× demand reduction**
- Detection/streaming decoupling — **100% → 36%** processor occupancy
- Avoidance that survives sensor failure by requiring no angular measurement

---

*Figures 1 and the tables in §2–§4, §6–§7 are measured on the hardware. Figure 3
is the projection model the controller runs. Figure 4's total is measured; the
split between stages is approximate. Navigation success rate is not reported —
no trial series has been run.*

