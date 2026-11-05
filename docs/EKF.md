# EKF (0.9.0): attitude + velocity + position + IMU biases in one filter

`src/core/ekf.h`. Runs **in parallel** with the old estimators (`nav.ekf = SHADOW`, the default) and only writes to the blackbox / telemetry for comparison. `nav.ekf = NAV`: Position Hold and GPS Rescue take position, velocity and heading from the EKF while it has GPS (optical flow still goes the old way). The attitude used by the control loop **is still the AHRS's** in both modes.

## 1. How it differs from the existing Kalman filters

| | Old estimators (`ahrs.h` + `horizontal_est.h` + `vertical_est.h`) | EKF |
|---|---|---|
| Attitude | Mahony, separate | a state of the same filter |
| Position / velocity | linear Kalman, treats attitude as correct | same filter, with cross-covariance against attitude |
| Heading when the compass is noisy | lost (`YAW_FROM_MAG` off), Position Hold unusable | GPS velocity corrects heading every time the craft accelerates |
| Gyro / accel bias | gyro: Mahony integral; accel z: in the altitude filter | 6 states, estimated jointly |

"Extended": rotating acceleration into the earth frame is nonlinear in attitude, so the model is linearised around the current estimate at every step. "Error state": the filter estimates the **small error** of a nominal state that is integrated fully nonlinearly (the quaternion stays a quaternion). This is the form PX4 EKF2 uses (Solà 2017, global angular error).

- 15 error states: angle (3, earth frame), NED velocity (3), NED position (3), gyro bias (3), accel bias (3).
- Prediction from the gyro / accel sums of **every IMU sample** (accumulated by the flight task, ~4 kHz), every 4 ms (250 Hz).
- Measurements:
  - GPS N/E position and N/E velocity (D if u-blox), compared against the state from `gps_delay_s` earlier (0.5 s of history kept; every correction and every reset moves that history with the state, so the next fix does not see the same jump again - 0.44.0);
  - baro (altitude);
  - compass: **heading only**, Jacobian with respect to the angle of the horizontal magnetic field (not the Euler yaw angle: when the craft is tilted the two differ). At most 10 Hz, sigma 0.5 rad like ArduPilot's EK3_YAW_M_NSE: compass error near the motors is a slow offset, not white noise;
  - no GPS for more than 1 s: "stationary in place" (sigma 10 m, EK3_NOAID_M_NSE) so tilt does not drift;
  - disarmed and stationary: zero velocity.
- Every measurement is gated (GPS 5σ, baro 5σ, compass 3σ). GPS position / baro rejected continuously for 5 s: the state jumps to follow (like ArduPilot's ResetHeight / ResetPosition).
- Accel X/Y bias is tightly constrained (like PX4 EKF2): on a multirotor thrust is always along the z axis, so these two biases are almost unobservable. This is a precaution for the real board; **the PC tests cannot prove it is needed**: left free, test_ekf still passes (at first I saw a 29° heading error that did not converge, but that was a velocity-attitude coupling bug in F, now fixed).
- Ground alignment: tilt from the accelerometer, heading from the compass (waits up to 3 s); without a compass it takes the AHRS heading with a large uncertainty.

## 2. Cost

The prediction step only multiplies the ~40 non-zero elements of F (≈ 1000 multiply-adds, not 6750). On PC: ~1 µs/step. On the ESP32-S3 expected 30–60 µs per step, running in the sensor thread **outside the lock**, not touching the control cycle. The real figure is shown in the Position tab ("One EKF step costs") and the `estimate.ekf_us` stream: **not yet measured on the board**.

## 3. Verification on PC

`test/test_ekf.cpp` (13 checks) with a simulated quadcopter whose true state is known exactly: the attitude is what a multirotor must have to fly that trajectory (the accelerometer only sees thrust), 250 Hz IMU with noise and bias, 10 Hz or 1 Hz GPS with delay. `WB_VERBOSE=1 ./build/test_ekf` prints the numbers:

| Scenario | Result |
|---|---|
| hover 60 s | tilt error ≤ 0.37°, heading ≤ 0.30°, position ≤ 0.18 m |
| figure 8, 30 × 15 m, tilt up to ~25° | tilt ≤ 0.43°, heading ≤ 0.53°, position ≤ 0.20 m |
| no compass, starts 29° off | flying figure 8: heading converges to ≤ 0.83° |
| compass disturbed by thrust (alone 20.7° RMS error) | EKF 8.9° RMS error (in steady hover nothing can distinguish a static compass offset) |
| yaw gyro off by 1°/s | learns 0.99°/s |
| GPS 1 Hz | position ≤ 0.60 m (10 Hz: 0.18 m), tilt ≤ 0.43° |
| GPS jumps 25 m for 3 s | rejected, estimate off ≤ 0.21 m; a permanent jump is followed by the state after 5 s |
| hover 60 s without GPS | tilt ≤ 0.2° (without the "stationary in place" constraint: 0.9°) |
| stationary, accel z off by 0.3 m/s² | learns 0.29 |

`test_fw` group `ekf` (real firmware on the virtual board):
- SHADOW: aligns correctly to a 60° heading on the ground; flying legs with Position Hold, EKF heading off from truth ≤ 4°, position ≤ 1.5 m, tilt ≤ 3°.
- NAV: Position Hold holds within 0.8 m for 10 s, pushing the stick moves exactly along the nose.
- NAV, **position resets** (0.44.0): when the GPS solution jumps (multipath, a new satellite set) the EKF is reset to it after 5 s of rejections; Position Hold and Loiter move their target by the same step, so the craft stays where it is (12 m jump: within 0.06 m; before 0.44.0 it ended 2 m off, because the target was moved by the old estimator's step). A switch between the EKF and the old estimator (the EKF starts or stops aiding) is handled the same way, with the difference between the two positions as the step (`src/core/nav/nav_source.h`, as ArduPilot reports a lane switch).
- **NaN / Inf measurements** (0.44.0): a GPS fix, a baro height or a compass reading with a NaN / Inf is never fused (a NaN innovation passes every gate) and never reset to - before, 5 s of a NaN GPS would have reset the state to NaN. A NaN vertical speed alone only drops the vertical part. Every variance stays above zero and no correlation above 1 under a minute of 'exact' (1e-6) measurements (`test_ekf` covariance group).
- NAV, **no compass**: at first Position Hold is refused (the old estimator does not know heading); after a few accelerations in Altitude Hold the EKF gets heading from GPS velocity (0.5° off), Position Hold can be engaged and holds within 0.19 m for 8 s. This is where the EKF does something the old estimator cannot.
- OFF: does not run.
- `test_threads` (ThreadSanitizer) running with the EKF: no access outside the lock.

## 4. Replaying real flight logs

`build/ekf_replay fast.csv slow.csv [out.csv] [--acc-noise x] [--gps-delay s] [--no-mag]` (`tools/ekf_replay.cpp`). With the 2026-10-11 log (firmware 0.8.7):

- **EKF and AHRS attitude agree**: roll 1.3°, pitch 1.9°, heading 3.3° RMS (about 2° of that is level trim, which the AHRS has and the raw accelerometer does not).
- **Velocity minus raw GPS velocity, just before each fix is used**: EKF 1.30 m/s (1.11 with `--acc-noise 1`), firmware 1.23 m/s. With 1 Hz GPS both are around 1 m/s: the GPS rate is the limit, exactly as analysed in docs/NAV.md section 11.
- **Altitude**: EKF at most 7.5 m; firmware 0.8.7 drifted up to 290 m (bug fixed in 0.8.8).
- **Heading while the compass is disturbed (10–27 s)**: EKF differs from AHRS by up to −9.6°. The log has no true heading to tell which one is right.
- Limits of this replay:
  - the log only has **instantaneous** 250 Hz accelerometer samples, so vibration is aliased (on the board the EKF receives the sum of all 4 kHz samples);
  - the gyro in the log is filtered and in pilot convention (pitch sign inverted, the tool flips it back);
  - coordinates in the CSV are rounded (11 m), so only GPS velocity is used.

**CSV export fixed in 0.9.0**: previously `"%g"` kept only 6 digits, so latitude 20.8634 meant an 11 m step. Now every column has enough decimals for its resolution.

## 5. Test flights in order
1. Leave SHADOW, GPS ≥ 5 Hz (0.8.8 requests the rate itself if `gps.auto_config` is on; NMEA needs baud ≥ 38400). Fly as usual, with accelerations and heading changes, and send the `.wblog` log (or the CSV exported from app 0.9.0).
2. I compare `ekf_yaw` with `yaw`, `ekf_vn/ve` with GPS velocity, `ekf_pn/pe` with `pos_n/pos_e`. If the EKF is clearly better, try NAV at a safe altitude in an open area, ready to flip the Position Hold switch off.
