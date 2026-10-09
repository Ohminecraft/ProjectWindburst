// The same formulas the firmware uses, for the previews (motor diagram, rate curve, throttle curve).
// tools/ui_reference.cpp prints the firmware's own numbers and app/tests/test_gui_math.py compares them with this file,
// so a preview can not silently drift away from what the craft does.

// Frame geometry: motor angle in degrees clockwise from the nose (seen from above), and prop direction (+1 = counter-clockwise).
// Source: src/core/mixer.h, mixer_row().
// Quads follow Betaflight's default build ("props in"): X = 1 rear-right CW, 2 front-right CCW, 3 rear-left CCW, 4 front-left CW.
export const LAYOUTS = {
  QUAD_X: { angles: [135, 45, 225, 315], spin: [-1, 1, 1, -1] },
  QUAD_PLUS: { angles: [180, 90, 270, 0], spin: [-1, 1, 1, -1] },
  HEX_X: { angles: [30, 90, 150, 210, 270, 330], spin: [1, -1, 1, -1, 1, -1] },
};
export const motorCount = (layout) => LAYOUTS[layout].angles.length;

// Mixer row of motor m: how much it answers to +roll (right wing down), +pitch (nose down), +yaw (nose right).
export function mixerRow(layout, m, propsOut = false) {
  const L = LAYOUTS[layout], rad = Math.PI / 180;
  let rmax = 0, pmax = 0;
  for (const a of L.angles) { rmax = Math.max(rmax, Math.abs(Math.sin(a * rad))); pmax = Math.max(pmax, Math.abs(Math.cos(a * rad))); }
  let roll = -Math.sin(L.angles[m] * rad) / rmax, pitch = -Math.cos(L.angles[m] * rad) / pmax;
  if (Math.abs(roll) < 1e-4) roll = 0;
  if (Math.abs(pitch) < 1e-4) pitch = 0;
  return { roll, pitch, yaw: L.spin[m] * (propsOut ? -1 : 1) };
}

// Stick (-1..1 after deadband) -> deg/s, Betaflight "Actual" rates. Source: src/core/rc.h, apply_actual_rate(). expoPct 0..100.
export function actualRate(centerSens, maxRate, expoPct, s) {
  const expo = expoPct * 0.01, a = Math.abs(s), s5 = s * s * s * s * s;
  const e = a * (s5 * expo + s * (1 - expo));
  const extra = Math.max(0, maxRate - centerSens);
  return s * centerSens + extra * e;
}

// Receiver microseconds -> stick -1..1 with deadband. Source: rc.h, RcInterpreter::process(). deadbandPct 0..20.
export function stick(us, deadbandPct) {
  const db = deadbandPct * 0.01;
  const v = Math.max(-1, Math.min(1, (us - 1500) / 500));
  if (Math.abs(v) <= db) return 0;
  return (v - Math.sign(v) * db) / (1 - db);
}

// Throttle stick 0..1 -> throttle command 0..1. Source: rc.h, RcInterpreter::curve(). mid / expo / limit in percent.
export function throttleCurve(midPct, expoPct, limitPct, t) {
  const m = midPct * 0.01, ex = expoPct * 0.01, e = t - 0.5;
  const x = e * (1 - ex) + ex * e * e * e * 4, base = 0.5 + x;
  const out = base <= 0.5 ? base * (m / 0.5) : m + (base - 0.5) * ((1 - m) / 0.5);
  return Math.max(0, Math.min(1, out)) * (limitPct * 0.01);
}
export const throttle01 = (us) => Math.max(0, Math.min(1, (us - 1000) / 1000));

// Attitude as the telemetry reports it -> rotation matrix body (forward, right, down) -> world (north, east, down), for the 3D view.
// Source: src/core/ahrs.h, euler_user() (the one place the pilot's sign convention is applied).
// roll: right wing down +, pitch: nose DOWN + (the firmware's pilot convention), yaw: nose right +. Returns body -> world.
export function rotation(rollDeg, pitchDeg, yawDeg) {
  const RAD = Math.PI / 180;
  const ph = rollDeg * RAD, th = -pitchDeg * RAD, ps = yawDeg * RAD;
  const cph = Math.cos(ph), sph = Math.sin(ph), cth = Math.cos(th), sth = Math.sin(th), cps = Math.cos(ps), sps = Math.sin(ps);
  return [
    [cps * cth, cps * sth * sph - sps * cph, cps * sth * cph + sps * sph],
    [sps * cth, sps * sth * sph + cps * cph, sps * sth * cph - cps * sph],
    [-sth, cth * sph, cth * cph],
  ];
}

// Beeper sound patterns as the firmware plays them (src/hal/beeper/beeper_core.h): durations in units of 10 ms, alternating on, off, on ...
// A leading 0 = the pattern starts with its pause. tools/ui_reference prints the firmware's table; test_gui_math.py compares.
export const BEEP_PATTERNS = {
  GYRO_CALIBRATED: [20, 10, 20, 10, 20, 10],
  RX_LOST: [50, 50],
  RX_LOST_LANDING: [10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70],
  DISARMING: [15, 5, 15, 5],
  ARMING: [30, 5, 5, 5],
  ARMING_GPS_FIX: [5, 5, 15, 5, 5, 5, 15, 30],
  ARMING_GPS_NO_FIX: [30, 5, 30, 5, 30, 5],
  BAT_CRIT_LOW: [50, 2],
  BAT_LOW: [25, 50],
  RX_SET: [10, 10],
  ACC_CALIBRATION: [5, 5, 5, 5],
  ACC_CALIBRATION_FAIL: [20, 15, 35, 5],
  READY_BEEP: [4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5],
  ARMED: [0, 245, 10, 5],
  SYSTEM_INIT: [3, 5, 3, 5],
  ARM_REFUSED: [40, 10, 10, 10, 10, 10],
  BLACKBOX_ERASE: [5, 5, 5, 5],
};

// The motors as the sounder (DShot beacon): one ESC tone per "on" part of a pattern, each taking BEACON_SLOT_MS; the pauses stay.
// Conditions of an armed craft can never be played by motors. Source: beeper_core.h (kMotorNever, kBeaconIntervalMs, Beeper::slot_ms).
export const BEACON_SLOT_MS = 450;
export const MOTOR_NEVER = ['RX_LOST_LANDING', 'ARMING', 'ARMING_GPS_FIX', 'ARMING_GPS_NO_FIX', 'ARMED', 'USB'];
// [number of tones, milliseconds from the first tone to the last] when the motors play pattern `name` once
export function motorTones(name) {
  const seq = BEEP_PATTERNS[name];
  if (!seq) return [0, 0];
  let tones = 0, t = 0, first = 0, last = 0;
  seq.forEach((d, i) => {
    if (d === 0) return;
    if (i % 2 === 0) { if (!tones) first = t; last = t; tones += 1; t += BEACON_SLOT_MS; } else t += d * 10;
  });
  return [tones, last - first];
}
