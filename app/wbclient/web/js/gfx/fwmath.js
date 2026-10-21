// The same formulas the firmware uses, for the previews (motor diagram, rate curve, throttle curve).
// tools/ui_reference.cpp prints the firmware's own numbers and app/tests/test_gui_math.py compares them with this file,
// so a preview can not silently drift away from what the craft does.

// Frame geometry: motor angle in degrees clockwise from the nose (seen from above), and prop direction (+1 = counter-clockwise).
// Source: src/core/mixer.h, mixer_row().
// Quads follow Betaflight's default build ("props in"): X = 1 rear-right CW, 2 front-right CCW, 3 rear-left CCW, 4 front-left CW.
// The frames after HEX_X (0.8.2) are Betaflight's tables (rows: roll, pitch, yaw per motor, src/core/mixer.h layout_table());
// angles / r (arm fraction) / under (lower motor of a coaxial pair) only place them in the drawing.
const D = 0.7071068;
export const LAYOUTS = {
  QUAD_X: { angles: [135, 45, 225, 315], spin: [-1, 1, 1, -1] },
  QUAD_PLUS: { angles: [180, 90, 270, 0], spin: [-1, 1, 1, -1] },
  HEX_X: { angles: [30, 90, 150, 210, 270, 330], spin: [1, -1, 1, -1, 1, -1] },
  HEX_PLUS: { angles: [120, 60, 240, 300, 0, 180], spin: [1, -1, 1, -1, 1, -1],
    rows: [[-0.866025, 0.5, 1], [-0.866025, -0.5, -1], [0.866025, 0.5, 1], [0.866025, -0.5, -1], [0, -1, 1], [0, 1, -1]] },
  HEX_X_BF: { angles: [150, 30, 210, 330, 90, 270], spin: [1, 1, -1, -1, -1, 1],
    rows: [[-0.5, 0.866025, 1], [-0.5, -0.866025, 1], [0.5, 0.866025, -1], [0.5, -0.866025, -1], [-1, 0, -1], [1, 0, 1]] },
  HEX_H: { angles: [135, 45, 225, 315, 90, 270], r: [1, 1, 1, 1, D, D], spin: [-1, 1, 1, -1, 1, -1],
    rows: [[-1, 1, -1], [-1, -1, 1], [1, 1, 1], [1, -1, -1], [0, 0, 0], [0, 0, 0]] },
  Y6: { angles: [180, 60, 300, 180, 60, 300], under: [0, 0, 0, 1, 1, 1], spin: [1, -1, -1, -1, 1, 1],
    rows: [[0, 1.333333, 1], [-1, -0.666667, -1], [1, -0.666667, -1], [0, 1.333333, -1], [-1, -0.666667, 1], [1, -0.666667, 1]] },
  Y4: { angles: [180, 45, 180, 315], r: [D, 1, D, 1], under: [0, 0, 1, 0], spin: [-1, 1, 1, -1],
    rows: [[0, 1, -1], [-1, -1, 0], [0, 1, 1], [1, -1, 0]] },
  OCTO_X8: { angles: [135, 45, 225, 315, 135, 45, 225, 315], under: [0, 0, 0, 0, 1, 1, 1, 1], spin: [-1, 1, 1, -1, 1, -1, -1, 1],
    rows: [[-1, 1, -1], [-1, -1, 1], [1, 1, 1], [1, -1, -1], [-1, 1, 1], [-1, -1, -1], [1, 1, -1], [1, -1, 1]] },
  OCTO_FLAT_PLUS: { angles: [315, 45, 135, 225, 0, 90, 180, 270], spin: [1, 1, 1, 1, -1, -1, -1, -1],
    rows: [[0.707107, -0.707107, 1], [-0.707107, -0.707107, 1], [-0.707107, 0.707107, 1], [0.707107, 0.707107, 1], [0, -1, -1], [-1, 0, -1], [0, 1, -1], [1, 0, -1]] },
  OCTO_FLAT_X: { angles: [292.5, 22.5, 112.5, 202.5, 337.5, 67.5, 157.5, 247.5], spin: [1, 1, 1, 1, -1, -1, -1, -1],
    rows: [[1, -0.414178, 1], [-0.414178, -1, 1], [-1, 0.414178, 1], [0.414178, 1, 1], [0.414178, -1, -1], [-1, -0.414178, -1], [-0.414178, 1, -1], [1, 0.414178, -1]] },
  VTAIL4: { angles: [150, 45, 210, 315], r: [0.9, 1, 0.9, 1], spin: [1, -1, -1, 1],
    rows: [[-0.58, 0.58, 1], [-0.46, -0.39, -0.5], [0.58, 0.58, -1], [0.46, -0.39, 0.5]] },
  ATAIL4: { angles: [160, 45, 200, 315], r: [0.9, 1, 0.9, 1], spin: [1, 1, -1, -1],
    rows: [[0, 1, 1], [-1, -1, 0], [0, 1, -1], [1, -1, 0]] },
  QUAD_X_1234: { angles: [315, 45, 135, 225], spin: [-1, 1, -1, 1], rows: [[1, -1, -1], [-1, -1, 1], [-1, 1, -1], [1, 1, 1]] },
};
export const motorCount = (layout) => (LAYOUTS[layout] || LAYOUTS.QUAD_X).angles.length;

// Mixer row of motor m: how much it answers to +roll (right wing down), +pitch (nose down), +yaw (nose right).
export function mixerRow(layout, m, propsOut = false) {
  const L = LAYOUTS[layout], rad = Math.PI / 180;
  if (L.rows) {                                            // Betaflight's table, each axis scaled so that its largest |value| is 1
    const mx = [0, 1, 2].map((k) => Math.max(...L.rows.map((r) => Math.abs(r[k]))));
    const v = L.rows[m].map((x, k) => (mx[k] > 0 ? x / mx[k] : 0));
    return { roll: v[0], pitch: v[1], yaw: v[2] * (propsOut ? -1 : 1) };
  }
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

// The LED strip, layout MANUAL: the colors at t (ms), the same integer sums as app::LedLogic::render() (src/app/led_logic.h).
// led: the section (count, brightness, fn[] names, color[] 0xRRGGBB, warnings, speed).
// st: { armed, ready, failsafe, batt_critical, batt_warning, stall, calibrating, mode (ACRO / ANGLE / ...), batt (% or 255), thr (%) }
const LED_RGB = { RED: [255, 0, 0], GREEN: [0, 255, 0], BLUE: [0, 0, 255], AMBER: [255, 110, 0], CYAN: [0, 200, 255], PURPLE: [170, 0, 255], TEAL: [0, 255, 140], ORANGE: [255, 60, 0], WHITE: [255, 255, 255] };
const LED_MODE = { ANGLE: 'BLUE', STABILIZE: 'CYAN', ALTHOLD: 'PURPLE', POSHOLD: 'TEAL', RESCUE: 'ORANGE' };
const ledScale = (c, k) => c.map((v) => Math.floor(v * k / 255));
export const ledModeColor = (mode) => LED_RGB[LED_MODE[mode] || 'WHITE'];
export function ledHue(d) {
  d %= 360; const x = Math.floor((d % 60) * 255 / 60);
  return [[255, x, 0], [255 - x, 255, 0], [0, 255, x], [0, 255 - x, 255], [x, 0, 255], [255, 0, 255 - x]][Math.floor(d / 60)];
}
export function ledColors(led, st, t) {
  const n = Math.max(0, Math.min(32, led.count));
  let c, lit = true;
  if (st.failsafe) { c = LED_RGB.RED; lit = Math.floor(t / 100) % 2 === 0; }
  else if (st.batt_critical) { c = LED_RGB.RED; const p = t % 1000; lit = p < 100 || (p >= 200 && p < 300); }
  else if (st.stall) c = Math.floor(t / 250) % 2 ? LED_RGB.AMBER : LED_RGB.RED;
  else if (st.calibrating) { const p = t % 1500, k = p < 750 ? p : 1500 - p; c = ledScale(LED_RGB.CYAN, 40 + Math.floor(k * 215 / 750)); }
  else if (!st.armed) { if (st.ready) c = LED_RGB.GREEN; else { c = LED_RGB.AMBER; lit = Math.floor(t / 500) % 2 === 0; } }
  else { c = ledModeColor(st.mode); if (st.mode === 'RESCUE') lit = Math.floor(t / 250) % 2 === 0; if (st.batt_warning && t % 2000 < 200) c = LED_RGB.AMBER; }
  const status = lit ? c : [0, 0, 0];
  let out = Array.from({ length: n }, () => status);
  if (!(led.warnings && (st.failsafe || st.batt_critical || st.stall))) {
    const speed = Math.max(1, Math.min(10, led.speed)), cy = Math.floor(10000 / speed), ph = t % cy;
    const fns = led.fn.slice(0, n), nr = fns.filter((f) => f === 'RAINBOW').length, nl = fns.filter((f) => f === 'LARSON').length;
    let kr = 0, kl = 0, head = 0;
    if (nl > 1) { const span = 2 * (nl - 1), x = Math.floor(ph * span / cy); head = x <= nl - 1 ? x : span - x; }
    const off = [0, 0, 0];
    out = fns.map((f, i) => {
      const v = Number(led.color[i]) >>> 0, col = [(v >> 16) & 255, (v >> 8) & 255, v & 255];
      switch (f) {
        case 'STATUS': return status;
        case 'COLOR': return col;
        case 'MODE': return ledModeColor(st.mode);
        case 'ARMED': return st.armed ? col : off;
        case 'BLINK': return ph < Math.floor(cy / 2) ? col : off;
        case 'STROBE': return t % Math.floor(cy / 2) < 60 ? col : off;
        case 'BREATHE': { const k = ph < Math.floor(cy / 2) ? Math.floor(ph * 2 * 255 / cy) : Math.floor((cy - ph) * 2 * 255 / cy); return ledScale(col, 20 + Math.floor(k * 235 / 255)); }
        case 'RAINBOW': return ledHue(Math.floor(ph * 360 / cy) + Math.floor(kr++ * 360 / nr));
        case 'LARSON': { const d = Math.abs(kl++ - head); return d === 0 ? col : d === 1 ? ledScale(col, 50) : off; }
        case 'BATTERY': return st.batt <= 100 ? [Math.floor(255 * (100 - st.batt) / 100), Math.floor(255 * st.batt / 100), 0] : off;
        case 'THROTTLE': { const p = Math.min(100, st.thr), r = Math.floor(255 * p / 100); return [r, 0, 255 - r]; }
        default: return off;
      }
    });
  }
  const k = Math.floor(Math.min(100, led.brightness) * 255 / 100);
  return out.map((x) => ledScale(x, k));
}

// ---- the stick preview (Receiver and Rate tabs): what the craft would do with these sticks
// Acro: the body turns at the commanded rates. R (body -> world, rows north / east / down, as rotation()) times the turn of one
// step: user rates (roll right, pitch nose down, yaw nose right, deg/s) are the FRD body rates (p, -q, r) - the same integration
// as the firmware's Ahrs with the gyro alone (src/core/ahrs.h; test_gui_math compares).
export function acroStep(R, rateDps, dt) {
  const D = Math.PI / 180, w = [rateDps[0] * D * dt, -rateDps[1] * D * dt, rateDps[2] * D * dt];
  const th = Math.hypot(w[0], w[1], w[2]);
  if (th < 1e-12) return R.map((r) => r.slice());
  const k = w.map((v) => v / th), s = Math.sin(th), c = 1 - Math.cos(th);
  const K = [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]];
  const E = [0, 1, 2].map((i) => [0, 1, 2].map((j) => (i === j ? 1 : 0) + s * K[i][j] + c * (K[i][0] * K[0][j] + K[i][1] * K[1][j] + K[i][2] * K[2][j])));
  const out = R.map((row) => [0, 1, 2].map((j) => row[0] * E[0][j] + row[1] * E[1][j] + row[2] * E[2][j]));
  // keep it a rotation: Gram-Schmidt on the columns (body axes in world)
  const col = (j) => [out[0][j], out[1][j], out[2][j]], norm = (v) => { const n = Math.hypot(v[0], v[1], v[2]); return v.map((x) => x / n); };
  const x = norm(col(0)); let y = col(1); const d = x[0] * y[0] + x[1] * y[1] + x[2] * y[2]; y = norm(y.map((v, i) => v - d * x[i]));
  const z = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
  return [0, 1, 2].map((i) => [x[i], y[i], z[i]]);
}
// Angle: the tilt the stick asks for (deg) - the firmware's AngleController target: angle_limit x rate / max rate, limited.
export function angleTarget(limitDeg, centerSens, maxRate, expoPct, s) {
  const t = maxRate > 0 ? limitDeg * actualRate(centerSens, maxRate, expoPct, s) / maxRate : 0;
  return Math.max(-limitDeg, Math.min(limitDeg, t));
}
