// The Mission tab's plan <-> the firmware's `mission` section (nav/mission.h, ArduPilot AUTO meanings). Pure functions: no page,
// no store - app/tests/test_mission_plan.py runs them in node.
//
// A plan as the tab edits it:
//   { takeoff_alt, speed (m/s, 0 = the rescue's speed), rtl (back home and land after the last point), rc_lost,
//     points: [ { lat, lon, alt, act, ... } ] }
//   act 'fly'    fly to the point, hold `hold` s                          -> WAYPOINT(p = hold)
//   act 'servo'  fly there, hover `hover` s, then each step: the servo to `deg`, wait `wait` s
//                                                                         -> WAYPOINT, LOITER_TIME(hover), per step LOITER_TIME(wait) + SERVO
//                (a DO command runs when the NAV command before it starts: the servo moves, then the wait - AP_Mission::update)
//   act 'land'   land at the point, motors off, wait `wait` s, take off to `alt` again -> LAND, DELAY, TAKEOFF
// The section holds 32 commands; a longer plan is refused (an Error that says so).

export const MAX_COMMANDS = 32;
const E7 = 1e7;
const M_PER_DEG = 111319.5;

const round1 = (v) => Math.round(v * 10) / 10;
const e7 = (deg) => Math.round(deg * E7);

// degrees <-> pulse with the servo section's angle_min / angle_max at min_us / max_us (reverse swaps the ends); i: 0..3
export function degToUs(servo, i, deg) {
  const lo = servo.min_us[i], hi = servo.max_us[i], a0 = servo.angle_min[i], a1 = servo.angle_max[i];
  let f = a1 !== a0 ? (deg - a0) / (a1 - a0) : 0.5;
  f = Math.min(1, Math.max(0, f));
  if (servo.reverse[i]) f = 1 - f;
  return Math.round(lo + f * (hi - lo));
}
export function usToDeg(servo, i, us) {
  const lo = servo.min_us[i], hi = servo.max_us[i], a0 = servo.angle_min[i], a1 = servo.angle_max[i];
  let f = hi !== lo ? (us - lo) / (hi - lo) : 0.5;
  if (servo.reverse[i]) f = 1 - f;
  return Math.round(a0 + f * (a1 - a0));
}

// the commands a plan becomes, as [cmd, {lat, lon, alt, p, arg}] (lat / lon in degrees, 0 = none)
function commands(plan, servo) {
  const c = [];
  const add = (cmd, o = {}) => c.push({ cmd, lat: 0, lon: 0, alt: 0, p: 0, arg: 0, ...o });
  add('TAKEOFF', { alt: plan.takeoff_alt });
  if (plan.speed > 0) add('SPEED', { p: Math.round(plan.speed * 10) });
  for (const pt of plan.points || []) {
    const at = { lat: pt.lat, lon: pt.lon };
    if (pt.act === 'land') {
      add('LAND', at); add('DELAY', { p: Math.round(pt.wait || 0) }); add('TAKEOFF', { alt: pt.alt });
    } else if (pt.act === 'servo') {
      add('WAYPOINT', { ...at, alt: pt.alt });
      if (pt.hover > 0) add('LOITER_TIME', { p: Math.round(pt.hover) });
      for (const s of pt.steps || []) {
        add('LOITER_TIME', { p: Math.round(s.wait || 0) });
        add('SERVO', { arg: s.servo, p: servo ? degToUs(servo, s.servo - 1, s.deg) : Math.round(s.deg) });
      }
    } else add('WAYPOINT', { ...at, alt: pt.alt, p: Math.round(pt.hold || 0) });
  }
  if (plan.rtl) add('RTL');
  return c;
}
export const commandCount = (plan) => commands(plan, null).length;

// plan -> the section's values (as the app's settings carry them: enum names, heights in m)
export function encodePlan(plan, servo) {
  const c = commands(plan, servo);
  if (c.length > MAX_COMMANDS) throw new Error(`${c.length} > ${MAX_COMMANDS}`);
  const s = { count: c.length, rc_lost: plan.rc_lost || 'CONTINUE', cmd: [], arg: [], alt: [], lat: [], lon: [], p: [] };
  for (let i = 0; i < MAX_COMMANDS; i++) {
    const x = c[i] || { cmd: 'NONE', lat: 0, lon: 0, alt: 0, p: 0, arg: 0 };
    s.cmd.push(x.cmd); s.arg.push(x.arg); s.alt.push(round1(x.alt || 0)); s.lat.push(e7(x.lat || 0)); s.lon.push(e7(x.lon || 0)); s.p.push(x.p);
  }
  return s;
}

// the section -> a plan (the patterns encodePlan writes; anything else becomes the nearest point it can)
export function decodePlan(s, servo) {
  const n = Math.min(s.count || 0, MAX_COMMANDS);
  const at = (i) => ({ cmd: s.cmd[i], lat: s.lat[i] / E7, lon: s.lon[i] / E7, alt: s.alt[i], p: s.p[i], arg: s.arg[i] });
  const plan = { takeoff_alt: 10, speed: 0, rtl: false, rc_lost: s.rc_lost || 'CONTINUE', points: [] };
  let i = 0;
  if (n > 0 && s.cmd[0] === 'TAKEOFF') { plan.takeoff_alt = s.alt[0]; i = 1; }
  while (i < n) {
    const x = at(i);
    if (x.cmd === 'SPEED') { plan.speed = x.p / 10; i++; continue; }
    if (x.cmd === 'RTL') { plan.rtl = true; i++; continue; }
    if (x.cmd === 'LAND') {
      const pt = { lat: x.lat, lon: x.lon, alt: plan.takeoff_alt, act: 'land', wait: 0 }; i++;
      if (i < n && s.cmd[i] === 'DELAY') { pt.wait = s.p[i]; i++; }
      if (i < n && s.cmd[i] === 'TAKEOFF') { pt.alt = s.alt[i]; i++; }
      plan.points.push(pt); continue;
    }
    if (x.cmd === 'WAYPOINT') {
      i++;
      const pt = { lat: x.lat, lon: x.lon, alt: x.alt, act: 'fly', hold: x.p };
      // hover, then (LOITER_TIME + SERVO) pairs
      if (i < n && s.cmd[i] === 'LOITER_TIME' && !(i + 1 < n && s.cmd[i + 1] === 'SERVO')) { pt.act = 'servo'; pt.hover = s.p[i]; pt.steps = []; delete pt.hold; i++; }
      while (i + 1 < n && s.cmd[i] === 'LOITER_TIME' && s.cmd[i + 1] === 'SERVO') {
        if (pt.act !== 'servo') { pt.act = 'servo'; pt.hover = 0; pt.steps = []; delete pt.hold; }
        const k = s.arg[i + 1];
        pt.steps.push({ servo: k, deg: servo && k >= 1 && k <= 4 ? usToDeg(servo, k - 1, s.p[i + 1]) : s.p[i + 1], wait: s.p[i] });
        i += 2;
      }
      plan.points.push(pt); continue;
    }
    i++;                                                       // (a command this tab does not make: skipped)
  }
  return plan;
}

// metres between two points (equirectangular: plenty for a few km)
export function distance(a, b) {
  const dn = (b.lat - a.lat) * M_PER_DEG, de = (b.lon - a.lon) * M_PER_DEG * Math.cos(a.lat * Math.PI / 180);
  return Math.hypot(dn, de);
}

// how far the plan flies from home and roughly how long it takes: the legs at `speed`, the climbs at 2 m/s, the landings at
// about 1 m/s, and every wait
export function estimate(plan, home, speed) {
  const v = speed > 0 ? speed : 5;
  let d = 0, t = (plan.takeoff_alt || 0) / 2, prev = home;
  for (const pt of plan.points || []) {
    d += distance(prev, pt); prev = pt;
    if (pt.act === 'fly') t += pt.hold || 0;
    else if (pt.act === 'servo') t += (pt.hover || 0) + (pt.steps || []).reduce((a, s) => a + (s.wait || 0), 0);
    else if (pt.act === 'land') t += (pt.alt || 0) / 1 + (pt.wait || 0) + (pt.alt || 0) / 2;
  }
  if (plan.rtl && home) { d += distance(prev, home); t += (prev.alt || plan.takeoff_alt || 0) / 1; }
  t += d / v;
  return { distance_m: Math.round(d), time_s: Math.round(t) };
}
