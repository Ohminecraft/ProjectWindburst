// Playing a flight log back like Betaflight's Blackbox Explorer (0.31): a playhead that runs through the log at a chosen speed,
// the chart window centred on it, an overview of the whole flight to jump around in. Pure functions (no page):
// tests/test_gui_playback.py runs them in node.

export const SPEEDS = [0.05, 0.1, 0.25, 0.5, 1, 2, 4];           // x real time (Blackbox Explorer: 5 % .. 400 %)

// The playhead after dt seconds of wall time at `speed`: { t, ended } - it stops at the end of the log.
export function advance(t, speed, dt, span) {
  const next = t + Math.max(0, dt) * speed;
  return next >= span[1] ? { t: span[1], ended: true } : { t: Math.max(span[0], next), ended: false };
}

// The chart window of `width` seconds with the playhead in its middle, kept inside the log; null = the whole log fits.
export function windowAround(t, width, span) {
  const whole = span[1] - span[0];
  if (!(width > 0) || width >= whole) return null;
  let a = t - width / 2;
  if (a < span[0]) a = span[0];
  if (a + width > span[1]) a = span[1] - width;
  return [a, a + width];
}

// A position on the overview bar (x pixels from its left edge, w pixels wide) as a time of the log.
export function seekX(x, w, span) {
  const f = w > 0 ? Math.min(1, Math.max(0, x / w)) : 0;
  return span[0] + f * (span[1] - span[0]);
}

// One step of the arrow keys: one record of the log (dt) forwards / backwards, `n` of them with Shift (Blackbox Explorer: a frame).
export function stepT(t, dir, dt, span, n = 1) {
  const s = Math.max(dt, 1e-4) * n * (dir < 0 ? -1 : 1);
  return Math.min(span[1], Math.max(span[0], t + s));
}

// The next / previous speed in SPEEDS (the + / - keys).
export function nextSpeed(speed, dir) {
  let i = SPEEDS.indexOf(speed);
  if (i < 0) i = SPEEDS.indexOf(1);
  return SPEEDS[Math.min(SPEEDS.length - 1, Math.max(0, i + (dir > 0 ? 1 : -1)))];
}

// Is the window [a, b] inside what was fetched [fa, fb] with a margin of a quarter of its width on both sides (as far as the log
// reaches, span)? Else it is fetched again - ahead of the playhead, not when the line already ends on screen.
export function covered(a, b, fa, fb, span) {
  const m = (b - a) * 0.25;
  return fa <= Math.max(a - m, span[0]) + 1e-9 && fb >= Math.min(b + m, span[1]) - 1e-9;
}
// What to fetch for the window [a, b] while playing: one more width ahead, half a width behind (inside the log).
export function prefetch(a, b, span) {
  const w = b - a;
  return [Math.max(span[0], a - w / 2), Math.min(span[1], b + w)];
}
