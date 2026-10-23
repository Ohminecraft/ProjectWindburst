// A live recording seen through a window that moves with the wall clock (0.31): the chart scrolls smoothly at 60 frames a
// second, whatever the rhythm in which the records come in. Pure functions (no page): tests/test_gui_follow.py runs them in node.
//
// The data reaches the page in bursts: the firmware sends a frame every 20 ms, but the link (a UART bridge, the USB driver of
// the computer), the server and the chart's own fetches bunch them up - seen on a board as a chart that jumped about once a
// second. So the window does not end AT the newest record; it ends a little behind it (`lag`), runs on with the clock between
// two bursts, and eases back onto that point when it drifts. The lag adapts: about the time between two bursts that brought new
// data (so the window never runs into the empty space past the newest record), never less than 0.1 s, never more than 2 s.

export const LAG_MIN = 0.1, LAG_MAX = 2.0;

// After a fetch that moved the newest record on: the new lag, from the time since the previous such fetch (seconds).
export function nextLag(lag, burst_gap) {
  if (!(burst_gap > 0)) return lag;
  const want = Math.min(LAG_MAX, Math.max(LAG_MIN, burst_gap * 1.25));
  // grows at once (a gap longer than the lag would empty the right edge), shrinks slowly (one short gap is not a trend)
  return want > lag ? want : lag + (want - lag) * 0.2;
}

// One animation frame: where the window ends now. disp = where it ended last frame (null: not started), newest = the time of
// the newest record (s), lag (s), dt = seconds since the last frame.
export function followStep(disp, newest, lag, dt) {
  const target = newest - lag;
  if (disp === null || !Number.isFinite(disp) || Math.abs(target - disp) > 3 * Math.max(lag, 0.5)) return Math.min(target, newest);
  const step = Math.min(Math.max(dt, 0), 0.1);
  // runs with the clock, a little faster when behind the target and slower when ahead of it (at most +-40 %): a burst of data
  // changes the speed gently, it never makes the window jump
  const k = Math.max(-0.4, Math.min(0.4, 0.5 * (target - disp) / Math.max(lag, 0.2)));
  let d = disp + step * (1 + k);
  if (d > newest) d = newest;                                          // never past the data
  if (d < disp) d = disp;                                              // and never backwards: a window that jerks back is worse than one that waits
  return d;
}
