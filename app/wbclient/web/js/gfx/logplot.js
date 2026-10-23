// Flight-log charts: several panels stacked on one time axis. Each panel has one y axis (one unit) and a legend that reads the
// values under the pointer. The data comes reduced from the server (wbclient/logview.py): the samples themselves when they fit
// the width, otherwise the lowest and highest value of each pixel column, so a zoomed-out chart still shows every spike.
//
//   drag            zoom into the dragged span          double-click    the whole log
//   Ctrl + wheel    zoom around the pointer             Shift + drag    move along the time axis
//   click           put the playhead there              Space           play / pause (0.31, like Betaflight's Blackbox Explorer)
//   <- / ->         one record back / on (Shift: ten)   + / -           faster / slower       Home / End   start / end of the log
// The overview bar under the tools shows the whole flight: click or drag on it to jump there (gfx/playback.js).
import { h, track, token } from '../ui.js';
import { tr } from '../i18n.js';
import { niceStep } from './scope.js';
import { followStep, nextLag, LAG_MAX } from './follow.js';
import { SPEEDS, advance, windowAround, seekX, stepT, nextSpeed, covered, prefetch } from './playback.js';

export const SERIES_COLORS = ['--series-1', '--series-2', '--series-3', '--series-4', '--series-5', '--series-6', '--series-7', '--series-8'];
const PAD_L = 58, PAD_R = 10, PAD_T = 6, AXIS_H = 20, PANEL_H = 150;

// ---- pure helpers (also used by the tests through the page)
export function timeText(t, span) {
  const d = span < 0.05 ? 4 : span < 0.5 ? 3 : span < 20 ? 2 : 1;
  return t.toFixed(d) + ' s';
}
export function valueText(v, unit) {
  if (v === null || v === undefined || !Number.isFinite(v)) return '–';
  const a = Math.abs(v);
  const text = a >= 1000 ? v.toFixed(0) : a >= 100 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : v.toFixed(3);
  return unit ? `${text} ${unit}` : text;
}
// index of the sample of `t` (ascending) nearest to `at`
export function nearest(t, at) {
  let lo = 0, hi = t.length - 1;
  if (hi < 0) return -1;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (t[mid] <= at) lo = mid; else hi = mid; }
  return Math.abs(t[hi] - at) < Math.abs(t[lo] - at) ? hi : lo;
}
// what a series holds at time `at`: { t, lo, hi } (lo === hi for a single sample), or null where the log has a hole
export function sampleAt(s, at) {
  if (!s || !s.t.length) return null;
  const i = nearest(s.t, at);
  const reach = s.v ? Math.max(s.dt * 3, 1e-6) : Math.max(s.dt * 3, (s.t[Math.min(i + 1, s.t.length - 1)] - s.t[Math.max(i - 1, 0)]));
  if (Math.abs(s.t[i] - at) > reach) return null;
  return s.v ? { t: s.t[i], lo: s.v[i], hi: s.v[i] } : { t: s.t[i], lo: s.lo[i], hi: s.hi[i] };
}

// source(fields, t0, t1, px) -> Promise of the server's view ({ t0, t1, span, series })
export function logPlot({ source }) {
  let panels = [];             // [{ id, title, unit, series: [{ field, name }], range: [lo, hi] | null, parts: { cv, legend } }]
  let events = [];             // [{ t, text }]
  let span = [0, 1];           // first and last second of the log
  let view = null;             // [t0, t1] on screen; null = the whole log
  let data = {};               // field -> series of the last reply
  let dataRange = [0, 1];      // the time range `data` was fetched for
  let cursor = null;           // time under the pointer
  let brush = null;            // { x0, x1, pan } while dragging
  let followSec = 0;           // > 0: keep showing the newest `followSec` seconds (live recording)
  let ticket = 0, timer = 0, alive = true, loading = false, again = false;
  // following a live recording: the window's end moves with the clock (follow.js); disp = where it ends, lag = how far behind
  // the newest record, newestSeen / newestWall = the newest record so far and when it arrived (performance.now, s)
  let disp = null, lag = 0.3, newestSeen = -1, newestWall = 0, raf = 0, lastFrame = 0, lastPaint = 0;
  // playback: playT = the playhead (s, null = none), speed (x real time), the overview of the whole log (ovField: its series)
  let playT = null, playing = false, speed = 1, playRaf = 0, playLast = 0, onPlay = null, ovField = null, overview = null, ovTicket = 0;

  const rangeText = h('span', { class: 'num plot-range', id: 'plot-range' });
  const btn = (id, label, title, fn) => { const b = h('button', { type: 'button', class: 'btn quiet', id, title }, label); b.addEventListener('click', fn); return b; };
  const tools = h('div', { class: 'plot-tools' },
    btn('plot-all', tr('Toàn bộ'), tr('Xem cả log (hoặc bấm đúp vào đồ thị)'), () => api.setRange(null)),
    btn('plot-in', tr('Phóng to'), tr('Phóng to quanh giữa (hoặc kéo chuột trên đồ thị)'), () => zoom(0.5)),
    btn('plot-out', tr('Thu nhỏ'), '', () => zoom(2)),
    btn('plot-left', '◀', tr('Lùi về trước'), () => pan(-0.5)),
    btn('plot-right', '▶', tr('Tới về sau'), () => pan(0.5)),
    rangeText);
  const playBtn = btn('plot-play', '▶', tr('Phát / dừng (phím cách)'), () => setPlaying(!playing));
  const speedSel = h('select', { id: 'plot-speed', 'aria-label': tr('Tốc độ phát') }, SPEEDS.map((s) => h('option', { value: String(s) }, `${Math.round(s * 100)} %`)));
  speedSel.value = '1';
  speedSel.addEventListener('change', () => { speed = Number(speedSel.value) || 1; });
  const ovCv = h('canvas', { class: 'plot-overview', id: 'plot-overview', 'aria-label': tr('Toàn bộ chuyến bay: bấm hoặc kéo để nhảy tới') });
  const transport = h('div', { class: 'plot-transport', id: 'plot-transport' },
    btn('plot-start', '⏮', tr('Về đầu log (Home)'), () => seek(span[0])),
    btn('plot-back', '◀|', tr('Lùi một bản ghi (←, Shift: mười)'), () => step(-1, 1)),
    playBtn,
    btn('plot-fwd', '|▶', tr('Tới một bản ghi (→, Shift: mười)'), () => step(1, 1)),
    speedSel, ovCv);
  const stack = h('div', { class: 'plot-stack' });
  const empty = h('p', { class: 'note', hidden: true }, tr('Chọn ít nhất một nhóm đồ thị ở trên.'));
  const el = h('div', { class: 'logplot', id: 'logplot', tabindex: '0' }, tools, transport, empty, stack,
    h('p', { class: 'help' }, tr('Kéo chuột để phóng to một đoạn, bấm đúp để xem cả log. Ctrl + lăn chuột: phóng to quanh con trỏ. Shift + kéo: dịch theo thời gian.')),
    h('p', { class: 'help' }, tr('Phát lại như Blackbox Explorer: bấm vào đồ thị để đặt vạch phát, phím cách để phát / dừng, ← → từng bản ghi, + − đổi tốc độ. Thanh trên cùng là cả chuyến bay: bấm hoặc kéo để nhảy tới.')));

  const cur = () => view || span;
  const plotW = () => Math.max(50, (stack.clientWidth || 800) - PAD_L - PAD_R);
  const X = (t) => { const [a, b] = cur(); return PAD_L + ((t - a) / (b - a)) * plotW(); };
  const T = (x) => { const [a, b] = cur(); return a + ((x - PAD_L) / plotW()) * (b - a); };

  function clampView(a, b) {
    const whole = span[1] - span[0];
    let w = Math.min(Math.max(b - a, 0.002), Math.max(whole, 0.002));
    let lo = a;
    if (lo < span[0]) lo = span[0];
    if (lo + w > span[1]) lo = Math.max(span[0], span[1] - w);
    return (w >= whole - 1e-9) ? null : [lo, lo + w];
  }
  function zoom(k, around) {
    const [a, b] = cur();
    const c = around === undefined ? (a + b) / 2 : around;
    api.setRange(clampView(c - (c - a) * k, c + (b - c) * k), true);
  }
  function pan(frac) { const [a, b] = cur(); const d = (b - a) * frac; api.setRange(clampView(a + d, b + d), true); }

  // ---- data
  function fields() { return [...new Set(panels.flatMap((p) => p.series.map((s) => s.field)))]; }
  function request(delay = 70) { clearTimeout(timer); timer = setTimeout(load, delay); }
  async function load() {
    if (!alive) return;
    const names = fields();
    if (!names.length || !stack.clientWidth) { if (!names.length) data = {}; paint(); return; }       // nothing to show, or not on screen
    if (loading) { again = true; return; }
    loading = true;
    const mine = ++ticket;
    try {
      // in follow mode the server is asked for "the last N seconds": only it knows where the recording ends right now
      const want = followSec ? [null, null] : (playing && view) ? prefetch(view[0], view[1], span) : (view || [null, null]);
      const res = await source(names, want[0], want[1], Math.round(plotW()), followSec ? followSec + LAG_MAX : 0);
      if (!alive || mine !== ticket) return;
      span = res.span;
      if (followSec) {
        const wall = performance.now() / 1000;
        if (span[1] > newestSeen) { if (newestWall) lag = nextLag(lag, wall - newestWall); newestSeen = span[1]; newestWall = wall; }
        followView(); startFollow();
      }
      data = res.series; dataRange = [res.t0, res.t1];
      el.dataset.loaded = String(mine);
      paint();
    } catch (e) {
      if (alive && mine === ticket) { el.dataset.error = (e && e.code) || 'ERROR'; }
    } finally {
      loading = false;
      if (again) { again = false; request(0); }
    }
  }

  // ---- playback (Blackbox Explorer)
  function defaultWidth() { return Math.min((span[1] - span[0]) / 3, 4); }
  function centerOnPlay() { const w = view ? view[1] - view[0] : defaultWidth(); view = windowAround(playT, w, span); }
  function notifyPlay() { if (onPlay && playT !== null) onPlay(playT, playing); el.dataset.playhead = playT === null ? '' : playT.toFixed(4); }
  function syncButtons() { playBtn.textContent = playing ? '⏸' : '▶'; playBtn.setAttribute('aria-pressed', playing ? 'true' : 'false'); el.dataset.playing = playing ? '1' : '0'; }
  function playTick(ts) {
    playRaf = 0;
    if (!alive || !playing) return;
    const now = ts / 1000, dt = playLast ? Math.min(now - playLast, 0.1) : 0; playLast = now;
    const r = advance(playT, speed, dt, span); playT = r.t;
    centerOnPlay();
    if (r.ended) { playing = false; syncButtons(); }
    if (view && !loading && !covered(view[0], view[1], dataRange[0], dataRange[1], span)) request(0);
    paint(); notifyPlay();
    if (playing) playRaf = requestAnimationFrame(playTick);
  }
  function setPlaying(on) {
    if (followSec > 0 || !(span[1] > span[0])) on = false;
    if (on && !playing) {
      if (playT === null || playT >= span[1]) playT = view ? (view[0] + view[1]) / 2 : span[0];
      if (!view) view = windowAround(playT, defaultWidth(), span);
      playLast = 0; playing = true; request(0);
      if (!playRaf) playRaf = requestAnimationFrame(playTick);
    } else if (!on && playing) { playing = false; if (playRaf) cancelAnimationFrame(playRaf); playRaf = 0; }
    syncButtons(); paint(); notifyPlay();
  }
  function seek(t) {
    if (followSec > 0) { followSec = 0; stopFollow(); }
    playT = Math.min(span[1], Math.max(span[0], t));
    if (view || playing) centerOnPlay();
    paint(); request(60); notifyPlay();
  }
  function recordDt() { for (const k of Object.keys(data)) if (data[k] && data[k].dt > 0) return data[k].dt; return 0.004; }
  function step(dir, n) { if (playing) setPlaying(false); seek(stepT(playT === null ? (cursor !== null ? cursor : cur()[0]) : playT, dir, recordDt(), span, n)); }
  async function loadOverview() {
    if (!ovField || !alive) { overview = null; paintOverview(); return; }
    const mine = ++ovTicket;
    try {
      const res = await source([ovField], null, null, Math.max(100, Math.round(ovCv.clientWidth || 600)), 0);
      if (mine !== ovTicket || !alive) return;
      overview = res.series[ovField] || null; span = res.span;
    } catch (e) { overview = null; }
    paintOverview();
  }
  function paintOverview() {
    const w = ovCv.clientWidth, hh = ovCv.clientHeight;
    if (!w || !hh) return;
    const dpr = window.devicePixelRatio || 1;
    if (ovCv.width !== Math.round(w * dpr) || ovCv.height !== Math.round(hh * dpr)) { ovCv.width = Math.round(w * dpr); ovCv.height = Math.round(hh * dpr); }
    const ctx = ovCv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, hh);
    const whole = Math.max(span[1] - span[0], 1e-6), XO = (t) => ((t - span[0]) / whole) * w;
    ctx.fillStyle = token('--line'); ctx.globalAlpha = 0.35; ctx.fillRect(0, 0, w, hh); ctx.globalAlpha = 1;
    const d = overview;
    if (d && d.t.length) {
      const los = d.v || d.lo, his = d.v || d.hi; let lo = Infinity, hi = -Infinity;
      for (let i = 0; i < d.t.length; i++) { lo = Math.min(lo, los[i]); hi = Math.max(hi, his[i]); }
      if (!(hi > lo)) { hi = lo + 1; }
      const YO = (v) => hh - 3 - ((v - lo) / (hi - lo)) * (hh - 6);
      ctx.strokeStyle = token('--muted'); ctx.lineWidth = 1; ctx.beginPath();
      for (let i = 0; i < d.t.length; i++) { const x = XO(d.t[i]); ctx.moveTo(x, YO(los[i])); ctx.lineTo(x, YO(his[i]) - 0.5); }
      ctx.stroke();
    }
    for (const ev of events) { const x = Math.round(XO(ev.t)) + 0.5; ctx.strokeStyle = token('--muted'); ctx.setLineDash([2, 2]); ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, hh); ctx.stroke(); ctx.setLineDash([]); }
    const [a, b] = cur();
    ctx.fillStyle = token('--blue'); ctx.globalAlpha = 0.18; ctx.fillRect(XO(a), 0, Math.max(2, XO(b) - XO(a)), hh); ctx.globalAlpha = 1;
    if (playT !== null) { const x = Math.round(XO(playT)) + 0.5; ctx.strokeStyle = token('--bad'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, hh); ctx.stroke(); }
    ovCv.dataset.t0 = a.toFixed(3); ovCv.dataset.t1 = b.toFixed(3);
  }
  {
    let dragging = false;
    const at = (e) => seekX(e.clientX - ovCv.getBoundingClientRect().left, ovCv.clientWidth, span);
    ovCv.addEventListener('pointerdown', (e) => { if (e.button !== 0) return; dragging = true; ovCv.setPointerCapture(e.pointerId); const t = at(e); seek(t); if (!view) { view = windowAround(t, defaultWidth(), span); request(0); } });
    ovCv.addEventListener('pointermove', (e) => { if (dragging) seek(at(e)); });
    const done = () => { dragging = false; };
    ovCv.addEventListener('pointerup', done); ovCv.addEventListener('pointercancel', done);
  }
  el.addEventListener('keydown', (e) => {
    if (e.target !== el && e.target.tagName !== 'CANVAS') return;          // (the speed list keeps its own keys)
    if (e.key === ' ') { e.preventDefault(); setPlaying(!playing); }
    else if (e.key === 'ArrowRight' || e.key === 'ArrowLeft') { e.preventDefault(); step(e.key === 'ArrowRight' ? 1 : -1, e.shiftKey ? 10 : 1); }
    else if (e.key === '+' || e.key === '=' || e.key === '-') { speed = nextSpeed(speed, e.key === '-' ? -1 : 1); speedSel.value = String(speed); }
    else if (e.key === 'Home') { e.preventDefault(); seek(span[0]); }
    else if (e.key === 'End') { e.preventDefault(); seek(span[1]); }
  });

  // ---- following a live recording
  function followView() {
    const end = disp === null ? span[1] : disp;
    view = end - followSec <= span[0] ? (span[1] - span[0] <= followSec ? null : [span[0], span[0] + followSec]) : [end - followSec, end];
  }
  function followTick(ts) {
    raf = 0;
    if (!alive || followSec <= 0) return;
    const now = ts / 1000, dt = lastFrame ? now - lastFrame : 0; lastFrame = now;
    disp = followStep(disp, span[1], lag, dt);
    followView();
    if (now - lastPaint >= 0.03) { lastPaint = now; paint(); }               // about 30 pictures a second
    raf = requestAnimationFrame(followTick);
  }
  function startFollow() { if (!raf && followSec > 0 && alive) { lastFrame = 0; raf = requestAnimationFrame(followTick); } }
  function stopFollow() { if (raf) cancelAnimationFrame(raf); raf = 0; disp = null; newestSeen = -1; newestWall = 0; lag = 0.3; }

  // ---- drawing
  function yRange(p) {
    if (p.range) return p.range;
    let lo = Infinity, hi = -Infinity;
    const [a, b] = cur();
    for (const s of p.series) {
      const d = data[s.field];
      if (!d) continue;
      const los = d.v || d.lo, his = d.v || d.hi;
      for (let i = 0; i < d.t.length; i++) { if (d.t[i] < a || d.t[i] > b) continue; if (los[i] < lo) lo = los[i]; if (his[i] > hi) hi = his[i]; }
    }
    if (!Number.isFinite(lo)) return [-(p.min || 2) / 2, (p.min || 2) / 2];
    const need = p.min || Math.max(Math.abs(hi) * 0.02, 1e-6);      // never blow sensor noise up to the full height of the panel
    if (hi - lo < need) { const mid = (hi + lo) / 2; lo = mid - need / 2; hi = mid + need / 2; }
    const pad = (hi - lo) * 0.06;
    return [lo - pad, hi + pad];
  }
  function drawSeries(ctx, d, Y, color) {
    ctx.strokeStyle = color; ctx.lineJoin = 'round'; ctx.lineCap = 'round'; ctx.lineWidth = 2;
    ctx.beginPath();
    const gap = Math.max(d.dt * 3, 1e-6);
    if (d.v) {
      let pen = false, prev = 0;
      for (let i = 0; i < d.t.length; i++) {
        const x = X(d.t[i]), y = Y(d.v[i]);
        if (pen && d.t[i] - prev <= gap) ctx.lineTo(x, y); else ctx.moveTo(x, y);       // a hole in the log is a hole in the line
        pen = true; prev = d.t[i];
      }
      if (d.t.length === 1) ctx.arc(X(d.t[0]), Y(d.v[0]), 1, 0, 7);
    } else {
      const colT = d.t.length > 1 ? (d.t[d.t.length - 1] - d.t[0]) / (d.t.length - 1) : 0;
      let prev = -Infinity;
      for (let i = 0; i < d.t.length; i++) {
        const x = X(d.t[i]);
        if (d.t[i] - prev > Math.max(gap, colT * 3)) ctx.moveTo(x, Y(d.lo[i])); else ctx.lineTo(x, Y(d.lo[i]));
        ctx.lineTo(x, Y(d.hi[i]));
        prev = d.t[i];
      }
    }
    ctx.stroke();
  }
  function paintPanel(p, last) {
    const cv = p.parts.cv;
    const w = cv.clientWidth, hh = cv.clientHeight;
    if (!w || !hh) return;
    const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(hh * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(hh * dpr); }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, hh);
    const ph = hh - PAD_T - (last ? AXIS_H : 4), pw = plotW();
    let [lo, hi] = yRange(p);
    const step = niceStep(hi - lo, 3.5);
    if (!p.range) { lo = Math.floor(lo / step) * step; hi = Math.ceil(hi / step) * step; }
    const Y = (v) => PAD_T + (1 - (v - lo) / (hi - lo)) * ph;
    const line = token('--line'), muted = token('--muted');
    ctx.font = `11px ${token('--ui')}`;
    // y grid and labels
    ctx.textBaseline = 'middle'; ctx.textAlign = 'right'; ctx.lineWidth = 1;
    for (let k = Math.ceil(lo / step - 1e-9); k * step <= hi + step * 1e-6; k++) {
      const v = Number((k * step).toPrecision(12));
      const y = Math.round(Y(v)) + 0.5;
      ctx.strokeStyle = line; ctx.globalAlpha = Math.abs(v) < step * 1e-6 ? 1 : 0.55;
      ctx.beginPath(); ctx.moveTo(PAD_L, y); ctx.lineTo(PAD_L + pw, y); ctx.stroke();
      ctx.globalAlpha = 1; ctx.fillStyle = muted;
      ctx.fillText(String(Number(v.toPrecision(6))), PAD_L - 7, y);
    }
    // time grid (labels under the last panel only)
    const [a, b] = cur();
    const tStep = niceStep(b - a, Math.max(3, Math.floor(pw / 110)));
    ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic';
    for (let k = Math.ceil(a / tStep - 1e-9); k * tStep <= b + tStep * 1e-6; k++) {
      const t = k * tStep;
      const x = Math.round(X(t)) + 0.5;
      ctx.strokeStyle = line; ctx.globalAlpha = 0.4;
      ctx.beginPath(); ctx.moveTo(x, PAD_T); ctx.lineTo(x, PAD_T + ph); ctx.stroke();
      ctx.globalAlpha = 1;
      if (last) { ctx.fillStyle = muted; ctx.fillText(timeText(t, b - a), Math.min(Math.max(x, PAD_L + 16), PAD_L + pw - 18), hh - 5); }
    }
    ctx.save(); ctx.beginPath(); ctx.rect(PAD_L, PAD_T, pw, ph); ctx.clip();
    // events: a dashed line through every panel, the name in the first one
    ctx.setLineDash([3, 3]); ctx.strokeStyle = muted; ctx.lineWidth = 1;
    let free = -Infinity;
    for (const ev of events) {
      if (ev.t < a || ev.t > b) continue;
      const x = Math.round(X(ev.t)) + 0.5;
      ctx.beginPath(); ctx.moveTo(x, PAD_T); ctx.lineTo(x, PAD_T + ph); ctx.stroke();
      if (p === panels[0] && x > free) {
        ctx.setLineDash([]);
        const tw = ctx.measureText(ev.text).width;
        const left = Math.min(x + 4, PAD_L + pw - tw - 4);
        ctx.fillStyle = token('--panel'); ctx.globalAlpha = 0.85; ctx.fillRect(left - 2, PAD_T + 1, tw + 4, 14); ctx.globalAlpha = 1;
        ctx.fillStyle = token('--ink'); ctx.textAlign = 'left'; ctx.textBaseline = 'top'; ctx.fillText(ev.text, left, PAD_T + 2);
        ctx.setLineDash([3, 3]); free = left + tw + 10;
      }
    }
    ctx.setLineDash([]);
    p.series.forEach((s, i) => { const d = data[s.field]; if (d && d.t.length) drawSeries(ctx, d, Y, token(SERIES_COLORS[i % SERIES_COLORS.length])); });
    // the playhead: a solid line through every panel
    if (playT !== null && playT >= a && playT <= b) {
      const x = Math.round(X(playT)) + 0.5;
      ctx.strokeStyle = token('--bad'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, PAD_T); ctx.lineTo(x, PAD_T + ph); ctx.stroke();
    }
    // crosshair with a ringed dot on every series
    if (cursor !== null && cursor >= a && cursor <= b) {
      const x = Math.round(X(cursor)) + 0.5;
      ctx.strokeStyle = muted; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, PAD_T); ctx.lineTo(x, PAD_T + ph); ctx.stroke();
      p.series.forEach((s, i) => {
        const at = sampleAt(data[s.field], cursor);
        if (!at) return;
        ctx.fillStyle = token(SERIES_COLORS[i % SERIES_COLORS.length]); ctx.strokeStyle = token('--panel'); ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(X(at.t), Y((at.lo + at.hi) / 2), 4, 0, 7); ctx.fill(); ctx.stroke();
      });
    }
    if (brush && !brush.pan) {
      const x0 = Math.min(brush.x0, brush.x1), x1 = Math.max(brush.x0, brush.x1);
      ctx.fillStyle = token('--blue'); ctx.globalAlpha = 0.14; ctx.fillRect(x0, PAD_T, x1 - x0, ph); ctx.globalAlpha = 1;
    }
    ctx.restore();
  }
  function paintLegend(p) {
    const lt = (playing || cursor === null) && playT !== null ? playT : cursor;       // playing: the values under the playhead
    p.series.forEach((s, i) => {
      const at = lt === null ? null : sampleAt(data[s.field], lt);
      const cell = p.parts.values[i];
      // several samples share this pixel column when zoomed out: the legend then says from where to where they go
      const one = at && valueText(at.lo, '') === valueText(at.hi, '');
      const text = !data[s.field] ? '' : at === null ? (lt === null ? '' : '–')
        : (one ? valueText(at.lo, p.unit) : `${valueText(at.lo, '')} … ${valueText(at.hi, p.unit)}`);
      if (cell.textContent !== text) cell.textContent = text;
    });
  }
  function paint() {
    if (!alive) return;
    const [a, b] = cur();
    rangeText.textContent = `${timeText(a, b - a)} – ${timeText(b, b - a)}` + (cursor !== null ? '   ·   ' + tr('con trỏ {t}', { t: timeText(cursor, (b - a) / 20) }) : '');
    el.dataset.t0 = a.toFixed(4); el.dataset.t1 = b.toFixed(4); el.dataset.cursor = cursor === null ? '' : cursor.toFixed(4);
    el.dataset.whole = view ? '0' : '1';
    panels.forEach((p, i) => { paintPanel(p, i === panels.length - 1); paintLegend(p); });
    paintOverview();
    transport.hidden = followSec > 0 || panels.length === 0;
  }

  // ---- pointer
  function wire(cv) {
    const xOf = (e) => e.clientX - cv.getBoundingClientRect().left;
    cv.addEventListener('pointermove', (e) => {
      const x = Math.min(Math.max(xOf(e), PAD_L), PAD_L + plotW());
      if (brush) {
        if (brush.pan) {
          const dt = ((brush.x1 - x) / plotW()) * (brush.view[1] - brush.view[0]);
          view = clampView(brush.view[0] + dt, brush.view[1] + dt);
          request(120);
        } else brush.x1 = x;
      }
      cursor = T(x);
      paint();
    });
    cv.addEventListener('pointerleave', () => { if (!brush) { cursor = null; paint(); } });
    cv.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      el.focus({ preventScroll: true });
      const x = Math.min(Math.max(xOf(e), PAD_L), PAD_L + plotW());
      brush = { x0: x, x1: x, pan: e.shiftKey, view: [...cur()], playing };
      if (playing) setPlaying(false);
      cv.setPointerCapture(e.pointerId);
    });
    const up = () => {
      if (!brush) return;
      const b = brush; brush = null;
      if (b.pan) { followSec = 0; request(0); } else if (Math.abs(b.x1 - b.x0) > 6) api.setRange(clampView(T(Math.min(b.x0, b.x1)), T(Math.max(b.x0, b.x1))), true);
      else if (followSec <= 0) { playT = T(b.x0); notifyPlay(); if (b.playing) setPlaying(true); }        // a click: the playhead goes there
      paint();
    };
    cv.addEventListener('pointerup', up);
    cv.addEventListener('pointercancel', up);
    cv.addEventListener('dblclick', () => api.setRange(null, true));
    cv.addEventListener('wheel', (e) => {
      if (!e.ctrlKey) return;                       // a plain wheel scrolls the page
      e.preventDefault();
      zoom(e.deltaY > 0 ? 1.4 : 1 / 1.4, T(Math.min(Math.max(xOf(e), PAD_L), PAD_L + plotW())));
    }, { passive: false });
  }

  const ro = new ResizeObserver(() => { paint(); request(150); if (ovField) loadOverview(); });
  ro.observe(stack);
  track(() => { alive = false; clearTimeout(timer); stopFollow(); if (playRaf) cancelAnimationFrame(playRaf); ro.disconnect(); });

  const api = {
    el,
    // panels: [{ id, title, unit, series: [{ field, name }], range: fixed [lo, hi], min: smallest height of the y axis }]
    setPanels(next) {
      panels = next.map((p) => {
        const values = p.series.map(() => h('span', { class: 'num plot-val' }));
        const cv = h('canvas', { class: 'cv plot-cv', 'data-panel': p.id, role: 'img', 'aria-label': `${p.title}: ${p.series.map((s) => s.name).join(', ')}` });
        const legend = h('div', { class: 'plot-legend' }, p.series.map((s, i) => h('span', { class: 'legend-item', 'data-field': s.field },
          h('i', { style: { background: `var(${SERIES_COLORS[i % SERIES_COLORS.length]})` } }), s.name, values[i])));
        wire(cv);
        return { ...p, parts: { cv, legend, values } };
      });
      stack.replaceChildren(...panels.map((p, i) => h('div', { class: 'plot-panel', 'data-panel': p.id },
        h('div', { class: 'plot-head' }, h('strong', {}, p.title), p.unit ? h('span', { class: 'unit' }, p.unit) : null, p.parts.legend),
        Object.assign(p.parts.cv, { style: `height:${PANEL_H + (i === panels.length - 1 ? AXIS_H : 0)}px` }))));
      empty.hidden = panels.length > 0; tools.hidden = panels.length === 0;
      request(0);
    },
    setEvents(list) { events = list; paint(); },
    setSpan(next) { span = next; },
    // range [t0, t1] or null for the whole log; byUser = the pilot moved the chart, so it stops following a live recording
    setRange(range, byUser) {
      if (byUser) { followSec = 0; stopFollow(); }
      view = range ? clampView(range[0], range[1]) : null;
      paint(); request();
    },
    // show the surroundings of a moment: the zoom the pilot already chose, or a third of the log (at most 4 s) when all of it is on screen
    centerOn(t) { const w = view ? view[1] - view[0] : Math.min((span[1] - span[0]) / 3, 4); api.setRange([t - w / 2, t + w / 2], true); },
    follow(seconds) { stopFollow(); if (playing) setPlaying(false); followSec = seconds; request(0); },
    // playback
    play(on) { setPlaying(on === undefined ? !playing : !!on); },
    seek,
    playhead: () => playT,
    isPlaying: () => playing,
    setSpeed(s) { speed = s; speedSel.value = String(s); },
    onPlayhead(fn) { onPlay = fn; },
    // the field drawn on the overview bar (the whole flight); null: none
    setOverview(field) { ovField = field; overview = null; loadOverview(); },
    following: () => followSec > 0,
    refresh() { request(0); },
    clear() { stopFollow(); if (playing) setPlaying(false); playT = null; overview = null; ovTicket++; data = {}; view = null; cursor = null; span = [0, 1]; followSec = 0; ticket++; paint(); },
    paint,
  };
  return api;
}
