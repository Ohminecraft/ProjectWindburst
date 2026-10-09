// A scrolling line chart for live signals (gyro vibration, ...). One y axis, a legend, and a crosshair that reads the values under the pointer.
import { h, track, token } from '../ui.js';
import { tr } from '../i18n.js';

export function niceStep(range, target) {
  const raw = range / target, mag = Math.pow(10, Math.floor(Math.log10(raw))), n = raw / mag;
  return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
}

// series: [{ name, color: '--series-1' }]   windowMs: visible time span   unit: text after the numbers
export function scope({ series, windowMs = 2000, unit = '', height = 220, minRange = 10 }) {
  const data = [];                 // [t_ms, v0, v1, ...]
  let hoverX = null;
  const cv = h('canvas', { class: 'cv', style: { height: height + 'px' }, role: 'img', 'aria-label': tr('Đồ thị ') + series.map((s) => s.name).join(', ') });
  const legend = h('div', { class: 'legend' }, series.map((s) => h('span', { class: 'legend-item' }, h('i', { style: { background: `var(${s.color})` } }), s.name)));
  const read = h('div', { class: 'legend-read num' });
  const el = h('div', { class: 'scope' }, h('div', { class: 'legend-row' }, legend, read), cv);

  function draw() {
    const w = cv.clientWidth, hh = cv.clientHeight;
    if (!w || !hh) return;
    const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(hh * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(hh * dpr); }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, hh);
    const padL = 46, padR = 8, padT = 8, padB = 20, pw = w - padL - padR, ph = hh - padT - padB;
    const tEnd = data.length ? data[data.length - 1][0] : 0, t0 = tEnd - windowMs;
    let lo = Infinity, hi = -Infinity;
    for (const row of data) { if (row[0] < t0) continue; for (let i = 1; i < row.length; i++) { if (row[i] < lo) lo = row[i]; if (row[i] > hi) hi = row[i]; } }
    if (!Number.isFinite(lo)) { lo = -minRange / 2; hi = minRange / 2; }
    if (hi - lo < minRange) { const mid = (hi + lo) / 2; lo = mid - minRange / 2; hi = mid + minRange / 2; }
    const step = niceStep(hi - lo, 4);
    lo = Math.floor(lo / step) * step; hi = Math.ceil(hi / step) * step;
    const X = (t) => padL + ((t - t0) / windowMs) * pw, Y = (v) => padT + (1 - (v - lo) / (hi - lo)) * ph;
    ctx.font = `11px ${token('--ui')}`; ctx.textBaseline = 'middle';
    for (let v = lo; v <= hi + step * 0.001; v += step) {
      ctx.strokeStyle = token('--line'); ctx.globalAlpha = Math.abs(v) < step * 0.001 ? 1 : 0.55; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(padL, Math.round(Y(v)) + 0.5); ctx.lineTo(w - padR, Math.round(Y(v)) + 0.5); ctx.stroke();
      ctx.globalAlpha = 1; ctx.fillStyle = token('--muted'); ctx.textAlign = 'right';
      ctx.fillText(String(Number(v.toPrecision(6))), padL - 6, Y(v));
    }
    ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic'; ctx.fillStyle = token('--muted');
    for (let s = 0; s <= windowMs; s += windowMs / 4) ctx.fillText(s === 0 ? tr('bây giờ') : `-${(s / 1000).toFixed(1)} s`, X(tEnd - s) - (s === 0 ? 18 : 0), hh - 5);
    ctx.save(); ctx.beginPath(); ctx.rect(padL, padT, pw, ph); ctx.clip();
    series.forEach((s, i) => {
      ctx.strokeStyle = token(s.color); ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.beginPath();
      let pen = false;
      for (const row of data) { if (row[0] < t0) continue; const x = X(row[0]), y = Y(row[i + 1]); if (pen) ctx.lineTo(x, y); else { ctx.moveTo(x, y); pen = true; } }
      ctx.stroke();
    });
    ctx.restore();
    // crosshair: values of the sample nearest to the pointer
    if (hoverX !== null && data.length) {
      const t = t0 + ((hoverX - padL) / pw) * windowMs;
      let best = data[data.length - 1];
      for (const row of data) if (Math.abs(row[0] - t) < Math.abs(best[0] - t)) best = row;
      const x = X(best[0]);
      ctx.strokeStyle = token('--muted'); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + ph); ctx.stroke();
      series.forEach((s, i) => { ctx.fillStyle = token(s.color); ctx.strokeStyle = token('--panel'); ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, Y(best[i + 1]), 4, 0, 7); ctx.fill(); ctx.stroke(); });
      read.textContent = `${((best[0] - tEnd) / 1000).toFixed(2)} s: ` + series.map((s, i) => `${s.name} ${best[i + 1].toFixed(1)}${unit}`).join(', ');
    } else if (data.length) {
      const last = data[data.length - 1];
      read.textContent = series.map((s, i) => `${s.name} ${last[i + 1].toFixed(1)}${unit}`).join(', ');
    } else read.textContent = tr('chưa có dữ liệu');
  }
  cv.addEventListener('pointermove', (e) => { hoverX = e.offsetX; });
  cv.addEventListener('pointerleave', () => { hoverX = null; });
  let raf = 0, alive = true;
  const loop = () => { if (!alive) return; draw(); raf = requestAnimationFrame(loop); };
  raf = requestAnimationFrame(loop);
  track(() => { alive = false; cancelAnimationFrame(raf); });

  return {
    el,
    push(t, values) {
      if (data.length && t < data[data.length - 1][0] - 1000) data.length = 0;      // the firmware restarted: its clock started again
      data.push([t, ...values]);
      const cut = t - windowMs * 1.5;
      while (data.length && data[0][0] < cut) data.shift();
    },
    clear() { data.length = 0; },
    stats(index) {                    // rms and peak-to-peak of one series over the visible window
      if (!data.length) return null;
      const t0 = data[data.length - 1][0] - windowMs;
      let n = 0, sum = 0, sq = 0, lo = Infinity, hi = -Infinity;
      for (const row of data) { if (row[0] < t0) continue; const v = row[index + 1]; n++; sum += v; sq += v * v; lo = Math.min(lo, v); hi = Math.max(hi, v); }
      const mean = sum / n;
      return { rms: Math.sqrt(Math.max(0, sq / n - mean * mean)), p2p: hi - lo, n };
    },
    drawNow: draw,
  };
}
