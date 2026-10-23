// Small read-outs drawn as instruments: a usage bar whose percentage rides at the end of its fill, a compass tape, and the GPS
// module's satellites (a sky plot and one signal bar per satellite). Each follows a telemetry stream.
import { h, listen } from '../ui.js';
import { store } from '../store.js';
import { tr } from '../i18n.js';

const SVGNS = 'http://www.w3.org/2000/svg';
function s(tag, attrs, ...kids) {
  const el = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs || {})) el.setAttribute(k, v);
  for (const kid of kids) el.append(kid.nodeType ? kid : document.createTextNode(kid));
  return el;
}
const follow = (stream, fn) => { listen('stream:' + stream, fn); if (store.streams[stream]) fn(store.streams[stream]); };

// ---- usage bar: value(v) -> { pct, caption } | null (nothing to show); the bar turns amber / red near full
export function usageBar(stream, value, id) {
  const fill = h('i'), label = h('b', { class: 'num' }, '–'), caption = h('span', { class: 'usage-cap' });
  const el = h('div', { class: 'usage', id }, h('div', { class: 'usage-track' }, fill, label), caption);
  follow(stream, (v) => {
    const r = value(v);
    el.classList.toggle('none', !r);
    if (!r) { label.textContent = '–'; fill.style.width = '0%'; caption.textContent = tr('không có'); return; }
    const p = Math.max(0, Math.min(100, r.pct));
    fill.style.width = p + '%'; label.textContent = `${Math.round(p)} %`;
    label.style.left = `clamp(0px, calc(${p}% - ${label.offsetWidth / 2}px), calc(100% - ${label.offsetWidth}px))`;
    el.dataset.level = p >= 90 ? 'bad' : p >= 75 ? 'warn' : 'ok';
    caption.textContent = r.caption || '';
  });
  return el;
}

// ---- compass tape: the heading under a fixed index; ticks every 5 degrees, the cardinal points and every 30 degrees named
export function compassTape(stream, heading) {
  const SPAN = 120, W = 360;                                      // degrees shown, viewBox width
  const strip = s('g', { class: 'tape-strip' });
  for (let d = -180; d <= 540; d += 5) {
    const x = (d * W) / SPAN, n = ((d % 360) + 360) % 360;
    const major = n % 30 === 0, card = n % 90 === 0;
    strip.append(s('line', { x1: x, x2: x, y1: card ? 6 : major ? 12 : 20, y2: 30, class: card ? 'card' : major ? 'major' : 'minor' }));
    if (major) strip.append(s('text', { x, y: 46, 'text-anchor': 'middle', class: card ? 'card' : '' }, card ? 'NESW'[n / 90] : String(n).padStart(3, '0')));
  }
  const value = h('span', { class: 'tape-value num' }, '–');
  const svg = s('svg', { viewBox: `${-W / 2} 0 ${W} 54`, class: 'tape', role: 'img', 'aria-label': tr('Thanh la bàn: hướng mũi máy bay') },
    strip, s('path', { d: 'M -7 0 L 7 0 L 0 10 Z', class: 'tape-index' }));
  follow(stream, (v) => {
    const hd = heading(v);
    if (hd === null || hd === undefined) return;
    const n = ((hd % 360) + 360) % 360;
    strip.setAttribute('transform', `translate(${(-n * W) / SPAN} 0)`);
    value.textContent = `${Math.round(n) % 360}°`;
  });
  return h('div', { class: 'compass-tape', id: 'compass-tape' }, svg, value);
}

// ---- satellites
export const GNSS = { 0: ['GPS', 'G'], 1: ['SBAS', 'S'], 2: ['Galileo', 'E'], 3: ['BeiDou', 'B'], 4: ['IMES', 'I'], 5: ['QZSS', 'Q'], 6: ['GLONASS', 'R'] };
const gnssOf = (g) => GNSS[g] || ['?', '?'];
const tip = (x) => ({ name: gnssOf(x.g)[0], sv: x.sv, cno: x.cno, el: x.el, az: x.az });
const strength = (cno) => (cno >= 35 ? 'ok' : cno >= 25 ? 'warn' : cno > 0 ? 'bad' : 'off');

export function satellitesView() {
  // sky plot: zenith in the middle, the horizon at the edge, north up
  const R = 100, sky = s('g');
  const plot = s('svg', { viewBox: '-118 -118 236 236', class: 'skyplot', role: 'img', 'aria-label': tr('Bầu trời: vị trí các vệ tinh') },
    s('circle', { r: R, class: 'ring' }), s('circle', { r: (R * 2) / 3, class: 'ring' }), s('circle', { r: R / 3, class: 'ring' }),
    s('line', { x1: -R, x2: R, y1: 0, y2: 0, class: 'ring' }), s('line', { y1: -R, y2: R, x1: 0, x2: 0, class: 'ring' }),
    ...['N', 'E', 'S', 'W'].map((t, i) => s('text', { x: [0, 110, 0, -110][i], y: [-106, 4, 114, 4][i], 'text-anchor': 'middle', class: 'dir' }, t)),
    sky);
  const bars = h('div', { class: 'sat-bars', id: 'sat-bars' });
  const summary = h('p', { class: 'sat-sum', id: 'sat-sum' }, tr('Đang chờ danh sách vệ tinh từ module…'));
  const legend = h('div', { class: 'legend wrap sat-legend' });
  const paint = (v) => {
    const n = v.count || 0, list = [];
    for (let i = 0; i < n; i++) list.push({ g: v.gnss[i], sv: v.sv[i], cno: v.cno[i], el: v.elev[i], az: v.azim[i], used: !!(v.flags[i] & 8), q: v.flags[i] & 7 });
    list.sort((a, b) => (a.g - b.g) || (a.sv - b.sv));
    sky.replaceChildren(...list.filter((x) => x.cno > 0 && x.el >= 0).map((x) => {
      const r = R * (1 - x.el / 90), a = (x.az * Math.PI) / 180, px = r * Math.sin(a), py = -r * Math.cos(a);
      const [, tag] = gnssOf(x.g);
      return s('g', { class: `sat g${x.g} ${x.used ? 'used' : ''}`, transform: `translate(${px.toFixed(1)} ${py.toFixed(1)})` },
        s('circle', { r: 4 + Math.min(6, x.cno / 8) }), s('text', { y: -10, 'text-anchor': 'middle' }, `${tag}${x.sv}`));
    }));
    bars.replaceChildren(...list.map((x) => {
      const [, tag] = gnssOf(x.g);
      return h('div', { class: `sat-bar ${strength(x.cno)} ${x.used ? 'used' : ''}`, title: x.used ? tr('{name} {sv}: {cno} dBHz, cao {el}°, hướng {az}°, dùng cho fix', tip(x)) : tr('{name} {sv}: {cno} dBHz, cao {el}°, hướng {az}°', tip(x)) },
        h('b', { class: 'num' }, x.cno ? String(x.cno) : ''), h('i', { style: { height: `${Math.min(100, (x.cno / 50) * 100)}%` } }), h('span', {}, `${tag}${x.sv}`));
    }));
    const used = list.filter((x) => x.used).length, seen = list.filter((x) => x.cno > 0).length;
    const avg = seen ? Math.round(list.filter((x) => x.cno > 0).reduce((t, x) => t + x.cno, 0) / seen) : 0;
    summary.textContent = n ? tr('{used} vệ tinh dùng cho fix, {seen} nhận được tín hiệu, {all} trong danh sách; tín hiệu trung bình {avg} dBHz', { used, seen, all: n, avg })
      : store.streams.status && store.streams.status.armed ? tr('Danh sách tắt khi đang arm (như Betaflight), có lại sau khi disarm.') : tr('Module chưa gửi danh sách vệ tinh (NMEA không có GSV, hoặc chưa cấu hình xong).');
    const kinds = [...new Set(list.map((x) => x.g))];
    legend.replaceChildren(...kinds.map((g) => h('span', { class: `legend-item g${g}` }, h('i'), gnssOf(g)[0])));
  };
  follow('satellites', paint);
  return h('div', { class: 'sats', id: 'satellites' }, summary,
    h('div', { class: 'sat-row' }, plot, h('div', { class: 'sat-right' }, bars, legend,
      h('p', { class: 'help' }, tr('Cột: cường độ tín hiệu (dBHz) - từ 35 là tốt, dưới 25 là yếu. Viền đậm: vệ tinh đang dùng cho fix. Trên bầu trời: giữa là đỉnh đầu, mép là chân trời, phía trên là hướng bắc.')))));
}
