// Rates: how far the craft turns for how much stick, and the throttle curve. Live sections: an edit acts at once.
import { store } from '../store.js';
import { h, block, fields, row, form, listen, control, canvas, token } from '../ui.js';
import { AXES } from '../labels.js';
import { actualRate, stick, throttleCurve, throttle01 } from '../gfx/fwmath.js';
import { tr } from '../i18n.js';
import { stickPreview } from '../gfx/stickpreview.js';

const SERIES = ['--series-1', '--series-2', '--series-3'];

function axisTable(section, cols) {
  return h('table', { class: 'grid axis' },
    h('thead', {}, h('tr', {}, h('th', {}), cols.map(([, label]) => h('th', {}, label)))),
    h('tbody', {}, AXES.map((ax, i) => h('tr', {}, h('th', { scope: 'row' }, h('i', { class: 'swatch', style: { background: `var(${SERIES[i]})` } }), ax),
      cols.map(([name]) => h('td', {}, control(section, name, { index: i })))))));
}

// Rate curve: stick deflection -> commanded rotation rate, one line per axis, with a dot where each stick is right now.
function rateChart() {
  let hover = null;
  const read = h('div', { class: 'legend-read num', id: 'rate-read' });
  const cv = canvas(260, (ctx, w, hh) => {
    const r = store.settings.rates, rx = store.settings.receiver, rc = store.streams.rc;
    const padL = 48, padR = 46, padT = 10, padB = 24, pw = w - padL - padR, ph = hh - padT - padB;
    const top = Math.max(...r.max_rate, ...r.center_sens, 100);
    const yMax = Math.ceil(top / 100) * 100;
    const X = (s) => padL + ((s + 1) / 2) * pw, Y = (v) => padT + (1 - (v + yMax) / (2 * yMax)) * ph;
    ctx.font = `11px ${token('--ui')}`; ctx.lineWidth = 1;
    for (const v of [-yMax, -yMax / 2, 0, yMax / 2, yMax]) {
      ctx.strokeStyle = token('--line'); ctx.globalAlpha = v === 0 ? 1 : 0.55;
      ctx.beginPath(); ctx.moveTo(padL, Math.round(Y(v)) + 0.5); ctx.lineTo(padL + pw, Math.round(Y(v)) + 0.5); ctx.stroke();
      ctx.globalAlpha = 1; ctx.fillStyle = token('--muted'); ctx.textAlign = 'right'; ctx.textBaseline = 'middle'; ctx.fillText(String(v), padL - 6, Y(v));
    }
    ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic';
    for (const s of [-1, -0.5, 0, 0.5, 1]) {
      ctx.strokeStyle = token('--line'); ctx.globalAlpha = s === 0 ? 1 : 0.4;
      ctx.beginPath(); ctx.moveTo(Math.round(X(s)) + 0.5, padT); ctx.lineTo(Math.round(X(s)) + 0.5, padT + ph); ctx.stroke();
      ctx.globalAlpha = 1; ctx.fillStyle = token('--muted'); ctx.fillText(`${s * 100} %`, X(s), hh - 6);
    }
    const sticks = rc ? [rc.ch[rx.ch_roll], rc.ch[rx.ch_pitch], rc.ch[rx.ch_yaw]].map((us, i) => stick(us, i === 2 ? rx.yaw_deadband : rx.deadband, rx.mid_rc || 1500)) : [null, null, null];
    const ends = [];
    AXES.forEach((ax, i) => {
      const f = (s) => actualRate(r.center_sens[i], r.max_rate[i], r.expo[i], s);
      ctx.strokeStyle = token(SERIES[i]); ctx.lineWidth = 2; ctx.beginPath();
      for (let k = 0; k <= 200; k++) { const s = -1 + k / 100; if (k) ctx.lineTo(X(s), Y(f(s))); else ctx.moveTo(X(s), Y(f(s))); }
      ctx.stroke();
      ends.push({ y: Y(f(1)), ax, i });
      if (sticks[i] !== null) {
        ctx.fillStyle = token(SERIES[i]); ctx.strokeStyle = token('--panel'); ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(X(sticks[i]), Y(f(sticks[i])), 5, 0, 7); ctx.fill(); ctx.stroke();
      }
    });
    // names at the right end of each line, nudged apart when two lines end together
    ends.sort((a, b) => a.y - b.y);
    for (let k = 1; k < ends.length; k++) if (ends[k].y - ends[k - 1].y < 13) ends[k].y = ends[k - 1].y + 13;
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle'; ctx.fillStyle = token('--ink');
    for (const e of ends) ctx.fillText(e.ax, padL + pw + 6, e.y);
    if (hover !== null) {
      const s = Math.max(-1, Math.min(1, ((hover - padL) / pw) * 2 - 1));
      ctx.strokeStyle = token('--muted'); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(X(s), padT); ctx.lineTo(X(s), padT + ph); ctx.stroke();
      read.textContent = tr('Cần ở {n} %: ', { n: (s * 100).toFixed(0) }) + AXES.map((ax, i) => `${ax} ${actualRate(r.center_sens[i], r.max_rate[i], r.expo[i], s).toFixed(0)} °/s`).join(', ');
    } else if (rc) {
      read.textContent = tr('Cần lúc này: ') + AXES.map((ax, i) => `${ax} ${actualRate(r.center_sens[i], r.max_rate[i], r.expo[i], sticks[i]).toFixed(0)} °/s`).join(', ');
    } else read.textContent = '';
  });
  cv.id = 'rate-chart';
  cv.setAttribute('role', 'img'); cv.setAttribute('aria-label', tr('Đường cong rate của ba trục'));
  cv.addEventListener('pointermove', (e) => { hover = e.offsetX; cv.paint(); });
  cv.addEventListener('pointerleave', () => { hover = null; cv.paint(); });
  listen('settings:rates', () => cv.paint()); listen('settings:receiver', () => cv.paint()); listen('stream:rc', () => cv.paint());
  return h('div', { class: 'scope' }, h('div', { class: 'legend-row' },
    h('div', { class: 'legend' }, AXES.map((ax, i) => h('span', { class: 'legend-item' }, h('i', { style: { background: `var(${SERIES[i]})` } }), ax))), read), cv,
    h('p', { class: 'help' }, tr('Trục ngang: độ lệch cần (sau deadband). Trục dọc: tốc độ quay yêu cầu, °/s. Chấm tròn là vị trí cần lúc này.')));
}

function throttleChart() {
  const read = h('div', { class: 'legend-read num', id: 'thr-read' });
  const cv = canvas(200, (ctx, w, hh) => {
    const r = store.settings.rates, rx = store.settings.receiver, rc = store.streams.rc;
    const padL = 48, padR = 24, padT = 10, padB = 24, pw = w - padL - padR, ph = hh - padT - padB;
    const X = (t) => padL + t * pw, Y = (v) => padT + (1 - v) * ph;
    ctx.font = `11px ${token('--ui')}`; ctx.lineWidth = 1;
    for (const v of [0, 0.25, 0.5, 0.75, 1]) {
      ctx.strokeStyle = token('--line'); ctx.globalAlpha = 0.55;
      ctx.beginPath(); ctx.moveTo(padL, Math.round(Y(v)) + 0.5); ctx.lineTo(padL + pw, Math.round(Y(v)) + 0.5); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(Math.round(X(v)) + 0.5, padT); ctx.lineTo(Math.round(X(v)) + 0.5, padT + ph); ctx.stroke();
      ctx.globalAlpha = 1; ctx.fillStyle = token('--muted');
      ctx.textAlign = 'right'; ctx.textBaseline = 'middle'; ctx.fillText(`${v * 100} %`, padL - 6, Y(v));
      ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic'; ctx.fillText(`${v * 100} %`, X(v), hh - 6);
    }
    const f = (t) => throttleCurve(r.throttle_mid, r.throttle_expo, r.throttle_limit, t);
    ctx.strokeStyle = token('--series-1'); ctx.lineWidth = 2; ctx.beginPath();
    for (let k = 0; k <= 100; k++) { const t = k / 100; if (k) ctx.lineTo(X(t), Y(f(t))); else ctx.moveTo(X(t), Y(f(t))); }
    ctx.stroke();
    if (rc) {
      const t = throttle01(rc.ch[rx.ch_throttle]);
      ctx.fillStyle = token('--series-1'); ctx.strokeStyle = token('--panel'); ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(X(t), Y(f(t)), 5, 0, 7); ctx.fill(); ctx.stroke();
      read.textContent = tr('Cần ga {stick} % cho lệnh ga {cmd} %', { stick: (t * 100).toFixed(0), cmd: (f(t) * 100).toFixed(0) });
    } else read.textContent = '';
  });
  cv.id = 'thr-chart';
  cv.setAttribute('role', 'img'); cv.setAttribute('aria-label', tr('Đường cong ga'));
  listen('settings:rates', () => cv.paint()); listen('settings:receiver', () => cv.paint()); listen('stream:rc', () => cv.paint());
  return h('div', { class: 'scope' }, h('div', { class: 'legend-row' }, h('div', { class: 'legend' }, h('span', { class: 'legend-item' }, h('i', { style: { background: 'var(--series-1)' } }), tr('Lệnh ga theo cần ga'))), read), cv);
}

export default {
  id: 'rates',
  title: 'Rate',
  lead: tr('Rate và đường cong ga có tác dụng ngay khi sửa, nhưng chỉ được giữ lại khi bấm "Lưu".'),
  streams: { rc: 30, status: 10 },
  mount(view) {
    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Rate (kiểu Actual)'),
          axisTable('rates', [['center_sens', tr('Độ nhạy giữa (°/s)')], ['max_rate', tr('Tốc độ tối đa (°/s)')], ['expo', 'Expo (%)']]),
          rateChart()),
        block(tr('Máy bay theo cần'), stickPreview('ACRO'))),
      h('div', {},
        block(tr('Ga'), fields('rates', ['throttle_mid', 'throttle_expo', 'throttle_limit']), throttleChart(),
          h('p', { class: 'help' }, tr('Đường cong ga chỉ dùng khi bay ga tay. Altitude Hold dùng thẳng vị trí cần ga, theo mục bên dưới.'))),
        block(tr('Cần ga trong Altitude Hold'), fields('althold', ['mid_stick', 'deadband', 'low_stick', 'max_stick', 'climb_rate', 'descend_rate', 'accel_z', 'jerk_z']),
          h('p', { class: 'help' }, tr('Hệ số vòng gia tốc (P, I) ở tab PID.')),
          h('p', { class: 'help' }, tr('Cần ở "điểm giữ" ± vùng chết: giữ độ cao. Đẩy lên tới "cần tối đa": leo nhanh nhất; kéo xuống tới "cần thấp": hạ nhanh nhất, và trên mặt đất thì đứng yên. Ga treo (mức ga máy treo được) firmware tự đo, không cần nhập. Đường cong ga ở trên không áp dụng cho mode này.'))),
        block(tr('Angle và Stabilize'), form(row('rates', 'angle_limit')),
          h('p', { class: 'help' }, tr('Angle (Betaflight): góc nghiêng theo cần đi theo đường cong rate ở trên, nên nửa cần cho ít hơn nửa góc tối đa. Stabilize (ArduPilot): tỉ lệ thẳng với cần. Các hệ số của hai mode ở tab PID.'))))));
  },
};
