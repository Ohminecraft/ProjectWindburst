// Filters: what is taken out of the gyro signal before the PID sees it, and out of the D-term. A "live" section: an edit acts on
// the running firmware at once (disarmed) - the picture at the top shows where every filter sits right now.
import { store } from '../store.js';
import { h, block, fields, form, row, listen, note, control, sliderRow, readout, kv, canvas, token } from '../ui.js';
import { AXES } from '../labels.js';
import { tr } from '../i18n.js';

const mult = (v) => (v / 100).toFixed(2);
const AXIS_COLOR = ['--series-1', '--series-2', '--series-3'];

// Frequency map 0 .. 1000 Hz, two lanes (gyro, D-term): low-pass = everything to the right of its cutoff is shaded, a dynamic
// low-pass shows the range it moves in and where it is now; notches are vertical marks.
function filterMap() {
  const read = h('div', { class: 'legend-read num', id: 'filter-read' });
  const cv = canvas(190, (ctx, w, hh) => {
    const f = store.settings.filters, t = store.streams.tuning;
    if (!f) return;
    const padL = 64, padR = 14, padT = 8, padB = 22, pw = w - padL - padR, lane = (hh - padT - padB) / 2, FMAX = 1000;
    const X = (hz) => padL + Math.min(hz, FMAX) / FMAX * pw;
    ctx.font = `11px ${token('--ui')}`;
    for (let hz = 0; hz <= FMAX; hz += 100) {
      ctx.strokeStyle = token('--line'); ctx.globalAlpha = hz % 500 === 0 ? 1 : 0.5; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(Math.round(X(hz)) + 0.5, padT); ctx.lineTo(Math.round(X(hz)) + 0.5, padT + 2 * lane); ctx.stroke();
      ctx.globalAlpha = 1; ctx.fillStyle = token('--muted'); ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic';
      if (hz % 200 === 0) ctx.fillText(hz === FMAX ? `${hz} Hz` : String(hz), hz === FMAX ? X(hz) - 12 : X(hz), hh - 6);
    }
    const lanes = [[tr('Gyro'), padT], [tr('D-term'), padT + lane]];
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle'; ctx.fillStyle = token('--ink');
    for (const [name, y] of lanes) ctx.fillText(name, padL - 8, y + lane / 2);
    ctx.strokeStyle = token('--line'); ctx.beginPath(); ctx.moveTo(padL, padT + lane + 0.5); ctx.lineTo(padL + pw, padT + lane + 0.5); ctx.stroke();
    // a low-pass: light shade from its lowest possible cutoff, full shade from where it is now
    const lowpass = (y, now, lo, hi) => {
      if (!now) return;
      ctx.fillStyle = token('--muted');
      if (lo && hi && hi > lo) { ctx.globalAlpha = 0.10; ctx.fillRect(X(lo), y + 3, X(hi) - X(lo), lane - 6); }
      ctx.globalAlpha = 0.22; ctx.fillRect(X(now), y + 3, padL + pw - X(now), lane - 6);
      ctx.globalAlpha = 1; ctx.strokeStyle = token('--ink'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(X(now), y + 3); ctx.lineTo(X(now), y + lane - 3); ctx.stroke();
    };
    const mark = (y, hz, color, part = 1, dash = false) => {
      if (!hz) return;
      ctx.strokeStyle = token(color); ctx.lineWidth = 2; ctx.setLineDash(dash ? [3, 3] : []);
      ctx.beginPath(); ctx.moveTo(X(hz), y + lane - 3 - (lane - 6) * part); ctx.lineTo(X(hz), y + lane - 3); ctx.stroke(); ctx.setLineDash([]);
    };
    const gy = padT, dy = padT + lane;
    const g1dyn = f.gyro_lpf1_dyn_min_hz > 0, d1dyn = f.dterm_lpf1_dyn_min_hz > 0;
    const g1 = t ? t.gyro_lpf1_hz : (g1dyn ? f.gyro_lpf1_dyn_min_hz : f.gyro_lpf1_static_hz);
    const d1 = t ? t.dterm_lpf1_hz : (d1dyn ? f.dterm_lpf1_dyn_min_hz : f.dterm_lpf1_static_hz);
    lowpass(gy, g1, g1dyn ? f.gyro_lpf1_dyn_min_hz : 0, g1dyn ? f.gyro_lpf1_dyn_max_hz : 0);
    lowpass(gy, f.gyro_lpf2_hz, 0, 0);
    lowpass(dy, d1, d1dyn ? f.dterm_lpf1_dyn_min_hz : 0, d1dyn ? f.dterm_lpf1_dyn_max_hz : 0);
    lowpass(dy, f.dterm_lpf2_hz, 0, 0);
    if (f.gyro_notch1_hz && f.gyro_notch1_cutoff && f.gyro_notch1_cutoff < f.gyro_notch1_hz) mark(gy, f.gyro_notch1_hz, '--series-4');
    if (f.gyro_notch2_hz && f.gyro_notch2_cutoff && f.gyro_notch2_cutoff < f.gyro_notch2_hz) mark(gy, f.gyro_notch2_hz, '--series-4');
    if (f.dterm_notch_hz && f.dterm_notch_cutoff && f.dterm_notch_cutoff < f.dterm_notch_hz) mark(dy, f.dterm_notch_hz, '--series-4');
    if (f.dyn_notch_count > 0) {                               // the range the dynamic notches may move in, and where they are
      ctx.strokeStyle = token('--muted'); ctx.lineWidth = 1; ctx.setLineDash([2, 3]);
      ctx.strokeRect(X(f.dyn_notch_min_hz) + 0.5, gy + 2.5, X(f.dyn_notch_max_hz) - X(f.dyn_notch_min_hz), lane * 0.42); ctx.setLineDash([]);
      if (t) for (let a = 0; a < 3; a++) for (let p = 0; p < Math.min(3, f.dyn_notch_count); p++) mark(gy - lane * 0.55, t.dyn_notch_hz[a * 3 + p], AXIS_COLOR[a], 0.4);
    }
    if (t) for (const hz of t.rpm_hz) for (let k = 1; k <= f.rpm_harmonics; k++) mark(gy, hz * k, '--series-5', 0.45, true);
    const parts = [];
    if (g1) parts.push(tr('Gyro lowpass 1: {n} Hz', { n: Math.round(g1) }));
    if (d1) parts.push(tr('D-term lowpass 1: {n} Hz', { n: Math.round(d1) }));
    if (t && t.rpm_hz.some((v) => v > 0)) parts.push(tr('RPM notch: {list} Hz', { list: t.rpm_hz.filter((v) => v > 0).join(', ') }));
    read.textContent = parts.join('   ');
  });
  cv.id = 'filter-map'; cv.setAttribute('role', 'img'); cv.setAttribute('aria-label', tr('Vị trí của các bộ lọc trên trục tần số'));
  listen('settings:filters', () => cv.paint()); listen('stream:tuning', () => cv.paint());
  const item = (color, text, cls = '') => h('span', { class: 'legend-item ' + cls }, h('i', { style: { background: color.startsWith('--') ? `var(${color})` : color } }), text);
  return h('div', { class: 'scope' },
    h('div', { class: 'legend-row' }, h('div', { class: 'legend wrap' },
      item('--ink', tr('cutoff của lowpass (vùng xám bên phải bị lọc)')), item('--series-4', tr('notch tĩnh')),
      AXES.map((ax, i) => item(AXIS_COLOR[i], tr('dynamic notch {axis}', { axis: ax }))), item('--series-5', tr('RPM notch'))), read),
    cv,
    h('p', { class: 'help' }, tr('Dynamic lowpass trượt trong vùng xám nhạt theo ga (thấp khi ga thấp). Khung nét đứt: vùng dynamic notch được phép chạy. Vị trí "lúc này" chỉ có khi đang kết nối.')));
}

// a field whose value the slider owns is locked while that slider is on
function owned(sliderFlag, names) {
  return names.map((n) => {
    const r = row('filters', n);
    const input = r[1].querySelector('[data-field]');
    const lock = () => { const on = !!(store.settings.filters || {})[sliderFlag]; input.disabled = store.armed || on; input.title = on ? tr('Do thanh trượt tính') : ''; };
    listen('settings:filters', lock); listen('armed', lock); lock();
    return r;
  });
}

export default {
  id: 'filters',
  title: tr('Bộ lọc'),
  lead: tr('Bộ lọc có tác dụng ngay khi sửa (lúc disarm), và chỉ được giữ lại khi bấm "Lưu". Lọc nhiều hơn: motor mát hơn, máy trễ hơn. Lọc ít hơn: máy nhạy hơn, motor dễ nóng. Sau mỗi lần giảm lọc hãy bay thử 30 giây rồi sờ motor.'),
  streams: { status: 10, tuning: 10 },
  mount(view) {
    const rpmNote = h('p', { class: 'note', id: 'rpm-note' });
    const paintRpm = () => {
      const m = store.settings.motors || {};
      const on = !!m.bidirectional && String(m.protocol || '').startsWith('DSHOT');
      rpmNote.className = 'note ' + (on ? 'ok' : '');
      rpmNote.textContent = on ? tr('DShot hai chiều đang bật: RPM filter có số liệu để chạy.') : tr('RPM filter cần DShot hai chiều (tab Motor). Hiện đang tắt, nên các notch này không có tác dụng; dynamic notch và lowpass gánh toàn bộ.');
    };
    listen('settings:motors', paintRpm); paintRpm();
    const weights = h('div', { class: 'ctl-line' }, [0, 1, 2].map((i) => [h('span', { class: 'unit' }, `${i + 1}×`), control('filters', 'rpm_weights', { index: i, width: '70px' })]));
    view.append(
      block(tr('Bộ lọc đang đặt ở đâu'), filterMap()),
      h('div', { class: 'cols' },
        h('div', {},
          block(tr('Thanh trượt'),
            form(row('filters', 'slider_gyro'), sliderRow('filters', 'slider_gyro_mult', { fmt: mult, step: 5, mark: 100, enabled: () => !!(store.settings.filters || {}).slider_gyro }),
              row('filters', 'slider_dterm'), sliderRow('filters', 'slider_dterm_mult', { fmt: mult, step: 5, mark: 100, enabled: () => !!(store.settings.filters || {}).slider_dterm })),
            note(tr('Kéo sang trái: lọc nhiều hơn (cutoff thấp hơn). Sang phải: lọc ít hơn. Thanh trượt đang bật thì các ô cutoff của lowpass bị khoá và hiện kết quả; tắt thanh trượt để nhập tay. Notch, dynamic notch và RPM filter không thuộc thanh trượt.'))),
          block(tr('Gyro lowpass 1'), form(row('filters', 'gyro_lpf1_type'), owned('slider_gyro', ['gyro_lpf1_dyn_min_hz', 'gyro_lpf1_dyn_max_hz']), row('filters', 'gyro_lpf1_dyn_expo'), owned('slider_gyro', ['gyro_lpf1_static_hz']))),
          block(tr('Gyro lowpass 2'), form(row('filters', 'gyro_lpf2_type'), owned('slider_gyro', ['gyro_lpf2_hz']))),
          block(tr('Gyro notch tĩnh'), fields('filters', ['gyro_notch1_hz', 'gyro_notch1_cutoff', 'gyro_notch2_hz', 'gyro_notch2_cutoff']),
            h('p', { class: 'help' }, tr('Cho một cộng hưởng không đổi theo ga (khung, giá camera). Tìm tần số của nó bằng đồ thị phổ ở tab Blackbox.')))),
        h('div', {},
          block(tr('RPM filter'), rpmNote, fields('filters', ['rpm_harmonics', 'rpm_min_hz', 'rpm_q', 'rpm_fade_range_hz', 'rpm_lpf_hz']),
            form([h('label', {}, tr('Trọng số hài')), h('div', { class: 'ctl' }, weights, h('p', { class: 'help' }, tr('Phần trăm tác dụng của notch ở hài 1, 2, 3. Cánh 2 lá thường chỉ cần hài 1 và 2; đặt hài 3 về 0 để bớt trễ.')))])),
          block(tr('Dynamic notch'), fields('filters', ['dyn_notch_count', 'dyn_notch_q', 'dyn_notch_min_hz', 'dyn_notch_max_hz']),
            h('p', { class: 'help' }, tr('Tự tìm các đỉnh nhiễu còn lại trong gyro (cộng hưởng khung, nhiễu mà RPM filter không thấy) và đặt notch lên đó. Cần vòng lặp từ 2000 Hz.'))),
          block(tr('D-term lowpass 1'), form(row('filters', 'dterm_lpf1_type'), owned('slider_dterm', ['dterm_lpf1_dyn_min_hz', 'dterm_lpf1_dyn_max_hz']), row('filters', 'dterm_lpf1_dyn_expo'), owned('slider_dterm', ['dterm_lpf1_static_hz']))),
          block(tr('D-term lowpass 2'), form(row('filters', 'dterm_lpf2_type'), owned('slider_dterm', ['dterm_lpf2_hz']))),
          block(tr('D-term notch và yaw lowpass'), fields('filters', ['dterm_notch_hz', 'dterm_notch_cutoff', 'yaw_lowpass_hz'])),
          block(tr('Lúc này'), kv([
            [tr('Gyro lowpass 1'), readout('tuning', (t) => (t.gyro_lpf1_hz ? `${t.gyro_lpf1_hz} Hz` : tr('tắt')))],
            [tr('D-term lowpass 1'), readout('tuning', (t) => (t.dterm_lpf1_hz ? `${t.dterm_lpf1_hz} Hz` : tr('tắt')))],
            [tr('Dynamic notch roll'), readout('tuning', (t) => t.dyn_notch_hz.slice(0, 3).join('  ') + ' Hz')],
            [tr('Dynamic notch pitch'), readout('tuning', (t) => t.dyn_notch_hz.slice(3, 6).join('  ') + ' Hz')],
            [tr('Dynamic notch yaw'), readout('tuning', (t) => t.dyn_notch_hz.slice(6, 9).join('  ') + ' Hz')],
            [tr('RPM notch (hài 1) từng motor'), readout('tuning', (t) => (t.rpm_hz.some((v) => v > 0) ? t.rpm_hz.filter((v) => v > 0).join('  ') + ' Hz' : tr('không có')))],
          ])))));
  },
};
