// PID: the tuning sliders, the gains per axis, the PID controller options, Angle and Stabilize. A "live" section: an edit acts
// on the running firmware at once (disarmed), and is kept only after "Save".
import { store, setFields } from '../store.js';
import { h, block, fields, form, row, listen, note, control, sliderRow, readout, kv } from '../ui.js';
import { AXES } from '../labels.js';
import { tr } from '../i18n.js';

const SERIES = ['--series-1', '--series-2', '--series-3'];
const mult = (v) => (v / 100).toFixed(2);

// which axes the firmware computes from the sliders (their boxes are locked)
function sliderAxes() { const m = (store.settings.pid || {}).slider_mode; return m === 'RPY' ? 3 : (m === 'RP' ? 2 : 0); }

function gainTable() {
  const cols = [['p', 'P'], ['i', 'I'], ['d', 'D'], ['d_max', 'D Max'], ['f', 'Feedforward']];
  const cells = [];
  const table = h('table', { class: 'grid axis', id: 'pid-gains' },
    h('thead', {}, h('tr', {}, h('th', {}), cols.map(([, label]) => h('th', {}, label)))),
    h('tbody', {}, AXES.map((ax, i) => h('tr', {}, h('th', { scope: 'row' }, h('i', { class: 'swatch', style: { background: `var(${SERIES[i]})` } }), ax),
      cols.map(([name]) => { const el = control('pid', name, { index: i }); cells.push([el, i]); return h('td', {}, el); })))));
  // (registered after the controls' own listeners, so this lock has the last word)
  const lock = () => { const n = sliderAxes(); for (const [el, axis] of cells) { el.disabled = store.armed || axis < n; el.title = axis < n ? tr('Do thanh trượt tính') : ''; } };
  listen('settings:pid', lock); listen('armed', lock); lock();
  return table;
}

function sliders() {
  const on = () => sliderAxes() > 0;
  const s = (name) => sliderRow('pid', name, { fmt: mult, step: 5, mark: 100, enabled: on });
  const state = h('p', { class: 'help', id: 'slider-state' });
  const paint = () => {
    const n = sliderAxes();
    state.textContent = n === 0 ? tr('Thanh trượt đang tắt: mọi ô trong bảng bên dưới nhập tay.')
      : (n === 2 ? tr('Roll và pitch do thanh trượt tính; yaw nhập tay.') : tr('Cả ba trục do thanh trượt tính: các ô trong bảng bên dưới bị khoá và hiện kết quả.'));
  };
  listen('settings:pid', paint); paint();
  const reset = h('button', { type: 'button', class: 'btn', id: 'sliders-reset' }, tr('Đưa mọi thanh trượt về 1.00'));
  reset.addEventListener('click', () => {
    setFields('pid', { slider_d: 100, slider_pi: 100, slider_ff: 100, slider_d_max: 100, slider_i: 100, slider_pitch_d: 100, slider_pitch_pi: 100, slider_master: 100 });
  });
  const lockReset = () => { reset.disabled = store.armed || !on(); };
  listen('settings:pid', lockReset); listen('armed', lockReset); lockReset();
  return block(tr('Thanh trượt tune'),
    form(row('pid', 'slider_mode')), state,
    form(s('slider_d'), s('slider_pi'), s('slider_ff')),
    h('h3', {}, tr('Chỉnh sâu')),
    form(s('slider_d_max'), s('slider_i'), s('slider_pitch_d'), s('slider_pitch_pi'), s('slider_master')),
    h('div', { class: 'row-actions' }, reset),
    note(tr('1.00 là bộ số mặc định của Betaflight, vốn dành cho quad 5 inch lực đẩy lớn. Máy nặng hoặc lực đẩy yếu thường cần Master Multiplier từ 1.3 đến 2.0; đổi từng nấc 0.1 và bay thử, sờ motor sau mỗi lần: motor nóng là D quá cao hoặc lọc quá ít.')));
}

function live() {
  const pct = (v) => (v === undefined ? '–' : `${v} %`);
  return block(tr('Lúc này'),
    kv([
      [tr('TPA'), readout('tuning', (t) => pct(t.tpa))],
      [tr('Anti Gravity'), readout('tuning', (t) => t.anti_gravity.toFixed(1))],
      [tr('Trạng thái'), h('span', { id: 'air-state' }, readout('tuning', (t) => ({ GROUND: tr('trên mặt đất: motor ở idle'), AIR: tr('đang bay'), AIRMODE: tr('đang bay, cần ga ở min (airmode)') }[t.air_state] || t.air_state)))],
      [tr('PID sum (roll, pitch, yaw)'), readout('tuning', (t) => t.pid_sum.map((v) => v.toFixed(0)).join('  '))],
      [tr('Ga treo đo được'), h('span', { id: 'hover-read' }, readout('tuning', (t) => pct(t.hover)))],
    ]),
    h('p', { class: 'help' }, tr('Ga treo: firmware tự đo trong mọi mode bay (trước chuyến bay đầu là số đoán). Altitude Hold và failsafe dùng số này, không cần nhập.')),
    h('p', { class: 'help' }, tr('PID sum: 1000 là toàn bộ dải motor; giới hạn 500 (roll, pitch) và 400 (yaw). Số nhảy quanh giới hạn khi bay bình thường là hệ số quá cao hoặc motor quá yếu.')));
}

export default {
  id: 'tuning',
  title: 'PID',
  lead: tr('PID có tác dụng ngay khi sửa (kể cả lúc máy đang nối USB trên bàn), nhưng chỉ được giữ lại khi bấm "Lưu". Đơn vị và mặc định là của Betaflight: số từ một bản tune Betaflight dùng được thẳng ở đây.'),
  streams: { status: 10, tuning: 10 },
  mount(view) {
    const stab = h('table', { class: 'grid axis', id: 'stab-gains' },
      h('thead', {}, h('tr', {}, h('th', {}), h('th', {}, 'Stabilize P'))),
      h('tbody', {}, AXES.map((ax, i) => h('tr', {}, h('th', { scope: 'row' }, h('i', { class: 'swatch', style: { background: `var(${SERIES[i]})` } }), ax), h('td', {}, control('pid', 'stab_p', { index: i }))))));
    view.append(h('div', { class: 'cols' },
      h('div', {},
        sliders(),
        block(tr('PID từng trục'), gainTable(),
          h('p', { class: 'help' }, tr('Thang của Betaflight: P, I, D, D Max từ 0 đến 250, Feedforward từ 0 đến 1000. D Max lớn hơn D thì D tự tăng về phía D Max khi máy bị lắc hoặc cần gạt nhanh (Dynamic Damping).'))),
        live()),
      h('div', {},
        block(tr('Feedforward'), fields('pid', ['ff_jitter_factor', 'ff_smooth_factor', 'ff_averaging', 'ff_boost', 'ff_max_rate_limit', 'ff_transition', 'ff_yaw_hold_gain', 'ff_yaw_hold_time']),
          h('p', { class: 'help' }, tr('Feedforward lấy tốc độ gạt cần đẩy thẳng ra motor, nên máy quay cùng lúc với cần thay vì chờ sai lệch lớn lên. Tắt hẳn: đặt cột Feedforward trong bảng về 0.'))),
        block(tr('I-term relax'), fields('pid', ['iterm_relax', 'iterm_relax_type', 'iterm_relax_cutoff', 'iterm_windup'])),
        block(tr('Anti Gravity'), fields('pid', ['anti_gravity', 'anti_gravity_gain', 'anti_gravity_p_gain', 'anti_gravity_cutoff'])),
        store.schema.sectionByName.pid.fields.some((f) => f.n === 'acro_trainer_gain') ? block(tr('Acro trainer'), fields('pid', ['acro_trainer_angle_limit', 'acro_trainer_lookahead_ms', 'acro_trainer_gain']),
          h('p', { class: 'help' }, tr('Bật bằng công tắc ACRO TRAINER (tab Mode), chỉ có tác dụng trong Acro. Nhìn trước: máy quay càng nhanh thì càng phanh sớm để không vượt góc; gạt hết cần có thể vượt vài độ rồi kéo về.'))) : null,
        block(tr('Dynamic Damping (D Max)'), fields('pid', ['d_max_gain', 'd_max_advance'])),
        block(tr('TPA và ga'), fields('pid', ['tpa_mode', 'tpa_rate', 'tpa_breakpoint', 'tpa_low_rate', 'tpa_low_breakpoint', 'tpa_low_always', 'throttle_boost', 'throttle_boost_cutoff']),
          note(tr('Giới hạn công suất motor và dynamic idle (cần DShot hai chiều) ở tab Motor.'))),
        block(tr('Angle'), fields('pid', ['angle_strength', 'angle_feedforward', 'angle_ff_smoothing', 'angle_earth_ref']), form(row('rates', 'angle_limit'))),
        block(tr('Stabilize (kiểu ArduPilot)'), stab, fields('pid', ['stab_input_tc', 'stab_accel_rp', 'stab_accel_yaw', 'stab_rate_ff']),
          h('p', { class: 'help' }, tr('Stabilize P: tốc độ quay yêu cầu (°/s) trên mỗi độ sai lệch góc, riêng từng trục (ATC_ANG_RLL_P, ATC_ANG_PIT_P, ATC_ANG_YAW_P; mặc định 4,5). Yaw trong Stabilize giữ hướng mũi khi cần yaw ở giữa.'))))));
  },
};
