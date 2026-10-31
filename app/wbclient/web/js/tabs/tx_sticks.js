// The DIY transmitter, joysticks: the throttle mode, the calibration (move every stick to its ends, let go, finish), which axis
// drives which channel, reversing, dead band, expo; the raw ADC readings to check the wiring.
import { store, reloadSection } from '../store.js';
import { h, block, fields, form, row, listen, note, cmdButton, toast, readout } from '../ui.js';
import { tr } from '../i18n.js';

const AXES = ['joy1 X', 'joy1 Y', 'joy2 X', 'joy2 Y'];

export default {
  id: 'tx-sticks',
  title: 'Joystick',
  lead: tr('Hiệu chỉnh cần, gán trục cho từng kênh và chọn cách cần ga hoạt động.'),
  streams: { tx: 20 },
  mount(view) {
    // ---- calibration
    const state = h('span', { id: 'tx-cal-state' }, '–');
    const start = cmdButton(tr('Bắt đầu hiệu chỉnh'), 'STICK_CAL', { step: 1 }, { class: 'primary' }); start.id = 'tx-cal-start';
    const finish = cmdButton(tr('Xong'), 'STICK_CAL', { step: 2 }, {
      done: async (res) => { await reloadSection('tx_sticks'); toast(tr('Đã hiệu chỉnh {n} trục (đã lưu vào flash của tay).', { n: (res || {}).axes !== undefined ? res.axes : 4 }), 'ok'); },
      fail: (e) => toast(e.code === 'FAILED' ? tr('Không trục nào đổi: đẩy từng cần ra hết hai đầu rồi thả về giữa, sau đó bấm Xong.') : e.message, 'bad', 7000),
    }); finish.id = 'tx-cal-finish';
    const cancel = cmdButton(tr('Huỷ'), 'STICK_CAL', { step: 0 }); cancel.id = 'tx-cal-cancel';
    const paint = () => { const t = store.streams.tx; if (t) state.textContent = t.calibrating ? tr('đang hiệu chỉnh: đẩy mọi cần ra hết hai đầu, rồi thả về giữa') : tr('không chạy'); };
    listen('stream:tx', paint); paint();
    // ---- raw readings and the calibration per axis
    const raw = form(AXES.map((name, i) => [h('label', {}, name), h('div', { class: 'ctl' }, h('div', { class: 'ctl-line' },
      h('span', { class: 'num', id: 'tx-raw-' + i }, readout('tx', (t) => String(t.raw[i]))),
      h('span', { class: 'unit' }, tr('(ADC 0..4095)'))))]));
    const perAxis = AXES.map((name, i) => block(name, form(
      row('tx_sticks', 'adc_pin', { index: i, label: tr('Chân ADC') }),
      row('tx_sticks', 'cal_min', { index: i, label: tr('Đầu thấp') }),
      row('tx_sticks', 'cal_mid', { index: i, label: tr('Giữa') }),
      row('tx_sticks', 'cal_max', { index: i, label: tr('Đầu cao') }),
      row('tx_sticks', 'invert', { index: i, label: tr('Đảo chiều') }))));
    const map = form(['Roll', 'Pitch', 'Throttle', 'Yaw'].map((ch, i) => row('tx_sticks', 'map', { index: i, label: ch, choices: [0, 1, 2, 3], help: '' })));
    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Cần ga'), fields('tx_sticks', ['throttle_mode', 'throttle_rate']),
          note(tr('Joystick tự về giữa: ALT / POS khi bay ALT HOLD hoặc POS HOLD (cần ở giữa = giữ độ cao). Thủ công khi bay ANGLE / ACRO: đẩy lên thì ga tăng dần, buông ra thì ga đứng yên, và ga luôn về thấp nhất khi drone chưa arm.'))),
        block(tr('Hiệu chỉnh'), h('p', {}, tr('Trạng thái: '), state), h('div', { class: 'btn-row' }, start, finish, cancel),
          h('p', { class: 'help' }, tr('Bấm Bắt đầu, đẩy từng cần ra hết bốn phía, thả cho cần về giữa rồi bấm Xong. Trục nào không được đẩy đủ xa thì giữ hiệu chỉnh cũ.'))),
        block(tr('Gán trục cho kênh'), map, h('p', { class: 'help' }, tr('0 = joy1 X, 1 = joy1 Y, 2 = joy2 X, 3 = joy2 Y. Mỗi kênh một trục khác nhau.'))),
        block(tr('Cảm giác cần'), fields('tx_sticks', ['deadband', 'expo']))),
      h('div', {}, block(tr('Giá trị đọc được'), raw), ...perAxis)));
  },
};
