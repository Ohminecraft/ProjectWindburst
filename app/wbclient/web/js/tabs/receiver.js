// Receiver: live channels, protocol, channel map.
import { store, setFields } from '../store.js';
import { h, block, fields, form, row, listen, note, readout } from '../ui.js';
import { tr } from '../i18n.js';

function bars() {
  const box = h('div', { class: 'chans', id: 'chans' });
  const rows = [];
  const make = (label) => {
    const fill = h('i'), val = h('span', { class: 'num' }, '–'), name = h('span', { class: 'chan-name' }, label);
    const r = h('div', { class: 'chan' }, name, h('div', { class: 'bar', role: 'img' }, h('b', { class: 'mid' }), fill), val);
    box.append(r);
    return { fill, val, name, r };
  };
  const names = ['Roll', 'Pitch', 'Throttle', 'Yaw', 'AUX1', 'AUX2', 'AUX3', 'AUX4', 'AUX5', 'AUX6', 'AUX7', 'AUX8'];
  for (const n of names) rows.push(make(n));
  const paint = () => {
    const rc = store.streams.rc, m = store.settings.receiver;
    if (!rc || !m) return;
    const idx = [m.ch_roll, m.ch_pitch, m.ch_throttle, m.ch_yaw];
    for (let i = 0; i < 8; i++) idx.push(m.ch_aux0 + i);
    rows.forEach((r, i) => {
      const us = rc.ch[idx[i]];
      const pct = Math.max(0, Math.min(100, ((us - 900) / 1200) * 100));
      r.fill.style.width = pct + '%';
      r.val.textContent = us === undefined ? '–' : us;
      r.name.textContent = tr('{name} (kênh {n})', { name: names[i], n: idx[i] + 1 });
      r.r.dataset.us = us;
    });
  };
  listen('stream:rc', paint); listen('settings:receiver', paint); paint();
  return box;
}

// what to know about the chosen protocol (wiring, channel order, failsafe)
function protoHelp() {
  const el = h('p', { class: 'help', id: 'rx-proto-help' });
  const spek = () => tr('Spektrum (satellite DSMX / DSM2) nối vào RX của UART2, 3,3 V. Spektrum gửi kênh theo thứ tự TAER: chọn "TAER" ở Thứ tự kênh. Firmware chưa làm được việc bind: bind satellite bằng receiver chính hoặc tay phát trước.');
  const text = {
    SPEKTRUM: spek, SPEKTRUM1024: spek,
    PPM: () => tr('PPM: một dây tín hiệu vào chân "PPM" ở tab Chân và cổng (mức 3,3 V; receiver 5 V cần mạch chia áp). Số kênh (4 đến 12) firmware tự nhận sau khoảng nửa giây. UART2 để trống cho việc khác.'),
    PWM: () => tr('PWM: mỗi kênh một dây, chân đặt ở nhóm "Receiver PWM" của tab Chân và cổng (mức 3,3 V). Đặt failsafe của receiver là "không phát xung": receiver phát xung failsafe thì firmware không biết là đã mất sóng.'),
  };
  const paint = () => { const f = text[(store.settings.receiver || {}).protocol]; el.textContent = f ? f() : ''; el.hidden = !f; };
  listen('settings:receiver', paint); paint();
  return el;
}

export default {
  id: 'receiver',
  title: 'Receiver',
  lead: tr('Kiểm tra từng cần và công tắc trên tay điều khiển trước khi gán mode.'),
  streams: { rc: 30, status: 10 },
  mount(view) {
    const preset = h('select', { id: 'chmap' }, h('option', { value: '' }, tr('Chọn nhanh…')), h('option', { value: 'AETR' }, 'AETR (ELRS, FrSky, Futaba)'), h('option', { value: 'TAER' }, 'TAER (Spektrum, JR)'));
    preset.addEventListener('change', () => {
      if (preset.value === 'AETR') setFields('receiver', { ch_roll: 0, ch_pitch: 1, ch_throttle: 2, ch_yaw: 3 });
      if (preset.value === 'TAER') setFields('receiver', { ch_throttle: 0, ch_roll: 1, ch_pitch: 2, ch_yaw: 3 });
      preset.value = '';
    });
    const chSel = (name) => row('receiver', name, { unit: tr('(0 = kênh 1)') });
    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Kênh đang nhận'),
          h('div', { class: 'link-read' },
            h('span', {}, tr('Tín hiệu: '), readout('rc', (r) => (r.link ? tr('có') : tr('mất')))),
            h('span', {}, tr('Chất lượng: '), readout('rc', (r) => r.rssi + ' %')),
            h('span', {}, tr('Tốc độ khung: '), readout('rc', (r) => tr('{n} khung/s', { n: r.rate })))),
          bars(),
          h('p', { class: 'help' }, tr('Thang 900 đến 2100 µs, vạch giữa là 1500. Cần ga thấp nhất phải về gần 1000.')))),
      h('div', {},
        block(tr('Giao thức'),
          fields('receiver', ['protocol', 'rx_timeout_ms', 'deadband']),
          note(tr('CRSF dùng cho mọi receiver ELRS và Crossfire nối qua UART. Chân UART của receiver đặt ở tab Chân và cổng. Có chân TX (nối vào RX của receiver) thì firmware gửi telemetry CRSF về tay phát: pin, dòng, mAh, tư thế, chế độ bay, GPS, vario. Đổi giao thức cần lưu và khởi động lại.')),
          protoHelp()),
        block(tr('RC smoothing'), fields('receiver', ['rc_smoothing', 'rc_smoothing_factor', 'rc_smoothing_throttle']),
          h('p', { class: 'help' }, tr('Receiver chỉ gửi vị trí cần vài chục đến vài trăm lần mỗi giây; firmware nối các bậc đó thành đường cong trơn trước khi đưa vào PID. Tần số cắt tự theo tốc độ khung đo được. Có hiệu lực sau khi lưu và khởi động lại.'))),
        block(tr('Thứ tự kênh'),
          form([h('label', { for: 'chmap' }, tr('Kiểu thứ tự')), h('div', { class: 'ctl' }, preset)], chSel('ch_roll'), chSel('ch_pitch'), chSel('ch_throttle'), chSel('ch_yaw'), chSel('ch_aux0'))))));
  },
};
