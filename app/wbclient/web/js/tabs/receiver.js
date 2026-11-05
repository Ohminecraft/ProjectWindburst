// Receiver: live channels, protocol, channel map.
import { store, setFields } from '../store.js';
import { h, block, fields, form, row, listen, note, readout, cmdButton, toast, kv } from '../ui.js';
import { ENUM_LABELS } from '../labels.js';
import { tr } from '../i18n.js';
import { stickPreview } from '../gfx/stickpreview.js';

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

// nRF24 (the DIY transmitter): bind, its state, what the link looks like from here
function nrfBlock() {
  const state = h('b', { id: 'nrf-bind-state' }, '–'), lq = h('span', { class: 'num', id: 'nrf-lq' }, '–');
  const paint = () => { const rc = store.streams.rc; if (!rc) return; state.textContent = tr(ENUM_LABELS.bind_state[rc.bind_state] || rc.bind_state); lq.textContent = rc.lq + ' %'; };
  listen('stream:rc', paint); paint();
  const bind = cmdButton(tr('Bind nRF24'), 'BIND', { action: 1 }, { done: () => toast(tr('Đang nghe bind 30 giây: bấm "Bind với drone" ở tab Radio của tay phát.'), 'info', 7000) });
  bind.id = 'nrf-bind';
  // set by the bind / the transmitter's Radio tab: shown, not editable here (a hand edit would split the pair after a restart)
  const ch = h('span', { class: 'num', id: 'nrf-channel' }, '–'), rate = h('span', { id: 'nrf-rate' }, '–'), pps = h('span', { id: 'nrf-pps' }, '–');
  const paintCfg = () => { const r = store.settings.receiver; if (!r) return; ch.textContent = String(r.nrf_channel); rate.textContent = tr(ENUM_LABELS.nrf_rate[r.nrf_rate] || r.nrf_rate); pps.textContent = tr(ENUM_LABELS.nrf_pps[r.nrf_pps] || r.nrf_pps); };
  listen('settings:receiver', paintCfg); paintCfg();
  const el = block('nRF24', h('p', {}, tr('Trạng thái: '), state, ' · ', tr('Gói nhận được: '), lq), h('div', { class: 'btn-row' }, bind),
    kv([[tr('Kênh tần số nRF24'), ch], [tr('Tốc độ sóng nRF24'), rate], [tr('Số gói mỗi giây của tay phát'), pps]]),
    h('p', { class: 'help' }, tr('Ba giá trị trên do việc bind (và tab Radio của tay phát) đặt. Mạch chưa bind lần nào tự nghe bind trong 60 giây sau khi bật.')));
  el.id = 'nrf-block';
  const show = () => { el.hidden = (store.settings.receiver || {}).protocol !== 'NRF24'; };
  listen('settings:receiver', show); show();
  return el;
}

// what to know about the chosen protocol (wiring, channel order, failsafe)
function protoHelp() {
  const el = h('p', { class: 'help', id: 'rx-proto-help' });
  const spek = () => tr('Spektrum (satellite DSMX / DSM2) nối vào RX của UART2, 3,3 V. Spektrum gửi kênh theo thứ tự TAER: chọn "TAER" ở Thứ tự kênh. Firmware chưa làm được việc bind: bind satellite bằng receiver chính hoặc tay phát trước.');
  const text = {
    SPEKTRUM: spek, SPEKTRUM1024: spek,
    MAVLINK: () => tr('MAVLink: receiver ExpressLRS (3.5 trở lên) để ở chế độ MAVLink, nối vào UART2 như CRSF (RX và TX), 460800 baud. Một link mang cả cần điều khiển lẫn telemetry MAVLink về Mission Planner / QGroundControl (bật "Telemetry về tay điều khiển"); khi bộ đệm của receiver gần đầy, firmware tự gửi chậm lại. Cách cài đặt: docs/ELRS_MAVLINK.md.'),
    PPM: () => tr('PPM: một dây tín hiệu vào chân "PPM" ở tab Chân và cổng (mức 3,3 V; receiver 5 V cần mạch chia áp). Số kênh (4 đến 12) firmware tự nhận sau khoảng nửa giây. UART2 để trống cho việc khác.'),
    NRF24: () => tr('nRF24: module nRF24L01+ nối vào bus SPI thứ hai (chân ở tab Chân và cổng, nhóm Receiver nRF24), cấp 3,3 V và thêm tụ 10 µF sát chân nguồn của module. Dùng với tay phát DIY Windburst; bind ở khung nRF24 bên dưới. Không dùng cùng lúc được với thẻ SD kiểu SPI, cảm biến PMW3901 hay dải LED.'),
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
          h('div', { class: 'link-read', id: 'rc-errors' },
            h('span', {}, tr('Khung hỏng mỗi giây: '), readout('rc', (r) => String(r.bad_frames))),
            h('span', {}, tr('Tràn bộ đệm UART: '), readout('rc', (r) => String(r.uart_ovf))),
            h('span', {}, tr('Lỗi đường truyền UART: '), readout('rc', (r) => String(r.uart_line)))),
          h('p', { class: 'help' }, tr('Tốc độ khung thấp hơn tốc độ gói của receiver (ELRS: 150, 250, 500 Hz…): "Khung hỏng" tăng = byte bị mất hoặc sai. "Tràn bộ đệm" tăng = firmware đọc UART không kịp. "Lỗi đường truyền" tăng = nhiễu trên dây, sai baud hoặc sai đảo mức. Cả ba bằng 0 mà tốc độ vẫn thấp = receiver gửi ít khung hơn (xem tốc độ gói ở tay phát).')),
          bars(),
          h('p', { class: 'help' }, tr('Thang 900 đến 2100 µs, vạch giữa là 1500. Cần ga thấp nhất phải về gần 1000.'))),
        block(tr('Máy bay theo cần'), stickPreview('ANGLE'))),
      h('div', {},
        block(tr('Giao thức'),
          fields('receiver', ['protocol', 'rx_timeout_ms', 'deadband', 'yaw_deadband', 'deadband3d_throttle', 'min_check', 'mid_rc', 'max_check', 'telemetry']),
          note(tr('CRSF dùng cho mọi receiver ELRS và Crossfire nối qua UART. Chân UART của receiver đặt ở tab Chân và cổng. Có chân TX (nối vào RX của receiver) thì firmware gửi telemetry CRSF về tay phát: pin, dòng, mAh, tư thế, chế độ bay, GPS, vario. Đổi giao thức cần lưu và khởi động lại.')),
          protoHelp()),
        nrfBlock(),
        block(tr('RC smoothing'), fields('receiver', ['rc_smoothing', 'rc_smoothing_factor', 'rc_smoothing_throttle']),
          h('p', { class: 'help' }, tr('Receiver chỉ gửi vị trí cần vài chục đến vài trăm lần mỗi giây; firmware nối các bậc đó thành đường cong trơn trước khi đưa vào PID. Tần số cắt tự theo tốc độ khung đo được. Có hiệu lực sau khi lưu và khởi động lại.'))),
        block(tr('Thứ tự kênh'),
          form([h('label', { for: 'chmap' }, tr('Kiểu thứ tự')), h('div', { class: 'ctl' }, preset)], chSel('ch_roll'), chSel('ch_pitch'), chSel('ch_throttle'), chSel('ch_yaw'), chSel('ch_aux0'))))));
  },
};
