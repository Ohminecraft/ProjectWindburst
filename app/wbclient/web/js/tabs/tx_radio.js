// The DIY transmitter, radio: bind with a drone, RF channel / data rate / power / packet rate, the nRF24's pins on the transmitter.
import { store } from '../store.js';
import { h, block, fields, listen, note, cmdButton, toast } from '../ui.js';
import { ENUM_LABELS } from '../labels.js';
import { tr } from '../i18n.js';

export default {
  id: 'tx-radio',
  title: 'Radio',
  lead: tr('Ghép tay với drone và cài đặt sóng nRF24.'),
  streams: { tx: 10 },
  mount(view) {
    const state = h('b', { id: 'tx-bind-state' }, '–');
    const addr = h('span', { class: 'num', id: 'tx-bind-addr' }, '–');
    const paint = () => {
      const t = store.streams.tx, r = store.settings.tx_radio;
      if (t) state.textContent = tr(ENUM_LABELS.bind_state[t.bind_state] || t.bind_state);
      if (r) addr.textContent = r.bound ? r.addr.map((b) => b.toString(16).toUpperCase().padStart(2, '0')).join('') : tr('chưa bind');
    };
    listen('stream:tx', paint); listen('settings:tx_radio', paint); paint();
    const bind = cmdButton(tr('Bind với drone'), 'BIND', { action: 1 }, { class: 'primary', done: () => toast(tr('Đang bind: đặt tay sát drone. Drone chưa bind lần nào tự nghe trong 60 giây sau khi bật; nếu không, bấm "Bind nRF24" ở tab Receiver của drone.'), 'info', 8000) });
    bind.id = 'tx-bind';
    const stop = cmdButton(tr('Dừng bind'), 'BIND', { action: 0 }); stop.id = 'tx-bind-stop';
    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Ghép với drone'), h('p', {}, tr('Trạng thái: '), state, ' · ', tr('Địa chỉ cặp: '), addr), h('div', { class: 'btn-row' }, bind, stop),
          h('p', { class: 'help' }, tr('Mỗi lần bind, tay tạo một địa chỉ ngẫu nhiên mới: hai bộ tay và drone bật cạnh nhau không lái nhầm nhau. Bind chạy ở công suất thấp nhất, tối đa 30 giây; xong thì cả hai tự lưu.'))),
        block(tr('Sóng'), fields('tx_radio', ['rf_channel', 'rate', 'power', 'pps']),
          note(tr('Kênh tần số, tốc độ và số gói mỗi giây: tay gửi cài đặt mới cho drone trên cài đặt cũ, rồi cả hai cùng đổi. Chỉ đổi khi drone đang bật và chưa arm; drone không theo được trong 1 giây thì cả hai quay về cài đặt cũ. Công suất là của riêng tay: đổi ngay.')))),
      h('div', {},
        block(tr('Chân nRF24 trên tay phát'), fields('tx_radio', ['pin_sck', 'pin_miso', 'pin_mosi', 'pin_cs', 'pin_ce']),
          h('p', { class: 'help' }, tr('ESP32-C5: module nRF24 cấp 3,3 V, thêm tụ 10 µF sát chân nguồn của module. Đổi chân có hiệu lực sau khi lưu và khởi động lại.'))))));
  },
};
