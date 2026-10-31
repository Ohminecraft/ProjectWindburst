// The DIY transmitter (docs/TX.md), overview: where the sticks are, the eight channels it sends, the radio link to the drone and
// what the drone says about itself on the ACKs (battery, armed, flight mode, link quality).
import { store } from '../store.js';
import { h, block, listen, kv, note } from '../ui.js';
import { ENUM_LABELS } from '../labels.js';
import { tr } from '../i18n.js';

const pos = (us) => Math.max(0, Math.min(100, ((us || 1500) - 1000) / 10));      // 1000..2000 us -> 0..100 %

// one stick box: x from one channel, y from another (up = high)
function stick(id, label, chx, chy) {
  const dot = h('i', { class: 'tx-dot' });
  const box = h('div', { class: 'tx-stick', id, role: 'img', 'aria-label': label }, h('b', { class: 'tx-cross' }), dot);
  const paint = () => {
    const t = store.streams.tx; if (!t) return;
    dot.style.left = pos(t.ch[chx]) + '%'; dot.style.top = (100 - pos(t.ch[chy])) + '%';
  };
  listen('stream:tx', paint); paint();
  return h('figure', { class: 'tx-stick-fig' }, box, h('figcaption', {}, label));
}

function channels() {
  const names = ['Roll', 'Pitch', 'Throttle', 'Yaw', 'AUX1', 'AUX2', 'AUX3', 'AUX4'];
  const vals = names.map((_, i) => h('span', { class: 'num', id: 'tx-ch-' + i }, '–'));
  const paint = () => { const t = store.streams.tx; if (t) vals.forEach((v, i) => { v.textContent = String(t.ch[i]); }); };
  listen('stream:tx', paint); paint();
  return kv(names.map((n, i) => [n, vals[i]]));
}

// a throttle mode that does not suit the flight mode the drone flies in (self-centring sticks: centre = 1500)
export function throttleMismatch(mode, flying) {
  const holds = flying === 'ALTHOLD' || flying === 'POSHOLD';
  if (mode === 'ALTPOS' && !holds) return tr('Ga đang ở chế độ ALT / POS (cần ở giữa = 1500) mà drone đang bay {mode}: buông cần là ga 50 %. Chuyển ga sang Thủ công ở tab Joystick, hoặc bật ALT HOLD / POS HOLD trên drone.', { mode: tr(ENUM_LABELS.fly_mode[flying] || flying) });
  if (mode === 'MANUAL' && holds) return tr('Ga đang ở chế độ Thủ công mà drone đang ở {mode}: cần ga không còn nghĩa "giữa = giữ độ cao". Chuyển ga sang ALT / POS ở tab Joystick.', { mode: tr(ENUM_LABELS.fly_mode[flying] || flying) });
  return '';
}

export default {
  id: 'tx-overview',
  title: tr('Tổng quan'),
  lead: tr('Tay phát DIY: vị trí cần, các kênh đang gửi, sóng tới drone và những gì drone báo về.'),
  streams: { tx: 20 },
  mount(view) {
    const link = h('span', { id: 'tx-link' }, '–'), drone = h('div', { id: 'tx-drone' });
    const warn = h('p', { class: 'note warn', id: 'tx-throttle-warn', hidden: true });
    const paint = () => {
      const t = store.streams.tx; if (!t) return;
      link.textContent = t.link ? tr('đã nối · {ack} % gói được nhận · {pps} gói/s', { ack: t.ack_pct, pps: t.pps }) : tr('chưa nối với drone ({state})', { state: tr(ENUM_LABELS.bind_state[t.bind_state] || t.bind_state) });
      link.className = t.link ? 'ok' : 'bad';
      drone.replaceChildren(t.d_seen
        ? kv([[tr('Pin'), (t.d_batt_cv / 100).toFixed(2) + ' V'], [tr('Trạng thái'), t.d_armed ? 'ARMED' : tr('chưa arm')],
          [tr('Chế độ bay'), tr(ENUM_LABELS.fly_mode[t.d_mode] || t.d_mode)], [tr('Sóng phía drone'), t.d_lq + ' %'], [tr('Failsafe'), t.d_failsafe ? tr('đang failsafe') : tr('không')]])
        : h('p', { class: 'help' }, tr('Chưa nghe thấy drone. Drone phải bật, dùng receiver nRF24 và đã bind với tay này (tab Radio).')));
      const m = t.d_seen ? throttleMismatch((store.settings.tx_sticks || {}).throttle_mode, t.d_mode) : '';
      warn.textContent = m; warn.hidden = !m;
    };
    listen('stream:tx', paint); listen('settings:tx_sticks', paint);
    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Cần điều khiển'), h('div', { class: 'tx-sticks' }, stick('tx-stick-left', tr('Trái: yaw và ga'), 3, 2), stick('tx-stick-right', tr('Phải: roll và pitch'), 0, 1)), warn),
        block(tr('Kênh đang gửi'), channels(), h('p', { class: 'help' }, tr('Kênh 5 đến 8 là 1000 cho tới khi tay có công tắc.')))),
      h('div', {},
        block(tr('Sóng tới drone'), h('p', {}, link)),
        block(tr('Drone báo về'), drone,
          note(tr('Arm bằng cần (bật "Arm bằng cần" ở tab Mode của drone): kéo ga và pitch xuống hết, yaw và roll về hai phía ngược nhau, giữ 1 giây.'))))));
    paint();
  },
};
