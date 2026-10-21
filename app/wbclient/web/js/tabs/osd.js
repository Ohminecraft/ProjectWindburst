// OSD: what the goggles of a digital FPV system (DJI O3 / O4, Walksnail, HDZero) draw, sent by the firmware over MSP DisplayPort.
// The preview shows each element at its place on the goggles' character grid; drag an element (or focus it and use the arrow keys)
// to move it, the check box shows / hides it. Positions: Betaflight's encoding (column bits 0-5, row bits 6-10, shown bit 11).
import { store, setField } from '../store.js';
import { h, block, fields, control, listen, note, track } from '../ui.js';
import { tr } from '../i18n.js';

const VISIBLE = 0x800;
const col = (p) => p & 0x3f, row = (p) => (p >> 6) & 0x1f, shown = (p) => !!(p & VISIBLE);
const make = (c, r, v) => (c & 0x3f) | ((r & 0x1f) << 6) | (v ? VISIBLE : 0);

// osd.pos[] order = osd::Element (src/osd/osd.h). sample: what the element looks like in flight (the goggles draw the symbols
// with Betaflight's font; here a stand-in of the same width).
const ELEMENTS = [
  ['RSSI', tr('Chất lượng sóng (LQ)'), '▂99'],
  ['BATT', tr('Điện áp pin'), '▮16.4V'],
  ['CELL', tr('Điện áp trung bình mỗi cell'), '▮4.10V'],
  ['CURRENT', tr('Dòng'), '12.35A'],
  ['MAH', tr('mAh đã dùng'), '1234ⓜ'],
  ['MODE', tr('Mode bay'), 'ANGL'],
  ['NAME', tr('Tên máy bay'), null],
  ['FLY_TIME', tr('Thời gian bay'), '✈02:15'],
  ['ON_TIME', tr('Thời gian bật nguồn'), '⏻05:40'],
  ['SATS', tr('Số vệ tinh GPS'), '🛰12'],
  ['SPEED', tr('Tốc độ (GPS)'), '»36k'],
  ['HOME_DIR', tr('Hướng về nhà'), '↑'],
  ['HOME_DIST', tr('Khoảng cách về nhà'), '⌂123m'],
  ['ALT', tr('Độ cao'), '▲12.3m'],
  ['VARIO', tr('Tốc độ lên / xuống'), '↑1.2s'],
  ['THROTTLE', tr('Ga'), 'T46'],
  ['CROSSHAIRS', tr('Tâm ngắm'), '-+-'],
  ['HORIZON', tr('Đường chân trời giả'), '—————————'],
  ['WARNINGS', tr('Cảnh báo'), 'LOW BATTERY'],
  ['HEADING', tr('Hướng mũi'), '↑123'],
  ['PITCH', tr('Góc pitch'), 'P-12'],
  ['ROLL', tr('Góc roll'), 'R 05'],
  ['ESC_TEMP', tr('Nhiệt độ ESC'), '45C'],
  ['DISARMED', tr('Chữ DISARMED'), 'DISARMED'],
];
const CANVAS = { AUTO: [53, 20], HD_53X20: [53, 20], HDZERO_50X18: [50, 18], SD_30X16: [30, 16] };

function nameText() {
  const a = (store.settings.osd || {}).name || [];
  const z = a.indexOf(0);
  return String.fromCharCode(...(z < 0 ? a : a.slice(0, z))).toUpperCase() || null;
}

function preview() {
  const box = h('div', { class: 'osd-canvas', id: 'osd-canvas', role: 'application', 'aria-label': tr('Màn hình OSD: kéo một mục để dời nó') });
  const items = [];
  const size = () => CANVAS[(store.settings.osd || {}).canvas] || CANVAS.AUTO;
  const place = (el, p) => {
    const [cols, rows] = size();
    el.style.left = (col(p) / cols) * 100 + '%'; el.style.top = (row(p) / rows) * 100 + '%';
    el.hidden = !shown(p) || col(p) >= cols || row(p) >= rows;
  };
  const build = () => {
    const [cols, rows] = size(), pos = store.settings.osd.pos;
    box.style.setProperty('--cols', cols); box.style.setProperty('--rows', rows);
    items.length = 0;
    box.replaceChildren(...ELEMENTS.map(([id, label, sample], i) => {
      const text = id === 'NAME' ? nameText() || tr('(chưa đặt tên)') : sample;
      const el = h('div', { class: 'osd-item', tabindex: 0, 'data-el': id, title: label, 'aria-label': tr('{label}: cột {c}, hàng {r}', { label, c: col(pos[i]), r: row(pos[i]) }) }, text);
      el.style.setProperty('--len', [...text].length);
      place(el, pos[i]);
      // drag: the element's top-left cell follows the pointer
      el.addEventListener('pointerdown', (ev) => {
        if (store.armed) return;
        ev.preventDefault(); el.setPointerCapture(ev.pointerId); el.classList.add('drag');
        const r = box.getBoundingClientRect(), p0 = store.settings.osd.pos[i];
        const dx = ev.clientX - el.getBoundingClientRect().left, dy = ev.clientY - el.getBoundingClientRect().top;
        let cur = p0;
        const at = (e) => {
          const [c, rw] = size();
          const cc = Math.max(0, Math.min(c - 1, Math.round(((e.clientX - dx - r.left) / r.width) * c)));
          const rr = Math.max(0, Math.min(rw - 1, Math.round(((e.clientY - dy - r.top) / r.height) * rw)));
          return make(cc, rr, true);
        };
        const move = (e) => { cur = at(e); place(el, cur); };
        const up = () => { el.removeEventListener('pointermove', move); el.classList.remove('drag'); if (cur !== p0) setField('osd', 'pos', cur, i); };
        el.addEventListener('pointermove', move);
        el.addEventListener('pointerup', up, { once: true });
      });
      el.addEventListener('keydown', (ev) => {
        const d = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] }[ev.key];
        if (!d || store.armed) return;
        ev.preventDefault();
        const [c, rw] = size(), p = store.settings.osd.pos[i];
        setField('osd', 'pos', make(Math.max(0, Math.min(c - 1, col(p) + d[0])), Math.max(0, Math.min(rw - 1, row(p) + d[1])), true), i);
        requestAnimationFrame(() => box.querySelector(`[data-el="${id}"]`)?.focus());
      });
      items.push(el);
      return el;
    }));
  };
  // the characters scale with the box: one grid cell = width / columns
  const ro = new ResizeObserver(() => box.style.setProperty('--cell', box.clientWidth / size()[0] + 'px'));
  ro.observe(box); track(() => ro.disconnect());
  listen('settings:osd', build); build();
  return box;
}

function list() {
  const body = h('tbody', { id: 'osd-list' });
  const build = () => {
    const pos = store.settings.osd.pos;
    body.replaceChildren(...ELEMENTS.map(([id, label], i) => {
      const cb = h('input', { type: 'checkbox', class: 'switch', role: 'switch', 'data-show': id, 'aria-label': label });
      cb.checked = shown(pos[i]); cb.disabled = store.armed;
      cb.addEventListener('change', () => { const p = store.settings.osd.pos[i]; setField('osd', 'pos', cb.checked ? p | VISIBLE : p & ~VISIBLE, i); });
      return h('tr', {}, h('td', {}, cb), h('td', {}, label), h('td', { class: 'num' }, `${col(pos[i])}, ${row(pos[i])}`));
    }));
  };
  listen('settings:osd', build); listen('armed', build); build();
  return h('table', { class: 'grid osd-list' }, h('thead', {}, h('tr', {}, h('th', {}, tr('Hiện')), h('th', {}, tr('Mục')), h('th', {}, tr('Cột, hàng')))), body);
}

export default {
  id: 'osd',
  title: 'OSD',
  lead: tr('Thông tin hiện trong kính của hệ FPV digital (DJI O3 / O4, Walksnail, HDZero) qua MSP DisplayPort.'),
  streams: { status: 10 },
  mount(view) {
    if (!store.schema.sectionByName.osd || !store.settings.osd) { view.append(note(tr('Firmware này chưa có OSD (cần bản 0.21 trở lên).'), 'warn')); return; }
    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Kết nối'), fields('osd', ['type', 'canvas', 'baud', 'rate', 'name']),
          h('div', { class: 'pins' },
            h('label', { class: 'pin' }, h('span', {}, 'TX'), control('osd', 'tx_pin', { empty: 255, width: '92px' })),
            h('label', { class: 'pin' }, h('span', {}, 'RX'), control('osd', 'rx_pin', { empty: 255, width: '92px' }))),
          h('p', { class: 'help' }, tr('Nối TX của flight controller vào RX của bộ phát hình, RX vào TX của nó (bộ phát hỏi trạng thái arm để bắt đầu quay video). UART: Tự động lấy UART còn trống sau GPS, receiver, optical flow và dây telemetry ESC; Theo bảng (tab Chân và cổng): UART có chức năng DisplayPort. Trong kính chọn chế độ OSD "Custom" / "Betaflight" (DJI: Canvas mode / custom OSD). Có hiệu lực sau khi lưu và khởi động lại.'))),
        block(tr('Các mục'), list())),
      h('div', {},
        block(tr('Màn hình'), preview(),
          h('p', { class: 'help' }, tr('Kéo một mục để dời nó, hoặc bấm vào nó rồi dùng phím mũi tên. Ký hiệu ở đây chỉ là hình minh hoạ: kính vẽ bằng font Betaflight của nó. Canvas "Tự động" xem như 53 x 20; HDZero tự báo lưới 50 x 18 của nó, mục nào nằm ngoài lưới đó sẽ không hiện.'))))));
  },
};
