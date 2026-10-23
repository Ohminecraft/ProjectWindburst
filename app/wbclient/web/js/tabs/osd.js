// OSD: what the goggles of a digital FPV system (DJI O3 / O4, Walksnail, HDZero) draw, sent by the firmware over MSP DisplayPort.
// The preview is a goggles' view: each element at its place on the character grid, in a font of the same cells (gfx/osdglyph.js),
// with the values the craft reports now (examples when there are none); the background tilts with the craft. Drag an element (or
// focus it and use the arrow keys) to move it, the check box shows / hides it. Positions: Betaflight's encoding (column bits 0-5,
// row bits 6-10, shown bit 11).
import { store, setField } from '../store.js';
import { h, block, fields, control, listen, note, track } from '../ui.js';
import { tr } from '../i18n.js';
import { cellSvg } from '../gfx/osdglyph.js';

const VISIBLE = 0x800;
const col = (p) => p & 0x3f, row = (p) => (p >> 6) & 0x1f, shown = (p) => !!(p & VISIBLE);
const make = (c, r, v) => (c & 0x3f) | ((r & 0x1f) << 6) | (v ? VISIBLE : 0);

// ---- what each element shows (src/osd/osd.h element_text): an array of cells - a character, or a glyph ([name, arg])
const str = (s) => [...s].map((ch) => ({ ch }));          // characters; a plain string in a cell list is a glyph's name
const num = (v, d) => (Number.isFinite(v) ? v : 0).toFixed(d);
const mmss = (sec) => `${String(Math.floor(sec / 60) % 100).padStart(2, '0')}:${String(Math.floor(sec % 60)).padStart(2, '0')}`;
const heading = (deg) => ((Math.round(deg) % 360) + 360) % 360;
const BAR = ['W', 'l', 't', 'l', 'N', 'l', 't', 'l', 'E', 'l', 't', 'l', 'S', 'l', 't', 'l', 'W', 'l', 't', 'l', 'N', 'l', 't', 'l'];
function battGlyph(v) {
  if (v.battState === 'CRITICAL') return 'battCrit';
  const cell = v.cells ? v.volts / v.cells : 4.2;
  return ['batt', Math.max(0, Math.min(6, Math.floor(((cell - 3.3) * 8) / 1.0)))];
}
const CELLS = {
  RSSI: (v) => ['lq', ...str(String(Math.min(99, v.lq)))],
  BATT: (v) => [battGlyph(v), ...str(num(v.volts, v.volts >= 10 ? 1 : 2)), 'volt'],
  CELL: (v) => [battGlyph(v), ...str(num(v.cells ? v.volts / v.cells : 0, 2)), 'volt'],
  CURRENT: (v) => [...str(num(Math.abs(v.amps), 2)), 'amp'],
  MAH: (v) => [...str(String(Math.round(v.mah))), 'mah'],
  MODE: (v) => str(v.mode),
  NAME: (v) => str(v.name),
  FLY_TIME: (v) => ['flyM', ...str(mmss(v.flyS))],
  ON_TIME: (v) => ['onM', ...str(mmss(v.onS))],
  SATS: (v) => ['satL', 'satR', ...str(String(v.sats))],
  SPEED: (v) => ['speed', ...str(String(Math.round(v.speed * 3.6))), 'kph'],
  HOME_DIR: (v) => [['arrow', v.homeDir - v.heading]],
  HOME_DIST: (v) => ['home', ...str(String(Math.round(v.homeDist))), 'm'],
  ALT: (v) => ['alt', ...str(num(v.alt, 1)), 'm'],
  VARIO: (v) => [v.vz >= 0 ? 'up' : 'down', ...str(num(Math.abs(v.vz), 1)), 'mps'],
  THROTTLE: (v) => ['thr', ...str(String(Math.round(v.throttle * 100)))],
  CROSSHAIRS: () => ['ahLine', 'ahCenter', 'ahLine'],
  HORIZON: null,                                             // (drawn over 9 columns, below)
  WARNINGS: (v) => str(v.warning),
  HEADING: (v) => [['arrow', 0], ...str(String(heading(v.heading)).padStart(3, '0'))],
  PITCH: (v) => ['pitch', { ch: v.pitch < 0 ? '-' : ' ' }, ...str(String(Math.round(Math.abs(v.pitch))).padStart(2, '0'))],
  ROLL: (v) => ['roll', { ch: v.roll < 0 ? '-' : ' ' }, ...str(String(Math.round(Math.abs(v.roll))).padStart(2, '0'))],
  ESC_TEMP: (v) => [...str(String(v.escTemp)), 'c'],
  DISARMED: (v) => (v.armed ? [] : str('DISARMED')),
  POWER: (v) => str(`${Math.round(Math.abs(v.amps) * v.volts)}W`),
  FLIGHT_DIST: (v) => ['dist', ...str(String(Math.round(v.dist))), 'm'],
  GPS_LAT: (v) => ['lat', ...str((v.lat / 1e7).toFixed(7))],
  GPS_LON: (v) => ['lon', ...str((v.lon / 1e7).toFixed(7))],
  COMPASS_BAR: (v) => { const d = Math.round((heading(v.heading) * 16) / 360) % 16; return BAR.slice(d, d + 9).map((c) => (c === 'l' ? 'hdgLine' : c === 't' ? 'hdgTick' : { ch: c })); },
  SIDEBARS: null,                                            // (drawn over 15 columns, below)
  G_FORCE: (v) => str(`${num(v.g, 1)}G`),
  BATT_USAGE: (v) => ['pbStart', ...Array.from({ length: 10 }, (_, i) => { const left = v.remaining - i * 10; return left >= 10 ? 'pbFull' : left >= 5 ? 'pbHalf' : 'pbEmpty'; }), 'pbClose'],
};

// osd.pos[] order = osd::Element (src/osd/osd.h)
const ELEMENTS = [
  ['RSSI', tr('Chất lượng sóng (LQ)')], ['BATT', tr('Điện áp pin')], ['CELL', tr('Điện áp trung bình mỗi cell')], ['CURRENT', tr('Dòng')],
  ['MAH', tr('mAh đã dùng')], ['MODE', tr('Mode bay')], ['NAME', tr('Tên máy bay')], ['FLY_TIME', tr('Thời gian bay')],
  ['ON_TIME', tr('Thời gian bật nguồn')], ['SATS', tr('Số vệ tinh GPS')], ['SPEED', tr('Tốc độ (GPS)')], ['HOME_DIR', tr('Hướng về nhà')],
  ['HOME_DIST', tr('Khoảng cách về nhà')], ['ALT', tr('Độ cao')], ['VARIO', tr('Tốc độ lên / xuống')], ['THROTTLE', tr('Ga')],
  ['CROSSHAIRS', tr('Tâm ngắm')], ['HORIZON', tr('Đường chân trời giả')], ['WARNINGS', tr('Cảnh báo')], ['HEADING', tr('Hướng mũi')],
  ['PITCH', tr('Góc pitch')], ['ROLL', tr('Góc roll')], ['ESC_TEMP', tr('Nhiệt độ ESC')], ['DISARMED', tr('Chữ DISARMED')],
  ['POWER', tr('Công suất (W)')], ['FLIGHT_DIST', tr('Quãng đường đã bay')], ['GPS_LAT', tr('Vĩ độ')], ['GPS_LON', tr('Kinh độ')],
  ['COMPASS_BAR', tr('Thanh la bàn')], ['SIDEBARS', tr('Hai thanh bên của đường chân trời')], ['G_FORCE', tr('Gia tốc (G)')], ['BATT_USAGE', tr('Thanh pin còn lại')],
];
const CANVAS = { AUTO: [53, 20], HD_53X20: [53, 20], HDZERO_50X18: [50, 18], SD_30X16: [30, 16] };

function nameText() {
  const a = (store.settings.osd || {}).name || [];
  const z = a.indexOf(0);
  return String.fromCharCode(...(z < 0 ? a : a.slice(0, z))).toUpperCase();
}

// The values: what the craft reports now, or an example of each
function values() {
  const s = store.streams, st = s.status || {}, at = s.attitude || {}, b = s.battery || {}, g = s.gps || {}, e = s.estimate || {}, rc = s.rc || {}, sn = s.sensors || {};
  const live = !!store.conn.connected;
  const modes = st.modes || [];
  const mode = modes.includes('RESCUE') ? 'RESC' : modes.includes('POSHOLD') ? 'POSH' : modes.includes('ALTHOLD') ? 'ALTH' : modes.includes('STABILIZE') ? 'STAB' : modes.includes('ANGLE') ? 'ANGL' : live ? 'ACRO' : 'ANGL';
  const acc = sn.acc || [0, 0, 1];
  const warn = st.failsafe && st.failsafe !== 'NONE' ? 'FAILSAFE' : b.state === 'CRITICAL' ? 'LAND NOW' : b.state === 'WARNING' ? 'LOW BATTERY' : live && rc.link === false ? 'RXLOSS' : live ? '' : 'LOW BATTERY';
  return {
    armed: live ? !!st.armed : false, lq: live ? rc.rssi || 0 : 99, mode, name: nameText() || tr('(chưa đặt tên)'),
    volts: b.voltage || 16.4, cells: b.cells || 4, battState: b.state, amps: b.current ?? 12.35, mah: b.mah ?? 1234, remaining: b.remaining ?? 75,
    flyS: 135, onS: 340, sats: g.sats ?? 12, speed: (g.gs ?? 1000) / 100, homeDist: g.home_dist ?? 123, homeDir: (g.home_dir ?? 450) / 10,
    alt: e.z ?? 12.3, vz: e.vz ?? 1.2, throttle: st.throttle ?? 0.46, warning: warn, heading: at.yaw ?? 123, pitch: at.pitch ?? -12, roll: at.roll ?? 5,
    escTemp: 45, dist: 1234, lat: g.lat || 107654321, lon: g.lon || 1066543210, g: Math.hypot(acc[0], acc[1], acc[2]) || 1,
  };
}

// The artificial horizon (osd.h horizon()): 9 columns, each one of 9 levels on rows pos .. pos + 9; [dx, dy, level]
function horizonCells(v) {
  const out = [], roll = Math.max(-400, Math.min(400, Math.round(v.roll * 10)));
  let pitch = Math.max(-200, Math.min(200, Math.round(v.pitch * 10)));
  pitch = Math.trunc((pitch * 25) / 200) - 41;
  for (let x = -4; x <= 4; x++) { const y = Math.trunc((-roll * x) / 64) - pitch; if (y >= 0 && y <= 81) out.push([x, Math.floor(y / 9), y % 9]); }
  return out;
}

function preview() {
  const box = h('div', { class: 'osd-canvas', id: 'osd-canvas', role: 'application', 'aria-label': tr('Màn hình OSD: kéo một mục để dời nó') });
  const view = h('div', { class: 'osd-view', 'aria-hidden': 'true' }, h('div', { class: 'osd-world' }));
  const size = () => CANVAS[(store.settings.osd || {}).canvas] || CANVAS.AUTO;
  const esc = (c) => (c === '<' ? '&lt;' : c === '&' ? '&amp;' : c === ' ' ? '&nbsp;' : c);
  const paintCells = (el, cells) => { el.innerHTML = cells.map((c) => (c && c.ch !== undefined ? `<span>${esc(c.ch)}</span>` : `<span class="g">${cellSvg(c)}</span>`)).join(''); el.style.setProperty('--len', cells.length); };
  const place = (el, p) => {
    const [cols, rows] = size();
    el.style.left = (col(p) / cols) * 100 + '%'; el.style.top = (row(p) / rows) * 100 + '%';
    el.hidden = !shown(p) || col(p) >= cols || row(p) >= rows;
  };
  let items = [];
  const build = () => {
    const [cols, rows] = size(), pos = store.settings.osd.pos;
    box.style.setProperty('--cols', cols); box.style.setProperty('--rows', rows);
    items = ELEMENTS.map(([id, label], i) => {
      if (i >= pos.length) return null;
      const el = h('div', { class: 'osd-item', tabindex: 0, 'data-el': id, title: label, 'aria-label': tr('{label}: cột {c}, hàng {r}', { label, c: col(pos[i]), r: row(pos[i]) }) });
      place(el, pos[i]);
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
      return el;
    });
    box.replaceChildren(view, ...items.filter(Boolean));
    paint();
  };
  // the values change: redraw the text of every element (cheap: a few hundred cells), and tilt the camera's view
  const paint = () => {
    const v = values();
    ELEMENTS.forEach(([id], i) => {
      const el = items[i]; if (!el) return;
      if (id === 'HORIZON') { const cells = horizonCells(v); el.innerHTML = cells.map(([x, y, l]) => `<span class="g abs" style="--x:${x + 4};--y:${y}">${cellSvg(['ah', l])}</span>`).join(''); el.style.setProperty('--len', 1); el.classList.add('multi'); el.style.setProperty('--ox', -4); return; }
      if (id === 'SIDEBARS') {
        const cells = [];
        for (let y = -3; y <= 3; y++) cells.push([0, y + 3, 'ahDeco'], [14, y + 3, 'ahDeco']);
        cells.push([1, 3, 'ahLeft'], [13, 3, 'ahRight']);
        el.innerHTML = cells.map(([x, y, g]) => `<span class="g abs" style="--x:${x};--y:${y}">${cellSvg(g)}</span>`).join('');
        el.style.setProperty('--len', 1); el.classList.add('multi'); el.style.setProperty('--ox', -7); el.style.setProperty('--oy', -3); return;
      }
      paintCells(el, CELLS[id](v));
    });
    box.style.setProperty('--roll', `${-(v.roll || 0)}deg`); box.style.setProperty('--pitch', `${Math.max(-30, Math.min(30, v.pitch || 0)) * 1.2}%`);
  };
  // the characters scale with the box: one grid cell = width / columns
  const ro = new ResizeObserver(() => box.style.setProperty('--cell', box.clientWidth / size()[0] + 'px'));
  ro.observe(box); track(() => ro.disconnect());
  listen('settings:osd', build);
  for (const st of ['status', 'attitude', 'battery', 'gps', 'estimate', 'rc', 'sensors']) listen('stream:' + st, paint);
  build();
  return box;
}

function list() {
  const body = h('tbody', { id: 'osd-list' });
  const build = () => {
    const pos = store.settings.osd.pos;
    body.replaceChildren(...ELEMENTS.filter((_, i) => i < pos.length).map(([id, label], i) => {
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
  streams: { status: 10, attitude: 15, battery: 2, gps: 2, estimate: 5, rc: 5, sensors: 5 },
  mount(view) {
    if (!store.schema.sectionByName.osd || !store.settings.osd) { view.append(note(tr('Firmware này chưa có OSD (cần bản 0.21 trở lên).'), 'warn')); return; }
    const osdFields = ['type', 'canvas', 'baud', 'rate', 'name'].concat('stats' in store.settings.osd ? ['stats'] : []);
    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Màn hình'), preview(),
          h('p', { class: 'help' }, tr('Kéo một mục để dời nó, hoặc bấm vào nó rồi dùng phím mũi tên. Số hiện ở đây là số máy đang báo (chưa kết nối thì là ví dụ); ký hiệu vẽ theo đúng ô chữ của kính, kính dùng font Betaflight của nó. Canvas "Tự động" xem như 53 x 20; HDZero tự báo lưới 50 x 18 của nó, mục nào nằm ngoài lưới đó sẽ không hiện.'))),
        block(tr('Kết nối'), fields('osd', osdFields),
          h('div', { class: 'pins' },
            h('label', { class: 'pin' }, h('span', {}, 'TX'), control('osd', 'tx_pin', { empty: 255, width: '92px' })),
            h('label', { class: 'pin' }, h('span', {}, 'RX'), control('osd', 'rx_pin', { empty: 255, width: '92px' }))),
          h('p', { class: 'help' }, tr('Nối TX của flight controller vào RX của bộ phát hình, RX vào TX của nó (bộ phát hỏi trạng thái arm để bắt đầu quay video). UART: Tự động lấy UART còn trống sau GPS, receiver, optical flow và dây telemetry ESC; Theo bảng (tab Chân và cổng): UART có chức năng DisplayPort. Trong kính chọn chế độ OSD "Custom" / "Betaflight" (DJI: Canvas mode / custom OSD). Có hiệu lực sau khi lưu và khởi động lại.')))),
      h('div', {},
        block(tr('Các mục'), list()))));
  },
};
