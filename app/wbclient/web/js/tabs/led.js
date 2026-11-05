// LED strip, as Betaflight's (0.44; firmware src/app/ledstrip.h): a 16 x 16 grid where each LED is placed in wiring order, with
// its function, overlays, color and directions (Betaflight's 32-bit led config), the 16-color palette (Betaflight's HSV: s = 0 full
// color, 255 white), the mode / special colors. The same settings as the Betaflight App's LED Strip tab (section ledstrip).
import { store, setField, setFields } from '../store.js';
import { h, block, fields, listen, note } from '../ui.js';
import { ENUM_LABELS } from '../labels.js';
import { tr } from '../i18n.js';
import { ledHsvToRgb, ledDefine, ledFields } from '../gfx/fwmath.js';

const FUNCTIONS = ['COLOR', 'FLIGHT_MODE', 'ARM_STATE', 'BATTERY', 'RSSI', 'GPS', 'THRUST_RING', 'GPS_BAR', 'BATTERY_BAR', 'ALTITUDE'];
const FN_CODE = ['C', 'F', 'A', 'L', 'S', 'G', 'R', 'P', 'E', 'U'];        // Betaflight's CLI letters, shown on the grid
const OVERLAYS = ['THROTTLE', 'RAINBOW', 'LARSON_SCANNER', 'BLINK', null, 'INDICATOR', 'WARNING'];   // (VTX, bit 4: no VTX here)
const DIRECTIONS = ['N', 'E', 'S', 'W', 'U', 'D'];
const MODES = ['ORIENTATION', 'HEADFREE', 'HORIZON', 'ANGLE', 'MAG', 'BARO'];
const SPECIAL = ['DISARMED', 'ARMED', 'ANIMATION', 'BACKGROUND', 'BLINKBACKGROUND', 'GPSNOSATS', 'GPSNOLOCK', 'GPSLOCKED'];
const MAX = 32;

const cfg = () => store.settings.ledstrip;
const count = (leds) => { let n = 0; while (n < MAX && leds[n]) n++; return n; };      // a 0 ends the list (Betaflight)
const paletteRgb = (i) => { const c = cfg(); const [r, g, b] = ledHsvToRgb(c.color_h[i], c.color_s[i], c.color_v[i]); return `rgb(${r},${g},${b})`; };

// the tool: what a click on an empty cell places
const tool = { fn: 0, overlays: 0, color: 0, dirs: 0 };
let selected = -1;

function grid(onPick) {
  const el = h('div', { class: 'led-grid', id: 'led-grid', role: 'grid', 'aria-label': tr('Lưới LED 16 x 16, mũi máy ở phía trên') });
  const paint = () => {
    const leds = cfg().leds, n = count(leds), at = {};
    for (let i = 0; i < n; i++) { const f = ledFields(leds[i]); at[f.x + ',' + f.y] = i; }
    el.replaceChildren(...Array.from({ length: 256 }, (_, k) => {
      const x = k % 16, y = Math.floor(k / 16), i = at[x + ',' + y];
      const cell = h('button', { type: 'button', class: 'led-cell', 'data-x': x, 'data-y': y, 'aria-label': `${x},${y}` });
      if (i !== undefined) {
        const f = ledFields(leds[i]);
        cell.classList.add('on'); if (i === selected) cell.classList.add('sel');
        cell.dataset.led = i;
        cell.style.background = f.fn === 0 || f.fn === 6 || f.fn === 9 ? paletteRgb(f.color) : '';
        cell.append(h('span', { class: 'led-wire' }, String(i + 1)), h('span', { class: 'led-fn' }, FN_CODE[f.fn] || '?'));
      }
      cell.addEventListener('click', () => onPick(x, y, i));
      return cell;
    }));
  };
  listen('settings:ledstrip', paint); paint();
  el.repaint = paint;
  return el;
}

function toolbar(apply) {
  const fnSel = h('select', { id: 'led-fn', 'aria-label': tr('Chức năng') }, FUNCTIONS.map((f, i) => h('option', { value: i }, tr(ENUM_LABELS.led_bf_function[f] || f))));
  const ol = OVERLAYS.map((o, b) => (o ? h('label', { class: 'chk' }, h('input', { type: 'checkbox', 'data-ol': b }), tr(ENUM_LABELS.led_bf_overlay[o] || o)) : null)).filter(Boolean);
  const dirs = DIRECTIONS.map((d, b) => h('label', { class: 'chk' }, h('input', { type: 'checkbox', 'data-dir': b }), d));
  const colors = h('div', { class: 'led-palette-pick', id: 'led-color' });
  const paintColors = () => colors.replaceChildren(...Array.from({ length: 16 }, (_, i) => {
    const b = h('button', { type: 'button', class: 'swatch' + (i === tool.color ? ' sel' : ''), 'data-color': i, title: String(i), style: `background:${paletteRgb(i)}` });
    b.addEventListener('click', () => { tool.color = i; paintColors(); apply(); });
    return b;
  }));
  const read = () => {
    tool.fn = Number(fnSel.value);
    tool.overlays = 0; for (const c of ol) { const i = c.querySelector('input'); if (i.checked) tool.overlays |= 1 << Number(i.dataset.ol); }
    tool.dirs = 0; for (const c of dirs) { const i = c.querySelector('input'); if (i.checked) tool.dirs |= 1 << Number(i.dataset.dir); }
  };
  for (const e of [fnSel, ...ol.map((c) => c.querySelector('input')), ...dirs.map((c) => c.querySelector('input'))]) e.addEventListener('change', () => { read(); apply(); });
  const show = (f) => {            // the selected LED's settings into the tool
    fnSel.value = f.fn; for (const c of ol) { const i = c.querySelector('input'); i.checked = !!(f.overlays & (1 << Number(i.dataset.ol))); }
    for (const c of dirs) { const i = c.querySelector('input'); i.checked = !!(f.dirs & (1 << Number(i.dataset.dir))); }
    tool.fn = f.fn; tool.overlays = f.overlays; tool.dirs = f.dirs; tool.color = f.color; paintColors();
  };
  listen('settings:ledstrip', paintColors); paintColors();
  const el = h('div', { class: 'led-tools' },
    h('div', { class: 'row' }, h('span', {}, tr('Chức năng')), fnSel),
    h('div', { class: 'row' }, h('span', {}, tr('Overlay')), ...ol),
    h('div', { class: 'row' }, h('span', {}, tr('Hướng')), ...dirs),
    h('div', { class: 'row' }, h('span', {}, tr('Màu')), colors));
  return { el, show };
}

function layoutBlock() {
  const state = h('p', { class: 'help', id: 'led-state' });
  const say = () => { const n = count(cfg().leds); state.textContent = selected >= 0 ? tr('LED {i} trong {n} (theo thứ tự dây). Bấm ô trống để thêm LED tiếp theo.', { i: selected + 1, n }) : tr('{n} LED. Bấm ô trống để thêm LED tiếp theo (theo thứ tự dây), bấm một LED để chọn và sửa.', { n }); };
  const tb = toolbar(() => {        // a tool change edits the selected LED
    if (selected < 0) return;
    const f = ledFields(cfg().leds[selected]);
    setField('ledstrip', 'leds', ledDefine(f.x, f.y, tool.color, tool.dirs, tool.fn, tool.overlays), selected);
  });
  const g = grid((x, y, i) => {
    const leds = cfg().leds.slice();
    if (i !== undefined) { selected = i; tb.show(ledFields(leds[i])); say(); g.repaint(); return; }
    const n = count(leds); if (n >= MAX) return;
    leds[n] = ledDefine(x, y, tool.color, tool.dirs, tool.fn, tool.overlays);
    if (!leds[n]) { state.textContent = tr('LED này mã hoá thành 0 (= hết danh sách): đặt ở ô khác, hoặc chọn hướng / overlay / màu khác.'); return; }
    selected = n; setFields('ledstrip', { leds }); say();
  });
  const del = h('button', { type: 'button', class: 'btn', id: 'led-delete' }, tr('Xoá LED đang chọn'));
  del.addEventListener('click', () => {
    if (selected < 0) return;
    const leds = cfg().leds.slice(); leds.splice(selected, 1); leds.push(0); selected = -1; setFields('ledstrip', { leds }); say();
  });
  const clearAll = h('button', { type: 'button', class: 'btn quiet', id: 'led-clear' }, tr('Xoá hết'));
  clearAll.addEventListener('click', () => { selected = -1; setFields('ledstrip', { leds: Array(MAX).fill(0) }); say(); });
  listen('settings:ledstrip', say); say();
  return block(tr('Bố trí LED'), h('div', { class: 'cols' }, g, h('div', {}, tb.el, h('div', { class: 'btn-row' }, del, clearAll), state)),
    note(tr('Ô (0,0) là góc trên bên trái, mũi máy ở phía trên. Chữ trên LED là mã chức năng của Betaflight (C màu, F chế độ bay, A arm, L pin, S RSSI, G GPS, R vòng ga, P thanh GPS, E thanh pin, U độ cao). Một LED ở ô 0,0 chức năng Màu, màu 0, không overlay, không hướng sẽ thành số 0 = hết danh sách: đặt nó ở ô khác hoặc thêm hướng.')));
}

function paletteBlock() {
  const rows = Array.from({ length: 16 }, (_, i) => {
    const sw = h('span', { class: 'swatch' });
    const num = (name, max) => { const inp = h('input', { type: 'number', min: 0, max, 'data-hsv': name + i, 'aria-label': `${name.toUpperCase()} ${i}` });
      inp.addEventListener('change', () => setField('ledstrip', 'color_' + name, Math.max(0, Math.min(max, Number(inp.value) || 0)), i)); return inp; };
    const hh = num('h', 359), ss = num('s', 255), vv = num('v', 255);
    const paint = () => { const c = cfg(); hh.value = c.color_h[i]; ss.value = c.color_s[i]; vv.value = c.color_v[i]; sw.style.background = paletteRgb(i); };
    listen('settings:ledstrip', paint); paint();
    return h('tr', {}, h('td', {}, String(i)), h('td', {}, sw), h('td', {}, hh), h('td', {}, ss), h('td', {}, vv));
  });
  return block(tr('Bảng 16 màu'), h('table', { class: 'grid', id: 'led-palette' }, h('tr', {}, h('th', {}, '#'), h('th', {}, ''), h('th', {}, 'H'), h('th', {}, 'S'), h('th', {}, 'V')), ...rows),
    note(tr('HSV của Betaflight: H 0 – 359, S = 0 là màu đậm, 255 là trắng; V độ sáng.')));
}

function modeColorBlock() {
  const pick = (get, set, label) => {
    const sel = h('select', { 'aria-label': label }, Array.from({ length: 16 }, (_, i) => h('option', { value: i }, String(i))));
    sel.addEventListener('change', () => set(Number(sel.value)));
    const paint = () => { sel.value = get(); sel.style.borderLeft = `14px solid ${paletteRgb(Number(sel.value))}`; };
    listen('settings:ledstrip', paint); paint(); return sel;
  };
  const head = h('tr', {}, h('th', {}, ''), ...DIRECTIONS.map((d) => h('th', {}, d)));
  const rows = MODES.map((m, mi) => h('tr', {}, h('td', {}, tr(ENUM_LABELS.led_bf_mode[m] || m)),
    ...DIRECTIONS.map((_, d) => h('td', {}, pick(() => cfg().mode_colors[mi * 6 + d], (v) => setField('ledstrip', 'mode_colors', v, mi * 6 + d), `${m} ${DIRECTIONS[d]}`)))));
  const special = h('table', { class: 'grid', id: 'led-special' }, ...SPECIAL.map((sname, si) => h('tr', {}, h('td', {}, tr(ENUM_LABELS.led_bf_special[sname] || sname)),
    h('td', {}, pick(() => cfg().special[si], (v) => setField('ledstrip', 'special', v, si), sname)))));
  return block(tr('Màu theo chế độ và màu đặc biệt'), h('table', { class: 'grid', id: 'led-modes' }, head, ...rows), special,
    note(tr('Chức năng "Chế độ bay": LED lấy màu theo chế độ đang bay và theo hướng đầu tiên của nó. Màu đặc biệt: arm / chưa arm, nền, nền khi nháy, GPS.')));
}

export default {
  id: 'led',
  title: tr('LED'),
  lead: tr('Dải LED WS2812 như Betaflight: bố trí trên lưới, chức năng, overlay, bảng màu.'),
  available: () => !!(store.schema && store.schema.sections.find((x) => x.name === 'ledstrip')),
  mount(view) {
    view.append(
      block(tr('Cài đặt'), fields('ledstrip', ['profile', 'brightness', 'race_color', 'beacon_color', 'beacon_period_ms', 'beacon_percent', 'visual_beeper', 'visual_beeper_color', 'rainbow_delta', 'rainbow_freq', 'aux_channel']),
        note(tr('Số LED trên dải là số LED đặt trên lưới. Chân dữ liệu đặt ở tab Chân (LED). Có hiệu lực ngay khi chưa arm; lưu để giữ.'))),
      layoutBlock(), paletteBlock(), modeColorBlock());
  },
};
