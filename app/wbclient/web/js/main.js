// Page chrome: connection bar (port, baud, language), arm-readiness line, save buttons, tab rail. The tabs live in js/tabs/.
import { api, ApiError } from './api.js';
import { store, on, loadSchema, loadSettings, reloadSection, wantStreams, pushStreams, flushAll, handleEvent } from './store.js';
import { h, clear, toast, showError, disposeScope, errorText, failsafeText } from './ui.js';
import { BLOCKERS } from './labels.js';
import { tr, LANGS, getLang, setLang } from './i18n.js';
import { THEMES, getTheme, setTheme, ZOOMS, getZoom, setZoom, stepZoom } from './theme.js';
import { enterView } from './anim.js';
import { BUDGET, linkIsOpen, linkOpen, linkStep, linkDone, linkSuccess, linkFail } from './linkbox.js';
import overview from './tabs/overview.js';
import receiver from './tabs/receiver.js';
import modes from './tabs/modes.js';
import sensors from './tabs/sensors.js';
import motors from './tabs/motors.js';
import tuning from './tabs/tuning.js';
import rates from './tabs/rates.js';
import filters from './tabs/filters.js';
import nav from './tabs/nav.js';
import power from './tabs/power.js';
import blackbox from './tabs/blackbox.js';
import pins from './tabs/pins.js';
import osd from './tabs/osd.js';
import system from './tabs/system.js';

const TABS = [overview, receiver, modes, sensors, motors, tuning, rates, filters, nav, power, osd, blackbox, pins, system];
// the rail in the order of building a craft: put it together, tune it, what it does in the air, then its logs and the board itself
const GROUPS = [
  [tr('Lắp máy'), ['overview', 'receiver', 'modes', 'sensors', 'motors']],
  [tr('Tinh chỉnh'), ['tuning', 'rates', 'filters']],
  [tr('Khi bay'), ['nav', 'power', 'osd']],
  [tr('Log và mạch'), ['blackbox', 'pins', 'system']],
];
// one line drawing per tab (24 x 24, stroked in the text colour)
const ICONS = {
  overview: '<path d="M4 16a8 8 0 1 1 16 0"/><path d="M12 16l4.5-5"/><path d="M4 20h16"/>',
  receiver: '<path d="M12 21v-9"/><circle cx="12" cy="10" r="2"/><path d="M8.5 6.5a5 5 0 0 0 0 7M15.5 6.5a5 5 0 0 1 0 7M5.6 3.6a9 9 0 0 0 0 12.8M18.4 3.6a9 9 0 0 1 0 12.8"/>',
  modes: '<rect x="3" y="4.5" width="18" height="6.5" rx="3.25"/><circle cx="16.5" cy="7.75" r="1.7"/><rect x="3" y="13" width="18" height="6.5" rx="3.25"/><circle cx="7.5" cy="16.25" r="1.7"/>',
  sensors: '<rect x="7" y="7" width="10" height="10" rx="1.5"/><path d="M10 3v4M14 3v4M10 17v4M14 17v4M3 10h4M3 14h4M17 10h4M17 14h4"/>',
  motors: '<circle cx="12" cy="12" r="2.2"/><path d="M12 9.8C11 6 12 3 14.5 3S17 6.5 13.6 10.4M14 13.3c3.4 1.6 5.7 3.8 4.4 5.8s-4.7.3-6.2-4.3M10 13.4c-3 2.6-6.1 3.2-6.9 1s1.8-4.4 6.8-4.6"/>',
  tuning: '<path d="M5 4v16M12 4v16M19 4v16"/><rect x="3" y="13" width="4" height="3.2" rx="1"/><rect x="10" y="7" width="4" height="3.2" rx="1"/><rect x="17" y="15" width="4" height="3.2" rx="1"/>',
  rates: '<path d="M4 4v16h16"/><path d="M4 19c7 0 10-3 12-8s2.5-6 4-7"/>',
  filters: '<path d="M3 12c2-6 4-6 6 0s4 6 6 0 4-6 6 0"/><path d="M3 20h18"/>',
  nav: '<path d="M12 21s-6.5-5.6-6.5-10.5a6.5 6.5 0 0 1 13 0C18.5 15.4 12 21 12 21z"/><circle cx="12" cy="10.5" r="2.3"/>',
  power: '<rect x="3" y="7" width="16" height="10" rx="2"/><path d="M21 10.5v3M7 10v4M10.5 10v4"/>',
  osd: '<rect x="3" y="5" width="18" height="13" rx="2"/><path d="M6.5 9h4M13.5 14.5h4M6.5 14.5h2.5"/>',
  blackbox: '<path d="M3 4v16h18"/><path d="M6.5 15l4-5 3 3 5-7"/>',
  pins: '<rect x="4" y="9" width="16" height="7" rx="1.5"/><path d="M8 9V4.5M12 9V4.5M16 9V4.5M8 16v3.5M16 16v3.5"/>',
  system: '<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M7 9l3 3-3 3M12.5 15H17"/>',
};
const $ = (id) => document.getElementById(id);
let current = null;          // id of the tab on screen
let settingsReady = false;
let busy = false;            // connect / save / reboot in progress
let lastFailure = null;      // why the last "Kết nối" did not work (shown on the welcome screen until the next attempt)

// ------------------------------------------------------------------ connection bar
const portSel = h('select', { id: 'port', 'aria-label': tr('Cổng nối với flight controller') });
const baudSel = h('select', { id: 'baud', 'aria-label': tr('Tốc độ baud'), title: tr('Tốc độ baud của cổng UART. Mạch dùng USB gốc của ESP32-S3 thì chọn số nào cũng được.') });
const connectBtn = h('button', { type: 'button', class: 'btn primary', id: 'connect' }, tr('Kết nối'));
const refreshBtn = h('button', { type: 'button', class: 'btn quiet', id: 'refresh', title: tr('Tìm lại các cổng') }, tr('Tìm cổng'));
const fwLabel = h('span', { class: 'fw', id: 'fw' });
const mcuBadge = h('span', { class: 'mcu-badge', id: 'mcu', hidden: true, title: tr('Vi điều khiển (MCU) của flight controller') });
const armPill = h('span', { class: 'pill off', id: 'arm-pill' }, tr('Chưa kết nối'));
const readyLine = h('span', { class: 'ready', id: 'ready' });
const chips = h('div', { class: 'chips', id: 'chips' });
const saveBtn = h('button', { type: 'button', class: 'btn primary', id: 'save', hidden: true }, tr('Lưu'));
const revertBtn = h('button', { type: 'button', class: 'btn quiet', id: 'revert', hidden: true }, tr('Bỏ thay đổi'));
const banner = h('div', { class: 'banner', id: 'banner', hidden: true });
const langSel = h('select', { id: 'lang', class: 'lang', 'aria-label': 'Ngôn ngữ / Language' }, Object.entries(LANGS).map(([k, name]) => h('option', { value: k }, name)));
const themeSel = h('select', { id: 'theme', class: 'lang', 'aria-label': tr('Giao diện') }, Object.entries(THEMES).map(([k, name]) => h('option', { value: k }, name)));
const zoomSel = h('select', { id: 'zoom', class: 'lang', 'aria-label': tr('Cỡ giao diện'), title: tr('Cỡ giao diện (Ctrl + / Ctrl - / Ctrl 0, hoặc Ctrl + lăn chuột)') }, ZOOMS.map((z) => h('option', { value: z }, z + ' %')));

function buildBar() {
  $('bar').append(
    h('div', { class: 'brand' }, h('strong', {}, 'Windburst'), fwLabel, mcuBadge),
    h('div', { class: 'bar-main' },
      h('div', { class: 'link' }, portSel, baudSel, refreshBtn, connectBtn),
      h('div', { class: 'state' }, armPill, readyLine),
      chips,
      h('div', { class: 'actions' }, revertBtn, saveBtn),
      banner));
  langSel.value = getLang();
  langSel.addEventListener('change', () => setLang(langSel.value));
  themeSel.value = getTheme();
  themeSel.addEventListener('change', () => setTheme(themeSel.value));
  zoomSel.value = String(getZoom());
  // Applied once more after the page is up: WebView2 (the desktop window) works out 100vh before the zoom set in index.html
  // counts, and keeps that height until something changes the zoom - the frame came out too tall at start-up.
  const rezoom = () => { const z = getZoom(); if (z !== 100) { const st = document.documentElement.style; st.removeProperty('zoom'); void document.body.offsetHeight; setZoom(z); } };
  requestAnimationFrame(() => requestAnimationFrame(rezoom)); window.addEventListener('load', rezoom, { once: true });
  zoomSel.addEventListener('change', () => setZoom(Number(zoomSel.value)));
  const zoomBy = (dir) => { zoomSel.value = String(setZoom(dir ? stepZoom(getZoom(), dir) : 100)); };
  window.addEventListener('keydown', (e) => {
    if (!(e.ctrlKey || e.metaKey) || e.altKey) return;
    if (e.key === '+' || e.key === '=') zoomBy(1); else if (e.key === '-' || e.key === '_') zoomBy(-1); else if (e.key === '0') zoomBy(0); else return;
    e.preventDefault();
  });
  window.addEventListener('wheel', (e) => { if (!e.ctrlKey) return; e.preventDefault(); zoomBy(e.deltaY < 0 ? 1 : -1); }, { passive: false });
  refreshBtn.addEventListener('click', () => refreshPorts());
  portSel.addEventListener('change', pickCustom);
  baudSel.addEventListener('change', () => { try { localStorage.setItem('wb-baud', baudSel.value); } catch (e) { /* not remembered */ } });
  connectBtn.addEventListener('click', () => (store.conn.connected ? disconnect() : connect()));
  saveBtn.addEventListener('click', save);
  revertBtn.addEventListener('click', revert);
}

// ------------------------------------------------------------------ ports
let customPort = '';       // a port typed by hand: an unusual device name, or socket://host:port for a simulator on another port
let portsKey = '';
function portLabel(p) {
  if (p.sim) return tr('Mô phỏng wb_sim (127.0.0.1:5760)');
  if (p.wifi) return tr('WiFi của máy bay (điểm phát riêng, 192.168.4.1:5760)');
  // the name the device gives itself (USB iProduct) before the driver's ("USB Serial Device (COM5)")
  const name = p.product || p.description;
  const what = [name, p.chip && !String(name).includes(p.chip) ? p.chip : ''].filter(Boolean).join(', ');
  return p.port + (what ? ' - ' + what : '');
}
function pickCustom() {
  if (portSel.value !== '__custom__') return;
  const typed = (window.prompt(tr('Tên cổng (ví dụ COM7, /dev/ttyACM0, hoặc socket://127.0.0.1:5760):'), customPort) || '').trim();
  if (typed) customPort = typed;
  refreshPorts(true).then(() => { if (typed) portSel.value = typed; });
}
function saved(key) { try { return localStorage.getItem(key) || ''; } catch (e) { return ''; } }

// quiet = the periodic look for newly plugged boards: it must not disturb a list the user is reading, nor report errors
async function refreshPorts(force, quiet) {
  let res;
  try { res = await api.get('/api/ports'); } catch (e) { if (!quiet) showError(e); return; }
  if (!baudSel.options.length) {
    for (const b of res.bauds) baudSel.append(h('option', { value: b }, b + ' baud'));
    const want = saved('wb-baud') || String(res.default_baud);
    baudSel.value = [...baudSel.options].some((o) => o.value === want) ? want : String(res.default_baud);
  }
  if (store.conn.connected && store.conn.baud && [...baudSel.options].some((o) => o.value === String(store.conn.baud))) baudSel.value = String(store.conn.baud);
  const key = JSON.stringify([res.ports, store.conn.port, customPort]);
  if (key === portsKey && !force) return;
  if (quiet && document.activeElement === portSel) return;
  portsKey = key;
  const keep = portSel.value && portSel.value !== '__custom__' ? portSel.value : saved('wb-port');
  clear(portSel);
  const real = res.ports.filter((p) => !p.wifi);           // (the board's WiFi access point is always listed: it is never the default)
  if (!real.length) portSel.append(h('option', { value: '' }, tr('Không thấy cổng nào')));
  for (const p of res.ports) portSel.append(h('option', { value: p.port, 'data-likely': p.likely ? '1' : null }, portLabel(p)));
  for (const extra of [store.conn.port, customPort]) if (extra && ![...portSel.options].some((o) => o.value === extra)) portSel.append(h('option', { value: extra }, extra));
  portSel.append(h('option', { value: '__custom__' }, tr('Nhập cổng khác…')));
  const has = (v) => v && [...portSel.options].some((o) => o.value === v);
  const likely = res.ports.find((p) => p.likely);
  const pick = has(store.conn.port) ? store.conn.port : has(keep) ? keep : likely ? likely.port : real.length ? real[0].port : '';
  portSel.value = pick;
}

async function connect() {
  if (!portSel.value || portSel.value === '__custom__') { toast(tr('Chưa có cổng nào để kết nối. Cắm cáp USB rồi bấm "Tìm cổng".'), 'warn'); return; }
  busy = true; lastFailure = null; paintBar();
  const port = portSel.value;
  linkOpen(port);
  // (a bar that runs out is shown at once; the server's own answer follows and gives the reason)
  linkStep('find', tr('Đang tìm thiết bị trên {port}…', { port }), BUDGET.find, () => linkFail(tr('Hết thời gian: không có firmware Windburst nào trả lời trên cổng này.'), true));
  try {
    try { localStorage.setItem('wb-port', port); } catch (e) { /* not remembered */ }
    await api.post('/api/connect', { port, baud: Number(baudSel.value) || 115200 });
  } catch (e) {
    lastFailure = e;
    linkFail(errorText(e), e.code === 'NO_ANSWER');
    if (!store.conn.connected) show(current);
  }
  busy = false; paintBar();
}
async function disconnect() {
  busy = true; paintBar();
  try { await api.post('/api/disconnect'); } catch (e) { showError(e); }
  busy = false; paintBar();
}

async function save() {
  const flags = (store.streams.status || {}).settings_dirty || [];
  const reboot = flags.includes('REBOOT_REQUIRED');
  busy = true; paintBar();
  try {
    await flushAll();
    await api.post('/api/save');
    if (reboot) {
      toast(tr('Đã lưu. Đang khởi động lại flight controller…'));
      await api.post('/api/reboot');
      toast(tr('Đã lưu và khởi động lại.'), 'ok');
    } else toast(tr('Đã lưu vào flash.'), 'ok');
  } catch (e) { showError(e); }
  busy = false; paintBar();
}
async function revert() {
  if (!window.confirm(tr('Bỏ mọi thay đổi chưa lưu? Flight controller sẽ khởi động lại và nạp lại cấu hình trong flash.'))) return;
  busy = true; paintBar();
  try { await api.post('/api/reboot'); toast(tr('Đã nạp lại cấu hình trong flash.'), 'ok'); } catch (e) { showError(e); }
  busy = false; paintBar();
}

function chip(label, value, kind = '') { return h('span', { class: 'chip ' + kind }, h('span', { class: 'chip-k' }, label), h('span', { class: 'chip-v num' }, value)); }

// The server says WHY the link is in its state as a code; the words are chosen here.
function noteText(note) {
  if (!note) return '';
  if (note.code === 'PORT_LOST' && note.waiting) return tr('Mất kết nối với cổng {port}, đang chờ cổng quay lại (mạch đang khởi động lại?)', { port: note.port });
  if (note.code === 'PORT_LOST') return tr('Mất kết nối với cổng {port} (rút cáp?)', { port: note.port });
  if (note.code === 'RECONNECTED') return tr('Cổng {port} đã quay lại: đã kết nối lại', { port: note.port });
  if (note.code === 'NOT_RESPONDING') return tr('Firmware không trả lời (đang khởi động lại, treo, hoặc mất nguồn?)');
  if (note.code === 'REBOOTED') return tr('Flight controller vừa khởi động lại');
  if (note.code === 'STALLED') return tr('Dữ liệu từ firmware ngừng {s} giây nhưng firmware KHÔNG khởi động lại (chi tiết: tab Hệ thống, Nhật ký firmware)', { s: note.seconds });
  return note.code || '';
}

function paintBar() {
  const c = store.conn, st = store.streams.status;
  connectBtn.textContent = busy ? tr('Đang xử lý…') : (c.connected ? tr('Ngắt kết nối') : tr('Kết nối'));
  connectBtn.classList.toggle('primary', !c.connected);
  connectBtn.disabled = busy;
  portSel.disabled = baudSel.disabled = c.connected || busy;
  refreshBtn.disabled = c.connected || busy;
  fwLabel.textContent = c.info ? `${c.info.name} ${c.info.version}` : '';
  // which chip: the firmware says it from 0.25 on; before, only "STM32" or not
  const mcu = c.info ? (c.info.mcu || ((c.info.features || []).includes('STM32') ? 'STM32' : 'ESP32')) : '';
  mcuBadge.textContent = mcu; mcuBadge.hidden = !mcu; mcuBadge.dataset.mcu = mcu;
  clear(chips);
  let bannerText = '';
  if (!store.serverUp) bannerText = tr('Mất liên lạc với chương trình wbclient trên máy này. Chạy lại "python -m wbclient gui" rồi tải lại trang.');
  else if (c.connected && !c.responding) bannerText = noteText(c.note) || tr('Firmware không trả lời.');
  else if (!c.connected && c.note) bannerText = noteText(c.note);
  else if (c.connected && store.armed) bannerText = tr('Đang ARM. Firmware khóa mọi thay đổi cấu hình cho tới khi disarm.');
  banner.hidden = !bannerText; banner.textContent = bannerText;
  banner.classList.toggle('armed', c.connected && store.armed && store.serverUp && c.responding);

  if (!c.connected) {
    armPill.className = 'pill off'; armPill.textContent = tr('Chưa kết nối');
    readyLine.textContent = ''; saveBtn.hidden = true; revertBtn.hidden = true;
    return;
  }
  if (!st) { armPill.className = 'pill off'; armPill.textContent = tr('Đang chờ dữ liệu'); readyLine.textContent = ''; return; }
  if (st.armed) {
    armPill.className = 'pill bad'; armPill.textContent = 'ARMED';
    readyLine.textContent = st.failsafe !== 'NONE' ? failsafeText(st) : (st.modes.filter((m) => m !== 'ARM').join(', ') || 'Acro');
  } else if (st.blockers.length) {
    armPill.className = 'pill warn'; armPill.textContent = tr('Chưa arm được');
    readyLine.textContent = st.blockers.map((b) => (BLOCKERS[b] ? tr(BLOCKERS[b]) : b)).join('; ');
  } else {
    armPill.className = 'pill ok'; armPill.textContent = tr('Sẵn sàng arm');
    readyLine.textContent = '';
  }
  readyLine.title = readyLine.textContent;
  const rc = store.streams.rc;
  const rxOk = st.health.includes('RX');
  chips.append(chip('Receiver', rxOk ? (rc ? tr('{n} khung/s', { n: rc.rate }) : tr('có sóng')) : tr('mất sóng'), rxOk ? '' : 'bad'));
  chips.append(chip(tr('Pin'), st.cell_mv ? (st.cell_mv / 1000).toFixed(2) + ' V/cell' : tr('không đo'), ''));
  // below 80 % of the configured rate the firmware refuses to arm (blocker LOOP_RATE): show it before the pilot asks why
  const slow = st.loop_hz && st.loop_target && st.loop_hz < 0.8 * st.loop_target;
  const pid = st.pid_target && st.pid_target !== st.loop_target ? ` · PID ${st.pid_hz}` : '';      // (only when the PID runs slower than the gyro)
  chips.append(chip(tr('Vòng lặp'), st.loop_hz ? (st.loop_target ? `${st.loop_hz} / ${st.loop_target} Hz${pid}, ${st.loop_max_us} µs` : `${st.loop_hz} Hz, ${st.loop_max_us} µs`) : tr('đang đo'), slow ? 'bad' : ''));
  const dirty = st.settings_dirty.includes('DIRTY'), reboot = st.settings_dirty.includes('REBOOT_REQUIRED');
  saveBtn.hidden = !dirty && !reboot; revertBtn.hidden = !dirty;
  saveBtn.textContent = reboot ? tr('Lưu và khởi động lại') : tr('Lưu');
  saveBtn.disabled = busy || st.armed; revertBtn.disabled = busy || st.armed;
}

// ------------------------------------------------------------------ tabs
function buildRail() {
  const placed = new Set();
  const add = (tab) => {
    const a = h('a', { href: '#' + tab.id, class: 'tab', id: 'tab-' + tab.id, 'data-tab': tab.id }, h('span', {}, tab.title));
    if (ICONS[tab.id]) a.insertAdjacentHTML('afterbegin', `<svg viewBox="0 0 24 24" aria-hidden="true">${ICONS[tab.id]}</svg>`);
    $('rail').append(a); placed.add(tab.id);
  };
  for (const [name, ids] of GROUPS) {
    const tabs = ids.map((id) => TABS.find((t) => t.id === id)).filter(Boolean);
    if (!tabs.length) continue;
    $('rail').append(h('div', { class: 'rail-group' }, name));
    tabs.forEach(add);
  }
  TABS.filter((t) => !placed.has(t.id)).forEach(add);          // (a tab nobody put in a group still gets its place)
  $('rail').append(h('div', { class: 'rail-foot' }, themeSel, zoomSel, langSel));       // language: at the foot of the rail, out of the way of the status bar
}
function tabFromHash() {
  const id = location.hash.replace('#', '');
  return TABS.some((tab) => tab.id === id) ? id : TABS[0].id;
}
function show(id) {
  disposeScope();
  // which way the new tab lies on the rail from the old one: it slides in from that side (anim.js)
  const order = [...$('rail').querySelectorAll('a.tab')].map((a) => a.dataset.tab), dir = current ? Math.sign(order.indexOf(id) - order.indexOf(current)) : 0;
  current = id;
  const view = clear($('view')); view.scrollTop = 0;
  enterView(view, dir);
  for (const a of $('rail').querySelectorAll('a.tab')) { a.classList.toggle('active', a.dataset.tab === id); a.setAttribute('aria-current', a.dataset.tab === id ? 'page' : 'false'); }
  const tab = TABS.find((x) => x.id === id);
  document.title = `${tab.title} - Windburst`;
  if (!store.conn.connected && !tab.offline) { wantStreams('tab', null); view.append(connectHelp()); return; }
  if (store.conn.connected && !settingsReady) { view.append(h('p', { class: 'note' }, tr('Đang đọc cấu hình từ flight controller…'))); return; }
  view.append(h('h1', {}, tab.title), tab.lead ? h('p', { class: 'lead' }, tab.lead) : null);
  try { tab.mount(view); } catch (e) { console.error(e); view.append(h('p', { class: 'note bad' }, tr('Lỗi khi dựng tab: ') + e.message)); }
  wantStreams('tab', (store.conn.connected && tab.streams) || null);
}

// What to try when "Kết nối" failed, with what the port itself sent (boot text, a crash loop, or noise at the wrong baud rate).
function failureHelp(e) {
  const x = e.extra || {};
  const tips = [];
  if (e.code === 'OPEN_FAILED') {
    tips.push(tr('Cổng đang bị chương trình khác giữ? Đóng serial monitor của PlatformIO / Arduino IDE, hoặc cửa sổ configurator khác.'));
    tips.push(tr('Rút cáp, cắm lại, rồi bấm "Tìm cổng": tên cổng có thể đã đổi.'));
  } else if (e.code === 'NO_ANSWER') {
    tips.push(tr('Chọn đúng cổng của flight controller chưa? Cổng có chip Espressif / CP210x / CH34x được xếp lên đầu danh sách.'));
    tips.push(tr('Mạch dùng cầu USB-UART thì baud phải khớp với firmware (mặc định 230400). Thử baud khác ở ô bên cạnh ô cổng.'));
    tips.push(tr('Mạch đã nạp firmware Windburst chưa? Nếu vừa nạp xong, bấm nút reset rồi kết nối lại.'));
  } else if (e.code === 'SCHEMA_MISMATCH') {
    tips.push(tr('Nạp firmware build từ đúng thư mục dự án này (cùng protocol/wbp_schema.json), hoặc dùng app đi kèm bản firmware đó.'));
  }
  return h('div', { class: 'failure', id: 'failure' },
    h('h2', {}, tr('Lần kết nối vừa rồi không được')),
    h('p', { class: 'note bad' }, errorText(e)),
    tips.length ? h('ul', {}, tips.map((x2) => h('li', {}, x2))) : null,
    x.raw ? [h('p', { class: 'help' }, tr('Cổng đã gửi về những dữ liệu sau (không phải khung WBP). Chữ đọc được là log khởi động hoặc lỗi của ESP32; toàn ký tự lạ thường là sai baud.')),
      h('pre', { class: 'log', id: 'failure-raw' }, x.raw)] : (e.code === 'NO_ANSWER' ? h('p', { class: 'help' }, tr('Cổng không gửi về byte nào.')) : null));
}

function connectHelp() {
  return h('div', { class: 'welcome' },
    h('h1', {}, tr('Nối flight controller để bắt đầu')),
    h('ol', {},
      h('li', {}, tr('Cắm cáp USB vào MCU đã nạp firmware Windburst.')),
      h('li', {}, tr('Bấm "Tìm cổng", chọn cổng COM của mạch ở thanh trên cùng, rồi bấm "Kết nối".')),
      h('li', {}, tr('Tab Tổng quan cho biết firmware tìm thấy cảm biến nào và vì sao chưa arm được.'))),
    lastFailure ? failureHelp(lastFailure) : null,
    h('p', { class: 'help' }, tr('Chưa có mạch? Chạy build/wb_sim (firmware thật trên mạch ảo), bấm "Tìm cổng" và chọn "Mô phỏng wb_sim".')),
    h('p', { class: 'help' }, tr('Đường truyền là nhị phân nên không dùng chung được với serial monitor: đóng serial monitor trước khi kết nối.')));
}

// ------------------------------------------------------------------ events
async function onConn({ before }) {
  const c = store.conn;
  const fresh = c.connected && (!before.connected || before.boots !== c.boots || before.port !== c.port);
  if (!c.connected) { settingsReady = false; paintBar(); show(current); refreshPorts(true); return; }
  paintBar();
  if (before.connected && (c.stalls || 0) > (before.stalls || 0)) toast(noteText(c.note), 'warn');
  if (fresh) {
    if (c.note && c.note.code === 'RECONNECTED') toast(noteText(c.note), 'ok');
    lastFailure = null;
    refreshPorts(true);                   // a page opened on an existing connection shows the port that is in use
    settingsReady = false; show(current);
    // (also when the page opens on a link that is already up, or the port came back by itself: the box starts at "connected")
    if (!linkIsOpen()) linkOpen(c.port);
    linkDone('find', tr('Đã tìm thấy thiết bị'));
    linkDone('link', c.info ? tr('Đã kết nối: {fw}', { fw: `${c.info.name} ${c.info.version}` }) : tr('Đã kết nối'));
    let late = 0;
    linkStep('read', tr('Đang đọc cấu hình…'), BUDGET.read);
    try {
      await Promise.race([loadSettings(), new Promise((_, no) => { late = setTimeout(() => no(new ApiError('READ_TIMEOUT', tr('Hết thời gian: không đọc được cấu hình từ thiết bị.'))), BUDGET.read); })]);
      clearTimeout(late);
      settingsReady = true;
      linkSuccess(tr('Đã đọc cấu hình'));
      wantStreams('global', { status: 10, rc: 5 });
      await pushStreams(true);
      show(current);
    } catch (e) {
      clearTimeout(late);
      if (!store.conn.connected) { linkFail(errorText(e)); return; }
      // a board whose settings can not be read is of no use to the page: let go of it, the welcome screen says why
      lastFailure = e;
      linkFail(errorText(e), e.code === 'READ_TIMEOUT');
      disconnect();
    }
  }
}

async function start() {
  document.documentElement.lang = getLang();
  buildBar(); buildRail();
  try { await loadSchema(); } catch (e) { $('view').append(h('p', { class: 'note bad' }, errorText(e))); return; }
  current = tabFromHash();
  on('conn', onConn);
  on('stream:status', paintBar);
  on('stream:rc', paintBar);
  on('armed', paintBar);
  on('error', showError);
  // the firmware changed a section by itself and wrote it to flash (in-flight level trim): show what it holds now
  on('log', (rows) => {
    for (const [, text] of rows) {
      const m = /^settings: section (\w+) written by the firmware/.exec(text || '');
      if (m && store.settings[m[1]]) { reloadSection(m[1]).catch(() => {}); toast(tr('Firmware vừa tự lưu phần "{name}" (level trim khi bay).', { name: m[1] }), 'ok'); }
    }
  });
  window.addEventListener('hashchange', () => show(tabFromHash()));
  await refreshPorts(true);
  show(current);
  api.events(handleEvent, (up) => { if (up !== store.serverUp) { store.serverUp = up; paintBar(); } });
  // a board plugged in later shows up by itself
  setInterval(() => { if (!store.conn.connected && !busy && !document.hidden) refreshPorts(false, true); }, 2500);
}
start();
