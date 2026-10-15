// Page chrome: connection bar (port, baud, language), arm-readiness line, save buttons, tab rail. The tabs live in js/tabs/.
import { api } from './api.js';
import { store, on, loadSchema, loadSettings, reloadSection, wantStreams, pushStreams, flushAll, handleEvent } from './store.js';
import { h, clear, toast, showError, disposeScope, errorText, failsafeText } from './ui.js';
import { BLOCKERS } from './labels.js';
import { tr, LANGS, getLang, setLang } from './i18n.js';
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
import system from './tabs/system.js';

const TABS = [overview, receiver, modes, sensors, motors, tuning, rates, filters, nav, power, blackbox, pins, system];
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
const armPill = h('span', { class: 'pill off', id: 'arm-pill' }, tr('Chưa kết nối'));
const readyLine = h('span', { class: 'ready', id: 'ready' });
const chips = h('div', { class: 'chips', id: 'chips' });
const saveBtn = h('button', { type: 'button', class: 'btn primary', id: 'save', hidden: true }, tr('Lưu'));
const revertBtn = h('button', { type: 'button', class: 'btn quiet', id: 'revert', hidden: true }, tr('Bỏ thay đổi'));
const banner = h('div', { class: 'banner', id: 'banner', hidden: true });
const langSel = h('select', { id: 'lang', class: 'lang', 'aria-label': 'Ngôn ngữ / Language' }, Object.entries(LANGS).map(([k, name]) => h('option', { value: k }, name)));

function buildBar() {
  $('bar').append(
    h('div', { class: 'brand' }, h('strong', {}, 'Windburst'), fwLabel),
    h('div', { class: 'link' }, portSel, baudSel, refreshBtn, connectBtn),
    h('div', { class: 'state' }, armPill, readyLine),
    chips,
    h('div', { class: 'actions' }, revertBtn, saveBtn),
    banner);
  langSel.value = getLang();
  langSel.addEventListener('change', () => setLang(langSel.value));
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
  const what = [p.description, p.chip && !String(p.description).includes(p.chip) ? p.chip : ''].filter(Boolean).join(', ');
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
  try {
    try { localStorage.setItem('wb-port', portSel.value); } catch (e) { /* not remembered */ }
    await api.post('/api/connect', { port: portSel.value, baud: Number(baudSel.value) || 230400 });
  } catch (e) {
    lastFailure = e;
    showError(e);
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
  if (note.code === 'PORT_LOST') return tr('Mất kết nối với cổng {port} (rút cáp?)', { port: note.port });
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
  for (const tab of TABS) {
    $('rail').append(h('a', { href: '#' + tab.id, class: 'tab', id: 'tab-' + tab.id, 'data-tab': tab.id }, tab.title));
  }
  $('rail').append(h('div', { class: 'rail-foot' }, langSel));       // language: at the foot of the rail, out of the way of the status bar
}
function tabFromHash() {
  const id = location.hash.replace('#', '');
  return TABS.some((tab) => tab.id === id) ? id : TABS[0].id;
}
function show(id) {
  disposeScope();
  current = id;
  const view = clear($('view'));
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
      h('li', {}, tr('Cắm cáp USB vào mạch ESP32-S3 đã nạp firmware Windburst.')),
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
    lastFailure = null;
    refreshPorts(true);                   // a page opened on an existing connection shows the port that is in use
    settingsReady = false; show(current);
    try {
      await loadSettings();
      settingsReady = true;
      wantStreams('global', { status: 10, rc: 5 });
      await pushStreams(true);
      show(current);
    } catch (e) { showError(e); }
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
