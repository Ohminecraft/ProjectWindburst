// Blackbox: what the flight log records and where; the log memory of the craft (download, erase); recording a live stream over
// the cable; the logs kept on this computer; and the charts. The charts also work with no flight controller connected.
import { api } from '../api.js';
import { store, setField, command } from '../store.js';
import { h, clear, block, fields, listen, note, toast, showError, track, token } from '../ui.js';
import { ENUM_LABELS, BB_FIELDS, LOG_END_REASONS, AXES } from '../labels.js';
import { tr } from '../i18n.js';
import { logPlot, sampleAt } from '../gfx/logplot.js';

const LIVE = '@live';
const STATE_PILL = { OFF: 'off', NO_STORAGE: 'bad', NEEDS_ERASE: 'warn', IDLE: 'ok', LOGGING: 'ok', STREAMING: 'ok', ERASING: 'warn', FULL: 'warn' };
const FOLLOW_SECONDS = 10;
let lastOpen = null;          // { file, index }: the log on screen, shown again when the tab is rebuilt (connecting rebuilds it)

export function bytesText(n) {
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(n < 10240 ? 1 : 0)} kB`;
  return `${(n / 1048576).toFixed(n < 10485760 ? 2 : 1)} MB`;
}
export function spanText(sec) {
  if (!(sec >= 0)) return '–';
  if (sec < 60) return tr('{s} giây', { s: sec < 10 ? sec.toFixed(1) : Math.round(sec) });
  return tr('{m} phút {s} giây', { m: Math.floor(sec / 60), s: Math.round(sec % 60) });
}
function remember(key, value) { try { localStorage.setItem(key, value); } catch (e) { /* not remembered */ } }
function recall(key, fallback) { try { return localStorage.getItem(key) || fallback; } catch (e) { return fallback; } }

// ------------------------------------------------------------------ chart panels a log can show
// Every panel has one unit. A preset is offered when the log holds at least one of its fields.
export function presetsFor(names, motors) {
  const has = (n) => names.includes(n);
  const pick = (series) => series.filter((s) => has(s.field));
  const perAxis = (id, title, unit, min, make) => AXES.map((ax, i) => ({ id: `${id}-${i}`, title: `${title} ${ax}`, unit, min, series: pick(make(i)) }));
  const motorSeries = (field) => Array.from({ length: motors }, (_, i) => ({ field: `${field}[${i}]`, name: tr('Motor {n}', { n: i + 1 }) }));
  const list = [
    { id: 'rate', label: tr('Gyro và setpoint'), panels: perAxis('rate', tr('Tốc độ quay'), '°/s', 40, (i) => [{ field: `setpoint[${i}]`, name: 'Setpoint' }, { field: `gyro[${i}]`, name: 'Gyro' }]) },
    { id: 'pid', label: 'PID', panels: perAxis('pid', 'PID', '', 20, (i) => [{ field: `pid_p[${i}]`, name: 'P' }, { field: `pid_i[${i}]`, name: 'I' }, { field: `pid_d[${i}]`, name: 'D' }, { field: `pid_f[${i}]`, name: 'FF' }]) },
    { id: 'motors', label: 'Motor', panels: [{ id: 'motors', title: tr('Lệnh ra motor (0 đến 1)'), unit: '', range: [0, 1], series: pick(motorSeries('motor')) }] },
    { id: 'rc', label: tr('Cần điều khiển'), panels: [{ id: 'rc', min: 0.2, title: tr('Cần điều khiển'), unit: '', series: pick([{ field: 'stick[0]', name: 'Roll' }, { field: 'stick[1]', name: 'Pitch' }, { field: 'stick[2]', name: 'Yaw' },
      { field: 'stick[3]', name: tr('Cần ga') }, { field: 'throttle', name: tr('Ga sau AltHold / failsafe') }]) }] },
    { id: 'attitude', label: tr('Góc nghiêng'), panels: [{ id: 'att', min: 4, title: tr('Góc nghiêng'), unit: '°', series: pick([{ field: 'roll', name: 'Roll' }, { field: 'pitch', name: 'Pitch' }]) },
      { id: 'yaw', min: 10, title: tr('Hướng mũi (yaw)'), unit: '°', series: pick([{ field: 'yaw', name: 'Yaw' }]) }] },
    { id: 'noise', label: tr('Gyro trước và sau lọc'), panels: perAxis('noise', tr('Gyro'), '°/s', 40, (i) => [{ field: `gyro_raw[${i}]`, name: tr('Trước lọc') }, { field: `gyro[${i}]`, name: tr('Sau lọc') }]), needs: 'gyro_raw[0]' },
    { id: 'acc', label: 'Accelerometer', panels: [{ id: 'acc', min: 0.2, title: 'Accelerometer', unit: 'g', series: pick([{ field: 'acc[0]', name: 'X' }, { field: 'acc[1]', name: 'Y' }, { field: 'acc[2]', name: 'Z' }]) }] },
    { id: 'rpm', label: 'RPM', panels: [{ id: 'rpm', min: 1000, title: tr('Tốc độ motor'), unit: 'rpm', series: pick(motorSeries('rpm')) }] },
    { id: 'timing', label: tr('Vòng lặp'), panels: [{ id: 'timing', min: 100, title: tr('Thời gian vòng lặp'), unit: 'µs', series: pick([{ field: 'loop_us', name: tr('Thời gian chạy') }, { field: 'gap_us', name: tr('Khoảng cách giữa hai vòng') }]) },
      { id: 'stages', min: 100, title: tr('Từng khâu của một chu kỳ (lớn nhất giữa hai bản ghi)'), unit: 'µs', series: pick([{ field: 't_lock', name: tr('Chờ khoá') }, { field: 't_imu', name: tr('Đọc IMU') },
        { field: 't_core', name: tr('Tính toán, chu kỳ thường') }, { field: 't_est', name: tr('Tính toán, chu kỳ có bộ ước lượng') }, { field: 't_motor', name: tr('Gửi motor') }]) }] },
    { id: 'alt', label: tr('Độ cao'), panels: [{ id: 'alt', min: 1, title: tr('Độ cao'), unit: 'm', series: pick([{ field: 'est_z', name: tr('Ước lượng') }, { field: 'baro_alt', name: 'Barometer' }, { field: 'range', name: 'Lidar' }]) },
      { id: 'vz', min: 0.5, title: tr('Tốc độ lên xuống'), unit: 'm/s', series: pick([{ field: 'est_vz', name: tr('Ước lượng') }]) }] },
    { id: 'battery', label: tr('Pin'), panels: [{ id: 'vbat', min: 0.5, title: tr('Điện áp pin'), unit: 'V', series: pick([{ field: 'vbat', name: tr('Cả pin') }]) },
      { id: 'cell', min: 0.2, title: tr('Mỗi cell'), unit: 'V', series: pick([{ field: 'cell', name: tr('Mỗi cell') }]) },
      { id: 'current', min: 1, title: tr('Dòng'), unit: 'A', series: pick([{ field: 'current', name: tr('Dòng') }]) },
      { id: 'mah', min: 1, title: tr('Đã dùng'), unit: 'mAh', series: pick([{ field: 'mah', name: tr('Đã dùng') }]) }] },
    { id: 'gps', label: 'GPS', panels: [{ id: 'gspd', min: 1, title: tr('Tốc độ GPS'), unit: 'm/s', series: pick([{ field: 'gps_speed', name: tr('Tốc độ') }]) },
      { id: 'home', min: 5, title: tr('Khoảng cách tới home'), unit: 'm', series: pick([{ field: 'home_dist', name: tr('Khoảng cách') }]) },
      { id: 'sats', min: 4, title: tr('Số vệ tinh'), unit: '', series: pick([{ field: 'gps_sats', name: tr('Vệ tinh') }]) }] },
    { id: 'nav', label: tr('Vị trí'), panels: [
      { id: 'navvel', min: 0.5, title: tr('Vận tốc trên mặt đất'), unit: 'm/s', series: pick([{ field: 'vel_n', name: tr('Ước lượng, bắc') }, { field: 'vel_e', name: tr('Ước lượng, đông') }, { field: 'flow_fwd', name: tr('Optical flow, tiến') }, { field: 'flow_right', name: tr('Optical flow, sang phải') }]) },
      { id: 'navpos', min: 2, title: tr('Vị trí và điểm đang giữ'), unit: 'm', series: pick([{ field: 'pos_n', name: tr('Vị trí, bắc') }, { field: 'pos_e', name: tr('Vị trí, đông') }, { field: 'nav_target_n', name: tr('Điểm giữ, bắc') }, { field: 'nav_target_e', name: tr('Điểm giữ, đông') }]) },
      { id: 'navlean', min: 4, title: tr('Góc nghiêng autopilot yêu cầu'), unit: '°', series: pick([{ field: 'nav_roll', name: 'Roll' }, { field: 'nav_pitch', name: 'Pitch' }]) },
      { id: 'flowq', title: tr('Chất lượng bề mặt (optical flow)'), unit: '%', range: [0, 100], series: pick([{ field: 'flow_quality', name: tr('Chất lượng') }]) }] },
    { id: 'link', label: 'Receiver', panels: [{ id: 'rssi', title: 'RSSI', unit: '%', range: [0, 100], series: pick([{ field: 'rssi', name: 'RSSI' }]) },
      { id: 'rcrate', min: 20, title: tr('Khung receiver mỗi giây'), unit: 'Hz', series: pick([{ field: 'rc_rate', name: tr('Khung/s') }]) }] },
    { id: 'mag', label: tr('La bàn'), panels: [{ id: 'mag', min: 0.1, title: tr('La bàn'), unit: 'gauss', series: pick([{ field: 'mag[0]', name: 'X' }, { field: 'mag[1]', name: 'Y' }, { field: 'mag[2]', name: 'Z' }]) }] },
  ];
  for (const p of list) p.panels = p.panels.filter((x) => x.series.length);
  return list.filter((p) => p.panels.length && (!p.needs || has(p.needs)));
}

// One event of the log as text. `short` = the word drawn on the chart ('' = not worth a marker there).
export function eventText(name, data, enums) {
  const one = (list) => (list && list[data[0]] !== undefined ? list[data[0]] : String(data[0]));
  const u32 = () => (data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)) >>> 0;
  if (name === 'ARM') return { text: 'ARM', short: 'ARM' };
  if (name === 'DISARM') return { text: 'DISARM', short: 'DISARM' };
  if (name === 'FAILSAFE') { const v = one(enums.failsafe); return { text: `Failsafe: ${v}`, short: v === 'NONE' ? '' : `FAILSAFE ${v}` }; }
  if (name === 'MODES') { const bits = u32(); const on = enums.mode.filter((m, i) => (bits >> i) & 1); return { text: tr('Mode: {list}', { list: on.join(', ') || tr('không có') }), short: '' }; }
  if (name === 'BATTERY') { const v = one(enums.batt_state); const label = (ENUM_LABELS.batt_state || {})[v]; return { text: tr('Pin: {state}', { state: label ? tr(label) : v }), short: v === 'WARNING' || v === 'CRITICAL' ? tr('PIN {state}', { state: v }) : '' }; }
  if (name === 'AH_STATE') return { text: `Altitude Hold: ${one(enums.ah_state)}`, short: '' };
  if (name === 'DROPPED') { const n = u32(); return { text: tr('Firmware bỏ {n} bản ghi (hàng đợi đầy)', { n }), short: tr('MẤT {n}', { n }) }; }
  if (name === 'END') return { text: tr('Kết thúc: {why}', { why: LOG_END_REASONS[data[0]] ? tr(LOG_END_REASONS[data[0]]) : String(data[0]) }), short: '' };
  return { text: name, short: '' };
}

// ------------------------------------------------------------------ what is recorded (settings)
function settingsBlock() {
  const names = store.schema.enums.bb_field;
  const boxes = names.map((name) => {
    const meta = BB_FIELDS[name] || { label: name };
    const box = h('input', { type: 'checkbox', class: 'cond-on', id: 'bbf-' + name });
    box.addEventListener('change', () => {
      const cur = new Set(store.settings.blackbox.fields);
      if (box.checked) cur.add(name); else cur.delete(name);
      setField('blackbox', 'fields', names.filter((n) => cur.has(n)));
    });
    return { name, box, row: h('label', { class: 'bb-group', for: 'bbf-' + name }, box, h('span', {}, h('b', {}, tr(meta.label)), meta.help ? h('span', { class: 'help' }, ' ' + tr(meta.help)) : null)) };
  });
  const sync = () => { const on = new Set((store.settings.blackbox || {}).fields || []); for (const b of boxes) b.box.checked = on.has(b.name); };
  const lock = () => { for (const b of boxes) b.box.disabled = store.armed; };
  listen('settings:blackbox', sync); listen('armed', lock); sync(); lock();

  const cost = h('p', { class: 'help', id: 'bb-cost' });
  const paintCost = () => {
    const st = store.streams.blackbox, s = store.settings.blackbox;
    if (!st || !s) { cost.textContent = ''; return; }
    // records plus the frame around every 240 bytes of them, plus the ten slow records per second
    const rate = Math.round(s.rate_hz * st.record_bytes * 1.04 + 900);
    const free = Math.max(0, st.capacity_kb - st.used_kb) * 1024;
    cost.textContent = tr('Với lựa chọn này: {n} byte mỗi bản ghi, khoảng {rate} mỗi giây.', { n: st.record_bytes, rate: bytesText(rate) })
      + (st.capacity_kb ? ' ' + tr('Chỗ trống đủ cho khoảng {t}.', { t: spanText(free / rate) }) : '');
    cost.dataset.rate = String(rate);
  };
  listen('stream:blackbox', paintCost); listen('settings:blackbox', paintCost); paintCost();

  // the SD card's own settings are only shown while the card is the chosen device
  const sdBox = h('div', { id: 'bb-sd', hidden: true },
    fields('blackbox', ['sd_mode', 'sd_mhz']),
    h('p', { class: 'help' }, tr('Chân của thẻ SD gán ở tab Chân và cổng (cần lưu và khởi động lại). File log nằm trong thư mục /wblog trên thẻ, tên wb_00001.wblog, wb_00002.wblog...: rút thẻ ra cắm vào máy tính rồi thêm bằng nút "Thêm file .wblog…" bên dưới, hoặc tải qua cáp như log trong flash.')));
  const paintSd = () => { sdBox.hidden = (store.settings.blackbox || {}).device !== 'SDCARD'; };
  listen('settings:blackbox', paintSd); paintSd();

  return block(tr('Ghi gì, khi nào'),
    fields('blackbox', ['device', 'trigger', 'rate_hz']),
    sdBox,
    h('p', { class: 'help' }, tr('Các lựa chọn này có hiệu lực ngay; bấm "Lưu" để giữ sau khi tắt nguồn. Phiên đang ghi dở giữ cấu hình lúc nó bắt đầu.')),
    h('div', { class: 'bb-groups', id: 'bb-groups' }, boxes.map((b) => b.row)),
    cost,
    h('p', { class: 'help' }, tr('Trạng thái, mode, pin và sóng receiver luôn được ghi 10 lần mỗi giây. Khi ESP32 ghi flash, cả hai nhân dừng chừng 1 ms cho mỗi trang; nhóm "Thời gian vòng lặp" ghi lại chính độ trễ đó để bạn đo trên mạch thật. Ghi ra thẻ SD thì không có độ trễ này.')));
}

// ------------------------------------------------------------------ the log memory of the craft
function storageBlock(ctx) {
  const pill = h('span', { class: 'pill off', id: 'bb-state' }, '–');
  const usedText = h('span', { class: 'num', id: 'bb-used' }, '–');
  const bar = h('div', { class: 'bar meter', id: 'bb-bar', role: 'meter', 'aria-valuemin': 0, 'aria-valuemax': 100 }, h('i', {}));
  const why = h('p', { class: 'note todo', id: 'bb-why', hidden: true });
  const title = h('span', { id: 'bb-store-title' }, tr('Bộ nhớ log trên mạch'));
  const buffer = h('p', { class: 'help', id: 'bb-buffer', hidden: true });
  const list = h('tbody', {});
  const table = h('table', { class: 'grid', id: 'bb-sessions', hidden: true }, h('thead', {}, h('tr', {}, h('th', {}, tr('Phiên')), h('th', {}, tr('Dung lượng')), h('th', {}, ''))), list);
  const none = h('p', { class: 'help', id: 'bb-none' }, tr('Chưa có phiên nào trên mạch.'));
  const allBtn = h('button', { type: 'button', class: 'btn', id: 'bb-download-all' }, tr('Tải tất cả về máy'));
  const eraseBtn = h('button', { type: 'button', class: 'btn danger', id: 'bb-erase' }, tr('Xoá bộ nhớ log'));
  const progress = h('div', { class: 'bb-progress', id: 'bb-progress', hidden: true }, h('div', { class: 'bar meter' }, h('i', {})), h('span', { class: 'num' }));
  let sessions = [], known = -1, wasErasing = false, lastState = '';

  const busy = () => (store.job && store.job.state === 'running') || (store.rec && store.rec.on);
  const lock = () => {
    const st = store.streams.blackbox || {};
    const off = store.armed || busy() || st.state === 'ERASING';
    allBtn.disabled = off || !sessions.length;
    eraseBtn.disabled = off || st.state === 'NO_STORAGE' || !st.capacity_kb;
    for (const b of list.querySelectorAll('button')) b.disabled = off;
  };
  async function download(index) {
    try { await api.post('/api/blackbox/download', { index }); } catch (e) { showError(e); }
  }
  async function loadSessions() {
    try { sessions = (await api.get('/api/blackbox/sessions')).sessions; } catch (e) { sessions = []; }
    clear(list);
    for (const s of sessions) {
      const b = h('button', { type: 'button', class: 'btn quiet bb-get' }, tr('Tải về máy'));
      b.addEventListener('click', () => download(s.index));
      list.append(h('tr', { 'data-index': s.index }, h('th', { scope: 'row', class: 'num' }, `#${s.index + 1}`), h('td', { class: 'num' }, bytesText(s.size)), h('td', {}, b)));
    }
    table.hidden = !sessions.length; none.hidden = !!sessions.length;
    lock();
  }
  const paint = (st) => {
    const label = (ENUM_LABELS.bb_state || {})[st.state];
    pill.className = 'pill ' + (STATE_PILL[st.state] || 'off'); pill.dataset.state = st.state;
    pill.textContent = (label ? tr(label) : st.state) + (st.state === 'ERASING' ? ` ${st.erase_pct}%` : '');
    const card = st.device === 'SDCARD';
    title.textContent = card ? tr('Thẻ SD') : tr('Bộ nhớ log trên mạch'); title.dataset.device = st.device;
    none.textContent = card ? tr('Chưa có file log nào trên thẻ.') : tr('Chưa có phiên nào trên mạch.');
    // sizes come in kB so that a 32 GB card fits; for a card "used" counts everything on it, not only the logs
    const pct = st.capacity_kb ? Math.min(100, (100 * st.used_kb) / st.capacity_kb) : 0;
    bar.firstChild.style.width = pct + '%'; bar.setAttribute('aria-valuenow', pct.toFixed(0));
    usedText.textContent = st.capacity_kb ? tr('{used} trên {cap} ({pct}%)', { used: bytesText(st.used_kb * 1024), cap: bytesText(st.capacity_kb * 1024), pct: pct.toFixed(0) }) : '–';
    const text = {
      NO_STORAGE: card ? tr('Firmware không thấy thẻ SD (cứ 3 giây tìm lại một lần). Kiểm tra: thẻ đã cắm, định dạng FAT32 (không phải exFAT), đủ chân ở tab Chân và cổng, đúng kiểu nối; dây dài thì hạ "Tốc độ thẻ SD". Lý do firmware báo nằm ở tab Tổng quan.')
        : tr('Firmware không tìm thấy phân vùng flash dành cho log. Build firmware với một bảng phân vùng trong thư mục partitions/ (có phân vùng "wblog"), hoặc ghi trực tiếp qua cáp ở mục bên dưới.'),
      NEEDS_ERASE: tr('Phân vùng log đang chứa dữ liệu khác hoặc log hỏng. Firmware không ghi đè lên đó: bấm "Xoá bộ nhớ log" trước.'),
      FULL: card ? tr('Thẻ SD đã đầy. Tải log về máy rồi xoá để ghi tiếp.') : tr('Bộ nhớ log đã đầy. Tải log về máy rồi xoá để ghi tiếp.'),
      OFF: st.capacity_kb ? tr('Đang không ghi trên mạch: chọn "Flash trên mạch" hoặc "Thẻ SD" ở mục "Ghi vào đâu".') : '',
    }[st.state] || '';
    buffer.hidden = !card || st.state !== 'LOGGING';
    if (!buffer.hidden) {
      buffer.dataset.pct = String(st.buffer_pct);
      buffer.textContent = st.buffer_pct >= 100 ? tr('Bộ đệm ghi thẻ đầy 100 %: thẻ quá chậm so với tốc độ ghi, đang mất bản ghi. Giảm "Tốc độ ghi" hoặc đổi thẻ.')
        : tr('Bộ đệm ghi thẻ: cao nhất {pct} % trong giây vừa qua.', { pct: st.buffer_pct });
    }
    why.hidden = !text; why.textContent = text;
    if (st.sessions !== known || st.state !== lastState) { known = st.sessions; lastState = st.state; loadSessions(); }      // a session that just ended has its final size now
    if (wasErasing && st.state !== 'ERASING') toast(tr('Đã xoá xong bộ nhớ log.'), 'ok');
    wasErasing = st.state === 'ERASING';
    lock();
  };
  const paintJob = (job) => {
    const running = job && job.state === 'running';
    progress.hidden = !running;
    if (running) {
      const pct = job.total ? (100 * job.done) / job.total : 0;
      progress.firstChild.firstChild.style.width = pct + '%';
      progress.lastChild.textContent = tr('Đang tải: {done} / {total}', { done: bytesText(job.done), total: bytesText(job.total) });
    }
    lock();
  };
  allBtn.addEventListener('click', () => download(null));
  eraseBtn.addEventListener('click', async () => {
    const card = (store.streams.blackbox || {}).device === 'SDCARD';
    if (!window.confirm(card ? tr('Xoá mọi file log (wb_*.wblog trong thư mục /wblog) trên thẻ SD? Các file khác trên thẻ không bị đụng tới.')
      : tr('Xoá toàn bộ log trên mạch? Việc này mất từ vài giây đến vài phút; trong lúc đó không arm được.'))) return;
    try { await command('BLACKBOX_ERASE'); } catch (e) { showError(e); }
  });
  listen('stream:blackbox', paint); listen('armed', lock); listen('job', paintJob); listen('rec', lock);
  if (store.streams.blackbox) paint(store.streams.blackbox);
  paintJob(store.job);
  ctx.reloadSessions = loadSessions;

  return block(title,
    h('div', { class: 'bb-store' }, pill, h('div', { class: 'bb-used' }, bar, usedText)),
    why, buffer, table, none, progress,
    h('div', { class: 'btn-row' }, allBtn, eraseBtn),
    h('p', { class: 'help' }, tr('Log được tải qua chính đường truyền này và lưu vào thư mục log trên máy tính. Mạch không hiện thành ổ USB.')));
}

// ------------------------------------------------------------------ live recording over the cable
function liveBlock() {
  const rate = h('input', { type: 'number', class: 'num', id: 'rec-rate', min: 10, max: 2000, step: 10, value: recall('wb-rec-rate', '200') });
  const btn = h('button', { type: 'button', class: 'btn primary', id: 'rec-toggle' }, tr('Bắt đầu ghi'));
  const status = h('span', { class: 'num', id: 'rec-status' });
  const limit = h('p', { class: 'help', id: 'rec-limit' });
  let working = false;
  const paint = () => {
    const rec = store.rec || {};
    btn.textContent = rec.on ? tr('Dừng ghi') : tr('Bắt đầu ghi');
    btn.classList.toggle('primary', !rec.on); btn.classList.toggle('danger', !!rec.on);
    btn.disabled = working || (store.job && store.job.state === 'running');
    rate.disabled = !!rec.on;
    status.textContent = rec.on ? tr('{n} bản ghi, {t}', { n: rec.records, t: spanText(rec.duration) }) + (rec.gaps ? tr(', thiếu {n} chỗ', { n: rec.gaps }) : '') : '';
    status.dataset.records = rec.on ? String(rec.records) : '';
    const st = store.streams.blackbox;
    if (st && store.conn.baud) {
      limit.textContent = tr('Qua cầu USB-UART ở {baud} baud, đường truyền chở được khoảng {n} bản ghi mỗi giây với các nhóm đang chọn (cổng USB gốc của ESP32-S3 nhanh hơn nhiều). Yêu cầu nhiều hơn thì log có chỗ thiếu.',
        { baud: store.conn.baud, n: Math.floor((store.conn.baud / 10) * 0.8 / (st.record_bytes * 1.04)) });
    }
  };
  btn.addEventListener('click', async () => {
    working = true; paint();
    try {
      const on = !(store.rec && store.rec.on);
      const hz = Math.min(2000, Math.max(10, Number(rate.value) || 200));
      if (on) remember('wb-rec-rate', String(hz));
      const r = await api.post('/api/blackbox/record', { on, rate_hz: hz });
      store.rec = r.rec;
    } catch (e) { showError(e); }
    working = false; paint();
  });
  listen('rec', paint); listen('job', paint); listen('stream:blackbox', paint); paint();
  return block(tr('Ghi trực tiếp qua cáp'),
    h('p', { class: 'help' }, tr('Firmware gửi log thẳng về app thay vì ghi vào flash: xem đồ thị ngay lúc đang chạy, trên bàn hay khi bay có dây. Đang bay và đang ghi vào flash thì firmware từ chối, để không cắt ngang log của chuyến bay.')),
    h('div', { class: 'form' },
      h('label', { for: 'rec-rate' }, tr('Tốc độ ghi')), h('div', { class: 'ctl' }, h('div', { class: 'ctl-line' }, rate, h('span', { class: 'unit' }, tr('bản ghi/giây'))))),
    h('div', { class: 'btn-row' }, btn, status),
    limit);
}

// ------------------------------------------------------------------ logs on this computer
function libraryBlock(ctx) {
  const body = h('tbody', {});
  const table = h('table', { class: 'grid', id: 'logs', hidden: true },
    h('thead', {}, h('tr', {}, h('th', {}, tr('File')), h('th', {}, tr('Lúc lưu')), h('th', {}, tr('Phiên')), h('th', {}, tr('Thời lượng')), h('th', {}, tr('Dung lượng')), h('th', {}, ''))), body);
  const none = h('p', { class: 'help', id: 'logs-none' }, tr('Chưa có log nào. Tải log từ mạch về, ghi trực tiếp qua cáp, hoặc thêm một file .wblog có sẵn.'));
  const dir = h('p', { class: 'help', id: 'logs-dir' });
  const pick = h('input', { type: 'file', accept: '.wblog', id: 'log-file', hidden: true });
  const add = h('button', { type: 'button', class: 'btn', id: 'log-add' }, tr('Thêm file .wblog…'));
  add.addEventListener('click', () => pick.click());
  pick.addEventListener('change', async () => {
    const f = pick.files[0];
    pick.value = '';
    if (!f) return;
    try {
      const bytes = new Uint8Array(await f.arrayBuffer());
      let bin = '';
      for (let i = 0; i < bytes.length; i += 0x8000) bin += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000));
      const r = await api.post('/api/log/upload', { name: f.name, data: btoa(bin) });
      await refresh();
      ctx.open(r.file, 0);
    } catch (e) { showError(e); }
  });
  async function refresh() {
    let res;
    try { res = await api.get('/api/logs'); } catch (e) { showError(e); return; }
    dir.textContent = tr('Thư mục log: {dir}', { dir: res.dir });
    clear(body);
    for (const log of res.logs) {
      const open = h('button', { type: 'button', class: 'btn quiet log-open' }, tr('Xem'));
      open.addEventListener('click', () => ctx.open(log.file, 0));
      const del = h('button', { type: 'button', class: 'btn quiet log-delete' }, tr('Xoá'));
      del.addEventListener('click', async () => {
        if (!window.confirm(tr('Xoá file {file} khỏi máy tính?', { file: log.file }))) return;
        try { await api.post('/api/log/delete', { file: log.file }); if (ctx.current() === log.file) ctx.close(); await refresh(); } catch (e) { showError(e); }
      });
      body.append(h('tr', { 'data-file': log.file, class: ctx.current() === log.file ? 'open' : '' },
        h('th', { scope: 'row' }, log.file),
        h('td', { class: 'num' }, new Date(log.mtime * 1000).toLocaleString()),
        h('td', { class: 'num' }, String(log.sessions)),
        h('td', { class: 'num' }, spanText(log.duration)),
        h('td', { class: 'num' }, bytesText(log.size)),
        h('td', {}, open, h('a', { class: 'btn quiet', href: api.url('/api/log/file', { file: log.file }), download: log.file }, tr('Lưu file')), del)));
    }
    table.hidden = !res.logs.length; none.hidden = !!res.logs.length;
  }
  ctx.refreshLibrary = refresh;
  ctx.markOpen = () => { for (const r of body.querySelectorAll('tr')) r.classList.toggle('open', r.dataset.file === ctx.current()); };
  refresh();
  return block(tr('Log trên máy tính'), table, none, h('div', { class: 'btn-row' }, add, pick), dir);
}

// ------------------------------------------------------------------ the charts
function viewerBlock(ctx) {
  let file = null, index = 0, meta = null, chosen = new Set(recall('wb-log-presets', 'rate,motors').split(','));
  let liveTimer = 0;
  const title = h('h2', { id: 'viewer-title' }, tr('Đồ thị'));
  const sessionSel = h('select', { id: 'log-session', 'aria-label': tr('Phiên trong file') });
  const info = h('dl', { class: 'kv', id: 'log-info' });
  const warn = h('div', { id: 'log-warn' });
  const presetRow = h('div', { class: 'bb-presets', id: 'log-presets', role: 'group', 'aria-label': tr('Nhóm đồ thị') });
  const exports = h('div', { class: 'btn-row', id: 'log-exports' });
  const follow = h('label', { class: 'inline', id: 'log-follow', hidden: true }, h('input', { type: 'checkbox', class: 'switch', role: 'switch' }), tr('Bám theo dữ liệu mới'));
  const evBody = h('tbody', {});
  const evBox = h('details', { class: 'bb-events', id: 'log-events' }, h('summary', {}, tr('Sự kiện trong log')),
    h('table', { class: 'grid' }, h('thead', {}, h('tr', {}, h('th', {}, tr('Lúc')), h('th', {}, tr('Sự kiện')))), evBody));
  const holder = h('div', { id: 'viewer', hidden: true });
  const length = h('span', { id: 'log-length' });
  const idle = h('p', { class: 'help', id: 'viewer-idle' }, tr('Bấm "Xem" ở một log trong danh sách trên, hoặc bắt đầu ghi trực tiếp.'));

  const plot = logPlot({
    source: (names, t0, t1, px, tail) => api.query('/api/log/view', { file, session: index, fields: names.join(','), px,
      ...(tail ? { tail } : (t0 === null ? {} : { t0: t0.toFixed(5), t1: t1.toFixed(5) })) }),
  });

  // ---- the craft at the playhead (Blackbox Explorer's sticks and motors): fetched on its own, whatever panels are shown
  const stickL = h('canvas', { id: 'hud-left', role: 'img', 'aria-label': tr('Cần trái: yaw và ga') });
  const stickR = h('canvas', { id: 'hud-right', role: 'img', 'aria-label': tr('Cần phải: roll và pitch') });
  const motorsBox = h('div', { class: 'hud-motors', id: 'hud-motors' });
  const hudInfo = h('dl', { class: 'kv', id: 'hud-info' });
  const hud = h('div', { class: 'bb-hud', id: 'log-hud', hidden: true }, stickL, stickR, motorsBox, hudInfo);
  let hudFields = [], hudBusy = false, hudWant = null;
  function setupHud() {
    const s = session(), all = [...s.fields, ...s.slow_fields];
    const motors = all.filter((n) => /^motor\[\d+\]$/.test(n));
    hudFields = ['stick[0]', 'stick[1]', 'stick[2]', 'stick[3]', 'roll', 'pitch', 'yaw', 'throttle'].filter((n) => all.includes(n)).concat(motors);
    clear(motorsBox);
    motors.forEach((m, i) => motorsBox.append(h('div', { class: 'hud-motor', 'data-motor': String(i) }, h('i', { style: { height: '0%' } }), h('b', {}, String(i + 1)))));
    stickL.hidden = !all.includes('stick[2]') && !all.includes('stick[3]'); stickR.hidden = !all.includes('stick[0]');      // (a log without the stick group)
    hud.hidden = !hudFields.length || file === LIVE;
  }
  function drawStick(cv, x, y) {
    const w = cv.clientWidth, hh = cv.clientHeight;
    if (!w || !hh) return;
    const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(hh * dpr); }
    const c = cv.getContext('2d'); c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, w, hh);
    c.strokeStyle = token('--line'); c.lineWidth = 1;
    c.beginPath(); c.moveTo(w / 2, 6); c.lineTo(w / 2, hh - 6); c.moveTo(6, hh / 2); c.lineTo(w - 6, hh / 2); c.stroke();
    if (x === null || y === null) return;
    const px = w / 2 + Math.max(-1, Math.min(1, x)) * (w / 2 - 8), py = hh / 2 - Math.max(-1, Math.min(1, y)) * (hh / 2 - 8);
    c.fillStyle = token('--blue'); c.beginPath(); c.arc(px, py, 6, 0, 7); c.fill();
    cv.dataset.x = x.toFixed(3); cv.dataset.y = y.toFixed(3);
  }
  function paintHud(series, t) {
    const v = (f) => { const a = sampleAt(series[f], t); return a ? (a.lo + a.hi) / 2 : null; };
    const thr = v('stick[3]');
    drawStick(stickL, v('stick[2]'), thr === null ? null : thr * 2 - 1);            // mode 2: yaw and throttle (0..1) on the left
    drawStick(stickR, v('stick[0]'), v('stick[1]'));                                  // roll, pitch (+ = nose down = stick forward = up)
    for (const bar of motorsBox.children) {
      const m = v(`motor[${bar.dataset.motor}]`);
      bar.firstChild.style.height = m === null ? '0%' : `${Math.round(Math.max(0, Math.min(1, m)) * 100)}%`;
      bar.title = m === null ? '' : `${Math.round(m * 100)} %`;
    }
    const deg = (f) => { const x = v(f); return x === null ? '–' : `${x.toFixed(1)}°`; };
    clear(hudInfo);
    hudInfo.append(...[[tr('Lúc'), `${t.toFixed(3)} s`], ['Roll', deg('roll')], ['Pitch', deg('pitch')], ['Yaw', deg('yaw')],
      [tr('Ga ra motor'), v('throttle') === null ? '–' : `${Math.round(v('throttle') * 100)} %`]].flatMap(([k, x]) => [h('dt', {}, k), h('dd', { class: 'num' }, x)]));
    hud.dataset.t = t.toFixed(4);
  }
  async function hudAt(t) {
    hudWant = t;
    if (hudBusy || !hudFields.length || hud.hidden) return;
    hudBusy = true;
    try {
      while (hudWant !== null) {                     // the newest wish only: a playhead that runs ahead skips the steps in between
        const at = hudWant; hudWant = null;
        const res = await api.query('/api/log/view', { file, session: index, fields: hudFields.join(','), px: 16, t0: (at - 0.05).toFixed(5), t1: (at + 0.05).toFixed(5) });
        paintHud(res.series, at);
      }
    } catch (e) { /* the log was closed meanwhile */ } finally { hudBusy = false; }
  }
  plot.onPlayhead((t) => hudAt(t));
  // ---- PID tuning from this log (wbclient/tune.py): step response per axis, noise, what to change
  const TUNE_WHY = {
    OVERSHOOT: tr('vọt lố: thêm D để hãm'), OVERSHOOT_NOISY_D: tr('vọt lố nhưng D đã nhiễu: bớt P thay vì thêm D'), SLOW: tr('đáp ứng chậm, không vọt lố: thêm P (và feed-forward)'),
    RINGING: tr('dao động quanh đích sau đỉnh: bớt P'), STEADY_LOW: tr('không tới hết mức được yêu cầu: thêm I'), NOISE_NO_MORE_D: tr('gyro / D nhiễu: đừng tăng D, xem lại bộ lọc trước'),
  };
  const TUNE_NOTE = {
    TOO_SHORT: tr('Log quá ngắn để phân tích.'), RATE_LOW: tr('Tốc độ ghi dưới 250 bản ghi/giây: đáp ứng tính ra thô. Nên ghi từ 1000 bản ghi/giây.'),
    NO_SETPOINT_GYRO: tr('Log không có nhóm "Gyro và setpoint": bật nhóm này trong cài đặt blackbox rồi bay lại.'),
    NO_STICK_MOVES: tr('Trong log không có đoạn đánh cần đủ mạnh (trên 20 °/s): bay vài cú lật / đánh cần dứt khoát rồi ghi lại.'),
    NO_SETTINGS: tr('Log không đọc được cấu hình PID (firmware khác phiên bản): chỉ có đáp ứng, không có gợi ý số.'),
  };
  const PARAM = { p: 'P', i: 'I', d: 'D', d_max: 'D max', f: 'FF' };
  const tuneOut = h('div', { id: 'tune-out' });
  const tuneBtn = h('button', { type: 'button', class: 'btn', id: 'log-tune' }, tr('Phân tích PID từ log này'));
  const tuneBox = h('section', { class: 'bb-tune', id: 'log-tune-box' }, h('h3', {}, tr('Phân tích PID')),
    h('p', { class: 'help' }, tr('Như PIDtoolbox: đáp ứng bước của từng trục được tính lại từ các lần đánh cần trong log (đỉnh 1,0 = không vọt lố), kèm độ nhiễu, rồi gợi ý chỉnh P / I / D / FF. Mỗi lần chỉnh tối đa 25 %: bay, ghi log, phân tích lại.')),
    tuneBtn, tuneOut);
  function drawStep(cv, step) {
    const w = cv.clientWidth, hh = cv.clientHeight;
    if (!w || !hh) return;
    const dpr = window.devicePixelRatio || 1;
    cv.width = Math.round(w * dpr); cv.height = Math.round(hh * dpr);
    const c = cv.getContext('2d'); c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, w, hh);
    const tmax = step.t[step.t.length - 1] || 0.5, X = (t) => 4 + (t / tmax) * (w - 8), Y = (v) => hh - 4 - (Math.max(0, Math.min(1.6, v)) / 1.6) * (hh - 8);
    c.strokeStyle = token('--line'); c.lineWidth = 1; c.setLineDash([4, 3]); c.beginPath(); c.moveTo(4, Y(1)); c.lineTo(w - 4, Y(1)); c.stroke(); c.setLineDash([]);
    c.strokeStyle = token('--blue'); c.lineWidth = 2; c.beginPath();
    step.t.forEach((t, k) => { if (k) c.lineTo(X(t), Y(step.y[k])); else c.moveTo(X(t), Y(step.y[k])); }); c.stroke();
  }
  function showTune(r) {
    clear(tuneOut);
    const axes = h('div', { class: 'bb-tune-axes' });
    for (const [ax, v] of Object.entries(r.axes)) {
      const m = v.metrics, cv = h('canvas', { class: 'tune-step', 'data-axis': ax });
      axes.append(h('div', { class: 'tune-axis', 'data-axis': ax }, h('strong', {}, ax[0].toUpperCase() + ax.slice(1)), cv,
        h('dl', { class: 'kv' }, ...(m ? [[tr('Đỉnh'), m.peak.toFixed(2)], [tr('Tới nửa bước'), m.latency_ms === null ? '–' : `${m.latency_ms} ms`], [tr('Giá trị cuối'), m.steady.toFixed(2)],
          [tr('Nhiễu gyro trên 80 Hz'), v.gyro_noise === null ? '–' : `${v.gyro_noise} °/s`], [tr('Phần của D'), v.d_share === null ? '–' : `${Math.round(v.d_share * 100)} %`]]
          : [[tr('Đáp ứng'), tr('không tính được')]]).flatMap(([k, x]) => [h('dt', {}, k), h('dd', { class: 'num' }, x)]))));
      if (v.step) requestAnimationFrame(() => drawStep(cv, v.step));
    }
    tuneOut.append(axes);
    for (const n of r.notes) tuneOut.append(note(TUNE_NOTE[n] || n, 'todo'));
    if (r.suggestions.length) {
      const body = h('tbody', {});
      for (const s of r.suggestions) body.append(h('tr', { 'data-param': `${s.axis}.${s.param}` }, h('td', {}, s.axis), h('td', {}, PARAM[s.param] || s.param),
        h('td', { class: 'num' }, s.new === null ? String(s.now) : `${s.now} → ${s.new}`), h('td', {}, TUNE_WHY[s.why] || s.why)));
      tuneOut.append(h('table', { class: 'grid', id: 'tune-suggestions' }, h('thead', {}, h('tr', {}, h('th', {}, tr('Trục')), h('th', {}, tr('Tham số')), h('th', {}, tr('Gợi ý')), h('th', {}, tr('Vì sao')))), body));
      const apply = r.suggestions.filter((s) => s.new !== null);
      if (r.sliders) tuneOut.append(note(tr('Cấu hình của chuyến bay này dùng thanh trượt (slider) để tính PID: chỉnh bằng thanh trượt ở tab Tuning theo hướng gợi ý, hoặc tắt thanh trượt trước khi nhập số.')));
      else if (apply.length && store.conn.connected && store.settings.pid) {
        const b = h('button', { type: 'button', class: 'btn primary', id: 'tune-apply' }, tr('Đưa gợi ý vào cấu hình (chưa lưu)'));
        b.addEventListener('click', () => {
          const idx = { roll: 0, pitch: 1, yaw: 2 };
          for (const s of apply) setField('pid', s.param, s.new, idx[s.axis]);
          toast(tr('Đã đưa {n} giá trị vào cấu hình. Bấm Lưu để giữ lại.', { n: apply.length }));
        });
        tuneOut.append(h('div', { class: 'btn-row' }, b));
      }
    } else if (!r.notes.length) tuneOut.append(note(tr('Đáp ứng tốt: không cần chỉnh gì.')));
    tuneOut.dataset.done = '1';
  }
  tuneBtn.addEventListener('click', async () => {
    tuneBtn.disabled = true; clear(tuneOut); tuneOut.dataset.done = ''; tuneOut.append(h('p', { class: 'help' }, tr('Đang tính…')));
    try { showTune(await api.query('/api/log/tune', { file, session: index })); } catch (e) { clear(tuneOut); showError(e); } finally { tuneBtn.disabled = false; }
  });

  function overviewField() {
    const s = session(), all = [...s.fields, ...s.slow_fields];
    return ['stick[3]', 'throttle', 'gyro[0]'].find((n) => all.includes(n)) || s.fields.find((n) => n !== 't') || null;
  }
  follow.firstChild.addEventListener('change', () => { if (follow.firstChild.checked) plot.follow(FOLLOW_SECONDS); else plot.setRange(null, true); });

  function session() { return meta.sessions[index]; }
  function buildPanels() {
    const s = session();
    const all = presetsFor([...s.fields, ...s.slow_fields], Number((s.fields.filter((n) => n.startsWith('motor[')).length) || 4));
    clear(presetRow);
    for (const p of all) {
      const box = h('input', { type: 'checkbox', id: 'preset-' + p.id });
      box.checked = chosen.has(p.id);
      box.addEventListener('change', () => {
        if (box.checked) chosen.add(p.id); else chosen.delete(p.id);
        remember('wb-log-presets', [...chosen].join(','));
        plot.setPanels(all.filter((x) => chosen.has(x.id)).flatMap((x) => x.panels));
      });
      presetRow.append(h('label', { class: 'bb-preset', for: 'preset-' + p.id }, box, p.label));
    }
    plot.setPanels(all.filter((x) => chosen.has(x.id)).flatMap((x) => x.panels));
  }
  function paintInfo() {
    const s = session();
    clear(info); clear(warn); clear(exports); clear(evBody);
    const live = file === LIVE;
    length.textContent = tr('{n} bản ghi, {t}', { n: s.records, t: spanText(s.duration) });
    info.append(...[
      [tr('Firmware'), s.firmware],
      [tr('Ghi vào'), s.to === 'stream' ? tr('app, qua cáp') : tr('flash trên mạch') + (s.session ? tr(', phiên thứ {n}', { n: s.session }) : '')],
      [tr('Thời lượng'), length],
      [tr('Tốc độ ghi'), tr('{n} bản ghi/giây', { n: s.rate_hz }) + (s.real_rate_hz && Math.abs(s.real_rate_hz - s.rate_hz) > 0.1 * s.rate_hz ? tr(' theo cấu hình, thực tế {n}', { n: Math.round(s.real_rate_hz) }) : '')],
    ].flatMap(([k, v]) => [h('dt', {}, k), h('dd', {}, v)]));
    if (s.real_rate_hz && s.real_rate_hz < 0.9 * s.rate_hz) warn.append(note(tr('Log thưa hơn cấu hình: vòng điều khiển lúc đó chạy chậm hơn tần số đã đặt, hoặc (khi ghi qua cáp) firmware đã giảm tốc độ ghi cho vừa đường truyền.'), 'todo'));
    if (s.gaps) warn.append(note(tr('Log thiếu dữ liệu ở {n} chỗ (khung bị mất trên đường truyền, hoặc hỏng trong flash). Đồ thị để trống ở những chỗ đó.', { n: s.gaps }), 'todo'));
    if (s.dropped) warn.append(note(tr('Firmware đã bỏ {n} bản ghi vì ghi không kịp. Giảm tốc độ ghi hoặc bỏ bớt nhóm.', { n: s.dropped }), 'todo'));
    if (!live && !s.complete) warn.append(note(tr('Phiên này không có dấu kết thúc: mất nguồn khi đang ghi, hoặc file bị cắt. Dữ liệu trước đó vẫn dùng được.'), 'todo'));
    if (store.schema && s.schema && parseInt(s.schema, 16) !== parseInt(store.schema.hash, 16)) warn.append(note(tr('Log này do một bản firmware có bộ cấu hình khác ghi ra: đồ thị vẫn đúng, nhưng phần cấu hình kèm theo không đọc được đầy đủ.')));
    const link = (id, label, path, params) => h('a', { class: 'btn quiet', id, href: api.url(path, { file, session: index, ...params }) }, label);
    exports.append(link('log-csv', tr('Xuất CSV'), '/api/log/csv', {}), link('log-csv-slow', tr('CSV dữ liệu chậm (pin, GPS, độ cao)'), '/api/log/csv', { slow: 1 }),
      link('log-settings', tr('Cấu hình của chuyến bay này'), '/api/log/settings', {}));
    const marks = [];
    for (const [t, name, data] of s.events) {
      const e = eventText(name, data, store.schema.enums);
      const go = h('button', { type: 'button', class: 'btn quiet num ev-go' }, `${t.toFixed(2)} s`);
      go.addEventListener('click', () => plot.centerOn(t));
      evBody.append(h('tr', { 'data-event': name }, h('td', {}, go), h('td', {}, e.text)));
      // a session opens with the state of that moment; "DISARM" there is no event of the flight
      if (e.short && !(name === 'DISARM' && t <= s.events[0][0])) marks.push({ t, text: e.short });
    }
    evBox.hidden = !s.events.length;
    plot.setEvents(marks);
  }
  async function open(name, sessionIndex, quiet) {
    let next;
    try { next = await api.query('/api/log/meta', { file: name }); } catch (e) { if (name !== LIVE && !quiet) showError(e); return false; }
    file = name; meta = next; index = Math.min(sessionIndex || 0, meta.sessions.length - 1);
    clear(sessionSel);
    meta.sessions.forEach((s, i) => sessionSel.append(h('option', { value: i }, tr('Phiên {n} trong file ({t})', { n: i + 1, t: spanText(s.duration) }))));
    sessionSel.value = String(index); sessionSel.hidden = meta.sessions.length < 2;
    title.textContent = name === LIVE ? tr('Đồ thị: đang ghi trực tiếp') : tr('Đồ thị: {file}', { file: name });
    holder.hidden = false; idle.hidden = true; holder.dataset.file = name;
    follow.hidden = name !== LIVE; follow.firstChild.checked = name === LIVE;
    plot.clear();
    paintInfo(); buildPanels(); setupHud(); clear(tuneOut); tuneBox.hidden = name === LIVE;
    plot.setOverview(name === LIVE ? null : overviewField());
    if (name === LIVE) plot.follow(FOLLOW_SECONDS);
    lastOpen = name === LIVE ? null : { file: name, index };
    ctx.markOpen();
    return true;
  }
  function close() { lastOpen = null; file = null; meta = null; holder.hidden = true; idle.hidden = false; holder.dataset.file = ''; title.textContent = tr('Đồ thị'); plot.clear(); plot.setPanels([]); ctx.markOpen(); }
  sessionSel.addEventListener('change', () => { index = Number(sessionSel.value); lastOpen = { file, index }; plot.clear(); paintInfo(); buildPanels(); setupHud(); plot.setOverview(overviewField()); });

  // a live recording: open it as soon as its header is there, keep the chart moving, and show the saved file when it stops
  let opening = false, wasOn = false;
  async function onRec(rec) {
    const stopped = wasOn && !rec.on;
    wasOn = !!rec.on;
    if (rec.on) {
      if (file !== LIVE && !opening && rec.records > 0) { opening = true; await open(LIVE, 0); opening = false; }
      if (file === LIVE) {
        follow.firstChild.checked = plot.following();
        length.textContent = tr('{n} bản ghi, {t}', { n: rec.records, t: spanText(rec.duration) });
      }
    } else if (file === LIVE) {
      await ctx.refreshLibrary();
      if (rec.file) await open(rec.file, 0); else close();
    } else if (stopped && rec.file) ctx.refreshLibrary();
  }
  listen('rec', onRec);
  // (new records are fetched five times a second; between two fetches the window keeps moving by itself - gfx/follow.js)
  liveTimer = setInterval(() => { if (file === LIVE && store.rec && store.rec.on && plot.following()) plot.refresh(); }, 200);
  track(() => clearInterval(liveTimer));
  if (store.rec && store.rec.on) onRec(store.rec);
  else if (lastOpen) open(lastOpen.file, lastOpen.index, true);

  ctx.open = open; ctx.close = close; ctx.current = () => file;
  holder.append(h('div', { class: 'bb-viewer-head' }, sessionSel, follow), info, warn, presetRow, hud, plot.el, exports, evBox, tuneBox);
  return h('section', { class: 'block', id: 'viewer-block' }, title, idle, holder);
}

export default {
  id: 'blackbox',
  title: 'Blackbox',
  lead: tr('Ghi lại chuyến bay (gyro, setpoint, PID, motor, pin, GPS) rồi xem bằng đồ thị; hoặc xem trực tiếp khi đang nối cáp.'),
  streams: { blackbox: 5, status: 10 },
  offline: true,           // the logs on this computer can be viewed with no flight controller connected
  mount(view) {
    const ctx = { open: async () => false, close() {}, current: () => null, refreshLibrary: async () => {}, reloadSessions: async () => {}, markOpen() {} };
    const linked = store.conn.connected;
    const viewer = viewerBlock(ctx);
    if (linked) view.append(h('div', { class: 'cols' }, h('div', {}, settingsBlock()), h('div', {}, storageBlock(ctx), liveBlock())));
    else view.append(note(tr('Chưa kết nối flight controller: vẫn xem được các log đã lưu trên máy tính này.')));
    view.append(libraryBlock(ctx), viewer);
    // a finished download lands in the list and opens
    let lastJob = store.job && store.job.state;
    listen('job', async (job) => {
      const was = lastJob; lastJob = job.state;
      if (was !== 'running' || job.state === 'running') return;
      if (job.state === 'done') {
        toast(tr('Đã tải log về: {file}', { file: job.file }), 'ok');
        await ctx.refreshLibrary();
        ctx.open(job.file, 0);
      } else toast(tr('Tải log thất bại ({why}).', { why: job.error }), 'bad', 6500);
    });
  },
};
