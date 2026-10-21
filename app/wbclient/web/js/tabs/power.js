// Battery and beeper: what the pack reads right now, the per-cell limits, the divider calibration; which situations beep and how.
import { store, setField, setFields, command } from '../store.js';
import { h, block, fields, control, clear, listen, track, note, readout, toast, showError } from '../ui.js';
import { ENUM_LABELS, BEEP_CONDITIONS } from '../labels.js';
import { BEEP_PATTERNS, MOTOR_NEVER, motorTones, ledColors } from '../gfx/fwmath.js';
import { tr } from '../i18n.js';

const SVG = 'http://www.w3.org/2000/svg';
const svg = (tag, attrs, ...kids) => { const el = document.createElementNS(SVG, tag); for (const [k, v] of Object.entries(attrs || {})) el.setAttribute(k, v); el.append(...kids); return el; };
const volts = (v, d = 2) => (v === undefined || v === null ? '–' : Number(v).toFixed(d));
const STATE_PILL = { NOT_PRESENT: 'off', OK: 'ok', WARNING: 'warn', CRITICAL: 'bad' };

// ---- one cell on a scale: red below the minimum, amber up to the warning level, green up to a full cell
function cellGauge() {
  const W = 520, H = 46, X0 = 6, X1 = W - 6;
  const zone = { crit: svg('rect', { class: 'g-crit', y: 14, height: 14 }), warn: svg('rect', { class: 'g-warn', y: 14, height: 14 }), ok: svg('rect', { class: 'g-ok', y: 14, height: 14 }) };
  const marker = svg('path', { class: 'g-mark', d: 'M0 8 L5 0 L-5 0 Z' });
  const ticks = ['middle', 'middle', 'end'].map((anchor) => svg('text', { class: 'g-tick', y: 42, 'text-anchor': anchor }));
  const root = svg('svg', { class: 'gauge', id: 'batt-gauge', viewBox: `0 0 ${W} ${H}`, role: 'img' }, zone.crit, zone.warn, zone.ok, ...ticks, marker);
  const paint = () => {
    const s = store.settings.battery; const b = store.streams.battery;
    if (!s) return;
    const min = s.cell_min_mv / 1000, warn = Math.max(min, s.cell_warning_mv / 1000), max = Math.max(warn + 0.05, s.cell_max_mv / 1000);
    const lo = min - 0.3, x = (v) => X0 + (X1 - X0) * Math.min(1, Math.max(0, (v - lo) / (max - lo)));
    const put = (el, a, c) => { el.setAttribute('x', x(a)); el.setAttribute('width', Math.max(0, x(c) - x(a))); };
    put(zone.crit, lo, min); put(zone.warn, min, warn); put(zone.ok, warn, max);
    [min, warn, max].forEach((v, i) => { ticks[i].setAttribute('x', x(v)); ticks[i].textContent = v.toFixed(2); });
    const have = b && b.cells > 0;
    marker.style.display = have ? '' : 'none';
    if (have) marker.setAttribute('transform', `translate(${x(b.cell)} 5)`);
    root.dataset.cell = have ? b.cell.toFixed(2) : '';
    root.setAttribute('aria-label', have ? tr('Mỗi cell {v} V; cảnh báo dưới {warn} V, tối thiểu {min} V, đầy {max} V', { v: b.cell.toFixed(2), warn: warn.toFixed(2), min: min.toFixed(2), max: max.toFixed(2) })
      : tr('Chưa có số đo pin'));
  };
  listen('settings:battery', paint); listen('stream:battery', paint); paint();
  return root;
}

// ---- on / off bars of one beep pattern (unit 10 ms, like the firmware table: on, off, on, off ...)
function patternView(name) {
  const seq = BEEP_PATTERNS[name];
  if (!seq) return h('span', { class: 'muted' }, '–');
  const K = 0.55, total = seq.reduce((a, b) => a + b, 0);
  const root = svg('svg', { class: 'beep-pattern', width: Math.max(24, Math.round(total * K)), height: 14, role: 'img', 'aria-label': tr('Mẫu âm dài {s} giây', { s: (total / 100).toFixed(2) }) });
  root.append(svg('rect', { class: 'bp-base', x: 0, y: 6, width: total * K, height: 2 }));
  let t = 0;
  seq.forEach((d, i) => { if (i % 2 === 0 && d > 0) root.append(svg('rect', { class: 'bp-on', x: t * K, y: 1, width: Math.max(1.5, d * K), height: 12, rx: 1 })); t += d; });
  return root;
}

function batteryColumn() {
  const pill = h('span', { class: 'pill off', id: 'batt-state' }, '–');
  const big = h('span', { class: 'num', id: 'batt-volts' }, '–');
  const cells = h('span', { class: 'num', id: 'batt-cells' }, '–');
  const perCell = h('span', { class: 'num', id: 'batt-cell' }, '–');
  const source = h('span', { id: 'batt-source' }, '–');
  const amps = h('span', { class: 'num', id: 'batt-amps' }, '–');
  const mah = h('span', { class: 'num', id: 'batt-mah' }, '–');
  const left = h('span', { class: 'num', id: 'batt-left' }, '–');
  const paint = (b) => {
    const label = (ENUM_LABELS.batt_state || {})[b.state];
    pill.className = 'pill ' + (STATE_PILL[b.state] || 'off'); pill.textContent = label ? tr(label) : b.state; pill.dataset.state = b.state;
    const have = b.cells > 0;
    big.textContent = have ? volts(b.voltage) + ' V' : '–';
    cells.textContent = have ? `${b.cells}S` : '–';
    perCell.textContent = have ? volts(b.cell) + ' V' : '–';
    const src = { ADC: tr('chân ADC'), ESC: tr('ESC (DShot hai chiều)'), NONE: tr('không đo'), DRONECAN: tr('module nguồn DroneCAN') }[b.source] || b.source;
    const csrc = { ADC: tr('cảm biến dòng'), ESC: tr('ESC (telemetry DShot mở rộng)'), NONE: tr('không đo'), DRONECAN: tr('module nguồn DroneCAN') }[b.current_source] || b.current_source || '–';
    source.textContent = tr('Nguồn đo điện áp: {src}. Nguồn đo dòng: {csrc}.', { src, csrc });
    const hasCurrent = b.current_source === 'ADC' || b.current_source === 'ESC' || b.current_source === 'DRONECAN';
    amps.textContent = hasCurrent ? volts(b.current, 1) + ' A' : '–';
    mah.textContent = hasCurrent ? `${b.mah} mAh` : '–';
    left.textContent = have ? `${b.remaining} %` : '–';
  };
  listen('stream:battery', paint);
  if (store.streams.battery) paint(store.streams.battery);

  // ---- divider calibration: divider = voltage measured with a multimeter / voltage at the pin
  const measured = h('input', { type: 'number', class: 'num', id: 'batt-measured', step: '0.01', min: '1', max: '60', inputmode: 'decimal', placeholder: tr('ví dụ 16.42'), style: { width: '124px' } });
  const calib = h('button', { type: 'button', class: 'btn', id: 'batt-calibrate' }, tr('Tính hệ số chia'));
  calib.addEventListener('click', () => {
    const b = store.streams.battery; const v = Number(measured.value);
    if (!b || b.source !== 'ADC' || !(b.pin_mv > 20)) { toast(tr('Chân ADC chưa đo được điện áp nào. Cắm pin và kiểm tra chân "ADC đo pin" ở tab Chân và cổng.'), 'bad'); return; }
    if (!(v >= 1 && v <= 60)) { toast(tr('Nhập điện áp pin đo bằng đồng hồ (1 đến 60 V).'), 'bad'); return; }
    const k = Math.round((v / (b.pin_mv / 1000)) * 100) / 100;
    setField('battery', 'divider', k);
    toast(tr('Hệ số chia mới: {k}. Số đọc sẽ khớp sau vài giây; bấm "Lưu" để giữ lại.', { k: k.toFixed(2) }), 'ok');
  });
  // ---- current sensor calibration: a known current (clamp meter, bench supply) -> the scale. amps x 1000 = pin_mV x 10000 / scale + offset_mA
  const measuredA = h('input', { type: 'number', class: 'num', id: 'curr-measured', step: '0.1', min: '0.5', max: '300', inputmode: 'decimal', placeholder: tr('ví dụ 5.0'), style: { width: '124px' } });
  const calibA = h('button', { type: 'button', class: 'btn', id: 'curr-calibrate' }, tr('Tính hệ số cảm biến'));
  calibA.addEventListener('click', () => {
    const b = store.streams.battery; const a = Number(measuredA.value); const s = store.settings.battery || {};
    if (!b || !(b.current_mv > 0)) { toast(tr('Chân cảm biến dòng chưa đo được điện áp nào. Kiểm tra chân "ADC đo dòng" ở tab Chân và cổng (cần khởi động lại sau khi gán chân).'), 'bad'); return; }
    if (!(a >= 0.5 && a <= 300)) { toast(tr('Nhập dòng đo bằng đồng hồ (0,5 đến 300 A), lúc motor đang quay đều.'), 'bad'); return; }
    const denom = a * 1000 - (s.current_offset || 0);
    const k = Math.round((b.current_mv * 10000) / denom);
    if (!(Math.abs(k) >= 1 && Math.abs(k) <= 16000)) { toast(tr('Hệ số tính được ({k}) nằm ngoài khoảng cho phép: kiểm tra độ lệch và dòng nhập vào.', { k }), 'bad'); return; }
    setField('battery', 'current_scale', k);
    toast(tr('Hệ số cảm biến mới: {k}. Bấm "Lưu" để giữ lại.', { k }), 'ok');
  });
  const lock = () => { calib.disabled = measured.disabled = calibA.disabled = measuredA.disabled = store.armed; };
  listen('armed', lock); lock();

  return h('div', {},
    block(tr('Pin đang đo'),
      h('div', { class: 'big-read' },
        h('div', {}, h('span', { class: 'k' }, tr('Điện áp')), big),
        h('div', {}, h('span', { class: 'k' }, tr('Số cell')), cells),
        h('div', {}, h('span', { class: 'k' }, tr('Mỗi cell')), perCell),
        h('div', {}, h('span', { class: 'k' }, tr('Trạng thái')), pill)),
      h('div', { class: 'big-read' },
        h('div', {}, h('span', { class: 'k' }, tr('Dòng')), amps),
        h('div', {}, h('span', { class: 'k' }, tr('Đã dùng')), mah),
        h('div', {}, h('span', { class: 'k' }, tr('Còn lại')), left)),
      cellGauge(),
      h('p', { class: 'help' }, source)),
    block(tr('Ngưỡng'),
      fields('battery', ['source', 'cells', 'cell_max_mv', 'cell_warning_mv', 'cell_min_mv', 'warning_delay', 'critical_delay', 'land_on_critical']),
      h('p', { class: 'help' }, tr('Các ngưỡng có hiệu lực ngay, không cần khởi động lại. Dưới mức cảnh báo thì firmware không cho arm.'))),
    block(tr('Hiệu chỉnh số đo (ADC)'),
      h('p', { class: 'help' }, tr('Pin nối qua cầu chia áp vào một chân ADC. Đo điện áp pin bằng đồng hồ, nhập vào đây, app tính hệ số chia từ điện áp đang có ở chân.')),
      fields('battery', ['divider']),
      h('div', { class: 'form' },
        h('label', {}, tr('Điện áp ở chân ADC')), h('div', { class: 'ctl' }, h('div', { class: 'ctl-line' }, readout('battery', (b) => (b.source === 'ADC' || b.pin_mv ? `${b.pin_mv} mV` : tr('chưa gán chân'))))),
        h('label', { for: 'batt-measured' }, tr('Điện áp đo bằng đồng hồ')), h('div', { class: 'ctl' }, h('div', { class: 'ctl-line' }, measured, h('span', { class: 'unit' }, 'V'), calib))),
      note(tr('Điện áp ở chân ADC không được vượt 3,1 V khi pin đầy. Pin 6S (25,2 V) cần hệ số chia từ 8,2 trở lên.'))),
    block(tr('Đo dòng và dung lượng'),
      h('p', { class: 'help' }, tr('Dòng lấy từ cảm biến dòng nối vào một chân ADC, hoặc là tổng dòng các ESC báo về (cần DShot hai chiều và extended telemetry ở tab Motor). Số mAh đã dùng tính từ lúc cấp nguồn.')),
      fields('battery', ['current_source', 'current_scale', 'current_offset', 'capacity_mah']),
      h('div', { class: 'form' },
        h('label', {}, tr('Điện áp ở chân cảm biến dòng')), h('div', { class: 'ctl' }, h('div', { class: 'ctl-line' }, readout('battery', (b) => (b.current_mv ? `${b.current_mv} mV` : tr('chưa gán chân'))))),
        h('label', { for: 'curr-measured' }, tr('Dòng đo bằng đồng hồ')), h('div', { class: 'ctl' }, h('div', { class: 'ctl-line' }, measuredA, h('span', { class: 'unit' }, 'A'), calibA))),
      note(tr('Hiệu chỉnh: tháo cánh, cho motor quay đều ở tab Motor, đo dòng bằng ampe kìm hoặc đọc trên nguồn, nhập vào đây. Hai số này có tác dụng ngay, không cần khởi động lại.'))));
}

function beeperColumn() {
  const names = store.schema.enums.beeper_cond;
  const condLabel = (i) => (i ? (BEEP_CONDITIONS[names[i - 1]] ? tr(BEEP_CONDITIONS[names[i - 1]].label) : names[i - 1]) : tr('im lặng'));
  const now = readout('battery', (b) => condLabel(b.beeping), '');
  now.id = 'beep-now';
  const motorNow = readout('battery', (b) => condLabel(b.motor_beeping), '');
  motorNow.id = 'motor-beep-now';
  const silence = h('button', { type: 'button', class: 'btn quiet', id: 'beep-silence' }, tr('Im lặng'));
  silence.addEventListener('click', async () => { try { await command('BEEP', { condition: null }); } catch (e) { showError(e); } });

  // one row per condition, a switch for each sounder: the buzzer on its pin, and the motors (the ESCs beep them)
  const toggle = (field, name, id, label) => {
    const box = h('input', { type: 'checkbox', class: 'cond-on', id, 'aria-label': label });
    box.addEventListener('change', () => {
      const cur = new Set(store.settings.beeper[field]);
      if (box.checked) cur.add(name); else cur.delete(name);
      setField('beeper', field, names.filter((n) => cur.has(n)));
    });
    return box;
  };
  const rows = names.map((name) => {
    const meta = BEEP_CONDITIONS[name] || { label: name };
    const box = toggle('conditions', name, 'cond-' + name, tr(meta.label));
    const never = MOTOR_NEVER.includes(name);
    const motorBox = never ? null : toggle('motor_conditions', name, 'mcond-' + name, tr('Motor: {what}', { what: tr(meta.label) }));
    const [tones] = motorTones(name);
    const tryBtn = BEEP_PATTERNS[name] ? h('button', { type: 'button', class: 'btn quiet beep-try' }, tr('Thử')) : null;
    if (tryBtn) tryBtn.addEventListener('click', async () => { try { await command('BEEP', { condition: name }); } catch (e) { showError(e); } });
    return { name, box, motorBox, tryBtn, row: h('tr', { 'data-cond': name },
      h('td', {}, box),
      h('td', { class: 'motor-cond' }, motorBox || h('span', { class: 'muted', title: tr('Motor đang quay (hoặc sắp quay) thì không kêu được') }, '–')),
      h('th', { scope: 'row' }, h('label', { for: 'cond-' + name }, tr(meta.label)), meta.help ? h('p', { class: 'help' }, tr(meta.help)) : null),
      h('td', {}, patternView(name), motorBox && tones ? h('span', { class: 'help motor-tones' }, ' ' + tr('motor: {n} tiếng', { n: tones })) : null),
      h('td', {}, tryBtn)) };
  });
  const sync = () => {
    const b = store.settings.beeper || {}; const on = new Set(b.conditions || []), mon = new Set(b.motor_conditions || []);
    for (const r of rows) { r.box.checked = on.has(r.name); if (r.motorBox) r.motorBox.checked = mon.has(r.name); }
  };
  const lock = () => { for (const r of rows) { r.box.disabled = store.armed; if (r.motorBox) r.motorBox.disabled = store.armed; if (r.tryBtn) r.tryBtn.disabled = store.armed; } silence.disabled = store.armed; };
  listen('settings:beeper', sync); listen('armed', lock); sync(); lock();

  // what the craft has to make noise with (pins and ESC protocol need a reboot, so the stored settings are what runs)
  const hasBuzzer = () => !!(store.settings.pins && store.settings.pins.buzzer !== 255);
  const dshot = () => /^(DSHOT|PROSHOT)/.test((store.settings.motors || {}).protocol || '');
  const have = h('p', { class: 'note', id: 'sounders' });
  const paintHave = () => {
    const b = hasBuzzer(), m = dshot();
    have.className = 'note ' + (b || m ? 'ok' : 'todo');
    have.dataset.buzzer = b ? '1' : '0'; have.dataset.motors = m ? '1' : '0';
    have.textContent = b && m ? tr('Máy này có còi và motor kêu được (ESC dùng DShot). Mỗi tình huống chọn riêng cho từng loại.')
      : m ? tr('Chưa gán chân còi: tiếng báo do motor phát (ESC dùng DShot). Motor chỉ kêu được khi chưa arm.')
      : b ? tr('Chỉ có còi: ESC đang dùng giao thức {p}, không nhận lệnh beacon. Muốn motor kêu thì chọn một giao thức DShot ở tab Motor.', { p: (store.settings.motors || {}).protocol || '?' })
      : tr('Máy này chưa có gì để kêu: chưa gán chân còi, và ESC không dùng DShot nên motor không kêu được.');
  };
  paintHave();                                   // pins and motors are edited on other tabs: this one is rebuilt whenever it is opened or the settings are re-read

  return h('div', {},
    block(tr('Tiếng báo'),
      have,
      h('p', { class: 'help' }, tr('Còi đang kêu: '), now, ' · ', tr('motor đang phát: '), motorNow)),
    block(tr('Khi nào thì kêu'),
      h('p', { class: 'help' }, tr('Hai cột dấu chọn: còi gắn trên mạch, và motor (ESC làm motor rung thành tiếng bíp). Nút "Thử" phát âm đó một lần trên cả hai, kể cả khi nó đang bị tắt.')),
      h('table', { class: 'grid beeps', id: 'beeps' },
        h('thead', {}, h('tr', {}, h('th', {}, tr('Còi')), h('th', {}, 'Motor'), h('th', {}, tr('Tình huống')), h('th', {}, tr('Mẫu âm')), h('th', {}, ''))),
        h('tbody', {}, rows.map((r) => r.row))),
      h('div', { class: 'btn-row' }, silence)),
    block(tr('Motor kêu (DShot beacon)'),
      h('p', { class: 'help' }, tr('ESC dùng chính motor để phát tiếng bíp, nên máy không cần còi. Mỗi tiếng dài khoảng nửa giây: motor phát đúng SỐ tiếng của mẫu âm (3 tiếng = gyro hiệu chuẩn xong, 2 tiếng = vừa disarm), không phát được nhịp nhanh như còi.')),
      fields('beeper', ['dshot_beacon_tone']),
      note(tr('Motor chỉ kêu khi chưa arm và sau khi disarm 1,2 giây. Lúc đang bay không có tiếng báo nào từ motor: pin yếu khi bay phải xem trên tay điều khiển hoặc gắn thêm còi.'))),
    block(tr('Còi gắn trên mạch'),
      hasBuzzer() ? null : h('p', { class: 'help', id: 'no-buzzer' }, tr('Chưa gán chân cho còi (tab Chân và cổng). Các ô dưới đây chỉ có tác dụng khi có còi.')),
      fields('beeper', ['type', 'inverted', 'frequency', 'volume'])),
    store.schema.sections.find((x) => x.name === 'led') ? block(tr('Đèn LED (WS2812)'),
      fields('led', ['count', 'brightness', 'layout']),
      note(tr('Dải LED WS2812 / SK6812 nối vào chân LED (tab Chân, nhóm Khác). Xanh lá: sẵn sàng arm; vàng nhấp nháy: còn điều gì chặn arm; khi bay: màu theo mode (Acro trắng, Angle xanh dương, Stabilize xanh lơ, Altitude Hold tím, Position Hold xanh ngọc, GPS Rescue cam nhấp nháy); đỏ nháy nhanh: failsafe; đỏ nháy đôi: pin cạn. Gửi qua bus SPI thứ hai, nên không dùng được cùng thẻ SD kiểu SPI hoặc PMW3901. Có hiệu lực sau khi lưu và khởi động lại.')),
      fieldOf('led', 'fn') ? ledManual() : null) : null);
}

// ---- LED strip, layout MANUAL: a function and a color for each LED, a preview that runs the firmware's effects (fwmath.js ledColors)
const fieldOf = (section, name) => (store.schema.sectionByName[section] || { fields: [] }).fields.find((f) => f.n === name);
const hex = (v) => '#' + (Number(v) >>> 0 & 0xFFFFFF).toString(16).padStart(6, '0');
function ledManual() {
  const names = store.schema.enums.led_function;
  const allFn = h('select', { id: 'led-all-fn', 'aria-label': tr('Chức năng') }, names.map((n) => h('option', { value: n }, tr((ENUM_LABELS.led_function || {})[n] || n))));
  const allCol = h('input', { type: 'color', id: 'led-all-color', value: '#ffffff', 'aria-label': tr('Màu') });
  const allBtn = h('button', { type: 'button', class: 'btn', id: 'led-all' }, tr('Áp cho mọi LED'));
  allBtn.addEventListener('click', () => {
    const v = parseInt(allCol.value.slice(1), 16);
    setFields('led', { fn: Array(32).fill(allFn.value), color: Array(32).fill(v) });
  });
  const lockAll = () => { allFn.disabled = allCol.disabled = allBtn.disabled = store.armed; };
  listen('armed', lockAll); lockAll();
  const strip = h('div', { class: 'led-strip', id: 'led-preview' });
  const state = h('select', { id: 'led-state', 'aria-label': tr('Trạng thái xem thử') },
    h('option', { value: 'ready' }, tr('Chưa arm, sẵn sàng')), h('option', { value: 'armed' }, tr('Đang bay (Angle), ga 60 %, pin 70 %')), h('option', { value: 'failsafe' }, 'Failsafe'));
  const tbody = h('tbody', {});
  let shown = '';
  const rows = () => {
    const led = store.settings.led; const key = `${led.layout}/${led.count}`;
    if (key === shown) return; shown = key;
    clear(tbody); clear(strip);
    for (let i = 0; i < Math.min(32, led.count); i++) {
      const pick = h('input', { type: 'color', 'data-led-color': i, 'aria-label': tr('Màu LED {n}', { n: i + 1 }) });
      pick.addEventListener('change', () => setField('led', 'color', parseInt(pick.value.slice(1), 16), i));
      const sync = () => { if (document.activeElement !== pick) pick.value = hex(store.settings.led.color[i]); };
      const lock = () => { pick.disabled = store.armed; };
      listen('settings:led', sync); listen('armed', lock); sync(); lock();
      tbody.append(h('tr', {}, h('td', { class: 'num' }, String(i + 1)), h('td', {}, control('led', 'fn', { index: i })), h('td', {}, pick)));
      strip.append(h('span', { class: 'led-dot' }));
    }
    box.hidden = led.layout !== 'MANUAL';
  };
  const box = h('div', { id: 'led-manual' },
    h('h3', {}, tr('Tự gán từng LED')),
    h('p', { class: 'help' }, tr('Mỗi LED một chức năng: trạng thái (như kiểu "Mọi LED cùng màu"), màu cố định, màu theo mode, sáng khi arm, nháy, chớp, thở, cầu vồng, quét qua lại, mức pin, mức ga. Cầu vồng và quét chạy qua những LED có cùng chức năng đó, theo thứ tự trên dải.')),
    fields('led', ['warnings', 'speed']),
    h('div', { class: 'btn-row' }, allFn, allCol, allBtn),
    h('div', { class: 'ctl-line' }, h('span', {}, tr('Xem thử:')), state), strip,
    h('table', { class: 'grid', id: 'led-table' }, h('thead', {}, h('tr', {}, h('th', {}, 'LED'), h('th', {}, tr('Chức năng')), h('th', {}, tr('Màu')))), tbody));
  listen('settings:led', rows); rows();
  const t0 = performance.now();
  const timer = setInterval(() => {
    const led = store.settings.led; if (!led || box.hidden) return;
    const st = { armed: state.value === 'armed', ready: true, failsafe: state.value === 'failsafe', mode: state.value === 'armed' ? 'ANGLE' : 'ACRO', batt: 70, thr: state.value === 'armed' ? 60 : 0 };
    const cs = ledColors(led, st, Math.floor(performance.now() - t0));
    strip.querySelectorAll('.led-dot').forEach((d, i) => { if (cs[i]) d.style.background = `rgb(${cs[i].join(',')})`; });
  }, 40);
  track(() => clearInterval(timer));
  return box;
}

export default {
  id: 'power',
  title: tr('Pin và còi'),
  lead: tr('Điện áp pin, các ngưỡng cảnh báo, và tiếng báo (còi hoặc motor) cho từng tình huống.'),
  streams: { battery: 5, status: 10 },
  mount(view) {
    view.append(h('div', { class: 'cols' }, batteryColumn(), beeperColumn()));
  },
};
