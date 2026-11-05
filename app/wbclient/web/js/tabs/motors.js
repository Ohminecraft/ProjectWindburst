// Motors: frame layout, ESC protocol, spin direction, motor order, motor test with live RPM, and the gyro vibration graph.
import { store, setField, setFields, flush, wantStreams, command } from '../store.js';
import { h, block, fields, control, listen, note, toast, showError, track, readout, cmdButton, errorText, kv } from '../ui.js';
import { LAYOUTS, motorCount } from '../gfx/fwmath.js';
import { ENUM_LABELS } from '../labels.js';
import { scope } from '../gfx/scope.js';
import { tr } from '../i18n.js';
import { api } from '../api.js';

const SVGNS = 'http://www.w3.org/2000/svg';
function s(tag, attrs, ...kids) {
  const el = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs || {})) el.setAttribute(k, v);
  for (const kid of kids) el.append(kid.nodeType ? kid : document.createTextNode(kid));
  return el;
}

// Top view of the frame: motor numbers, the direction each prop must turn, and a ring per motor (for the live command).
// Returns the svg, the ring of each motor and its group (the motor wizard clicks on those).
function frameSvg(m) {
    const L = LAYOUTS[m.layout], n = L.angles.length;
    const svg = s('svg', { viewBox: '-150 -152 300 310', role: 'img', 'aria-label': tr('Khung {layout} nhìn từ trên xuống, {n} motor', { layout: m.layout, n }) });
    svg.append(s('path', { d: 'M0 -150 l-12 20 h24 z', class: 'nose' }), s('text', { x: 0, y: -116, class: 'nose-t', 'text-anchor': 'middle' }, tr('mũi')));
    const fills = [], groups = [];
    // where motor i is drawn: the end of its arm (r: fraction of a full arm). A coaxial pair (Y6, X8, Y4) sits side by side
    // across its arm - the upper motor clockwise of it, the lower one the other way - so that neither disc covers the other
    const coax = !!(L.under && L.under.some((u) => u));
    const disc = coax ? 28 : n > 6 ? 26 : n > 4 ? 32 : 36, R = coax ? 104 : 92;
    const armEnd = (i) => { const a = (L.angles[i] * Math.PI) / 180, rr = L.r ? L.r[i] : 1; return [Math.sin(a) * R * rr, -Math.cos(a) * R * rr, a]; };
    const paired = (i) => coax && L.angles.some((ang, j) => j !== i && ang === L.angles[i] && (L.r ? L.r[j] === L.r[i] : true));
    const P = L.angles.map((ang, i) => {
      const [x, y, a] = armEnd(i);
      if (!paired(i)) return [x, y];
      const side = L.under[i] ? -1 : 1, off = disc + 6;                  // across the arm: (cos a, sin a) on the screen
      return [x + Math.cos(a) * off * side, y + Math.sin(a) * off * side];
    });
    // motors on arms that are close together (A-tail, V-tail) are pushed apart until their discs clear each other
    const gap = 2 * disc + 8;
    for (let it = 0; it < 20; it++) {
      for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) {
        const dx = P[j][0] - P[i][0], dy = P[j][1] - P[i][1], d = Math.hypot(dx, dy) || 1;
        if (d >= gap) continue;
        const push = (gap - d) / 2, ux = dx / d, uy = dy / d;
        P[i][0] -= ux * push; P[i][1] -= uy * push; P[j][0] += ux * push; P[j][1] += uy * push;
      }
    }
    const pos = (i) => P[i];
    const arms = new Set();
    L.angles.forEach((ang, i) => {
      const [x, y] = paired(i) ? armEnd(i) : P[i], key = `${Math.round(x)},${Math.round(y)}`;        // (a pair shares its arm)
      if (arms.has(key)) return;
      arms.add(key);
      svg.append(s('line', { x1: 0, y1: 0, x2: x, y2: y, class: 'arm' }));
    });
    svg.append(s('rect', { x: -20, y: -30, width: 40, height: 60, rx: 6, class: 'hub' }));
    L.angles.forEach((ang, i) => {
      const [x, y] = pos(i), under = !!(L.under && L.under[i]), k = disc / 36;
      const ccw = (L.spin[i] > 0) !== !!m.props_out;          // +1 = counter-clockwise prop; props-out flips every motor
      const g = s('g', { transform: `translate(${x} ${y})`, 'data-motor': i, 'data-spin': ccw ? 'ccw' : 'cw', 'data-under': under ? '1' : '0' });
      const C = 2 * Math.PI * disc;
      const fill = s('circle', { r: disc, class: 'cmd', 'stroke-dasharray': `0 ${C}`, transform: 'rotate(-90)' });
      fills.push({ fill, C });
      // arrow along the top of the disc, on a circle just inside the ring and centred on the motor: it runs over the top from
      // one side to the other (left to right = clockwise) and its head follows the circle
      const r = disc - 9 * k, dir = ccw ? -1 : 1, deg = Math.PI / 180;
      const from = (ccw ? -35 : -145) * deg, to = (ccw ? -145 : -35) * deg, head = 9 * k;
      const at = (t) => [r * Math.cos(t), r * Math.sin(t)];
      const stop = to - dir * (head / r);                                   // the line ends where the head starts
      const [x0, y0] = at(from), [x1, y1] = at(stop), [px, py] = at(to);
      const arc = `M ${x0.toFixed(2)} ${y0.toFixed(2)} A ${r.toFixed(2)} ${r.toFixed(2)} 0 0 ${ccw ? 0 : 1} ${x1.toFixed(2)} ${y1.toFixed(2)}`;
      const tx = -Math.sin(to) * dir, ty = Math.cos(to) * dir, nx = Math.cos(to), ny = Math.sin(to), w = 5.5 * k;    // tangent, outward normal
      const bx = px - tx * head, by = py - ty * head;
      const tip = `M ${(px + tx * 2 * k).toFixed(2)} ${(py + ty * 2 * k).toFixed(2)} L ${(bx + nx * w).toFixed(2)} ${(by + ny * w).toFixed(2)} L ${(bx - nx * w).toFixed(2)} ${(by - ny * w).toFixed(2)} z`;
      g.append(s('circle', { r: disc, class: under ? 'disc under' : 'disc' }), fill, s('path', { d: arc, class: 'spin' }), s('path', { d: tip, class: 'spin-tip' }),
        s('text', { y: 10 * k, 'text-anchor': 'middle', class: 'mnum', style: `font-size: ${(26 * k).toFixed(1)}px` }, String(i + 1) + (under ? '↓' : '')),
        s('text', { y: disc + 16, 'text-anchor': 'middle', class: 'mtag' }, ''));
      groups.push(g);
      svg.append(g);
    });
    return { svg, fills, groups };
}

function diagram() {
  const box = h('div', { class: 'frame', id: 'frame' });
  let fills = [];
  const build = () => {
    const f = frameSvg(store.settings.motors); fills = f.fills;
    box.replaceChildren(f.svg, h('p', { class: 'help' }, tr('Mũi tên là chiều mỗi cánh PHẢI quay (nhìn từ trên xuống). Vòng tô đậm là lệnh đang gửi tới ESC.')));
  };
  const live = (mo) => { fills.forEach(({ fill, C }, i) => fill.setAttribute('stroke-dasharray', `${(mo.cmd[i] || 0) * C} ${C}`)); };
  let sig = '';
  listen('settings:motors', () => { const m = store.settings.motors, k = m.layout + m.props_out; if (k !== sig) { sig = k; build(); } });
  listen('stream:motors', live);
  sig = store.settings.motors.layout + store.settings.motors.props_out; build();
  return box;
}

function motorTest() {
  const n = () => motorCount(store.settings.motors.layout);
  const values = new Array(8).fill(0);
  const safe = h('input', { type: 'checkbox', id: 'props-off' });
  const sliders = h('div', { class: 'test-sliders', id: 'test-sliders' });
  const master = h('input', { type: 'range', min: 0, max: 100, step: 1, value: 0, id: 'test-all', disabled: true, 'aria-label': tr('Tất cả motor') });
  const masterVal = h('span', { class: 'num' }, '0 %');
  const stop = h('button', { type: 'button', class: 'btn danger', id: 'test-stop' }, tr('Dừng tất cả'));
  let inputs = [], timer = null, sending = false, failed = false, dirty = false, live = false;

  const zero = () => { values.fill(0); inputs.forEach((x) => { x.input.value = 0; x.val.textContent = '0 %'; }); master.value = 0; masterVal.textContent = '0 %'; };
  const enable = () => { const on = safe.checked && !store.armed; inputs.forEach((x) => { x.input.disabled = !on; }); master.disabled = !on; };
  const halt = async () => { zero(); live = false; try { await command('MOTOR_STOP'); } catch (e) { /* not connected: nothing is spinning through us */ } };

  // The firmware stops a motor that is not refreshed for 0.5 s, so a frozen page can never leave a motor running.
  // A slider moved while a command is on its way is not dropped: the newest values go out as soon as that command is answered
  // (dragged from 0 to 100 % in one go, the motor gets 100 % at once, not at the next 200 ms refresh). Brought back to 0, the
  // motors are stopped at once (before 0.35.1 nothing was sent: they ran on at the old value until the 0.5 s timeout).
  async function tick() {
    if (!safe.checked) return;
    if (sending) { dirty = true; return; }
    const count = n(), active = values.slice(0, count);
    const on = active.some((v) => v > 0);
    if (!on && !live) return;
    sending = true; dirty = false;
    try {
      if (!on) await command('MOTOR_STOP');
      else if (active.every((v) => v === active[0])) await command('MOTOR_TEST', { motor: 255, value: active[0] * 10 });
      else for (let i = 0; i < count; i++) await command('MOTOR_TEST', { motor: i, value: active[i] * 10 });
      live = on; failed = false;
    } catch (e) {
      if (!failed) showError(e);
      failed = true; zero(); live = false;
    }
    sending = false;
    if (dirty) tick();
  }

  const build = () => {
    inputs = [];
    sliders.replaceChildren(...Array.from({ length: n() }, (_, i) => {
      const input = h('input', { type: 'range', min: 0, max: 100, step: 1, value: 0, disabled: true, 'data-test': i, 'aria-label': `Motor ${i + 1}` });
      const val = h('span', { class: 'num' }, '0 %');
      input.addEventListener('input', () => { values[i] = Number(input.value); val.textContent = values[i] + ' %'; tick(); });
      inputs.push({ input, val });
      return h('div', { class: 'test-row' }, h('span', { class: 'test-n' }, `Motor ${i + 1}`), input, val,
        h('span', { class: 'num rpm' }, readout('motors', (m) => (store.settings.motors.bidirectional || store.settings.motors.protocol === 'DRONECAN' ? `${m.rpm[i]} rpm` : ''))));
    }));
    enable();
  };
  master.addEventListener('input', () => {
    const v = Number(master.value); masterVal.textContent = v + ' %';
    for (let i = 0; i < n(); i++) { values[i] = v; inputs[i].input.value = v; inputs[i].val.textContent = v + ' %'; }
    tick();
  });
  safe.addEventListener('change', () => { enable(); if (!safe.checked) halt(); });
  stop.addEventListener('click', halt);
  timer = setInterval(tick, 200);
  const hidden = () => { if (document.hidden && values.some((v) => v > 0)) halt(); };
  document.addEventListener('visibilitychange', hidden);
  track(() => { clearInterval(timer); document.removeEventListener('visibilitychange', hidden); if (values.some((v) => v > 0)) command('MOTOR_STOP').catch(() => {}); });
  listen('armed', () => { if (store.armed) zero(); enable(); });
  let count = n();
  listen('settings:motors', () => { if (n() !== count) { count = n(); zero(); build(); } });
  build();

  return block('Test motor',
    h('label', { class: 'tag-warn', for: 'props-off' }, safe, h('span', {}, h('strong', {}, tr('Đã tháo hết cánh quạt.')), tr(' Motor sẽ quay thật khi kéo thanh trượt.'))),
    sliders,
    h('div', { class: 'test-row all' }, h('span', { class: 'test-n' }, tr('Tất cả')), master, masterVal, stop),
    h('p', { class: 'help' }, tr('Kéo từng thanh để biết motor số mấy nằm ở đâu và quay chiều nào. Sai vị trí thì sửa "Thứ tự motor"; sai chiều thì bật "Đảo chiều" cho motor đó.')));
}

// Motor wizard (Betaflight Configurator 4.3+): the order and the spin directions are found by looking at the motors, not typed in.
//   Order:     each motor output turns in turn (slowly); click where on the frame the turning motor sits. At the end the new
//              motors.order is written; the firmware takes it at once (FlightApp::apply_motor_order), "Lưu" keeps it.
//   Direction: click a motor to turn it (or turn all), compare with the arrow, "Đảo chiều" sends the ESC its new direction
//              (DShot 7 / 8 + save) and sets motors.reversed_mask.
// Every 200 ms the test is refreshed: the firmware stops a motor that is not refreshed for 0.5 s.
function wizard() {
  const box = h('div', { class: 'frame wizard', id: 'wiz-frame' });
  const msg = h('p', { class: 'note', id: 'wiz-msg' }, tr('Tháo cánh, đánh dấu "Đã tháo hết cánh quạt" ở khối Test motor, rồi chọn một bước.'));
  const pct = h('input', { type: 'number', class: 'num', id: 'wiz-pct', min: 3, max: 25, step: 1, value: 8, 'aria-label': tr('Ga khi quay thử (%)') });
  pct.style.width = '64px';
  const bOrder = h('button', { type: 'button', class: 'btn', id: 'wiz-order' }, tr('Sắp xếp lại thứ tự motor'));
  const bDir = h('button', { type: 'button', class: 'btn', id: 'wiz-dir' }, tr('Chiều quay motor'));
  const bStop = h('button', { type: 'button', class: 'btn danger', id: 'wiz-stop' }, tr('Dừng'));
  const all = h('input', { type: 'checkbox', id: 'wiz-all' });
  const bFlip = h('button', { type: 'button', class: 'btn', id: 'wiz-flip', disabled: true }, tr('Đảo chiều motor này'));
  const dirRow = h('div', { class: 'btn-row', id: 'wiz-dir-row', hidden: true }, h('label', { class: 'inline' }, all, tr(' Quay tất cả')), bFlip);
  let mode = null, groups = [], timer = null, busy = false;
  let k = 0, cur = [], next = [], sel = -1;                      // order: the output turning now, the old order, the new one; direction: the motor picked

  const n = () => motorCount(store.settings.motors.layout);
  const value = () => Math.max(3, Math.min(25, Number(pct.value) || 8)) * 10;
  const say = (text, kind = '') => { msg.className = 'note ' + kind; msg.textContent = text; };
  const tag = (i, text, cls) => { const g = groups[i]; if (!g) return; g.querySelector('.mtag').textContent = text; g.dataset.state = cls || ''; };
  const draw = () => {
    const f = frameSvg(store.settings.motors); groups = f.groups;
    groups.forEach((g, i) => g.addEventListener('click', () => pick(i)));
    box.replaceChildren(f.svg);
    if (mode === 'order') next.forEach((slot, p) => { if (slot !== null) tag(p, 'Slot ' + (slot + 1), 'done'); });
    if (mode === 'dir') paintDir();
  };
  const paintDir = () => { const mask = store.settings.motors.reversed_mask; for (let i = 0; i < n(); i++) tag(i, (mask >> i) & 1 ? tr('đảo') : '', i === sel || all.checked ? 'on' : ''); };
  async function tick() {
    if (!mode || busy) return;
    busy = true;
    try {
      if (mode === 'order') await command('MOTOR_TEST', { motor: k, value: value() });
      else if (all.checked) await command('MOTOR_TEST', { motor: 255, value: value() });
      else if (sel >= 0) await command('MOTOR_TEST', { motor: sel, value: value() });
    } catch (e) { showError(e); await stop(); }
    busy = false;
  }
  async function stop(text) {
    const was = mode;
    mode = null; clearInterval(timer); timer = null; sel = -1; dirRow.hidden = true;
    if (was) { try { await command('MOTOR_STOP'); } catch (e) { /* not connected */ } }
    draw();
    if (text) say(text, 'ok'); else if (was) say(tr('Đã dừng. Không có gì bị đổi.'));
  }
  // what must hold before a motor may turn
  const ready = () => {
    const st = store.streams.status, flags = st ? String(st.settings_dirty) : '';
    if (!store.conn.connected) return tr('Chưa kết nối.');
    if (store.armed) return tr('Máy đang arm.');
    if (!(document.getElementById('props-off') || {}).checked) return tr('Đánh dấu "Đã tháo hết cánh quạt" ở khối Test motor trước.');
    if (flags.includes('REBOOT_REQUIRED')) return tr('Có cài đặt đang chờ khởi động lại (giao thức, kiểu khung ...): bấm "Lưu và khởi động lại" trước, wizard chạy trên cấu hình motor đang chạy.');
    return '';
  };
  async function start(which) {
    await stop();
    const why = ready();
    if (why) { say(why, 'bad'); return; }
    if (which === 'dir' && !/^DSHOT/.test(String(store.settings.motors.protocol))) { say(tr('Đổi chiều bằng lệnh chỉ làm được với DShot. Với PWM / Oneshot / Multishot hãy đổi hai trong ba dây motor.'), 'bad'); return; }
    mode = which;
    if (which === 'order') {
      cur = store.settings.motors.order.slice(); next = new Array(n()).fill(null); k = 0;
      say(tr('Motor đang quay: bấm vào vị trí của nó trên hình ({i} / {n}).', { i: 1, n: n() }));
    } else {
      sel = -1; dirRow.hidden = false; all.checked = false;
      say(tr('Bấm vào một motor để nó quay, so với mũi tên trên hình. Quay sai thì bấm "Đảo chiều motor này".'));
    }
    draw();
    timer = setInterval(tick, 200); tick();
  }
  async function pick(p) {
    if (mode === 'order') {
      if (next[p] !== null) { toast(tr('Vị trí này đã có motor. Bấm vào vị trí của motor đang quay.'), 'warn'); return; }
      next[p] = cur[k]; tag(p, 'Slot ' + (cur[k] + 1), 'done'); k++;
      try { await command('MOTOR_STOP'); } catch (e) { /* the next tick shows it */ }
      if (k < n()) { say(tr('Motor đang quay: bấm vào vị trí của nó trên hình ({i} / {n}).', { i: k + 1, n: n() })); return; }
      const order = cur.slice(); next.forEach((slot, i) => { order[i] = slot; });
      mode = null; clearInterval(timer); timer = null;
      setFields('motors', { order }); await flush('motors');
      draw();
      const changed = order.some((v, i) => v !== cur[i]);
      say(changed ? tr('Xong: thứ tự mới đang chạy. Kiểm tra bằng thanh test motor, rồi bấm "Lưu" để giữ.') : tr('Xong: thứ tự không đổi, các motor đã nằm đúng chỗ.'), 'ok');
    } else if (mode === 'dir') {
      if (all.checked) { await flip(p); return; }
      sel = p; bFlip.disabled = false; paintDir();
      try { await command('MOTOR_STOP'); } catch (e) { /* the next tick turns the new one */ }
    }
  }
  async function flip(p) {
    const mask = store.settings.motors.reversed_mask, rev = !((mask >> p) & 1);
    try {
      await command('MOTOR_DIRECTION', { motor: p, reversed: rev ? 1 : 0 });
      setField('motors', 'reversed_mask', rev ? (mask | (1 << p)) : (mask & ~(1 << p)));
      paintDir();
      toast(tr('Motor {n}: đã gửi chiều mới cho ESC. Bấm "Lưu" để giữ.', { n: p + 1 }), 'ok');
    } catch (e) { showError(e); }
  }
  bOrder.addEventListener('click', () => start('order'));
  bDir.addEventListener('click', () => start('dir'));
  bStop.addEventListener('click', () => stop());
  bFlip.addEventListener('click', () => { if (sel >= 0) flip(sel); });
  all.addEventListener('change', () => { sel = -1; bFlip.disabled = true; paintDir(); if (!all.checked) command('MOTOR_STOP').catch(() => {}); });
  const lock = () => { bOrder.disabled = bDir.disabled = store.armed; if (store.armed && mode) stop(); };
  listen('armed', lock); lock();
  let sig = '';
  listen('settings:motors', () => { const m = store.settings.motors, s2 = m.layout + m.props_out; if (s2 !== sig) { sig = s2; if (mode) stop(); else draw(); } else if (mode === 'dir') paintDir(); });
  const hidden = () => { if (document.hidden && mode) stop(); };
  document.addEventListener('visibilitychange', hidden);
  track(() => { document.removeEventListener('visibilitychange', hidden); if (mode) { clearInterval(timer); command('MOTOR_STOP').catch(() => {}); } });
  sig = store.settings.motors.layout + store.settings.motors.props_out; draw();
  return block(tr('Motor wizard'),
    h('p', { class: 'help' }, tr('Thay cho chọn tay "Slot phần cứng" và "Đảo chiều". Sắp xếp: từng motor quay lần lượt, bấm vào vị trí của motor đang quay trên hình. Chiều quay (chỉ DShot): bấm vào motor để nó quay, quay ngược mũi tên thì bấm "Đảo chiều motor này". Thứ tự mới chạy ngay; bấm "Lưu" để giữ.')),
    h('div', { class: 'btn-row' }, bOrder, bDir, bStop, h('label', { class: 'inline' }, tr('Ga khi quay thử'), ' ', pct, ' %')),
    dirRow, msg, box);
}

// "Motor i is wired to hardware slot k" (motors.order). Shown 1-based like the Slot boxes of the pins tab.
function slotSelect(i) {
  const sel = h('select', { 'data-order': i, 'aria-label': tr('Slot phần cứng của motor {n}', { n: i + 1 }) }, Array.from({ length: 8 }, (_, k) => h('option', { value: k }, 'Slot ' + (k + 1))));
  const sync = () => { sel.value = store.settings.motors.order[i]; sel.disabled = store.armed; };
  sel.addEventListener('change', () => setField('motors', 'order', Number(sel.value), i));
  listen('settings:motors', sync); sync();
  return sel;
}

function perMotor() {
  const body = h('tbody', { id: 'motor-rows' });
  const build = () => {
    const m = store.settings.motors, n = motorCount(m.layout);
    body.replaceChildren(...Array.from({ length: n }, (_, i) => {
      const rev = h('input', { type: 'checkbox', class: 'switch', role: 'switch', 'data-rev': i, 'aria-label': tr('Đảo chiều motor {n}', { n: i + 1 }) });
      rev.checked = !!((store.settings.motors.reversed_mask >> i) & 1);
      rev.disabled = store.armed;
      rev.addEventListener('change', async () => {
        const mask = store.settings.motors.reversed_mask;
        setField('motors', 'reversed_mask', rev.checked ? (mask | (1 << i)) : (mask & ~(1 << i)));
        try { await command('MOTOR_DIRECTION', { motor: i, reversed: rev.checked ? 1 : 0 }); toast(tr('Đã gửi lệnh đổi chiều cho ESC {n}. Bấm "Lưu" để giữ.', { n: i + 1 }), 'ok'); }
        catch (e) { toast(e.code === 'NOT_SUPPORTED' ? tr('Chỉ đổi chiều bằng lệnh được với DShot. Với PWM / Oneshot hãy đổi hai dây motor.') : errorText(e), 'warn', 6000); }
      });
      return h('tr', {},
        h('th', { scope: 'row' }, `Motor ${i + 1}`),
        h('td', {}, readout('motors', (mo) => Math.round(mo.cmd[i] * 100) + ' %')),
        // '–' while the rpm is stale (rpm_ok bit clear): a 0 there would read as "motor stopped"
        h('td', {}, readout('motors', (mo) => (!(m.bidirectional || m.esc_sensor || m.protocol === 'DRONECAN') ? '–' : mo.rpm_ok === undefined || ((mo.rpm_ok >> i) & 1) ? String(mo.rpm[i]) : '–'))),
        h('td', {}, (() => {                       // the share of frames without a good reply: amber from 1 %, red from 10 % (100 %: the ESC does not answer)
          const el = readout('motors', (mo) => (m.bidirectional ? mo.err[i] + ' %' : '–'));
          el.dataset.motorErr = String(i);
          listen('stream:motors', (mo) => { const e = m.bidirectional ? mo.err[i] : 0; el.dataset.level = e >= 10 ? 'bad' : e >= 1 ? 'warn' : ''; });
          return el; })()),
        h('td', {}, readout('motors', (mo) => (mo.temp[i] ? mo.temp[i] + ' °C' : '–'))),
        h('td', {}, readout('motors', (mo) => (mo.amps && mo.amps[i] ? mo.amps[i].toFixed(1) + ' A' : '–'))),
        h('td', {}, slotSelect(i)),
        h('td', {}, rev),
        h('td', {}, cmdButton(tr('Kêu'), 'MOTOR_BEACON', { motor: i }, { class: 'quiet', fail: (e) => toast(e.code === 'NOT_SUPPORTED' ? tr('ESC chỉ kêu được qua DShot.') : errorText(e), 'warn') })));
    }));
  };
  let sig = '';
  const maybe = () => { const m = store.settings.motors, k = [m.layout, m.bidirectional, m.esc_sensor, m.reversed_mask].join(); if (k !== sig) { sig = k; build(); } };
  listen('settings:motors', maybe); listen('armed', build); maybe();
  return block(tr('Từng motor'),
    h('table', { class: 'grid motors' },
      h('thead', {}, h('tr', {}, ['', tr('Lệnh'), 'RPM', tr('Lỗi telemetry'), tr('Nhiệt độ ESC'), tr('Dòng'), tr('Slot phần cứng'), tr('Đảo chiều'), ''].map((t) => h('th', {}, t)))), body),
    h('p', { class: 'help' }, tr('Slot phần cứng: motor này nối vào chân "Slot" nào ở tab Chân và cổng. RPM có khi bật DShot hai chiều hoặc dây telemetry ESC; lỗi telemetry chỉ với DShot hai chiều. Dòng: extended DShot telemetry hoặc dây telemetry ESC.')),
    h('p', { class: 'help' }, tr('"Khung bỏ qua": vòng điều khiển gọi nhanh hơn một khung DShot kèm lượt trả lời có thể đi hết (DShot300 hai chiều cần 182 µs, DShot600 cần 116 µs) - hạ tần số vòng hoặc dùng DShot nhanh hơn. "Lượt trả lời bị lỡ": firmware đổi chiều chân tín hiệu không kịp.')),
    h('div', { class: 'link-read', id: 'dshot-state' }, h('span', {}, tr('DShot hai chiều: '), readout('motors', (mo) => {
      const label = ENUM_LABELS.bidir_state[mo.bidir]; let text = label ? tr(label) : String(mo.bidir);
      if (mo.bidir === 'ACTIVE') {
        // which ESCs send replies at all: one that never answers has no bidirectional firmware, or its signal wire is the problem
        const n = motorCount(store.settings.motors.layout), silent = [];
        for (let i = 0; i < n; i++) if (!((mo.answers >> i) & 1)) silent.push(i + 1);
        text += silent.length ? tr(', ESC không trả lời: {list}', { list: silent.join(', ') }) : tr(', mọi ESC đều trả lời');
      }
      return text;
    })),
      h('span', {}, tr('Khung DShot bỏ qua trong giây vừa rồi: '), readout('motors', (mo) => String(mo.skipped))),
      h('span', {}, tr('Lượt trả lời bị lỡ: '), readout('motors', (mo) => String(mo.late))),
      h('span', {}, tr('Trả lời nhận được / hỏng mỗi giây: '), readout('motors', (mo) => `${mo.replies} / ${mo.garbled}`)),
      h('span', {}, readout('motors', (mo) => (mo.no_start === undefined ? '' : tr('không có bit start: {a}, sai số bit: {b}, sai checksum: {c}', { a: mo.no_start, b: mo.bad_bits, c: mo.bad_crc }))))),
    h('div', { class: 'link-read', id: 'esc-wire-state' }, h('span', {}, tr('Dây telemetry ESC, mỗi giây: '), readout('motors', (mo) => (store.settings.motors.esc_sensor
      ? tr('{ok} trả lời, {crc} hỏng CRC, {lost} ESC không trả lời', { ok: mo.tlm_frames, crc: mo.tlm_crc, lost: mo.tlm_timeouts }) : tr('tắt'))))),
    h('div', { class: 'link-read' }, h('span', {}, tr('Điện áp ESC báo: '), readout('motors', (mo) => (mo.volt[0] ? mo.volt[0].toFixed(2) + ' V' : tr('không có')))),
      h('span', {}, 'Dynamic idle: ', readout('motors', (mo) => (mo.dyn_idle * 100).toFixed(1) + ' %'))));
}

// The gyro's vibration: roll, pitch and yaw at once, one colour each - raw (what the frame does) or filtered (what the PID sees).
// Below it the RMS of every axis, raw and filtered, over the last 2 s.
function vibration() {
  const AX = ['Roll', 'Pitch', 'Yaw'];
  const enable = h('input', { type: 'checkbox', class: 'switch', role: 'switch', id: 'vib-on' });
  const which = h('select', { id: 'vib-signal', 'aria-label': tr('Tín hiệu') }, h('option', { value: 'raw' }, tr('Gyro thô')), h('option', { value: 'filt' }, tr('Sau bộ lọc')));
  const sc = scope({ series: AX.map((a, i) => ({ name: a, color: `--series-${i + 1}` })), windowMs: 2000, unit: ' °/s', height: 230, minRange: 8 });
  const stats = h('div', { class: 'help num', id: 'vib-stats' }, '');
  // the last 2 s of both signals, every axis (for the RMS table: the scope keeps only what it draws)
  const N = 1000; const buf = [0, 1, 2, 3, 4, 5].map(() => new Float32Array(N)); let at = 0, filled = 0;
  enable.addEventListener('change', () => { wantStreams('vibration', enable.checked ? { gyro_fast: 500 } : null); if (!enable.checked) { sc.clear(); filled = 0; } });
  which.addEventListener('change', () => sc.clear());
  listen('fast', (rows) => {
    const off = which.value === 'raw' ? 1 : 4;
    for (const r of rows) {
      sc.push(r[0], [r[off], r[off + 1], r[off + 2]]);
      for (let k = 0; k < 6; k++) buf[k][at] = r[1 + k];
      at = (at + 1) % N; if (filled < N) filled++;
    }
  });
  const rms = (k) => { let a = 0, m = 0; for (let i = 0; i < filled; i++) m += buf[k][i]; m /= filled; for (let i = 0; i < filled; i++) { const d = buf[k][i] - m; a += d * d; } return Math.sqrt(a / filled); };
  const t = setInterval(() => {
    if (filled < 10) { stats.textContent = ''; return; }
    stats.replaceChildren(kv(AX.map((a, i) => [a, tr('thô {raw} °/s RMS, sau lọc {filt} °/s RMS', { raw: rms(i).toFixed(1), filt: rms(3 + i).toFixed(1) })])));
  }, 500);
  track(() => { clearInterval(t); wantStreams('vibration', null); });
  return block(tr('Độ rung (gyro)'),
    h('div', { class: 'btn-row' }, h('label', { class: 'inline' }, enable, tr(' Bật đồ thị (500 mẫu/giây)')), which),
    sc.el, stats,
    h('p', { class: 'help' }, tr('Ba trục cùng lúc, mỗi trục một màu. Chạy từng motor ở tab này và xem gyro thô: motor hoặc cánh nào làm gyro rung mạnh hơn hẳn các motor khác là motor cần kiểm tra (cong trục, lỏng ốc, cánh mẻ).')));
}

export default {
  id: 'motors',
  title: tr('Motor'),
  lead: tr('Kiểu khung, ESC, chiều quay và thứ tự motor. Tháo cánh trước khi test.'),
  streams: { motors: 20, status: 10 },
  mount(view) {
    const pulse = h('div', { id: 'pulse-fields' }, fields('motors', ['pulse_disarmed', 'pulse_armed', 'pulse_max', 'pulse_hz']),
      h('p', { class: 'help' }, tr('Với PWM, Oneshot, Multishot và brushed không có "Motor idle": ga thấp nhất khi đã arm chính là "Xung khi arm" (như min_throttle của Betaflight).')));
    const dshot = h('div', { id: 'dshot-fields' }, fields('motors', ['bidirectional', 'ext_pullup', 'pole_count', 'edt', 'esc_sensor']),
      h('p', { class: 'help' }, tr('Dynamic idle (cần DShot hai chiều) ở tab PID.')));
    const idle = h('div', { id: 'idle-field' }, fields('motors', ['idle_x10']));
    const tooMany = h('p', { class: 'note bad', id: 'dshot-count' });
    const canNote = h('p', { class: 'help', id: 'can-esc-note' }, tr('ESC DroneCAN: lệnh ga đi trên bus CAN (esc.RawCommand, 400 lần/giây), ESC index = slot motor (thứ tự motor vẫn áp dụng). Không cần chân motor; CAN bus phải bật (tab Cảm biến). Vòng quay, điện áp, dòng, nhiệt độ lấy từ esc.Status của ESC. Không có lệnh DShot (beacon, đổi chiều): chiều quay đặt trong cấu hình của ESC.'));
    const pulse3d = h('div', { id: 'pulse3d-fields' }, fields('motors', ['deadband3d_low', 'deadband3d_high', 'neutral3d', 'limit3d_low', 'limit3d_high']));
    const which = () => {
      const m = store.settings.motors, d = /^(DSHOT|PROSHOT)/.test(String(m.protocol));
      const can = m.protocol === 'DRONECAN';
      pulse3d.hidden = d || can || m.protocol === 'BRUSHED' || !m.mode3d;
      pulse.hidden = d || can; dshot.hidden = !d; idle.hidden = !(d || can); canNote.hidden = !can;
      const n = motorCount(m.layout); tooMany.hidden = !(d && n > 4);
      tooMany.textContent = tr('Khung này có {n} motor: DShot / ProShot trên ESP32-S3 chỉ chạy được 4 (4 bộ phát RMT). Chọn PWM, Oneshot hoặc Multishot.', { n });
    };
    listen('settings:motors', which); which();
    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Khung'), diagram(), fields('motors', ['layout', 'props_out'])),
        motorTest(),
        wizard(),
        vibration()),
      h('div', {},
        block('ESC', fields('motors', ['protocol']), idle, fields('motors', ['output_limit']), tooMany, canNote, pulse, dshot,
          note(tr('Kiểu khung và giao thức ESC có hiệu lực sau khi lưu và khởi động lại; thứ tự motor và chiều quay có hiệu lực ngay (lưu để giữ). DShot qua RMT của ESP32-S3 chạy được tối đa 4 motor.'))),
        perMotor(),
        block(tr('Motor dừng giữa chừng'), fields('motors', ['stall_disarm', 'stall_min_rpm', 'stall_drop', 'stall_min_cmd', 'stall_ms']),
          h('div', { class: 'link-read', id: 'stall-state' }, h('span', {}, tr('Lần gần nhất: '), readout('motors', (mo) => (mo.stall_motor ? tr('motor {n} đã dừng (máy đã disarm)', { n: mo.stall_motor }) : tr('chưa có lần nào'))))),
          note(tr('Cần tốc độ của từng motor: DShot hai chiều hoặc dây telemetry ESC. Motor không có tốc độ thì không bị xét. Có hiệu lực sau khi lưu và khởi động lại.'))),
        block(tr('Crash flip (lật lại sau khi rơi)'), fields('motors', ['crashflip_rate', 'crashflip_motor_pct', 'crashflip_auto_rearm']),
          note(tr('Gán công tắc CRASHFLIP ở tab Chế độ. Chỉ chạy với ESC DShot (lệnh 21 đảo chiều quay, không lưu vào ESC; lệnh 20 trả lại). Có hiệu lực sau khi lưu và khởi động lại.'))),
        block(tr('Mode 3D (motor quay hai chiều)'), fields('motors', ['mode3d', 'switched_mode3d']), pulse3d,
          note(tr('DShot: 3D dùng 48..1047 cho chiều ngược, 1048..2047 cho chiều thuận, không cần các xung ở trên. Deadband ga 3D ở tab Receiver, công tắc 3D ở tab Chế độ. Bật 3D thì crash flip, runaway lúc cất cánh và dynamic idle tắt. Có hiệu lực sau khi lưu và khởi động lại.'))),
        escToolBlock(),
        servos())));
  },
};

// ESC passthrough: the ESC tools (BLHeliSuite32, ESC Configurator, AM32 Configurator) on this same port, after this app has let go of it.
function escToolBlock() {
  const btn = h('button', { type: 'button', class: 'btn', id: 'esc-tool-mode' }, tr('Nhả cổng cho BLHeliSuite32'));
  const msg = h('div', { id: 'esc-tool-result' });
  btn.addEventListener('click', async () => {
    try {
      await api.post('/api/disconnect');
      msg.replaceChildren(h('p', { class: 'note ok' }, tr('Xong. App đã nhả cổng: mở BLHeliSuite32 / ESC Configurator trên cổng này rồi bấm Connect.')));
    } catch (e) { showError(e); }
  });
  return block(tr('Cấu hình ESC (BLHeliSuite32, ESC Configurator)'),
    h('p', { class: 'help', id: 'esc-passthrough-help' }, tr('Firmware có ESC passthrough như Betaflight. Cắm pin, tháo cánh, bấm nút dưới (app sẽ tự ngắt kết nối), rồi mở BLHeliSuite32 / ESC Configurator / AM32 Configurator trên cùng cổng và bấm Connect. Motor ngừng phát trong lúc đó; bấm Disconnect trong công cụ thì firmware tự khởi động lại. Không làm được khi đang ARM.')),
    h('div', { class: 'btn-row' }, btn), msg);
}

// Servos: each follows one RC channel, or the mission (DO_SET_SERVO). Live settings; the pins are on the Pins tab.
function servos() {
  const live = (i) => readout('motors', (m) => (m.servo && m.servo[i] ? `${m.servo[i]} µs` : tr('tắt')));
  return block('Servo',
    h('p', { class: 'help' }, tr('Mỗi servo đi theo một kênh RC: kênh ở 1000 µs cho xung "nhỏ nhất", ở 2000 µs cho xung "lớn nhất". Có tác dụng ngay; chân servo gán ở tab Chân và cổng (cần lưu và khởi động lại). Nguồn servo lấy từ BEC 5 / 6 V riêng, nối chung GND.')),
    h('p', { class: 'help' }, tr('Kênh "Nhiệm vụ": chỉ nhiệm vụ điều khiển servo này (bước servo ở tab Nhiệm vụ). Góc ở xung nhỏ nhất / lớn nhất cho app đổi số độ ra xung; trước khi nhiệm vụ đụng tới, servo đứng ở "Xung ban đầu".')),
    h('p', { class: 'help' }, tr('Kênh "Nhiệm vụ": chỉ nhiệm vụ điều khiển servo này (bước servo ở tab Nhiệm vụ). Góc ở xung nhỏ nhất / lớn nhất cho app đổi số độ ra xung; trước khi nhiệm vụ đụng tới, servo đứng ở "Xung ban đầu".')),
    h('table', { class: 'grid', id: 'servo-table' },
      h('tr', {}, h('th', {}, ''), h('th', {}, tr('Kênh RC')), h('th', {}, tr('Nhỏ nhất (µs)')), h('th', {}, tr('Lớn nhất (µs)')), h('th', {}, tr('Đảo chiều')),
        h('th', {}, tr('Góc ở xung nhỏ nhất (°)')), h('th', {}, tr('Góc ở xung lớn nhất (°)')), h('th', {}, tr('Xung ban đầu (µs)')), h('th', {}, tr('Đang phát'))),
      [0, 1, 2, 3].map((i) => h('tr', {}, h('td', {}, `Servo ${i + 1}`), h('td', {}, control('servo', 'source', { index: i })),
        h('td', {}, control('servo', 'min_us', { index: i, width: '80px' })), h('td', {}, control('servo', 'max_us', { index: i, width: '80px' })),
        h('td', {}, control('servo', 'reverse', { index: i })),
        h('td', {}, control('servo', 'angle_min', { index: i, width: '70px' })), h('td', {}, control('servo', 'angle_max', { index: i, width: '70px' })), h('td', {}, control('servo', 'init_us', { index: i, width: '80px' })),
        h('td', { id: `servo-us-${i}` }, live(i))))),
    fields('servo', ['rate_hz']));
}
