// Overview: is every module there, why can't it arm, and does the 3D model move the way the craft moves in your hand.
import { store, emit } from '../store.js';
import { h, block, kv, readout, listen, failsafeText } from '../ui.js';
import { craftView } from '../gfx/craft3d.js';
import { BLOCKERS } from '../labels.js';
import { api } from '../api.js';
import { tr } from '../i18n.js';

const MODULES = [
  ['imu', 'IMU (gyro + accelerometer)'], ['baro', 'Barometer'], ['range', 'Lidar / sonar'], ['mag', tr('La bàn')],
  ['gps', 'GPS'], ['rx', 'Receiver'], ['motors', 'Motor / ESC'], ['battery', tr('Đo pin')], ['beeper', tr('Tiếng báo (còi / motor)')], ['blackbox', 'Blackbox'],
];
// lines of the boot report that are only there when there is something to say: a flow sensor is selected, a pin is wrong
const TROUBLE = [['flow', 'Optical flow'], ['pins', tr('Bảng chân')]];
const f = (v, d) => (v === undefined || v === null ? '–' : Number(v).toFixed(d));
const vec = (a, d) => (a ? a.map((x) => f(x, d)).join('   ') : '–');
const withId = (id, el) => { el.id = id; return el; };

function moduleTable() {
  const body = h('tbody');
  const paint = () => {
    // a module may have more than one line (blackbox: flash partition and SD card): it is missing only if every line says FAILED
    const parts = {};
    for (const l of store.conn.report || []) { const i = l.indexOf(': '); (parts[l.slice(0, i)] = parts[l.slice(0, i)] || []).push(l.slice(i + 2)); }
    const lines = Object.fromEntries(Object.entries(parts).map(([k, v]) => [k, v.join(' · ')]));
    const allFailed = (key) => (parts[key] || []).every((t) => t.includes('FAILED'));
    const extra = TROUBLE.filter(([key]) => lines[key] !== undefined);
    const flowRows = extra.filter(([key]) => key === 'flow'), others = extra.filter(([key]) => key !== 'flow');
    const at = MODULES.findIndex(([key]) => key === 'gps');        // the flow sensor belongs next to the other position sensors
    body.replaceChildren(...[...MODULES.slice(0, at + 1), ...flowRows, ...MODULES.slice(at + 1), ...others].map(([key, label]) => {
      const text = lines[key];
      const failed = text === undefined || allFailed(key);
      const warned = !failed && /WARNING|FAILED/.test(text);     // one of several lines failed (the SD card is missing, the flash is there)
      const off = text !== undefined && /disabled/.test(text);
      const detail = text === undefined ? tr('firmware không báo cáo') : text.replace(/(^| · )(FAILED|WARNING) - /g, '$1');
      return h('tr', { 'data-module': key },
        h('th', { scope: 'row' }, label),
        h('td', {}, h('span', { class: 'pill ' + (off ? 'off' : failed ? 'bad' : warned ? 'warn' : 'ok') }, off ? tr('Tắt') : failed ? (key === 'pins' ? tr('Lỗi') : tr('Không có')) : warned ? tr('Cảnh báo') : tr('Có'))),
        h('td', { class: 'detail' }, detail));
    }));
  };
  listen('conn', paint); paint();
  return h('table', { class: 'grid modules' }, h('thead', {}, h('tr', {}, h('th', {}, 'Module'), h('th', {}, tr('Trạng thái')), h('th', {}, tr('Firmware báo')))), body);
}

export default {
  id: 'overview',
  title: tr('Tổng quan'),
  lead: tr('Firmware tìm thấy gì lúc khởi động, vì sao chưa arm được, và các số đo đang chạy.'),
  streams: { attitude: 30, sensors: 10, gps: 2, estimate: 10, status: 10, battery: 2 },
  mount(view) {
    const craft = craftView(() => {
      const a = store.streams.attitude || {};
      return { roll: a.roll, pitch: a.pitch, yaw: a.yaw, layout: (store.settings.motors || {}).layout };
    }, 320);
    const recenter = h('button', { type: 'button', class: 'btn quiet', id: 'recenter' }, tr('Quay mũi máy về phía xa'));
    recenter.addEventListener('click', () => craft.recenter());

    const blockers = h('ul', { class: 'blockers', id: 'blockers' });
    const paintBlockers = (st) => {
      const items = st.armed ? [h('li', { class: 'bad' }, tr('Đang ARM'))] :
        st.blockers.length ? st.blockers.map((b) => h('li', { 'data-blocker': b }, BLOCKERS[b] ? tr(BLOCKERS[b]) : b)) : [h('li', { class: 'ok' }, tr('Không còn gì chặn. Gạt công tắc ARM để bật motor.'))];
      blockers.replaceChildren(...items);
    };
    listen('stream:status', paintBlockers);
    if (store.streams.status) paintBlockers(store.streams.status);

    const again = h('button', { type: 'button', class: 'btn quiet' }, tr('Hỏi lại firmware'));
    again.addEventListener('click', async () => { try { store.conn.report = (await api.get('/api/report')).report; emit('conn', { before: store.conn }); } catch (e) { /* shown by the bar */ } });

    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Tư thế'),
          craft,
          h('div', { class: 'big-read' },
            h('div', {}, h('span', { class: 'k' }, 'Roll'), readout('attitude', (a) => f(a.roll, 1) + '°')),
            h('div', {}, h('span', { class: 'k' }, 'Pitch'), readout('attitude', (a) => f(a.pitch, 1) + '°')),
            h('div', {}, h('span', { class: 'k' }, 'Yaw'), readout('attitude', (a) => f(a.yaw, 1) + '°'))),
          h('p', { class: 'help' }, tr('Nghiêng máy sang phải thì roll dương, chúi mũi xuống thì pitch dương, quay mũi sang phải thì yaw tăng. Hình phải nghiêng đúng như máy trên tay; nếu sai, sửa "Hướng chip IMU" ở tab Cảm biến.')),
          recenter),
        block(tr('Vì sao chưa arm được'), blockers)),
      h('div', {},
        block('Module', moduleTable(), again),
        block(tr('Số đo'),
          h('p', { class: 'help' }, tr('"Thời gian từng khâu" là trung bình của giây vừa rồi cho một chu kỳ điều khiển; cộng lại phải nhỏ hơn 1 000 000 / tần số đã đặt (250 µs ở 4000 Hz). Khâu nào lớn bất thường là chỗ cần xem: đọc IMU (dây SPI, tốc độ SPI), tính toán (bộ lọc, RPM filter), gửi motor (DShot); "còn lại" gồm chờ luồng cảm biến nhả khoá, ghi log và telemetry. Số lớn nhất của từng khâu nằm trong log blackbox (nhóm "Thời gian vòng lặp").')),
          kv([
            ['Gyro (°/s)', readout('sensors', (s) => vec(s.gyro, 1))],
            ['Accelerometer (g)', readout('sensors', (s) => vec(s.acc, 3))],
            [tr('La bàn (gauss)'), readout('sensors', (s) => vec(s.mag, 3))],
            ['Barometer', readout('sensors', (s) => `${f(s.baro_alt, 2)} m, ${s.baro_pa} Pa, ${f(s.temp, 1)} °C`)],
            ['Lidar', readout('sensors', (s) => (s.range > 0 ? f(s.range, 3) + ' m' : tr('không có số đo')))],
            [tr('Độ cao ước lượng'), readout('estimate', (e) => (e.valid.includes('ALT') ? `${f(e.z, 2)} m, ${f(e.vz, 2)} m/s` : tr('chưa hợp lệ')))],
            ['GPS', readout('gps', (g) => (g.fix >= 2 ? tr('fix {fix}D, {sats} vệ tinh, HDOP {hdop}', { fix: g.fix, sats: g.sats, hdop: f(g.hdop, 2) }) : tr('chưa fix, {sats} vệ tinh ({state})', { sats: g.sats, state: g.state })))],
            [tr('Pin'), readout('battery', (b) => (b.cells ? tr('{v} V, {cells} cell, {cell} V mỗi cell', { v: f(b.voltage, 2), cells: b.cells, cell: f(b.cell, 2) }) : tr('không có pin')))],
            [tr('Thời gian một chu kỳ'), readout('status', (s) => tr('{us} µs, dài nhất {max} µs', { us: s.loop_us, max: s.loop_max_us }))],
            [tr('Chu kỳ điều khiển thực tế'), withId('loop-rate', readout('status', (s) => (s.loop_hz ? (s.loop_target ? tr('{hz} Hz (đặt {target} Hz), bỏ lỡ {n} chu kỳ trong giây vừa rồi', { hz: s.loop_hz, target: s.loop_target, n: s.overruns }) : `${s.loop_hz} Hz`) : tr('đang đo'))))],
            [tr('Thời gian từng khâu'), withId('stage-times', readout('status', (s) => (s.loop_target ? tr('đọc IMU {imu} · tính toán {core} · gửi motor {motor} · còn lại {rest} µs', { imu: s.t_imu, core: s.t_core, motor: s.t_motor, rest: s.t_rest }) : '–')))],
            [tr('Luồng cảm biến, lượt dài nhất'), withId('slow-max', readout('status', (s) => (s.loop_target ? `${s.t_slow_max} µs` : '–')))],
            ['Failsafe', withId('fs-state', readout('status', (s) => (s.failsafe === 'NONE' ? tr('không') : failsafeText(s).replace('Failsafe: ', ''))))],
            [tr('CPU (nhân chạy vòng bay)'), withId('cpu-load', readout('status', (s) => `${s.cpu} %`))],
            ['RAM', withId('ram-use', readout('status', (s) => tr('{pct} % đang dùng, còn {free} kB', { pct: s.ram, free: s.heap_free })))],
            ['PSRAM', withId('psram-use', readout('status', (s) => (s.psram_total ? tr('còn {free} trên {total} kB', { free: s.psram_free, total: s.psram_total }) : tr('không có'))))],
          ])))));
  },
};
