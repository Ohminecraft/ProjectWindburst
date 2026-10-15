// Sensors: calibration (gyro, 6-face accelerometer, level, compass), board alignment, module switches, GPS.
import { store } from '../store.js';
import { h, block, fields, form, row, listen, note, cmdButton, toast, readout, kv, canvas, token, errorText } from '../ui.js';
import { ENUM_LABELS } from '../labels.js';
import { tr } from '../i18n.js';

const FACES = ['+X', '−X', '+Y', '−Y', '+Z', '−Z'];

function calibration() {
  const cal = () => store.streams.calibration || {};
  // ---------------- gyro
  const gyroState = h('span', { class: 'cal-state', id: 'gyro-state' });
  // ---------------- accelerometer
  const faces = h('div', { class: 'faces', id: 'faces' }, FACES.map((n, i) => h('span', { class: 'face', 'data-face': i }, n)));
  const accState = h('p', { class: 'cal-state', id: 'acc-state' });
  const accStored = h('p', { class: 'help', id: 'acc-stored' });
  let accApplied = false;       // the stream reports DONE both after one face and after the final fit: remember which it was
  const finish = cmdButton(tr('Tính và áp dụng'), 'ACC_CAL_FINISH', null, {
    class: 'primary',
    done: () => { accApplied = true; paint(); toast(tr('Đã áp dụng hiệu chuẩn accelerometer. Bấm "Lưu" để giữ lại.'), 'ok'); },
    fail: (e) => toast(e.code === 'BAD_STATE' ? tr('Chưa đủ 6 mặt.') : e.code === 'FAILED' ? tr('Số đo 6 mặt không hợp lý (chip lỗi hoặc máy bị động). Hủy và làm lại.') : errorText(e), 'bad'),
  });
  finish.id = 'acc-finish';
  // ---------------- compass
  const magPts = [];
  const magState = h('p', { class: 'cal-state', id: 'mag-state' });
  const magStored = h('p', { class: 'help', id: 'mag-stored' });
  const cloud = canvas(180, (ctx, w, hh) => {
    const half = w / 2, s = Math.min(half, hh) * 0.42;
    let r = 0.2;
    for (const p of magPts) r = Math.max(r, Math.abs(p[0]), Math.abs(p[1]), Math.abs(p[2]));
    ctx.font = `11px ${token('--ui')}`; ctx.fillStyle = token('--muted'); ctx.textAlign = 'center';
    [['X / Y', 0, 1], ['X / Z', 0, 2]].forEach(([label, a, b], k) => {
      const cx = half * k + half / 2, cy = hh / 2 - 6;
      ctx.strokeStyle = token('--line'); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(cx, cy, s, 0, 7); ctx.stroke();
      ctx.fillStyle = token('--muted'); ctx.fillText(label, cx, hh - 4);
      ctx.fillStyle = token('--series-1');
      for (const p of magPts) { ctx.beginPath(); ctx.arc(cx + (p[a] / r) * s, cy - (p[b] / r) * s, 1.6, 0, 7); ctx.fill(); }
    });
  }, 'cloud');
  let magRefused = false;       // the last "finish" was refused: keep its reason on screen while the user turns the craft some more
  const magStart = cmdButton(tr('Bắt đầu'), 'MAG_CAL_START', null, { done: () => { magPts.length = 0; magRefused = false; magState.textContent = tr('Đang thu mẫu: xoay máy chậm quanh mọi trục, như vẽ số 8.'); } });
  const magFinish = cmdButton(tr('Tính và áp dụng'), 'MAG_CAL_FINISH', null, {
    class: 'primary',
    done: () => { magRefused = false; magState.textContent = tr('Hiệu chuẩn la bàn thành công. Bấm "Lưu" để giữ lại.'); toast(tr('Đã áp dụng hiệu chuẩn la bàn.'), 'ok'); },
    fail: (e) => {
      const reason = e.extra && e.extra.result && e.extra.result.reason;
      magRefused = e.code === 'FAILED';
      const why = ENUM_LABELS.mag_cal_result[reason] ? tr(ENUM_LABELS.mag_cal_result[reason]) : (reason || tr('thất bại'));
      magState.textContent = e.code === 'FAILED' ? tr('Chưa được: {why}. Xoay tiếp rồi bấm lại.', { why }) :
        e.code === 'BAD_STATE' ? tr('Chưa bấm "Bắt đầu".') : errorText(e);
    },
  });
  const magCancel = cmdButton(tr('Hủy'), 'MAG_CAL_CANCEL', null, { class: 'quiet', done: () => { magRefused = false; magState.textContent = ''; } });
  magStart.id = 'mag-start'; magFinish.id = 'mag-finish';

  const paint = () => {
    const c = cal(), st = store.streams.status, s = store.settings.calibration || {};
    // gyro
    if (c.kind === 'GYRO' && c.state === 'RUNNING') gyroState.textContent = tr('đang chạy {n} %', { n: c.progress });
    else if (st) gyroState.textContent = st.health.includes('GYRO_CAL') ? tr('đã hiệu chuẩn') : tr('chưa xong (máy phải đứng yên)');
    // accelerometer
    const mask = c.kind === 'ACCEL' ? c.mask : 0;
    for (const el of faces.children) el.classList.toggle('done', !!((mask >> Number(el.dataset.face)) & 1));
    const count = FACES.filter((_, i) => (mask >> i) & 1).length;
    if (accApplied) accState.textContent = tr('Đã tính xong và đang dùng hiệu chuẩn mới.');
    else if (c.kind === 'ACCEL') {
      accState.textContent = c.state === 'RUNNING' ? tr('Đang lấy mẫu {n} %, giữ yên…', { n: c.progress }) :
        c.state === 'FAILED' ? tr('Mặt này chưa lấy được: máy bị động hoặc chưa nằm hẳn lên một mặt. Đặt lại rồi bấm lần nữa.') :
        count === 6 ? tr('Đủ 6 mặt. Bấm "Tính và áp dụng".') : tr('Đã có {n}/6 mặt. Xoay máy sang mặt khác rồi bấm "Lấy mặt này".', { n: count });
    } else accState.textContent = '';
    accStored.textContent = s.acc_valid ? tr('Đang dùng hiệu chuẩn: offset {offset} g, scale {scale}.', { offset: s.acc_offset.map((x) => x.toFixed(3)).join(', '), scale: s.acc_scale.map((x) => x.toFixed(3)).join(', ') }) : tr('Chưa có hiệu chuẩn accelerometer (đang dùng số đo thô của chip).');
    // compass
    if (c.kind === 'MAG' && c.state === 'RUNNING') {
      if (c.raw && (!magPts.length || magPts[magPts.length - 1].some((v, i) => Math.abs(v - c.raw[i]) > 0.01))) { magPts.push([...c.raw]); if (magPts.length > 1500) magPts.shift(); cloud.paint(); }
      if (!magRefused) magState.textContent = tr('Đang thu mẫu ({n} % độ phủ): xoay máy chậm quanh mọi trục, như vẽ số 8.', { n: c.progress });
    }
    magStored.textContent = s.mag_valid ? tr('Đang dùng hiệu chuẩn la bàn (bán kính {r} gauss).', { r: s.mag_radius.toFixed(3) }) : tr('La bàn chưa hiệu chuẩn.');
  };
  listen('stream:calibration', paint); listen('stream:status', paint); listen('settings:calibration', paint);
  paint();

  return [
    block('Gyro',
      h('div', { class: 'btn-row' }, cmdButton(tr('Hiệu chuẩn gyro'), 'GYRO_CAL', null, { done: () => toast(tr('Đang hiệu chuẩn gyro, giữ máy đứng yên khoảng 1 giây.')) }), gyroState),
      h('p', { class: 'help' }, tr('Firmware tự làm việc này mỗi lần khởi động. Chỉ cần bấm lại nếu máy bị động lúc cấp nguồn.'))),
    block(tr('Accelerometer, 6 mặt'),
      h('p', { class: 'help' }, tr('Đặt máy lần lượt lên 6 mặt: nằm bình thường, lật úp, dựng mũi lên trời, chúi mũi xuống đất, nghiêng hẳn sang trái, nghiêng hẳn sang phải. Thứ tự tùy ý, firmware tự nhận ra mặt nào. Mỗi mặt giữ yên rồi bấm "Lấy mặt này".')),
      faces, accState,
      h('div', { class: 'btn-row' }, cmdButton(tr('Lấy mặt này'), 'ACC_CAL_POINT', null, { done: () => { accApplied = false; }, fail: (e) => toast(e.code === 'BUSY' ? tr('Đang lấy mẫu mặt trước, chờ một giây.') : errorText(e), 'bad') }), finish, cmdButton(tr('Hủy'), 'ACC_CAL_CANCEL', null, { class: 'quiet', done: () => { accApplied = false; } })),
      accStored),
    block(tr('Cân bằng (level trim)'),
      h('p', { class: 'help' }, tr('Đặt máy trên mặt phẳng ngang, giữ yên, rồi bấm. Firmware ghi độ lệch còn lại để Angle mode giữ máy thật phẳng. Lệch quá 20° thì đó là hướng lắp chip, sửa ở "Hướng chip IMU".')),
      h('div', { class: 'btn-row' }, cmdButton(tr('Lấy tư thế này làm phẳng'), 'LEVEL_TRIM', null, {
        done: () => toast(tr('Đã lấy level trim. Bấm "Lưu" để giữ lại.'), 'ok'),
        fail: (e) => toast(e.code === 'FAILED' ? tr('Không lấy được: máy đang động, hoặc nghiêng quá 20°.') : errorText(e), 'bad') })),
      form(row('calibration', 'acc_trim_roll', { label: 'Trim roll', unit: '°' }), row('calibration', 'acc_trim_pitch', { label: 'Trim pitch', unit: '°' })),
      h('h3', {}, tr('Level trim khi bay')),
      h('p', { class: 'help' }, tr('Máy vẫn trôi khi cần ở giữa? Gán mode "Level trim khi bay" cho một công tắc (tab Mode). Khi bay: giữ máy treo yên một chỗ bằng cần, bật công tắc ít nhất 2 giây rồi tắt, hạ cánh và disarm. Tư thế lúc treo yên thành "cân bằng" mới, hai số trim ở trên đổi theo và được tự lưu vào flash.')),
      kv([[tr('Trạng thái'), h('span', { id: 'inflight-trim' }, readout('calibration', (c) => (c.inflight_trim === 0 ? tr('chưa đo') : (c.inflight_trim >= 100 ? tr('đã đo xong, sẽ áp dụng khi disarm') : tr('đang đo: {n} %', { n: c.inflight_trim })))))]])),
    block(tr('La bàn'),
      h('p', { class: 'help' }, tr('Làm ngoài trời hoặc xa kim loại và dây nguồn. Bấm "Bắt đầu", xoay máy chậm theo mọi hướng cho các chấm phủ kín hai vòng tròn, rồi bấm "Tính và áp dụng".')),
      cloud, magState,
      h('div', { class: 'btn-row' }, magStart, magFinish, magCancel),
      magStored),
  ];
}

// the second IMU: its state, which chip(s) the loop flies on, and its gyro next to the first one's (for checking its orientation)
function imu2Text(s) {
  if (s.imu2 === undefined || s.imu2 === 'NONE') return tr('không có');
  const st = ENUM_LABELS.imu2_state[s.imu2] ? tr(ENUM_LABELS.imu2_state[s.imu2]) : s.imu2;
  if (s.imu2 !== 'OK') return st;
  const src = (s.gyro_src || []).join(' + ') || '–';
  const g = (v) => (v || []).map((x) => Number(x).toFixed(0)).join(' / ');
  return tr('{state}, vòng bay dùng {src}; gyro IMU 2 {g2} (IMU 1 {g1}) °/s', { state: st, src, g2: g(s.gyro2), g1: g(s.gyro) });
}

// the CAN bus: its state, and the nodes on it (id, name, health) - the one that gives the GNSS fix / the compass marked
function canText(c) {
  if (!c || c.state === undefined) return '–';
  const st = ENUM_LABELS.can_state[c.state] ? tr(ENUM_LABELS.can_state[c.state]) : c.state;
  if (c.state !== 'RUNNING' && c.state !== 'BUS_OFF') return st;
  let t = tr('{state}, node {id}; {rx} khung/s nhận, {tx} gửi, {drop} bỏ; lỗi TX {tec} / RX {rec}, bus-off {off} lần; đã cấp {alloc} node id', { state: st, id: c.node_id, rx: c.rx_fps, tx: c.tx_fps, drop: c.tx_dropped, tec: c.tec, rec: c.rec, off: c.bus_off, alloc: c.allocations });
  if (c.esc_sent || c.esc_mask) {                          // ESCs on the bus (motors.protocol DRONECAN)
    const heard = []; for (let i = 0; i < 8; i++) if (c.esc_mask & (1 << i)) heard.push(i);
    t += '. ' + tr('ESC: đã gửi {n} lệnh, bỏ {drop}; nghe thấy ESC {list}', { n: c.esc_sent, drop: c.esc_dropped, list: heard.length ? heard.join(', ') : tr('(chưa có)') });
  }
  return t;
}
const HEALTH = ['OK', 'WARNING', 'ERROR', 'CRITICAL'];
function paintCanNodes() {
  const box = document.getElementById('can-nodes'); const c = store.streams.can;
  if (!box || !c || !c.nodes) { if (box) box.replaceChildren(); return; }
  const dec = new TextDecoder(); const rows = [];
  for (let i = 0; i < Math.min(c.nodes, 8); i++) {
    const raw = c.names.slice(i * 16, i * 16 + 16); const z = raw.indexOf(0); const name = dec.decode(Uint8Array.from(z < 0 ? raw : raw.slice(0, z))) || tr('(chưa biết tên)');
    const uses = [c.gnss_node === c.node[i] ? 'GNSS' : '', c.mag_node === c.node[i] ? tr('la bàn') : '', c.baro_node === c.node[i] ? tr('khí áp kế') : '', c.batt_node === c.node[i] ? tr('pin') : ''].filter(Boolean).join(', ');
    const health = c.health[i] === 255 ? tr('mất kết nối') : (HEALTH[c.health[i]] || String(c.health[i]));
    const up = tr('chạy {s} s', { s: c.uptime[i] }), use = uses ? tr('đang dùng: {what}', { what: uses }) : '';
    rows.push([tr('Node {id}', { id: c.node[i] }), [name, health, up, use].filter(Boolean).join(' · ')]);
  }
  box.replaceChildren(kv(rows));
}

export default {
  id: 'sensors',
  title: tr('Cảm biến'),
  lead: tr('Hiệu chuẩn, hướng lắp chip và bật tắt từng module.'),
  streams: { calibration: 15, sensors: 10, gps: 2, status: 10, can: 2 },
  mount(view) {
    const f = (v, d) => (v === undefined || v === null ? '–' : Number(v).toFixed(d));
    view.append(h('div', { class: 'cols' },
      h('div', {}, calibration()),
      h('div', {},
        block(tr('IMU và vòng điều khiển'),
          fields('general', ['board_align', 'board_trim_roll', 'board_trim_pitch', 'board_trim_yaw', 'loop_hz', 'pid_denom', 'est_hz', 'max_arm_angle', 'airmode', 'airmode_start_throttle']),
          fields('sensors', ['gyro_use', 'imu2_align']),
          kv([[tr('IMU thứ hai'), h('span', { id: 'imu2-state' }, readout('sensors', (s) => imu2Text(s)))],
            [tr('Đang chạy'), h('span', { id: 'rates-now' }, readout('status', (s) => (s.pid_target ? tr('gyro {gyro} Hz, PID {pid} Hz', { gyro: s.loop_target, pid: s.pid_target }) : '–')))]]),
          note(tr('Các mục này có hiệu lực sau khi lưu và khởi động lại.'))),
        block('Module',
          fields('sensors', ['baro_enabled', 'baro_model', 'range_enabled', 'range_model', 'range_l1x_mode', 'mag_enabled', 'mag_model', 'mag_align', 'mag_trim_roll', 'mag_trim_pitch', 'mag_trim_yaw', 'mag_declination', 'gps_enabled']),
          note(tr('Bật hoặc tắt module cần lưu và khởi động lại. Module bật mà firmware không tìm thấy sẽ hiện "Không có" ở tab Tổng quan kèm lý do.'))),
        block('GPS',
          kv([
            [tr('Trạng thái'), readout('gps', (g) => `${g.state}, ${g.baud} baud`)],
            [tr('Tần số lời giải'), readout('gps', (g) => (g.rate > 0 ? `${f(g.rate, 1)} Hz` + (g.rate < 4.5 ? tr(' (chậm: Position Hold bằng GPS cần từ 5 Hz; module NMEA thì tăng baud lên 57600)') : '') : '–'))],
            ['Fix', readout('gps', (g) => (g.fix >= 2 ? tr('{fix}D, {sats} vệ tinh, HDOP {hdop}', { fix: g.fix, sats: g.sats, hdop: f(g.hdop, 2) }) : tr('chưa fix, {sats} vệ tinh', { sats: g.sats })))],
            [tr('Vị trí'), readout('gps', (g) => (g.fix >= 2 ? tr('{lat}, {lon}, cao {alt} m', { lat: f(g.lat, 7), lon: f(g.lon, 7), alt: f(g.alt, 1) }) : '–'))],
            ['Home', readout('gps', (g) => (g.home_dist || g.home_dir ? tr('cách {d} m, hướng {dir}°', { d: g.home_dist, dir: f(g.home_dir, 0) }) : tr('cách 0 m')))],
          ]),
          h('div', { class: 'btn-row' }, cmdButton(tr('Đặt home tại đây'), 'SET_HOME', null, {
            done: () => toast(tr('Đã đặt home.'), 'ok'),
            fail: (e) => toast(e.code === 'NO_SENSOR' ? tr('GPS chưa có fix đủ tốt để đặt home.') : errorText(e), 'bad') })),
          fields('gps', ['provider', 'baud', 'rate_hz', 'auto_baud', 'auto_config', 'min_sats', 'sbas', 'sbas_integrity', 'use_galileo', 'acquire_model', 'flight_model', 'utc_standard', 'set_home_once', 'use_3d_speed'], { baud: { choices: [9600, 19200, 38400, 57600, 115200, 230400] } })),
        store.schema.sections.find((x) => x.name === 'can') ? block(tr('CAN bus (DroneCAN)'),
          fields('can', ['mode', 'bitrate', 'node_id', 'dna_server']),
          h('div', { id: 'can-state' }, readout('can', (c) => canText(c))),
          h('div', { id: 'can-nodes' }), 
          note(tr('Cần một bộ thu phát CAN 3,3 V (SN65HVD230, TJA1051T/3...) và hai chân ở tab Chân (CAN bus). Module trên bus: GPS provider, loại la bàn, loại khí áp kế, nguồn đo điện áp / dòng chọn DRONECAN; ESC trên bus: giao thức ESC DRONECAN (tab Motor). Có hiệu lực sau khi lưu và khởi động lại.'))) : null)));
    listen('stream:can', () => paintCanNodes());
  },
};
