// Position: the optical flow sensor (what it measures right now, how it sits on the craft), the position estimate,
// Position Hold and GPS Rescue. Everything in the "nav" section is live; the sensor type (section "flow") is read at power-up.
import { store } from '../store.js';
import { h, block, fields, kv, readout, listen, note, canvas, token } from '../ui.js';
import { ENUM_LABELS } from '../labels.js';
import { tr } from '../i18n.js';

const f = (v, d) => (v === undefined || v === null ? '–' : Number(v).toFixed(d));
const withId = (id, el) => { el.id = id; return el; };
const label = (group, v) => (ENUM_LABELS[group] && ENUM_LABELS[group][v] ? tr(ENUM_LABELS[group][v]) : String(v));
const has = (list, name) => Array.isArray(list) && list.includes(name);

// The speed over the ground as the flow sensor sees it, drawn the way the pilot looks at the craft from above: the nose is up.
// Carry the craft forwards and the arrow must point up; to the right and it must point right. Exported for the tests.
export function dialGeometry(w, hh, fwd, right) {
  const cx = w / 2, cy = hh / 2, R = Math.max(10, Math.min(w, hh) / 2 - 24);
  const speed = Math.hypot(fwd || 0, right || 0);
  const full = speed > 1 ? Math.ceil(speed) : 1;                       // m/s at the outer ring
  return { cx, cy, R, full, x: cx + ((right || 0) / full) * R, y: cy - ((fwd || 0) / full) * R };
}
function flowDial() {
  const cv = canvas(210, (ctx, w, hh) => {
    const n = store.streams.nav;
    const used = !!n && n.flow_reject === 'OK';
    const g = dialGeometry(w, hh, used ? n.flow_fwd : 0, used ? n.flow_right : 0);
    ctx.strokeStyle = token('--line'); ctx.lineWidth = 1;
    for (const k of [0.5, 1]) { ctx.beginPath(); ctx.arc(g.cx, g.cy, g.R * k, 0, 7); ctx.stroke(); }
    ctx.beginPath(); ctx.moveTo(g.cx - g.R, g.cy); ctx.lineTo(g.cx + g.R, g.cy); ctx.moveTo(g.cx, g.cy - g.R); ctx.lineTo(g.cx, g.cy + g.R); ctx.stroke();
    ctx.font = `11px ${token('--ui')}`; ctx.fillStyle = token('--muted');
    ctx.textAlign = 'center'; ctx.fillText(tr('mũi máy'), g.cx, g.cy - g.R - 8);
    ctx.textAlign = 'left'; ctx.fillText(tr('phải'), g.cx + g.R + 5, g.cy + 4);
    ctx.fillText(`${g.full} m/s`, g.cx + 4, g.cy - g.R + 12); ctx.fillText(`${g.full / 2} m/s`, g.cx + 4, g.cy - g.R / 2 + 12);
    if (!used) return;
    ctx.strokeStyle = ctx.fillStyle = token('--series-1'); ctx.lineWidth = 3; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(g.cx, g.cy); ctx.lineTo(g.x, g.y); ctx.stroke();
    ctx.beginPath(); ctx.arc(g.x, g.y, 5, 0, 7); ctx.fill();
  }, 'flow-dial');
  cv.id = 'flow-dial';
  cv.setAttribute('role', 'img');
  cv.setAttribute('aria-label', tr('Tốc độ optical flow đang đo: mũi tên chỉ hướng máy đang đi, mũi máy ở phía trên'));
  listen('stream:nav', () => cv.paint());
  return cv;
}

// What the firmware said about the flow sensor when it started (boot report lines "flow: ...").
function flowReport() {
  const el = h('span', { id: 'flow-report' });
  const paint = () => {
    const lines = (store.conn.report || []).filter((l) => l.startsWith('flow: ')).map((l) => l.slice(6).replace(/^(FAILED|WARNING) - /, ''));
    const type = (store.settings.flow || {}).type;
    el.textContent = lines.length ? lines.join(' · ') : (type && type !== 'NONE' ? tr('firmware chưa khởi động lại từ khi chọn cảm biến') : tr('chưa chọn cảm biến'));
  };
  listen('settings:flow', paint); paint();          // (after a restart the settings are read again: that repaints it with the new report)
  return el;
}

function pill(id, stream, pick) {
  const el = h('span', { class: 'pill off', id }, '–');
  const up = (v) => { const [cls, text] = pick(v); el.className = 'pill ' + cls; if (el.textContent !== text) el.textContent = text; };
  listen('stream:' + stream, up);
  if (store.streams[stream]) up(store.streams[stream]);
  return el;
}

export default {
  id: 'nav',
  title: tr('Vị trí'),
  lead: tr('Cảm biến optical flow, ước lượng vị trí, giữ vị trí (Position Hold) và tự bay về (GPS Rescue).'),
  streams: { nav: 10, estimate: 10, sensors: 5, gps: 2, status: 10, attitude: 5 },
  mount(view) {
    const pair = (a, b, d, unit) => `${f(a, d)} / ${f(b, d)} ${unit}`;
    view.append(
      note(tr('Các chức năng ở tab này mới được kiểm chứng bằng mô phỏng trên máy tính, chưa bay thật. Lần đầu thử: bay thấp, chỗ rộng, tay sẵn sàng tắt công tắc Position Hold / Rescue để lấy lại quyền điều khiển.'), 'todo'),
      h('div', { class: 'cols' },
        h('div', {},
          block(tr('Cảm biến optical flow'),
            fields('flow', ['type', 'baud']),
            note(tr('Loại cảm biến và baud có hiệu lực sau khi lưu và khởi động lại. Chân nối: tab Chân và cổng, nhóm "Optical flow".')),
            kv([
              [tr('Firmware báo'), flowReport()],
              [tr('Số mẫu nhận được'), withId('flow-rate', readout('nav', (n) => tr('{n} mẫu/giây', { n: n.flow_rate })))],
              [tr('Bề mặt bên dưới'), withId('flow-quality', readout('nav', (n) => tr('chất lượng {q} / 100', { q: n.flow_quality })))],
              [tr('Mẫu mới nhất'), withId('flow-use', readout('nav', (n) => label('flow_reject', n.flow_reject)))],
              [tr('Tốc độ trên mặt đất'), withId('flow-speed', readout('nav', (n) => (n.flow_reject === 'OK' ? tr('tiến {fwd} m/s, sang phải {right} m/s', { fwd: f(n.flow_fwd, 2), right: f(n.flow_right, 2) }) : '–')))],
              ['Lidar', withId('flow-range', readout('sensors', (s) => (s.range > 0 ? f(s.range, 2) + ' m' : tr('không có số đo'))))],
            ]),
            flowDial(),
            h('p', { class: 'help' }, tr('Kiểm tra hướng lắp: cầm máy cách sàn 0,5 đến 1 m (sàn có hoa văn, đủ sáng), giữ phẳng. Đưa máy về phía mũi: mũi tên phải chỉ lên. Đưa sang phải: mũi tên chỉ sang phải. Mũi tên lệch một phần tư hay nửa vòng: đổi "Hướng lắp cảm biến". Trái phải ngược nhau: bật "Ảnh bị lật gương". Chỉ lắc máy tại chỗ thì mũi tên phải gần như đứng yên.')),
            fields('nav', ['flow_align', 'flow_flip', 'flow_scale', 'flow_quality_min', 'flow_max_height'])),
          block(tr('Ước lượng vị trí'),
            kv([
              [tr('Vận tốc lấy từ'), withId('est-source', readout('estimate', (e) => (has(e.valid, 'FLOW') ? tr('optical flow') : has(e.valid, 'VEL') ? 'GPS' : tr('chưa có'))))],
              [tr('Vận tốc (bắc / đông)'), withId('est-vel', readout('estimate', (e) => (has(e.valid, 'VEL') ? pair(e.vel_n, e.vel_e, 2, 'm/s') : '–')))],
              [tr('Vị trí so với home (bắc / đông)'), withId('est-pos', readout('estimate', (e) => (has(e.valid, 'POS') ? pair(e.pos_n, e.pos_e, 1, 'm') : tr('chưa có (cần GPS fix)'))))],
              [tr('Hướng mũi'), withId('est-heading', readout('estimate', (e) => (has(e.valid, 'YAW_FROM_MAG') ? tr('theo la bàn') : tr('chỉ theo gyro (không biết hướng bắc)'))))],
              [tr('Góc nghiêng ước lượng'), withId('est-tilt', readout('estimate', (e) => (has(e.valid, 'TILT_AIDED') ? tr('đang được chỉnh theo gia tốc đo từ GPS / optical flow') : tr('chỉ theo accelerometer'))))],
            ]),
            fields('nav', ['tilt_aid'])),
          block(tr('EKF (thử nghiệm)'),
            h('p', { class: 'help' }, tr('Bộ lọc gộp tư thế, vận tốc, vị trí và bias của IMU vào một: khi la bàn bị nhiễu, vận tốc GPS sửa được hướng mũi mỗi lần máy tăng tốc. Mặc định chỉ chạy song song và ghi vào blackbox (ekf_yaw, ekf_pn…) để so; chọn NAV thì Position Hold và GPS Rescue dùng nó. Chỉ chọn NAV sau khi log cho thấy nó tốt hơn.')),
            kv([
              [tr('Trạng thái'), withId('ekf-state', readout('estimate', (e) => (!has(e.ekf_flags, 'ALIGNED') ? tr('chưa chạy / chưa căn') : [has(e.ekf_flags, 'GPS_AIDING') ? tr('có GPS') : tr('không GPS'), has(e.ekf_flags, 'HEADING_OK') ? tr('biết hướng') : tr('chưa chắc hướng'), has(e.ekf_flags, 'USED_FOR_NAV') ? tr('đang dùng để giữ vị trí') : tr('chỉ ghi log')].join(', '))))],
              [tr('Hướng mũi: EKF / AHRS'), withId('ekf-yaw', readout('estimate', (e) => (has(e.ekf_flags, 'ALIGNED') ? `${f(e.ekf_yaw, 1)}° ± ${f(e.ekf_yaw_sigma, 1)} / ${f(((store.streams.attitude || {}).yaw), 1)}°` : '–')))],
              [tr('Vị trí EKF (bắc / đông)'), withId('ekf-pos', readout('estimate', (e) => (has(e.ekf_flags, 'GPS_AIDING') ? pair(e.ekf_pn, e.ekf_pe, 1, 'm') : '–')))],
              [tr('Một bước EKF tốn'), withId('ekf-us', readout('estimate', (e) => (e.ekf_us ? `${e.ekf_us} µs` : '–')))],
            ]),
            fields('nav', ['ekf'])),
          block(tr('Độ cao từ GPS'),
            h('p', { class: 'help' }, tr('Độ cao GPS trôi một đến vài mét mỗi phút, barometer thì ổn định hơn trong vài giây nhưng trôi theo thời tiết. Vì vậy mặc định độ cao GPS chỉ dùng khi không có barometer / lidar; còn vận tốc lên / xuống của GPS (u-blox) thì luôn có ích, nhất là trong gió cánh quạt và khi leo nhanh.')),
            kv([
              [tr('Độ cao GPS'), withId('est-gps-alt', readout('estimate', (e) => (has(e.valid, 'GPS_ALT') ? tr('đang dùng trong ước lượng độ cao') : tr('không dùng lúc này'))))],
              [tr('Vận tốc đứng GPS'), withId('est-gps-vz', readout('estimate', (e) => (has(e.valid, 'GPS_VZ') ? tr('đang dùng trong ước lượng độ cao') : tr('không dùng lúc này'))))],
            ]),
            fields('nav', ['gps_alt', 'gps_vel_z']),
            note(tr('Chỉ có điểm home (lúc arm, GPS đã fix) thì mới có độ cao GPS. Lần đầu dùng Altitude Hold chỉ với GPS: bay ở chỗ thoáng, cao trên 5 m, sẵn sàng tắt công tắc.')))),
        h('div', {},
          block('Position Hold',
            h('p', { class: 'help' }, tr('Buông cần roll / pitch thì máy tự phanh rồi đứng tại chỗ, chống lại gió; đẩy cần thì bay như Angle. Cần một trong hai: optical flow + lidar (trong nhà, bay thấp), hoặc GPS đã fix + la bàn đã hiệu chuẩn. Gán công tắc ở tab Mode và nên bật cùng Altitude Hold.')),
            kv([
              [tr('Trạng thái'), pill('ph-state', 'nav', (n) => (has(n.flags, 'FENCE') ? ['warn', tr('chạm hàng rào: đang giữ ở mép, đẩy cần về phía home để bay tiếp')] : has(n.flags, 'POSHOLD') ? ['ok', tr('đang giữ vị trí')] : has(n.flags, 'POSHOLD_SLOW_GPS') ? ['warn', tr('GPS gửi quá ít lời giải mỗi giây để giữ vị trí (xem tần số GPS ở tab Cảm biến)')] : has(n.flags, 'POSHOLD_UNAVAILABLE') ? ['warn', tr('công tắc đang bật nhưng chưa có gì để giữ vị trí')] : ['off', tr('tắt')]))],
              ['Roll / Pitch', withId('ph-axes', readout('nav', (n) => (has(n.flags, 'POSHOLD') ? `${label('ph_axis', n.ph_roll)} / ${label('ph_axis', n.ph_pitch)}` : '–')))],
              [tr('Góc nghiêng autopilot yêu cầu (roll / pitch)'), withId('ph-lean', readout('nav', (n) => (has(n.flags, 'NAV_LEAN') ? pair(n.lean_roll, n.lean_pitch, 1, '°') : '–')))],
              [tr('Điểm đang giữ (bắc / đông)'), withId('ph-target', readout('nav', (n) => (has(n.flags, 'POSHOLD') ? pair(n.target_n, n.target_e, 1, 'm') : '–')))],
              [tr('Bù gió đã học (bắc / đông)'), withId('ph-wind', readout('nav', (n) => (has(n.flags, 'POSHOLD') ? pair(n.wind_n, n.wind_e, 2, 'm/s²') : '–')))],
            ]),
            fields('nav', ['ph_angle_max', 'ph_brake_rate', 'ph_brake_angle', 'pos_p', 'vel_p', 'vel_i', 'vel_d', 'ph_min_gps_hz', 'fence_radius', 'fence_alt_max', 'loiter_speed'])),
          block(tr('GPS Rescue và failsafe'),
            h('p', { class: 'help' }, tr('Rescue, theo cách của ArduPilot và PX4. Bay về (RTL): leo lên độ cao bay về (gần home thì leo ít, theo hình nón), bay thẳng về home với mũi hướng theo đường bay, hạ xuống độ cao chờ, chờ vài giây, hạ cánh và tắt motor. Bay về theo đường cũ (SmartRTL): lần ngược lại đường vừa bay, vòng qua những chỗ đã vòng, ở độ cao đã bay. Hạ cánh tại chỗ: hạ ngay chỗ đang bay, giữ vị trí bằng GPS. Chạy khi bật công tắc GPS Rescue (tab Mode), hoặc khi mất sóng nếu chọn ở ô dưới. Sóng trở lại giữa chừng: gạt cần roll hoặc pitch quá 30 % để lấy lại quyền điều khiển. Pin dưới mức tối thiểu thì luôn hạ cánh tại chỗ.')),
            kv([
              [tr('Sẵn sàng bay về'), pill('rescue-ready', 'nav', (n) => (has(n.flags, 'RESCUE_AVAILABLE') ? ['ok', tr('có')] : ['off', tr('chưa')]))],
              [tr('Giai đoạn'), withId('rescue-phase', readout('nav', (n) => label('rescue_phase', n.rescue_phase) + (n.rescue_fail && n.rescue_fail !== 'NONE' ? ': ' + label('rescue_fail', n.rescue_fail) : '')))],
              [tr('Còn phải bay về'), withId('rescue-left', readout('nav', (n) => (has(n.flags, 'RESCUE') && n.rescue_left > 0 ? f(n.rescue_left, 0) + ' m' : '–')))],
              [tr('Khoảng cách tới home'), withId('rescue-home', readout('nav', (n) => (has((store.streams.estimate || {}).valid, 'POS') ? f(n.home_dist, 1) + ' m' : '–')))],
              [tr('Độ cao bay về lần này'), withId('rescue-alt', readout('nav', (n) => (n.return_alt > 0 ? f(n.return_alt, 1) + ' m' : '–')))],
            ]),
            h('p', { class: 'help' }, tr('"Sẵn sàng" chỉ có khi máy đang bay với điểm home (GPS đã fix lúc arm), la bàn và ước lượng độ cao. Đã cài Rescue (công tắc hoặc failsafe) thì firmware không cho arm khi GPS chưa fix hoặc không có la bàn.')),
            fields('nav', ['fs_procedure', 'fs_hold_s', 'batt_low_action', 'batt_crit_action', 'rescue_mode', 'rescue_alt_mode', 'rescue_return_alt', 'rescue_cone', 'rescue_min_start_dist', 'rescue_initial_climb',
              'rescue_ascend_rate', 'rescue_ground_speed', 'rescue_descend_alt', 'rescue_loiter', 'rescue_descend_rate', 'rescue_landing_alt', 'rescue_land_speed', 'rescue_min_sats', 'rescue_allow_no_fix'])))));
  },
};
