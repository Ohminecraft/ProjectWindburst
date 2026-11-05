// The craft as the sticks would fly it, live from the receiver (Receiver and Rate tabs). Nothing is sent to the firmware.
//   Acro   the body turns at the rates of the rate curve (Rate tab), the attitude adds up - like flying Acro
//   Angle  roll / pitch tilt as Angle mode commands them (angle_limit and the rate curve), yaw turns the heading
// "Level" puts it back level, nose away from the viewer.
import { store } from '../store.js';
import { h, listen } from '../ui.js';
import { craftView } from './craft3d.js';
import { actualRate, stick, acroStep, angleTarget, rotation } from './fwmath.js';
import { tr } from '../i18n.js';

const I3 = () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

export function stickPreview(mode0 = 'ACRO', height = 240) {
  let mode = mode0, R = I3(), heading = 0, last = 0;
  const sticks = () => {
    const rc = store.streams.rc, rx = store.settings.receiver;
    if (!rc || !rx || !rc.link) return [0, 0, 0];
    return [rx.ch_roll, rx.ch_pitch, rx.ch_yaw].map((c, i) => stick(rc.ch[c], i === 2 ? rx.yaw_deadband : rx.deadband, rx.mid_rc || 1500));
  };
  const step = () => {
    const now = performance.now(), dt = last ? Math.min(0.05, (now - last) / 1000) : 0; last = now;
    const r = store.settings.rates;
    if (!r) return { R };
    const s = sticks(), rate = [0, 1, 2].map((i) => actualRate(r.center_sens[i], r.max_rate[i], r.expo[i], s[i]));
    if (mode === 'ACRO') R = acroStep(R, rate, dt);
    else {
      heading = (heading + rate[2] * dt) % 360;
      R = rotation(angleTarget(r.angle_limit, r.center_sens[0], r.max_rate[0], r.expo[0], s[0]), angleTarget(r.angle_limit, r.center_sens[1], r.max_rate[1], r.expo[1], s[1]), heading);
    }
    return { R };
  };
  const cv = craftView(() => ({ ...step(), layout: (store.settings.motors || {}).layout }), height);
  cv.id = 'stick-preview';
  cv.setAttribute('aria-label', tr('Hình 3D của máy bay theo cần đang gạt'));
  const level = () => { R = I3(); heading = 0; };
  const sel = h('select', { id: 'preview-mode', 'aria-label': tr('Kiểu bay xem thử') },
    h('option', { value: 'ACRO' }, tr('Acro (quay theo rate)')), h('option', { value: 'ANGLE' }, tr('Angle (nghiêng theo cần)')));
  sel.value = mode;
  sel.addEventListener('change', () => { mode = sel.value; level(); });
  const btn = h('button', { type: 'button', class: 'btn quiet', id: 'preview-level' }, tr('Về cân bằng'));
  btn.addEventListener('click', level);
  const state = h('span', { class: 'help', id: 'preview-link' });
  const paint = () => { const rc = store.streams.rc; state.textContent = rc && rc.link ? '' : tr('Chưa có tín hiệu từ tay điều khiển.'); };
  listen('stream:rc', paint); paint();
  const box = h('div', { class: 'stick-preview' }, h('div', { class: 'btn-row' }, sel, btn, state), cv,
    h('p', { class: 'help' }, tr('Chỉ là hình mô phỏng trong app, không gửi gì xuống máy bay. Acro: máy quay đúng tốc độ của đường cong rate, thả cần thì đứng yên ở tư thế đó. Angle: góc nghiêng theo cần (giới hạn góc ở tab Rate), cần yaw xoay hướng mũi.')));
  box.level = level; box.setMode = (m) => { sel.value = m; mode = m; level(); };
  return box;
}
