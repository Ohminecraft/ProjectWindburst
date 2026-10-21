// A small 3D view of the craft that follows the attitude telemetry. Plain canvas, painter's algorithm, no library.
// Frames: body = forward / right / down, world = north / east / down. The camera sits behind and above the craft.
import { h, track, token } from '../ui.js';
import { LAYOUTS, rotation } from './fwmath.js';
import { tr } from '../i18n.js';

const RAD = Math.PI / 180;

const mul = (R, p) => [R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2], R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2], R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2]];

function ring(cx, cy, z, r, n = 18) {
  const pts = [];
  for (let i = 0; i < n; i++) { const a = (i / n) * 2 * Math.PI; pts.push([cx + r * Math.cos(a), cy + r * Math.sin(a), z]); }
  return pts;
}
function shade(hex, k) {
  const n = parseInt(hex.slice(1), 16), r = (n >> 16) & 255, g = (n >> 8) & 255, b = n & 255;
  const f = (c) => Math.max(0, Math.min(255, Math.round(c * k)));
  return `rgb(${f(r)},${f(g)},${f(b)})`;
}

// get(): { roll, pitch, yaw, layout } or { R, layout }.  The view yaw can be re-centred so the nose points away from the viewer.
export function craftView(get, height = 300) {
  let yaw0 = null;
  const cv = h('canvas', { class: 'cv craft', style: { height: height + 'px' }, role: 'img', 'aria-label': tr('Hình 3D của máy theo tư thế đang đo') });
  const el = 36 * RAD, cel = Math.cos(el), sel = Math.sin(el), DIST = 5.2;

  function draw() {
    const w = cv.clientWidth, hh = cv.clientHeight;
    if (!w || !hh) return;
    const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(hh * dpr); }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, hh);
    const a = get();
    if (yaw0 === null) yaw0 = a.yaw || 0;
    const R = a.R || rotation(a.roll || 0, a.pitch || 0, (a.yaw || 0) - yaw0);     // (a.R: a body -> world matrix of its own, the stick preview)
    const F = Math.min(w, hh * 1.5) * 1.55, cx = w / 2, cy = hh * 0.50;
    const proj = (p) => {                        // world (n, e, d) -> screen + depth
      const zc = p[0] * cel + p[2] * sel, yc = p[0] * sel - p[2] * cel, s = F / (DIST + zc);
      return [cx + p[1] * s, cy - yc * s, zc];
    };
    const cFrame = token('--craft-frame'), cFront = token('--craft-front'), cRear = token('--craft-rear'), cGrid = token('--line'), cInk = token('--ink');
    const polys = [];
    const add = (pts, fill, opts = {}) => {
      const wpts = opts.world ? pts : pts.map((p) => mul(R, p));
      const s = wpts.map(proj);
      let k = 1;
      if (!opts.flat) {                          // flat shading from the polygon's normal (light from above and a little ahead)
        const u = [wpts[1][0] - wpts[0][0], wpts[1][1] - wpts[0][1], wpts[1][2] - wpts[0][2]], v = [wpts[2][0] - wpts[0][0], wpts[2][1] - wpts[0][1], wpts[2][2] - wpts[0][2]];
        const n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        const len = Math.hypot(n[0], n[1], n[2]) || 1;
        k = 0.72 + 0.28 * Math.abs((n[0] * 0.3 + n[2] * -0.95) / len);
      }
      polys.push({ s, depth: s.reduce((q, p) => q + p[2], 0) / s.length + (opts.bias || 0), fill: opts.flat ? fill : shade(fill, k), alpha: opts.alpha || 1, stroke: opts.stroke });
    };

    // ground reference: a grid 1 unit below the craft, drawn first
    ctx.strokeStyle = cGrid; ctx.lineWidth = 1;
    const G = 1.05, span = 2.4;
    for (let i = -4; i <= 4; i++) {
      const t = (i / 4) * span;
      for (const [p, q] of [[[-span, t, G], [span, t, G]], [[t, -span, G], [t, span, G]]]) {
        const A = proj(p), B = proj(q);
        ctx.globalAlpha = i === 0 ? 0.9 : 0.45;
        ctx.beginPath(); ctx.moveTo(A[0], A[1]); ctx.lineTo(B[0], B[1]); ctx.stroke();
      }
    }
    ctx.globalAlpha = 1;

    const L = LAYOUTS[a.layout] || LAYOUTS.QUAD_X;
    // shadow of each prop on the ground: shows the tilt at a glance
    for (const [i, ang] of L.angles.entries()) {
      const rr = L.r ? L.r[i] : 1, c = mul(R, [rr * Math.cos(ang * RAD), rr * Math.sin(ang * RAD), 0]);
      const s = ring(c[0], c[1], G, 0.40, 14).map(proj);
      ctx.fillStyle = cInk; ctx.globalAlpha = 0.08;
      ctx.beginPath(); s.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath(); ctx.fill();
    }
    ctx.globalAlpha = 1;

    // arms
    L.angles.forEach((ang, i) => {
      const rr = L.r ? L.r[i] : 1, c = rr * Math.cos(ang * RAD), s = rr * Math.sin(ang * RAD), wd = 0.07;
      add([[-s * wd, c * wd, -0.03], [c - s * wd, s + c * wd, -0.03], [c + s * wd, s - c * wd, -0.03], [s * wd, -c * wd, -0.03]], cFrame);
      add([[-s * wd, c * wd, 0.03], [c - s * wd, s + c * wd, 0.03], [c + s * wd, s - c * wd, 0.03], [s * wd, -c * wd, 0.03]], cFrame);
    });
    // body: a box, longer along the nose axis
    const bx = 0.36, by = 0.2, zt = -0.12, zb = 0.07;
    const box = [[-bx, -by], [bx, -by], [bx, by], [-bx, by]];
    add(box.map(([x, y]) => [x, y, zt]), cFrame, { bias: -0.02 });
    add(box.map(([x, y]) => [x, y, zb]), cFrame);
    for (let i = 0; i < 4; i++) { const p = box[i], q = box[(i + 1) % 4]; add([[p[0], p[1], zt], [q[0], q[1], zt], [q[0], q[1], zb], [p[0], p[1], zb]], cFrame); }
    // nose arrow on top of the body
    add([[0.34, 0, zt - 0.012], [0.02, -0.14, zt - 0.012], [0.02, 0.14, zt - 0.012]], cFront, { flat: true, bias: R[2][2] > 0 ? -0.6 : 0.6 });   // on top unless the craft is upside down
    // motors and prop discs: the front ones carry the accent colour
    L.angles.forEach((ang, i) => {
      const rr = L.r ? L.r[i] : 1, under = L.under && L.under[i], x = rr * Math.cos(ang * RAD), y = rr * Math.sin(ang * RAD), front = x > 0.01;
      add(ring(x, y, under ? 0.10 : -0.10, 0.13, 12), cFrame, { bias: -0.03 });                 // (the lower motor of a coaxial pair: under the arm)
      add(ring(x, y, under ? 0.13 : -0.13, 0.40, 20), front ? cFront : cRear, { flat: true, alpha: 0.42, bias: -0.04, stroke: front ? cFront : cRear });
    });

    polys.sort((p, q) => q.depth - p.depth);
    for (const p of polys) {
      ctx.globalAlpha = p.alpha; ctx.fillStyle = p.fill;
      ctx.beginPath(); p.s.forEach((pt, i) => (i ? ctx.lineTo(pt[0], pt[1]) : ctx.moveTo(pt[0], pt[1]))); ctx.closePath(); ctx.fill();
      if (p.stroke) { ctx.globalAlpha = 0.9; ctx.strokeStyle = p.stroke; ctx.lineWidth = 1.5; ctx.stroke(); }
    }
    ctx.globalAlpha = 1;
  }

  let raf = 0, alive = true;
  const loop = () => { if (!alive) return; draw(); raf = requestAnimationFrame(loop); };
  raf = requestAnimationFrame(loop);
  track(() => { alive = false; cancelAnimationFrame(raf); });
  cv.recenter = () => { yaw0 = null; };
  cv.drawNow = draw;
  return cv;
}
