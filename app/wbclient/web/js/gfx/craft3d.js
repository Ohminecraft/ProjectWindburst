// A 3D view of the craft that follows the attitude telemetry: carbon frame, stack, camera, battery, motors and props of the
// configured layout. WebGL with a depth buffer, no library; the old flat painter's view stays as the fallback.
// Frames: body = forward / right / down, world = north / east / down. The camera sits behind and above the craft.
import { h, track, token } from '../ui.js';
import { LAYOUTS, rotation } from './fwmath.js';
import { tr } from '../i18n.js';

const RAD = Math.PI / 180;
const EL = 34 * RAD, CEL = Math.cos(EL), SEL = Math.sin(EL), DIST = 5.0, GROUND = 1.05;

const mul = (R, p) => [R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2], R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2], R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2]];

// palette slots: the vertex carries the slot, the colour comes from the page's tokens (light / dark follow by themselves)
const PAL = ['--craft-frame', '--craft-front', '--craft-rear', '--craft-motor', '--craft-dark', '--craft-battery', '--craft-metal', '--craft-glass'];
const [FRAME, FRONT, REAR, MOTOR, DARK, BATT, METAL, GLASS] = PAL.keys();

function rgb(css) {
  const s = (css || '').trim();
  if (/^#[0-9a-f]{6}$/i.test(s)) { const n = parseInt(s.slice(1), 16); return [(n >> 16) / 255, ((n >> 8) & 255) / 255, (n & 255) / 255]; }
  if (/^#[0-9a-f]{3}$/i.test(s)) return [1, 2, 3].map((i) => parseInt(s[i] + s[i], 16) / 255);
  const m = s.match(/rgba?\(([^)]+)\)/);
  return m ? m[1].split(',').slice(0, 3).map((x) => parseFloat(x) / 255) : [0.5, 0.5, 0.5];
}

// ------------------------------------------------------------------ geometry (body frame, 1 = motor distance)
// A mesh is a list of triangles: positions, flat normals, palette slot per vertex.
class Mesh {
  constructor() { this.p = []; this.n = []; this.c = []; }
  tri(a, b, c, slot) {
    const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    const l = Math.hypot(...n) || 1; n = n.map((x) => x / l);
    for (const p of [a, b, c]) { this.p.push(...p); this.n.push(...n); this.c.push(slot); }
  }
  quad(a, b, c, d, slot) { this.tri(a, b, c, slot); this.tri(a, c, d, slot); }
  // a convex outline (x, y) from z0 to z1; `xf` moves the finished points (tilted parts)
  prism(poly, z0, z1, slot, xf = null, capSlot = slot) {
    const T = xf || ((p) => p), top = poly.map(([x, y]) => T([x, y, z0])), bot = poly.map(([x, y]) => T([x, y, z1]));
    for (let i = 1; i + 1 < poly.length; i++) { this.tri(top[0], top[i], top[i + 1], capSlot); this.tri(bot[0], bot[i + 1], bot[i], capSlot); }
    for (let i = 0; i < poly.length; i++) { const j = (i + 1) % poly.length; this.quad(top[i], bot[i], bot[j], top[j], slot); }
  }
  cyl(cx, cy, z0, z1, r, slot, n = 20, capSlot = slot) { this.prism(circle(cx, cy, r, n), z0, z1, slot, null, capSlot); }
}
const circle = (cx, cy, r, n = 20) => Array.from({ length: n }, (_, i) => [cx + r * Math.cos((i / n) * 2 * Math.PI), cy + r * Math.sin((i / n) * 2 * Math.PI)]);
function rounded(w, d, r, n = 4) {            // a rectangle 2w x 2d with corners of radius r, as one convex outline
  const out = [];
  for (const [cx, cy, a0] of [[w - r, d - r, 0], [-w + r, d - r, 90], [-w + r, -d + r, 180], [w - r, -d + r, 270]]) {
    for (let i = 0; i <= n; i++) { const a = (a0 + (i / n) * 90) * RAD; out.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]); }
  }
  return out;
}
// rotation about the body's y axis (nose up for a > 0), then a move
const tiltY = (a, at) => (p) => { const c = Math.cos(a), s = Math.sin(a); return [at[0] + p[0] * c + p[2] * s, at[1] + p[1], at[2] - p[0] * s + p[2] * c]; };

function buildCraft(L) {
  const m = new Mesh(), discs = [];
  const zb0 = 0.035, zb1 = 0.07;              // bottom plate (z down: larger = lower)
  const zt0 = -0.115, zt1 = -0.095;           // top plate
  // bottom plate: the centre and one arm per motor, each ending in a round motor pad
  m.prism(rounded(0.3, 0.2, 0.06), zb0, zb1, FRAME);
  L.angles.forEach((ang, i) => {
    const rr = L.r ? L.r[i] : 1, a = ang * RAD, x = rr * Math.cos(a), y = rr * Math.sin(a), px = -Math.sin(a), py = Math.cos(a);
    const w0 = 0.075, w1 = 0.055, s0 = 0.12;
    m.prism([[s0 * Math.cos(a) - px * w0, s0 * Math.sin(a) - py * w0], [x - px * w1, y - py * w1], [x + px * w1, y + py * w1], [s0 * Math.cos(a) + px * w0, s0 * Math.sin(a) + py * w0]], zb0, zb1, FRAME);
    m.prism(circle(x, y, 0.1, 18), zb0, zb1, FRAME);
  });
  // top plate and its four standoffs
  m.prism(rounded(0.25, 0.15, 0.05), zt0, zt1, FRAME);
  for (const [x, y] of [[0.19, 0.1], [0.19, -0.1], [-0.19, 0.1], [-0.19, -0.1]]) m.cyl(x, y, zt1, zb0, 0.017, METAL, 10);
  // the stack: flight controller over ESC
  m.prism(rounded(0.1, 0.1, 0.012), -0.015, 0.03, DARK);
  m.prism(rounded(0.085, 0.085, 0.01), -0.05, -0.02, DARK);
  m.cyl(0.06, -0.06, -0.06, -0.05, 0.012, FRONT, 8);               // status LED
  // camera at the nose, tilted up 25 degrees: body, lens barrel, glass
  const cam = tiltY(25 * RAD, [0.27, 0, -0.025]);
  m.prism(rounded(0.05, 0.055, 0.012), -0.05, 0.05, DARK, cam);
  const lens = (p) => cam([p[2] + 0.05, p[0], p[1]]);                                   // a cylinder along the camera's x
  m.prism(circle(0, 0, 0.036, 16), 0, 0.05, METAL, lens);
  m.prism(circle(0, 0, 0.026, 16), 0.05, 0.056, GLASS, lens);
  // battery on the top plate, with its strap
  m.prism(rounded(0.24, 0.12, 0.03), -0.245, zt0, BATT);
  m.prism([[0.04, -0.13], [0.09, -0.13], [0.09, 0.13], [0.04, 0.13]], -0.25, zt0 + 0.005, DARK);
  m.prism([[-0.11, -0.13], [-0.06, -0.13], [-0.06, 0.13], [-0.11, 0.13]], -0.25, zt0 + 0.005, DARK);
  // nose arrow printed on the battery
  m.prism([[0.2, 0], [0.12, -0.05], [0.12, 0.05]], -0.252, -0.245, FRONT);
  // receiver antennas at the tail, swept back and up
  for (const side of [-1, 1]) {
    const ant = (p) => { const a = -40 * RAD, c = Math.cos(a), s = Math.sin(a); return [-0.26 - p[2] * c, side * 0.06 + p[0] + side * p[2] * 0.25, -0.1 + p[2] * s + p[1]]; };
    m.prism(circle(0, 0, 0.009, 8), 0, 0.3, DARK, ant);
    m.prism(circle(0, 0, 0.014, 8), 0.24, 0.31, METAL, ant);
  }
  // motors and props
  L.angles.forEach((ang, i) => {
    const rr = L.r ? L.r[i] : 1, a = ang * RAD, x = rr * Math.cos(a), y = rr * Math.sin(a), under = L.under && L.under[i], front = x > 0.01;
    const dir = under ? 1 : -1;                                                     // the motor stands up from the arm (or hangs below it)
    const base = under ? zb1 : zb0, z = (d) => base + dir * d;
    const lo = (p, q) => [Math.min(p, q), Math.max(p, q)];
    m.cyl(x, y, ...lo(z(0), z(0.03)), 0.075, DARK, 20);                             // stator base
    m.cyl(x, y, ...lo(z(0.03), z(0.1)), 0.09, MOTOR, 24);                           // bell
    m.cyl(x, y, ...lo(z(0.1), z(0.115)), 0.035, METAL, 12);                         // prop nut
    const pz = z(0.105), slot = front ? FRONT : REAR, spin = (L.spin && L.spin[i]) || 1;
    for (let b = 0; b < 3; b++) {                                                    // three blades with a little pitch
      const a0 = ang * RAD + (b * 2 * Math.PI) / 3 + i * 0.7, ca = Math.cos(a0), sa = Math.sin(a0), qa = -sa, qb = ca;
      const P = (r, w, t) => [x + ca * r + qa * w, y + sa * r + qb * w, pz + t];
      const tw = 0.012 * spin;
      m.quad(P(0.03, -0.03, -tw), P(0.3, -0.045, -tw * 0.6), P(0.3, 0.04, tw * 0.6), P(0.03, 0.02, tw), slot);
      m.quad(P(0.3, -0.045, -tw * 0.6), P(0.42, -0.025, -tw * 0.3), P(0.42, 0.02, tw * 0.3), P(0.3, 0.04, tw * 0.6), slot);
    }
    m.cyl(x, y, ...lo(pz - 0.01, pz + 0.01), 0.04, slot, 12);                       // prop hub
    discs.push({ x, y, z: pz, slot });
  });
  return { m, discs };
}

// ------------------------------------------------------------------ WebGL
const VS = `
attribute vec3 aP; attribute vec3 aN; attribute float aC;
uniform mat3 uR; uniform mat3 uC; uniform vec4 uProj; uniform vec3 uPal[8];
varying vec3 vN; varying vec3 vCol;
void main() {
  vec3 w = uR * aP;
  vec3 c = uC * w;
  float zc = c.z + ${DIST.toFixed(2)};
  gl_Position = vec4(c.x * uProj.x, c.y * uProj.y, uProj.z * zc + uProj.w, zc);
  vN = uR * aN;
  int k = int(aC + 0.5);
  vec3 col = uPal[0];
  for (int i = 1; i < 8; i++) if (i == k) col = uPal[i];
  vCol = col;
}`;
const FS = `
precision mediump float;
varying vec3 vN; varying vec3 vCol;
uniform vec3 uView; uniform float uAlpha; uniform float uFlat;
void main() {
  vec3 n = normalize(vN);
  if (dot(n, uView) > 0.0) n = -n;                       // the side that faces the camera
  vec3 L = normalize(vec3(0.35, -0.45, -1.0));           // light from above, ahead and to the left (north, east, down)
  float d = max(dot(n, L), 0.0);
  vec3 hv = normalize(L - uView);
  float s = pow(max(dot(n, hv), 0.0), 28.0) * 0.35;
  vec3 col = mix(vCol * (0.42 + 0.62 * d) + vec3(s), vCol, uFlat);
  gl_FragColor = vec4(col, uAlpha);
}`;

function glView(cv) {
  const gl = cv.getContext('webgl', { antialias: true, preserveDrawingBuffer: true, premultipliedAlpha: false, alpha: true });
  if (!gl) return null;
  const sh = (type, src) => { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; };
  const prog = gl.createProgram();
  try {
    gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS)); gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS)); gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
  } catch (e) { console.warn('craft view: WebGL program failed, flat view instead', e); return null; }
  const U = {}; for (const n of ['uR', 'uC', 'uProj', 'uPal', 'uView', 'uAlpha', 'uFlat']) U[n] = gl.getUniformLocation(prog, n);
  const A = { p: gl.getAttribLocation(prog, 'aP'), n: gl.getAttribLocation(prog, 'aN'), c: gl.getAttribLocation(prog, 'aC') };
  const buf = (arr) => { const b = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, b); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(arr), gl.STATIC_DRAW); return b; };
  const mesh = (m) => ({ p: buf(m.p), n: buf(m.n), c: buf(m.c), count: m.p.length / 3 });
  const bind = (M) => {
    for (const [k, size] of [['p', 3], ['n', 3], ['c', 1]]) { gl.bindBuffer(gl.ARRAY_BUFFER, M[k]); gl.enableVertexAttribArray(A[k]); gl.vertexAttribPointer(A[k], size, gl.FLOAT, false, 0, 0); }
  };
  let craft = null, craftKey = '', discMesh = null;
  const I3 = new Float32Array([1, 0, 0, 0, 1, 0, 0, 0, 1]);
  // world -> camera: x = east, y = up on the screen, z = depth (column-major for WebGL)
  const C = new Float32Array([0, SEL, CEL, 1, 0, 0, 0, -CEL, SEL]);
  const view = [CEL, 0, SEL];                             // the direction the camera looks, world

  function draw(a, w, hh) {
    const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(hh * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(hh * dpr); }
    gl.viewport(0, 0, cv.width, cv.height);
    gl.clearColor(0, 0, 0, 0); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(prog); gl.enable(gl.DEPTH_TEST);
    const L = LAYOUTS[a.layout] || LAYOUTS.QUAD_X, key = a.layout || 'QUAD_X';
    if (key !== craftKey) {
      const { m, discs } = buildCraft(L);
      craft = mesh(m); craftKey = key;
      const dm = new Mesh();
      for (const d of discs) { const c = circle(d.x, d.y, 0.43, 36); for (let i = 0; i < c.length; i++) dm.tri([d.x, d.y, d.z], [...c[i], d.z], [...c[(i + 1) % c.length], d.z], d.slot); }
      discMesh = mesh(dm);
    }
    const F = Math.min(w, hh * 1.5) * 1.5, near = 1.5, far = 9;
    gl.uniform4f(U.uProj, (2 * F) / w, (2 * F) / hh, (far + near) / (far - near), (-2 * far * near) / (far - near));
    gl.uniformMatrix3fv(U.uC, false, C);
    gl.uniform3f(U.uView, ...view);
    const pal = PAL.map((t) => rgb(token(t)));
    const line = rgb(token('--line'));
    // ground grid and prop shadows: world frame, one unit under the craft
    gl.uniformMatrix3fv(U.uR, false, I3);
    gl.uniform3fv(U.uPal, new Float32Array([...line, ...pal.slice(1).flat()]));
    gl.uniform1f(U.uFlat, 1); gl.uniform1f(U.uAlpha, 1);
    drawGround(a, L);
    // the craft
    const R = a.R || rotation(a.roll || 0, a.pitch || 0, (a.yaw || 0) - (a.yaw0 || 0));
    gl.uniformMatrix3fv(U.uR, false, new Float32Array([R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1], R[0][2], R[1][2], R[2][2]]));
    gl.uniform3fv(U.uPal, new Float32Array(pal.flat()));
    gl.uniform1f(U.uFlat, 0); gl.uniform1f(U.uAlpha, 1);
    gl.disable(gl.BLEND);
    bind(craft); gl.drawArrays(gl.TRIANGLES, 0, craft.count);
    // prop discs: see-through, after everything solid
    gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false);
    gl.uniform1f(U.uFlat, 1); gl.uniform1f(U.uAlpha, 0.16);
    bind(discMesh); gl.drawArrays(gl.TRIANGLES, 0, discMesh.count);
    gl.depthMask(true); gl.disable(gl.BLEND);
  }

  // the ground: thin strips GROUND below the craft's centre (strips, not lines: they get a depth like the rest)
  const groundBufs = {};
  function drawGround(a, L) {
    if (!groundBufs.grid || groundBufs.key !== craftKey) {
      const g = new Mesh(), span = 2.4;
      for (let i = -4; i <= 4; i++) {
        const t = (i / 4) * span, wd = i === 0 ? 0.008 : 0.004;
        g.quad([-span, t - wd, GROUND], [span, t - wd, GROUND], [span, t + wd, GROUND], [-span, t + wd, GROUND], 0);
        g.quad([t - wd, -span, GROUND], [t + wd, -span, GROUND], [t + wd, span, GROUND], [t - wd, span, GROUND], 0);
      }
      groundBufs.grid = mesh(g); groundBufs.key = craftKey;
    }
    bind(groundBufs.grid); gl.drawArrays(gl.TRIANGLES, 0, groundBufs.grid.count);
    // shadows of the props: where each motor is over the ground, tilted with the craft
    const R = a.R || rotation(a.roll || 0, a.pitch || 0, (a.yaw || 0) - (a.yaw0 || 0));
    const s = new Mesh();
    L.angles.forEach((ang, i) => {
      const rr = L.r ? L.r[i] : 1, c = mul(R, [rr * Math.cos(ang * RAD), rr * Math.sin(ang * RAD), 0]), ring = circle(c[0], c[1], 0.4, 24);
      for (let k = 0; k < ring.length; k++) s.tri([c[0], c[1], GROUND - 0.001], [...ring[k], GROUND - 0.001], [...ring[(k + 1) % ring.length], GROUND - 0.001], 1);
    });
    const sm = mesh(s);
    gl.uniform3fv(U.uPal, new Float32Array([0, 0, 0, ...rgb(token('--ink')), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]));
    gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false);
    gl.uniform1f(U.uAlpha, 0.08);
    bind(sm); gl.drawArrays(gl.TRIANGLES, 0, sm.count);
    gl.depthMask(true); gl.disable(gl.BLEND);
    for (const k of ['p', 'n', 'c']) gl.deleteBuffer(sm[k]);
  }
  return { draw };
}

// ------------------------------------------------------------------ the flat fallback (no WebGL)
function flatView(cv) {
  return {
    draw(a, w, hh) {
      const dpr = window.devicePixelRatio || 1;
      if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(hh * dpr); }
      const ctx = cv.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, hh);
      const R = a.R || rotation(a.roll || 0, a.pitch || 0, (a.yaw || 0) - (a.yaw0 || 0));
      const F = Math.min(w, hh * 1.5) * 1.5, cx = w / 2, cy = hh / 2;
      const proj = (p) => { const zc = p[0] * CEL + p[2] * SEL, yc = p[0] * SEL - p[2] * CEL, s = F / (DIST + zc); return [cx + p[1] * s, cy - yc * s]; };
      const L = LAYOUTS[a.layout] || LAYOUTS.QUAD_X;
      ctx.lineCap = 'round';
      L.angles.forEach((ang, i) => {
        const rr = L.r ? L.r[i] : 1, p = mul(R, [rr * Math.cos(ang * RAD), rr * Math.sin(ang * RAD), 0]), o = proj([0, 0, 0]), q = proj(p);
        ctx.strokeStyle = token('--craft-frame'); ctx.lineWidth = 8; ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
        ctx.fillStyle = token(Math.cos(ang * RAD) > 0.01 ? '--craft-front' : '--craft-rear'); ctx.globalAlpha = 0.5;
        ctx.beginPath(); ctx.arc(q[0], q[1], F * 0.4 / DIST, 0, 2 * Math.PI); ctx.fill(); ctx.globalAlpha = 1;
      });
    },
  };
}

// get(): { roll, pitch, yaw, layout } or { R, layout }.  The view yaw can be re-centred so the nose points away from the viewer.
export function craftView(get, height = 300) {
  let yaw0 = null;
  const cv = h('canvas', { class: 'cv craft', style: { height: height + 'px' }, role: 'img', 'aria-label': tr('Hình 3D của máy theo tư thế đang đo') });
  let view = null;
  function draw() {
    const w = cv.clientWidth, hh = cv.clientHeight;
    if (!w || !hh) return;
    if (!view) view = glView(cv) || flatView(cv);
    const a = get();
    if (yaw0 === null) yaw0 = a.yaw || 0;
    view.draw({ ...a, yaw0 }, w, hh);
  }
  let raf = 0, alive = true;
  const loop = () => { if (!alive) return; draw(); raf = requestAnimationFrame(loop); };
  raf = requestAnimationFrame(loop);
  track(() => { alive = false; cancelAnimationFrame(raf); });
  cv.recenter = () => { yaw0 = null; };
  cv.drawNow = draw;
  return cv;
}
