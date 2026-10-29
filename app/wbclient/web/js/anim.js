// Motion of the page, kept in one place:
//   - a tab coming on screen (enterView): each theme has its own way, set in app.css on .view-enter
//       Carbon / Swiss: slides in 12 px from the side of the rail it was picked on, fading in
//       HUD:            a bright scan line sweeps across and uncovers it
//       Glass:          fades in out of a blur
//       Field:          none (outdoors: contrast and speed first)
//   - a value that appears for the first time (decodeText): a row of ASCII "noise" as long as the value, replaced by the value
//     from left to right, as if it were decoded
// Nothing moves when the system asks for reduced motion, in the Field theme, or under automation (tests read text at once).
// The frame maths (scrambleFrame, decodeMs) is pure and tested with node (tests/test_gui_anim.py).

export const NOISE = '!<>-_\\/[]{}=+*^?#01';

// What a value looks like `progress` (0..1) into its decoding: the first round(progress x length) characters are the value's,
// the rest noise (spaces stay spaces). rnd: () => 0..1.
export function scrambleFrame(text, progress, rnd = Math.random) {
  const n = text.length, shown = Math.max(0, Math.min(n, Math.round(progress * n)));
  let out = text.slice(0, shown);
  for (let i = shown; i < n; i++) out += text[i] === ' ' ? ' ' : NOISE[Math.floor(rnd() * NOISE.length) % NOISE.length];
  return out;
}
// How long the decoding of a value takes (ms): longer values a little longer, never more than half a second.
export function decodeMs(length) { return Math.min(500, 150 + 25 * length); }

function still() {
  if (typeof document === 'undefined') return true;
  if (globalThis.navigator && navigator.webdriver) return true;
  if (document.documentElement.dataset.theme === 'field') return true;
  return !!(globalThis.matchMedia && matchMedia('(prefers-reduced-motion: reduce)').matches);
}

const running = new WeakMap();      // element -> its decoding (the target text can change while it runs)

// Sets el's text; animated the first time a value comes (the element still shows the placeholder or nothing).
export function decodeText(el, text, first = false) {
  const job = running.get(el);
  if (job) { job.text = text; return; }
  if (!first || still() || !text) { el.textContent = text; return; }
  const j = { text, t0: performance.now() }; running.set(el, j);
  const step = (now) => {
    const p = (now - j.t0) / decodeMs(j.text.length);
    if (p >= 1 || !el.isConnected) { el.textContent = j.text; running.delete(el); el.classList.remove('decoding'); return; }
    el.textContent = scrambleFrame(j.text, p);
    requestAnimationFrame(step);
  };
  el.classList.add('decoding');
  requestAnimationFrame(step);
}

// A tab's content has just been built in `view`; dir: +1 the new tab sits below the old one on the rail, -1 above, 0 unknown.
export function enterView(view, dir) {
  view.classList.remove('view-enter');
  if (still()) return;
  view.style.setProperty('--enter-dy', (dir < 0 ? -12 : 12) + 'px');
  void view.offsetWidth;            // (restarts the animation when the same class comes back)
  view.classList.add('view-enter');
  view.addEventListener('animationend', () => view.classList.remove('view-enter'), { once: true });
}
