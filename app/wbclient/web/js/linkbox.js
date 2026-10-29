// The connection box: while the page connects to a flight controller and reads its settings, a box in the middle of the
// window (the page behind it blurred) shows the steps - looking for the firmware on the port, connected (which firmware),
// reading the settings - and under them a bar that runs down over the time the current step may take. A bar that runs out
// means the step timed out: the device could not be read. A failed attempt stays on screen until the user closes it.
import { h } from './ui.js';
import { tr } from './i18n.js';

// How long a step may take (ms). find: the server opens the port and sends HELLO up to 4 times, 1 s each (gui.py connect,
// client.py hello); read: every settings section, normally well under a second.
export const BUDGET = { find: 4500, read: 8000 };
const STEPS = ['find', 'link', 'read'];

let box = null, card, title, list, msg, timer, closeBtn, closing = 0, expire = 0;
const rows = {};

function build() {
  title = h('h2', { id: 'linkbox-title' });
  list = h('ol', { class: 'link-steps' }, STEPS.map((id) => (rows[id] = h('li', { 'data-step': id, 'data-state': 'wait' }, h('i', { 'aria-hidden': 'true' }), h('span')))));
  msg = h('p', { class: 'linkbox-msg', id: 'linkbox-msg', hidden: true });
  timer = h('div', { class: 'link-timer', id: 'link-timer', role: 'progressbar', 'aria-label': tr('Thời gian chờ còn lại') }, h('i'));
  closeBtn = h('button', { type: 'button', class: 'btn', id: 'linkbox-close', hidden: true }, tr('Đóng'));
  card = h('div', { class: 'linkbox-card' }, title, list, msg, timer, h('div', { class: 'linkbox-foot' }, closeBtn));
  box = h('div', { class: 'linkbox', id: 'linkbox', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'linkbox-title', hidden: true }, card);
  closeBtn.addEventListener('click', linkClose);
  box.addEventListener('click', (e) => { if (e.target === box && box.dataset.state === 'fail') linkClose(); });
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && box && !box.hidden && box.dataset.state === 'fail') linkClose(); });
  document.body.append(box);
}

function setRow(id, state, text) {
  rows[id].dataset.state = state;
  rows[id].firstChild.textContent = state === 'done' ? '✓' : state === 'fail' ? '✕' : '';
  if (text !== undefined) rows[id].lastChild.textContent = text;
}

// the bar: full, then empty after `ms` (a CSS animation, so it can be stopped where it is)
function runBar(ms, onOut) {
  clearTimeout(expire);
  const bar = timer.firstChild;
  timer.className = 'link-timer';
  bar.style.animation = 'none'; void bar.offsetWidth; bar.style.animation = '';
  bar.style.animationDuration = ms + 'ms';
  timer.classList.add('running');
  expire = setTimeout(() => { timer.classList.add('out'); if (onOut) onOut(); }, ms);
}
function stopBar(kind) {
  clearTimeout(expire);
  timer.classList.remove('running');
  timer.classList.add('stopped', kind);
}

export function linkIsOpen() { return !!box && !box.hidden; }

// Show the box for a connection to `port` (all steps still to come). Calling it while it is open starts it over.
export function linkOpen(port) {
  if (!box) build();
  clearTimeout(closing);
  title.textContent = port ? tr('Kết nối {port}', { port }) : tr('Kết nối');
  setRow('find', 'wait', tr('Tìm thiết bị'));
  setRow('link', 'wait', tr('Kết nối'));
  setRow('read', 'wait', tr('Đọc cấu hình'));
  msg.hidden = true; closeBtn.hidden = true;
  box.dataset.state = 'run';
  box.classList.remove('closing');
  box.hidden = false;
}

// Step `id` begins (the steps before it are done): its text, and the bar runs down over `ms`. onOut: the bar ran out.
export function linkStep(id, text, ms, onOut) {
  if (!linkIsOpen()) linkOpen();
  for (const s of STEPS.slice(0, STEPS.indexOf(id))) if (rows[s].dataset.state !== 'done') setRow(s, 'done');
  setRow(id, 'run', text);
  msg.hidden = true; closeBtn.hidden = true;
  box.dataset.state = 'run';
  runBar(ms, onOut);
}

export function linkDone(id, text) {
  if (linkIsOpen()) setRow(id, 'done', text);
}

// All done: the bar stops, the box goes away a moment later (it no longer catches clicks meanwhile).
export function linkSuccess(text) {
  if (!linkIsOpen()) return;
  for (const s of STEPS) if (rows[s].dataset.state !== 'done') setRow(s, 'done');
  if (text) setRow('read', 'done', text);
  stopBar('ok');
  box.dataset.state = 'done';
  box.classList.add('closing');
  closing = setTimeout(() => { box.hidden = true; box.classList.remove('closing'); }, 450);
}

// The step under way failed (`why`): marked, the bar stops (red), the reason under the steps, and the box stays until closed.
export function linkFail(why, timedOut) {
  if (!linkIsOpen()) linkOpen();
  const id = STEPS.find((s) => rows[s].dataset.state === 'run') || STEPS.find((s) => rows[s].dataset.state === 'wait') || 'read';
  setRow(id, 'fail');
  stopBar(timedOut ? 'out' : 'bad');
  msg.textContent = why; msg.hidden = false;
  closeBtn.hidden = false;
  box.dataset.state = 'fail';
  closeBtn.focus();
}

export function linkClose() {
  if (!box) return;
  clearTimeout(expire); clearTimeout(closing);
  box.hidden = true; box.classList.remove('closing');
}
