// Small DOM helpers and the settings widgets. A widget is bound to one field of the store: it shows what the firmware holds
// and writes the user's edit back through setField().
import { store, on, fieldDef, fieldRange, setField, command, hasField } from './store.js';
import { FIELDS, ENUM_LABELS, NOT_IMPLEMENTED, ERRORS } from './labels.js';
import { tr } from './i18n.js';
import { decodeText } from './anim.js';

export function h(tag, attrs, ...kids) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v === null || v === undefined || v === false) continue;
    if (k === 'class') el.className = v;
    else if (k === 'text') el.textContent = v;
    else if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2), v);
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else if (v === true) el.setAttribute(k, '');
    else el.setAttribute(k, v);
  }
  for (const kid of kids.flat(Infinity)) {
    if (kid === null || kid === undefined || kid === false) continue;
    el.append(kid.nodeType ? kid : document.createTextNode(String(kid)));
  }
  return el;
}
export function clear(el) { while (el.firstChild) el.removeChild(el.firstChild); return el; }

// Everything a tab subscribes to is released when the tab is left.
let scope = [];
export function track(off) { scope.push(off); return off; }
export function disposeScope() { for (const off of scope) { try { off(); } catch (e) { /* ignore */ } } scope = []; }
export const listen = (evt, fn) => track(on(evt, fn));

export function toast(text, kind = 'info', ms = 4500) {
  const box = document.getElementById('toasts');
  const el = h('div', { class: 'toast ' + kind }, text);
  box.append(el);
  setTimeout(() => el.remove(), ms);
}
export function errorText(e) {
  const code = e && e.code;
  const x = (e && e.extra) || {};
  // codes of the local server (wbclient/gui.py): it sends the facts, the sentence is made here in the page's language
  if (code === 'OPEN_FAILED') return tr('Không mở được cổng {port}: {detail}', { port: x.port, detail: x.detail });
  if (code === 'NO_ANSWER') return tr('Mở được cổng {port} ở {baud} baud nhưng firmware Windburst không trả lời.', { port: x.port, baud: x.baud });
  if (code === 'SCHEMA_MISMATCH') return tr('Firmware và app không cùng phiên bản giao thức: {detail}', { detail: x.detail });
  if (code === 'BAD_VALUE') return tr('Giá trị không hợp lệ: {detail}', { detail: x.detail || e.message });
  if (code === 'NOT_A_BACKUP') return tr('File không phải bản sao lưu settings của Windburst.');
  if (code === 'TIMEOUT') return tr('Firmware không trả lời.');
  if (code && ERRORS[code]) return tr(ERRORS[code]) + (x.result && x.result.reason ? ` (${x.result.reason})` : '');
  return (e && e.message) || String(e);
}
export const showError = (e) => toast(errorText(e), 'bad', 6500);

// ---- number formatting
export function fmt(v, digits) {
  if (v === null || v === undefined || Number.isNaN(v)) return '';
  if (digits !== undefined) return Number(v).toFixed(digits);
  return String(v);
}
function fieldText(f, v) {
  if (v === null || v === undefined) return '';
  if (f.t === 'f32') return String(Number(Number(v).toPrecision(6)));
  if (f.scale) return Number(v).toFixed(Math.max(0, Math.ceil(-Math.log10(f.scale))));
  return String(v);
}

// ---- widgets
// control(): the bare input for one field. opts: index (array element), empty (numeric value shown as an empty box, e.g. 255 for pins),
//            labels (enum value -> text), choices (a number field shown as a list of these values), width, onchange(value)
export function control(section, name, opts = {}) {
  if (!hasField(section, name)) return h('span', { class: 'unit', 'data-missing': `${section}.${name}` }, tr('(firmware này chưa có)'));      // (an older firmware)
  const f = fieldDef(section, name);
  const idx = opts.index;
  const get = () => { const v = (store.settings[section] || {})[name]; return idx === undefined ? v : (v ? v[idx] : undefined); };
  const put = (v) => { setField(section, name, v, idx); if (opts.onchange) opts.onchange(v); };
  let el, sync;
  if (opts.choices) {                                     // a number field limited to a few values (opts.choices: [value, ...])
    el = h('select', {}, opts.choices.map((v) => h('option', { value: String(v) }, String(v))));
    el.addEventListener('change', () => put(Number(el.value)));
    sync = () => { const v = get(); if (v !== undefined && v !== null && !opts.choices.includes(v)) el.append(h('option', { value: String(v) }, String(v))); el.value = v === undefined || v === null ? '' : String(v); };
  } else if (f.text) {                                     // a text kept as a 0-padded byte array (UTF-8): general.usb_name
    el = h('input', { type: 'text', class: 'text', autocomplete: 'off', spellcheck: 'false', placeholder: opts.placeholder });
    const enc = new TextEncoder(), dec = new TextDecoder();
    el.addEventListener('change', () => {
      let s = opts.ascii ? el.value.replace(/[^\x20-\x7E]/g, '_') : el.value, b = enc.encode(s);     // ascii: printable ASCII only (a USB name)
      while (b.length > f.count - 1) { s = [...s].slice(0, -1).join(''); b = enc.encode(s); }     // (cut whole characters: one byte is the end mark)
      el.value = s;
      const a = Array.from(b); while (a.length < f.count) a.push(0);
      put(a);
    });
    sync = (force) => {
      if (!force && document.activeElement === el) return;
      const a = get() || []; const z = a.indexOf(0);
      el.value = dec.decode(Uint8Array.from(z < 0 ? a : a.slice(0, z)));
    };
  } else if (f.t === 'bool') {
    el = h('input', { type: 'checkbox', class: 'switch', role: 'switch' });
    el.addEventListener('change', () => put(el.checked));
    sync = () => { el.checked = !!get(); };
  } else if (f.enum) {
    const names = store.schema.enums[f.enum];
    const labels = opts.labels || ENUM_LABELS[f.enum] || {};
    const todo = NOT_IMPLEMENTED[f.enum] || [];
    el = h('select', {},
      f.optional ? h('option', { value: '' }, tr('(không dùng)')) : null,
      names.map((n) => h('option', { value: n }, (labels[n] ? tr(labels[n]) : n) + (todo.includes(n) ? tr(' (firmware chưa hỗ trợ)') : ''))));
    el.addEventListener('change', () => put(el.value === '' ? null : el.value));
    sync = () => { const v = get(); el.value = v === null || v === undefined ? '' : v; };
  } else {
    const r = fieldRange(f);
    el = h('input', { type: 'number', class: 'num', inputmode: 'decimal' });
    if (Number.isFinite(r.min)) el.min = r.min;
    if (Number.isFinite(r.max)) el.max = r.max;
    el.step = r.step === null ? 'any' : r.step;
    if (opts.empty !== undefined) el.placeholder = tr('trống');
    const commit = () => {
      if (el.value.trim() === '') { if (opts.empty !== undefined) put(opts.empty); else sync(true); return; }
      let v = Number(el.value);
      if (!Number.isFinite(v)) { sync(true); return; }
      if (Number.isFinite(r.min)) v = Math.max(r.min, v);
      if (Number.isFinite(r.max)) v = Math.min(r.max, v);
      if (v !== get()) put(v);
      sync(true);
    };
    el.addEventListener('change', commit);
    sync = (force) => {
      if (!force && document.activeElement === el) return;         // never rewrite a box the user is typing in
      const v = get();
      el.value = (opts.empty !== undefined && v === opts.empty) ? '' : fieldText(f, v);
    };
  }
  if (opts.width) el.style.width = opts.width;
  if (opts.id) el.id = opts.id;
  el.dataset.field = `${section}.${name}` + (idx === undefined ? '' : `[${idx}]`);
  const lock = () => { el.disabled = store.armed || (opts.enabled ? !opts.enabled() : false); };      // opts.enabled() -> false greys it out
  listen('settings:' + section, () => { sync(); lock(); });
  listen('armed', lock);
  sync(); lock();
  return el;
}

let uid = 0;
// A range slider bound to one field. The number next to it follows the thumb; the firmware gets the value when the thumb is let go
// (one settings write per move, not one per pixel). opts: fmt(value) -> text, step, enabled() -> false greys it out, id, mark (value of a tick)
export function slider(section, name, opts = {}) {
  const f = fieldDef(section, name), r = fieldRange(f);
  const text = opts.fmt || ((v) => String(v));
  const input = h('input', { type: 'range', min: r.min, max: r.max, step: opts.step || r.step || 1 });
  const out = h('output', { class: 'num' });
  if (opts.id) { input.id = opts.id; out.setAttribute('for', opts.id); }
  input.dataset.field = `${section}.${name}`;
  const get = () => (store.settings[section] || {})[name];
  let dragging = false;
  const sync = () => { if (dragging) return; const v = get(); if (v !== undefined && v !== null) { input.value = v; out.textContent = text(v); } };
  const lock = () => { input.disabled = store.armed || (opts.enabled ? !opts.enabled() : false); };
  input.addEventListener('input', () => { dragging = true; out.textContent = text(Number(input.value)); });
  input.addEventListener('change', () => { dragging = false; const v = Number(input.value); if (v !== get()) setField(section, name, v); sync(); });
  listen('settings:' + section, () => { sync(); lock(); });
  listen('armed', lock);
  sync(); lock();
  const box = h('div', { class: 'slider' }, input, out);
  if (opts.mark !== undefined && r.max > r.min) { box.dataset.mark = opts.mark; box.style.setProperty('--mark-f', String((opts.mark - r.min) / (r.max - r.min))); }
  return box;
}
// A field's help text. A short one stays under the control; a long one (more than kHelpInline characters) sits behind a "?"
// next to it and shows in a bubble on hover, keyboard focus or a tap - the page stays short, the text is one move away.
// key: what it explains ("section.field", "mode.NAME"), for the tests. long: true when it became a "?".
export const kHelpInline = 140;
export function isLongHelp(text) { return !!text && text.length > kHelpInline; }
export function helpTip(text, key) {
  const pop = h('span', { class: 'help-pop', role: 'tooltip', id: 'tip' + (++uid) }, text);
  const btn = h('button', { type: 'button', class: 'help-tip', 'aria-label': tr('Giải thích'), 'aria-describedby': pop.id }, '?');
  const wrap = h('span', { class: 'help-wrap' }, btn, pop);
  if (key) wrap.dataset.helpFor = key;
  const place = () => {                                  // fixed, below the "?" (above when there is no room), inside the window
    const r = btn.getBoundingClientRect(), w = pop.offsetWidth, ht = pop.offsetHeight, m = 8;
    const left = Math.min(Math.max(r.left + r.width / 2 - w / 2, m), window.innerWidth - w - m);
    const below = r.bottom + 6, top = below + ht + m > window.innerHeight && r.top - 6 - ht >= m ? r.top - 6 - ht : below;
    pop.style.left = Math.max(left, m) + 'px'; pop.style.top = top + 'px';
  };
  const show = () => { wrap.classList.add('open'); place(); };
  const hide = () => wrap.classList.remove('open');
  btn.addEventListener('mouseenter', show); wrap.addEventListener('mouseleave', () => { if (document.activeElement !== btn) hide(); });
  btn.addEventListener('focus', show); btn.addEventListener('blur', hide);
  btn.addEventListener('click', (e) => { e.preventDefault(); e.stopPropagation(); if (wrap.classList.contains('open') && document.activeElement !== btn) hide(); else show(); });
  btn.addEventListener('keydown', (e) => { if (e.key === 'Escape') { hide(); btn.blur(); } });
  return wrap;
}
export function helpFor(text, key) {
  if (!text) return null;
  if (isLongHelp(text)) return helpTip(text, key);
  const p = h('p', { class: 'help' }, text); if (key) p.dataset.helpFor = key; return p;
}

// The labelled form row of a slider (texts from labels.js).
export function sliderRow(section, name, opts = {}) {
  if (!hasField(section, name)) return [];                 // (a setting an older firmware does not have: no row at all)
  const meta = FIELDS[`${section}.${name}`] || {};
  const id = 'f' + (++uid);
  const help = meta.help ? tr(meta.help) : '', key = `${section}.${name}`;
  return [
    h('label', { for: id }, opts.label || (meta.label ? tr(meta.label) : name)),
    h('div', { class: 'ctl' }, isLongHelp(help) ? h('div', { class: 'ctl-line' }, slider(section, name, { ...opts, id }), helpFor(help, key)) : slider(section, name, { ...opts, id }),
      isLongHelp(help) ? null : helpFor(help, key)),
  ];
}

// One labelled row for a form: [label, control + unit + help]. Text comes from labels.js, falling back to the schema.
export function row(section, name, opts = {}) {
  if (!hasField(section, name)) return [];                 // (a setting an older firmware does not have: no row at all)
  const f = fieldDef(section, name);
  const meta = FIELDS[`${section}.${name}`] || {};
  const id = 'f' + (++uid);
  const input = control(section, name, { ...opts, id });
  // texts from labels.js are Vietnamese source strings; texts passed in by a tab are already in the page's language
  const label = opts.label || (meta.label ? tr(meta.label) : name);
  const unit = opts.unit !== undefined ? opts.unit : (meta.unit ? tr(meta.unit) : meta.unit);
  const help = opts.help !== undefined ? opts.help : (meta.help ? tr(meta.help) : (meta.label || opts.label ? '' : f.desc));
  const needsReboot = !store.schema.sectionByName[section].live;
  const long = isLongHelp(help), key = `${section}.${name}`;
  return [
    h('label', { for: id }, label),
    h('div', { class: 'ctl' },
      h('div', { class: 'ctl-line' }, input, unit ? h('span', { class: 'unit' }, unit) : null, opts.after || null, long ? helpFor(help, key) : null),
      long ? null : helpFor(help, key),
      needsReboot && opts.rebootNote ? h('p', { class: 'help' }, tr('Có hiệu lực sau khi lưu và khởi động lại.')) : null),
  ];
}
export function form(...rows) { return h('div', { class: 'form' }, rows); }
export function fields(section, names, opts = {}) { return form(names.map((n) => row(section, n, opts[n] || {}))); }

export function block(title, ...kids) {
  return h('section', { class: 'block' }, title ? h('h2', {}, title) : null, kids);
}
export function note(text, kind = '') { return h('p', { class: 'note ' + kind }, text); }

// A button that runs a firmware command. It is disabled while armed unless the schema says the command is allowed then.
export function cmdButton(label, name, args, opts = {}) {
  const def = store.schema.commandByName[name];
  const btn = h('button', { type: 'button', class: 'btn ' + (opts.class || '') }, label);
  btn.dataset.cmd = name;
  if (!def) { btn.disabled = true; btn.title = tr('Firmware này chưa có chức năng này: cập nhật firmware.'); btn.dataset.missing = '1'; return btn; }      // (an older firmware)
  btn.addEventListener('click', async () => {
    btn.disabled = true;
    try {
      const res = await command(name, typeof args === 'function' ? args() : args);
      if (opts.done) opts.done(res);
    } catch (e) {
      if (opts.fail) opts.fail(e); else showError(e);
    } finally { lock(); }
  });
  const lock = () => { btn.disabled = store.armed && !(def && def.armed_ok); };
  listen('armed', lock); lock();
  return btn;
}

// A value that follows a telemetry stream: readout('status', v => v.loop_us + ' µs')
export function readout(stream, render, cls = 'num') {
  const el = h('span', { class: cls }, '–');
  const up = (v) => { const t = render(v); if (t !== undefined && el.textContent !== t) decodeText(el, t, el.textContent === '–'); };      // (the first value comes in decoded: anim.js)
  listen('stream:' + stream, up);
  if (store.streams[stream]) up(store.streams[stream]);
  return el;
}
export function kv(pairs) {
  return h('dl', { class: 'kv' }, pairs.filter(Boolean).map(([k, v]) => [h('dt', {}, k), h('dd', {}, v)]));
}

// Canvas sized for the display's pixel ratio; draw(ctx, w, h) works in CSS pixels. Redraws when the element is resized.
export function canvas(height, draw, cls = '') {
  const cv = h('canvas', { class: 'cv ' + cls, style: { height: height + 'px' } });
  const paint = () => {
    const w = cv.clientWidth, hh = cv.clientHeight;
    if (!w || !hh) return;
    const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(hh * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(hh * dpr); }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, hh);
    draw(ctx, w, hh);
  };
  const ro = new ResizeObserver(paint);
  ro.observe(cv);
  track(() => ro.disconnect());
  cv.paint = paint;
  return cv;
}
// Colours for canvas drawing come from the CSS tokens so the charts follow the light / dark theme.
export function token(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

// "Failsafe: đang tự hạ cánh (mất sóng)" - the stage and why, from the status stream, in the pilot's words
export function failsafeText(st) {
  const stage = ENUM_LABELS.failsafe[st.failsafe] ? tr(ENUM_LABELS.failsafe[st.failsafe]) : st.failsafe;
  const why = (st.fs_cause || []).map((c) => (ENUM_LABELS.fs_cause[c] ? tr(ENUM_LABELS.fs_cause[c]) : c)).join(', ');
  // (0.32: what it does - hold, fly home, land, motors off - when that is more than its stage says)
  const act = st.fs_action && st.fs_action !== 'NONE' && st.fs_action !== 'WARN' && ENUM_LABELS.fs_action[st.fs_action] ? tr(ENUM_LABELS.fs_action[st.fs_action]) : '';
  return 'Failsafe: ' + stage + (act && st.failsafe === 'DESCENT' ? ` - ${act}` : '') + (why ? ` (${why})` : '');
}
