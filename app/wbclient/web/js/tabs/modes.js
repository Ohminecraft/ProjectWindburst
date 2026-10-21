// Modes: which aux switch position turns each flight mode on (Betaflight style ranges). The firmware has 16 entries;
// an entry is { mode, aux channel, low, high } and a mode is on while its aux channel is inside one of its ranges.
import { store, setFields } from '../store.js';
import { h, block, fields, listen, note, toast } from '../ui.js';
import { MODES } from '../labels.js';
import { tr } from '../i18n.js';

const MIN = 900, MAX = 2100, STEP = 25;
const pct = (us) => ((Math.max(MIN, Math.min(MAX, us)) - MIN) / (MAX - MIN)) * 100;

// Two-handle range slider with a marker for the live channel value.
function rangeSlider(lo, hi, onCommit) {
  const sel = h('div', { class: 'range-sel' }), mark = h('i', { class: 'range-mark' });
  const mk = (which) => h('button', { type: 'button', class: 'range-h', role: 'slider', 'aria-valuemin': MIN, 'aria-valuemax': MAX, 'aria-label': which === 'lo' ? tr('Đầu dưới của dải') : tr('Đầu trên của dải'), 'data-h': which });
  const hl = mk('lo'), hh = mk('hi');
  const track = h('div', { class: 'range' }, h('div', { class: 'range-track' }), sel, mark, hl, hh);
  const state = { lo, hi, dragging: false };
  const paint = () => {
    sel.style.left = pct(state.lo) + '%'; sel.style.width = Math.max(0, pct(state.hi) - pct(state.lo)) + '%';
    hl.style.left = pct(state.lo) + '%'; hh.style.left = pct(state.hi) + '%';
    hl.setAttribute('aria-valuenow', state.lo); hh.setAttribute('aria-valuenow', state.hi);
  };
  const setFrom = (which, us) => {
    us = Math.round(us / STEP) * STEP;
    if (which === 'lo') state.lo = Math.max(MIN, Math.min(us, state.hi - STEP)); else state.hi = Math.min(MAX, Math.max(us, state.lo + STEP));
    paint(); if (track.oninput2) track.oninput2(state.lo, state.hi);
  };
  for (const [el, which] of [[hl, 'lo'], [hh, 'hi']]) {
    el.addEventListener('pointerdown', (e) => {
      el.setPointerCapture(e.pointerId); state.dragging = true;
      const move = (ev) => { const r = track.getBoundingClientRect(); setFrom(which, MIN + ((ev.clientX - r.left) / r.width) * (MAX - MIN)); };
      const up = () => { el.removeEventListener('pointermove', move); el.removeEventListener('pointerup', up); state.dragging = false; onCommit(state.lo, state.hi); };
      el.addEventListener('pointermove', move); el.addEventListener('pointerup', up);
    });
    el.addEventListener('keydown', (e) => {
      const d = e.key === 'ArrowLeft' || e.key === 'ArrowDown' ? -STEP : e.key === 'ArrowRight' || e.key === 'ArrowUp' ? STEP : 0;
      if (!d) return;
      e.preventDefault(); setFrom(which, (which === 'lo' ? state.lo : state.hi) + d); onCommit(state.lo, state.hi);
    });
  }
  paint();
  return {
    el: track,
    set(l, hgh) { if (state.dragging) return; state.lo = l; state.hi = hgh; paint(); },
    mark(us) { if (us === undefined) { mark.hidden = true; return; } mark.hidden = false; mark.style.left = pct(us) + '%'; },
    onInput(fn) { track.oninput2 = fn; },
    lock(dis) { hl.disabled = hh.disabled = dis; },
  };
}

export default {
  id: 'modes',
  title: tr('Mode'),
  lead: tr('Gán công tắc AUX cho từng mode. Vạch đậm trên thanh là vị trí công tắc lúc này: gạt công tắc để xem nó rơi vào đâu.'),
  streams: { rc: 20, status: 10 },
  mount(view) {
    const list = h('div', { class: 'modes', id: 'modes' });
    let rows = [], signature = '';
    const m = () => store.settings.modes;
    const stickArm = () => !!(store.settings.general || {}).stick_arm;
    const hasStickArm = store.schema.sectionByName.general.fields.some((f) => f.n === 'stick_arm');

    function writeSlot(i, patch) {
      const cur = m(), out = { mode: [...cur.mode], aux: [...cur.aux], lo: [...cur.lo], hi: [...cur.hi] };
      for (const [k, v] of Object.entries(patch)) out[k][i] = v;
      setFields('modes', out);
    }
    function build() {
      const cur = m();
      rows = [];
      list.replaceChildren(...store.schema.enums.mode.map((name) => {
        const meta = MODES[name] || { label: name };
        const state = h('span', { class: 'pill off', 'data-mode-state': name }, tr('Tắt'));
        const ranges = h('div', { class: 'mode-ranges' });
        cur.mode.forEach((mm, i) => {
          if (mm !== name) return;
          const aux = h('select', { 'aria-label': tr('Kênh AUX') }, Array.from({ length: 8 }, (_, k) => h('option', { value: k }, 'AUX' + (k + 1))));
          aux.value = cur.aux[i];
          aux.addEventListener('change', () => writeSlot(i, { aux: Number(aux.value) }));
          const text = h('span', { class: 'num range-text' }, `${cur.lo[i]} – ${cur.hi[i]}`);
          const sl = rangeSlider(cur.lo[i], cur.hi[i], (lo, hi) => writeSlot(i, { lo, hi }));
          sl.onInput((lo, hi) => { text.textContent = `${lo} – ${hi}`; });
          const del = h('button', { type: 'button', class: 'btn quiet', title: tr('Xóa dải này') }, tr('Xóa'));
          del.addEventListener('click', () => writeSlot(i, { mode: null, aux: 0, lo: 1000, hi: 1000 }));
          const row = h('div', { class: 'mode-range', 'data-slot': i }, aux, sl.el, text, del);
          ranges.append(row);
          rows.push({ i, name, aux, sl, text, row, del });
        });
        const add = h('button', { type: 'button', class: 'btn quiet', 'data-add': name }, tr('Thêm dải'));
        add.addEventListener('click', () => {
          const free = m().mode.findIndex((x) => x === null);
          if (free < 0) { toast(tr('Đã dùng hết 16 dải. Xóa bớt một dải trước.'), 'warn'); return; }
          writeSlot(free, { mode: name, aux: 0, lo: 1700, hi: 2100 });
        });
        const none = !cur.mode.includes(name);
        return h('div', { class: 'mode' + (meta.todo ? ' todo' : ''), 'data-mode': name },
          h('div', { class: 'mode-head' }, h('h3', {}, tr(meta.label)), state, h('p', { class: 'help' }, meta.help ? tr(meta.help) : '')),
          h('div', { class: 'mode-body' }, ranges,
            none && name === 'ARM' ? h('p', { class: 'note ' + (stickArm() ? '' : 'bad'), id: 'no-arm-switch' }, stickArm() ? tr('Đang arm bằng cần: không cần công tắc ARM. Nên gán một công tắc ARM để có thể tắt motor ngay lập tức.') : tr('Chưa gán công tắc ARM: máy sẽ không arm được.')) : null,
            add));
      }));
      signature = JSON.stringify([cur.mode, cur.aux]);
      lock(); live();
    }
    function sync() {
      const cur = m();
      if (JSON.stringify([cur.mode, cur.aux]) !== signature) { build(); return; }
      for (const r of rows) { r.sl.set(cur.lo[r.i], cur.hi[r.i]); r.text.textContent = `${cur.lo[r.i]} – ${cur.hi[r.i]}`; }
    }
    function live() {
      const rc = store.streams.rc, rx = store.settings.receiver, cur = m(), st = store.streams.status;
      for (const r of rows) {
        const us = rc ? rc.ch[rx.ch_aux0 + cur.aux[r.i]] : undefined;
        r.sl.mark(us);
        r.row.classList.toggle('inside', us !== undefined && us >= cur.lo[r.i] && us <= cur.hi[r.i]);
      }
      if (st) for (const el of list.querySelectorAll('[data-mode-state]')) {
        const onNow = st.modes.includes(el.dataset.modeState);
        el.className = 'pill ' + (onNow ? 'ok' : 'off'); el.textContent = onNow ? tr('Đang bật') : tr('Tắt');
      }
    }
    function lock() {
      for (const r of rows) { r.aux.disabled = r.del.disabled = store.armed; r.sl.lock(store.armed); }
      for (const b of list.querySelectorAll('[data-add]')) b.disabled = store.armed;
    }
    listen('settings:modes', sync); listen('stream:rc', live); listen('stream:status', live); listen('armed', lock);
    build();
    if (hasStickArm) listen('settings:general', build);
    view.append(
      hasStickArm ? block(tr('Arm bằng cần'), fields('general', ['stick_arm'], { stick_arm: { rebootNote: true } })) : null,
      block(null, list),
      note(tr('Altitude Hold, Position Hold và GPS Rescue tự bật Angle. Mode không có dải nào thì luôn tắt. Thay đổi ở tab này có tác dụng ngay, nhưng vẫn cần bấm "Lưu" để giữ lại sau khi tắt nguồn.')));
  },
};
