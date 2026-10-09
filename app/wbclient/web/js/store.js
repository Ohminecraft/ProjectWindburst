// What the page knows: the protocol schema, the connection, the settings as the firmware holds them in RAM, the latest telemetry.
import { api } from './api.js';

export const store = {
  schema: null,
  conn: { connected: false, responding: false, port: null, note: '', info: null, report: [], boots: 0 },
  serverUp: true,
  settings: {},        // section -> values (engineering units, enum names)
  streams: {},         // stream -> latest values
  logs: [],            // [level, text]
  raw: '',             // what the port sent outside the protocol (boot text, crash dump), as text
  armed: false,
  job: null,           // the log download in progress or just finished (wbclient/gui.py)
  rec: null,           // the live blackbox recording: { on, file, records, duration, ... }
};

const listeners = new Map();
export function on(evt, fn) {
  if (!listeners.has(evt)) listeners.set(evt, new Set());
  listeners.get(evt).add(fn);
  return () => listeners.get(evt).delete(fn);
}
export function emit(evt, data) {
  const set = listeners.get(evt);
  if (set) for (const fn of [...set]) { try { fn(data); } catch (e) { console.error(evt, e); } }
}

// ---- schema helpers
export async function loadSchema() {
  const s = await api.get('/api/schema');
  s.sectionByName = Object.fromEntries(s.sections.map((x) => [x.name, x]));
  s.streamByName = Object.fromEntries(s.streams.map((x) => [x.name, x]));
  s.commandByName = Object.fromEntries(s.commands.map((x) => [x.name, x]));
  store.schema = s;
}
export const sectionDef = (section) => store.schema.sectionByName[section];
export function fieldDef(section, name) {
  const f = sectionDef(section).fields.find((x) => x.n === name);
  if (!f) throw new Error(`no field ${section}.${name}`);
  return f;
}
// limits of a field in the units the page shows (scaled)
export function fieldRange(f) {
  const k = f.scale || 1;
  const nat = { u8: [0, 255], u16: [0, 65535], u32: [0, 4294967295], i16: [-32768, 32767], i32: [-2147483648, 2147483647] }[f.t];
  const lo = f.min !== undefined ? f.min : (nat ? nat[0] : -Infinity);
  const hi = f.max !== undefined ? f.max : (nat ? nat[1] : Infinity);
  return { min: lo * k, max: hi * k, step: f.t === 'f32' ? null : k };
}

// ---- settings
export async function loadSettings() {
  store.settings = await api.get('/api/settings');
  emit('settings');
  for (const name of Object.keys(store.settings)) emit('settings:' + name);
}
export async function reloadSection(section) {
  const all = await api.get('/api/settings');
  store.settings[section] = all[section];
  emit('settings:' + section);
}

const pending = new Map();   // section -> { changes, timer }
// Change one field (or one element of an array field). The page shows the new value at once, the firmware gets it ~0.2 s later
// and answers with what it really stored (it clamps), which then replaces the value on the page.
export function setField(section, name, value, index) {
  const cur = store.settings[section];
  if (!cur) return;
  let full = value;
  if (index !== undefined && index !== null) { full = [...cur[name]]; full[index] = value; }
  cur[name] = full;
  let p = pending.get(section);
  if (!p) { p = { changes: {}, timer: null }; pending.set(section, p); }
  p.changes[name] = full;
  clearTimeout(p.timer);
  p.timer = setTimeout(() => flush(section), 180);
  emit('settings:' + section, { local: true });
}
// Several fields of one section at once (kept in one request so the firmware never sees a half-edited entry).
export function setFields(section, changes) {
  const cur = store.settings[section];
  if (!cur) return;
  Object.assign(cur, changes);
  let p = pending.get(section);
  if (!p) { p = { changes: {}, timer: null }; pending.set(section, p); }
  Object.assign(p.changes, changes);
  clearTimeout(p.timer);
  p.timer = setTimeout(() => flush(section), 180);
  emit('settings:' + section, { local: true });
}
const sending = new Map();   // section -> the request that is on its way
// Sends what was edited in a section. Requests of one section go out one after the other: a second edit made while the first
// is still on its way must reach the firmware AFTER it (the firmware answers each write with the whole section as it then is).
export async function flush(section) {
  const p = pending.get(section);
  if (!p) return sending.get(section);
  pending.delete(section);
  clearTimeout(p.timer);
  const before = sending.get(section) || Promise.resolve();
  const run = before.then(async () => {
    try {
      const r = await api.post('/api/set', { section, changes: p.changes });
      if (!pending.has(section) && sending.get(section) === run) store.settings[section] = r.values;     // newer edits are already on their way: keep showing those
      emit('settings:' + section);
    } catch (e) {
      emit('error', e);
      try { await reloadSection(section); } catch (e2) { /* disconnected */ }
    }
  });
  sending.set(section, run);
  await run;
  if (sending.get(section) === run) sending.delete(section);
}
export async function flushAll() { await Promise.all([...new Set([...pending.keys(), ...sending.keys()])].map(flush)); }
export const hasPending = () => pending.size > 0 || sending.size > 0;

// ---- telemetry subscriptions: every part of the page says what it needs, the firmware gets the union
const wants = new Map();     // owner -> { stream: hz }
let sent = '';
export function wantStreams(owner, rates) {
  if (rates) wants.set(owner, rates); else wants.delete(owner);
  pushStreams();
}
export async function pushStreams(force) {
  if (!store.conn.connected) { sent = ''; return; }
  const merged = {};
  for (const r of wants.values()) for (const [k, v] of Object.entries(r)) merged[k] = Math.max(merged[k] || 0, v);
  const key = JSON.stringify(Object.entries(merged).sort());
  if (key === sent && !force) return;
  sent = key;
  try { await api.post('/api/streams', { rates: merged }); } catch (e) { sent = ''; }
}

// ---- incoming events
export function handleEvent(msg) {
  if (msg.conn) {
    const before = store.conn;
    store.conn = msg.conn;
    if (!msg.conn.connected) { store.streams = {}; store.armed = false; sent = ''; }
    if (msg.conn.connected && (!before.connected || before.port !== msg.conn.port)) store.raw = '';      // a new connection starts a new transcript
    emit('conn', { before });
  }
  if (msg.s) {
    for (const [name, v] of Object.entries(msg.s)) {
      store.streams[name] = v;
      if (name === 'status') {
        const armed = !!v.armed;
        if (armed !== store.armed) { store.armed = armed; emit('armed', armed); }
      }
      emit('stream:' + name, v);
    }
  }
  if (msg.job) { store.job = msg.job; emit('job', msg.job); }
  if (msg.rec) { store.rec = msg.rec; emit('rec', msg.rec); }
  if (msg.f) emit('fast', msg.f);
  if (msg.raw) {
    store.raw = (store.raw + msg.raw).slice(-20000);
    emit('raw', msg.raw);
  }
  if (msg.log) {
    for (const row of msg.log) store.logs.push(row);
    if (store.logs.length > 600) store.logs.splice(0, store.logs.length - 600);
    emit('log', msg.log);
  }
}

export async function command(name, args) {
  const r = await api.post('/api/command', { name, args: args || {} });
  const def = store.schema.commandByName[name];
  if (def && def.writes) await reloadSection(def.writes);      // e.g. a finished calibration changed the calibration section
  return r.result;
}
