// Mission: the waypoint plan on a map (docs/MISSION.md; layout C of the design: the map fills the tab, everything else floats on
// it). Click the map to add a point, drag a point to move it, click one to say what the craft does there. Upload writes the
// `mission` section and saves it; Start / Stop send the MISSION command (the pilot arms first). The craft's GPS position, its
// heading and the way it flew are drawn live. The map tiles come through the local server, which keeps them on disk
// (wbclient/maptiles.py): a place looked at once is there again without a network.
import { store, reloadSection, command } from '../store.js';
import { api } from '../api.js';
import { h, toast, showError, listen, track } from '../ui.js';
import { ENUM_LABELS } from '../labels.js';
import { tr } from '../i18n.js';
import { encodePlan, decodePlan, estimate, distance, commandCount, MAX_COMMANDS } from '../mission_plan.js';

// the plan being edited: kept while the app runs (switching tabs does not lose it)
const plan = { takeoff_alt: 10, speed: 0, rtl: true, rc_lost: 'CONTINUE', points: [] };
const view = { base: 'sat', names: true, selected: -1, center: null, zoom: 17, trail: [], me: null };     // me: where this computer is ({lat, lon, acc})
const acts = () => ({ fly: tr('Bay tới'), servo: tr('Đứng yên · servo'), land: tr('Hạ cánh · chờ · cất cánh') });
const DEFAULT_CENTER = [21.0285, 105.8542];       // (no GPS yet, nothing saved: Hà Nội)

const num = (v, d = 0) => (v === undefined || v === null || Number.isNaN(Number(v)) ? '–' : Number(v).toFixed(d));
const gpsFix = () => { const g = store.streams.gps; return g && g.fix >= 3 && (g.lat || g.lon) ? { lat: g.lat, lon: g.lon } : null; };
const speedOf = () => (plan.speed > 0 ? plan.speed : Number((store.settings.nav || {}).rescue_ground_speed) || 7.5);
function fmtTime(s) { const m = Math.floor(s / 60), r = Math.round(s % 60); return m ? tr('{m} phút {r} s', { m, r }) : `${r} s`; }
function input(id, value, onChange, attrs = {}) {
  const el = h('input', { id, type: 'number', value: String(value), class: 'num', ...attrs });
  el.addEventListener('change', () => onChange(Number(el.value)));
  return el;
}

function newPoint(latlng) {
  const last = plan.points[plan.points.length - 1];
  return { lat: Number(latlng.lat.toFixed(7)), lon: Number(latlng.lng.toFixed(7)), alt: last ? last.alt : plan.takeoff_alt, act: 'fly', hold: 0, hover: 5, steps: [], wait: 60 };
}

export default {
  id: 'mission',
  title: tr('Nhiệm vụ'),
  lead: tr('Kế hoạch bay tự động: bấm lên bản đồ để thêm điểm, bấm vào điểm để chọn việc drone làm ở đó, rồi nạp vào drone. Arm bằng công tắc trước, sau đó bật công tắc MISSION hoặc bấm Bắt đầu.'),
  streams: { gps: 5, nav: 5, attitude: 5, estimate: 5, battery: 2 },
  mount(root) {
    const L = window.L;
    if (!L) { root.append(h('p', { class: 'note bad' }, tr('Không tải được thư viện bản đồ (Leaflet).'))); return; }
    root.classList.add('view-fill'); track(() => root.classList.remove('view-fill'));      // (app.css: the map fills the tab's area)
    const mapEl = h('div', { id: 'mission-map', class: 'mission-map' });
    const wrap = h('div', { class: 'mission-wrap', id: 'mission-wrap' }, mapEl);
    root.append(wrap);

    const fix = gpsFix();
    const center = view.center || (fix ? [fix.lat, fix.lon] : plan.points[0] ? [plan.points[0].lat, plan.points[0].lon] : DEFAULT_CENTER);
    const map = L.map(mapEl, { zoomControl: true }).setView(center, view.zoom);
    track(() => { view.center = [map.getCenter().lat, map.getCenter().lng]; view.zoom = map.getZoom(); map.remove(); });
    const tileUrl = (p) => api.url('/api/tile', { p }) + '&z={z}&x={x}&y={y}';
    const tiles = {
      sat: L.tileLayer(tileUrl('sat'), { maxZoom: 19, zIndex: 1, attribution: 'Esri World Imagery' }),
      osm: L.tileLayer(tileUrl('osm'), { maxZoom: 19, zIndex: 1, attribution: '© OpenStreetMap' }),
    };
    // the names over the satellite pictures (Google's satellite view with its labels): roads, then places; above any base map
    const names = L.layerGroup([
      L.tileLayer(tileUrl('road'), { maxZoom: 19, zIndex: 2, attribution: 'Esri Reference' }),
      L.tileLayer(tileUrl('lbl'), { maxZoom: 19, zIndex: 3 }),
    ]);
    tiles[view.base].addTo(map);

    const layer = L.layerGroup().addTo(map);           // the plan: route, points, distances
    const live = L.layerGroup().addTo(map);            // the craft and its track
    const trail = L.polyline(view.trail, { color: '#ff5a3c', weight: 3, opacity: 0.9, interactive: false }).addTo(live);
    const droneIcon = (yaw) => L.divIcon({ className: 'drone-marker', iconSize: [30, 30], html: `<svg viewBox="-15 -15 30 30" style="transform: rotate(${yaw || 0}deg)"><path d="M0 -12 L8 9 L0 4 L-8 9 Z"/></svg>` });
    let drone = null;

    // ---- the floating boxes
    const search = h('input', { id: 'mission-search', type: 'text', enterkeyhint: 'search', placeholder: tr('Tìm địa chỉ…'), 'aria-label': tr('Tìm địa chỉ') });
    const hits = h('div', { id: 'mission-hits', class: 'mission-hits' });
    // the chosen map is a primary button: every theme draws that one readable (a home-made 'on' colour was black on black in Swiss)
    const baseBtn = (b, text) => { const el = h('button', { type: 'button', class: 'btn ' + (view.base === b ? 'primary' : 'quiet'), id: 'map-' + b, 'aria-pressed': String(view.base === b) }, text); el.addEventListener('click', () => setBase(b)); return el; };
    const btn = (id, text, cls, fn) => { const el = h('button', { type: 'button', class: 'btn ' + cls, id }, text); el.addEventListener('click', fn); return el; };
    const meBtn = btn('map-me', tr('Vị trí của tôi'), 'quiet', locateMe);
    meBtn.title = tr('Vị trí của máy tính này, theo dịch vụ vị trí của Windows (GPS, Wi-Fi). Lần đầu Windows hỏi quyền.');
    const namesBtn = btn('map-names', tr('Tên đường'), 'quiet', () => { view.names = !view.names; paintNames(); });
    namesBtn.title = tr('Tên đường, phường, địa danh trên ảnh vệ tinh');
    const topLeft = h('div', { class: 'mission-float mission-tl', id: 'mission-bar' }, search, meBtn, baseBtn('sat', tr('Vệ tinh')), namesBtn, baseBtn('osm', tr('Đường phố')), hits);
    const linkPill = h('span', { class: 'pill off', id: 'mission-link' }, '–');
    const topRight = h('div', { class: 'mission-float mission-tr', id: 'mission-actions' }, linkPill,
      btn('mission-download', tr('Tải từ drone'), 'quiet', download), btn('mission-upload', tr('Nạp vào drone'), 'primary', upload),
      btn('mission-start', tr('Bắt đầu'), 'danger', () => start(true)), btn('mission-stop', tr('Dừng'), 'quiet', () => start(false)));
    const opts = h('div', { class: 'mission-float mission-opts', id: 'mission-opts' });
    const editor = h('div', { class: 'mission-float mission-editor', id: 'mission-editor', hidden: true });
    const hud = h('div', { class: 'mission-hud', id: 'mission-hud' });
    wrap.append(topLeft, topRight, opts, editor, hud);
    for (const el of [topLeft, topRight, opts, editor, hud]) { L.DomEvent.disableClickPropagation(el); L.DomEvent.disableScrollPropagation(el); }

    function setBase(b) {
      if (b === view.base) return;
      map.removeLayer(tiles[view.base]); view.base = b; tiles[b].addTo(map);
      for (const k of ['sat', 'osm']) { const el = document.getElementById('map-' + k); el.className = 'btn ' + (k === b ? 'primary' : 'quiet'); el.setAttribute('aria-pressed', String(k === b)); }
      paintNames();
    }
    // (the street map has its names drawn in: the button is there with the satellite pictures only)
    function paintNames() {
      const on = view.base === 'sat' && view.names;
      if (on !== map.hasLayer(names)) { if (on) names.addTo(map); else map.removeLayer(names); }
      namesBtn.hidden = view.base !== 'sat';
      namesBtn.className = 'btn ' + (view.names ? 'primary' : 'quiet'); namesBtn.setAttribute('aria-pressed', String(view.names));
    }
    paintNames();

    // ---- address search (OpenStreetMap Nominatim through the local server)
    search.addEventListener('keydown', async (ev) => {
      if (ev.key !== 'Enter') return;
      hits.replaceChildren();
      try {
        const r = await api.query('/api/geocode', { q: search.value });
        if (!r.hits.length) { hits.append(h('div', { class: 'help' }, tr('Không tìm thấy (hoặc không có mạng).'))); return; }
        for (const x of r.hits) {
          const a = h('button', { type: 'button', class: 'hit' }, x.name);
          a.addEventListener('click', () => { map.setView([x.lat, x.lon], 17); hits.replaceChildren(); });
          hits.append(a);
        }
      } catch (e) { showError(e); }
    });

    // ---- where this computer is (navigator.geolocation: in the desktop window that is Windows' location service - GPS, Wi-Fi
    // around - which asks the user the first time; desktop.py lets the page ask without a box of WebView2's own)
    const meLayer = L.layerGroup().addTo(map);
    let alive = true; track(() => { alive = false; });
    function paintMe() {
      meLayer.clearLayers();
      const m = view.me; if (!m) return;
      L.circle([m.lat, m.lon], { radius: m.acc, color: '#1a73e8', weight: 1, fillOpacity: 0.12, interactive: false }).addTo(meLayer);
      L.circleMarker([m.lat, m.lon], { radius: 7, color: '#fff', weight: 2, fillColor: '#1a73e8', fillOpacity: 1, className: 'me-marker', interactive: false }).addTo(meLayer);
    }
    function locateMe() {
      hits.replaceChildren();
      if (!navigator.geolocation) { hits.append(h('div', { class: 'help' }, tr('Máy này không cho biết vị trí.'))); return; }
      meBtn.disabled = true;
      navigator.geolocation.getCurrentPosition((pos) => {
        if (!alive) return;
        meBtn.disabled = false;
        view.me = { lat: pos.coords.latitude, lon: pos.coords.longitude, acc: pos.coords.accuracy || 0 };
        paintMe(); map.setView([view.me.lat, view.me.lon], Math.max(map.getZoom(), 17));
        toast(tr('Vị trí của bạn (sai số khoảng {m} m).', { m: Math.round(view.me.acc) }), 'ok');
      }, (err) => {
        if (!alive) return;
        meBtn.disabled = false;
        const why = err.code === 1 ? tr('Windows chưa cho phép lấy vị trí. Bật trong Cài đặt > Quyền riêng tư và bảo mật > Vị trí: "Dịch vụ vị trí" và "Cho phép ứng dụng máy tính truy cập vị trí".')
          : err.code === 3 ? tr('Chờ vị trí quá lâu. Bật Wi-Fi (Windows dò vị trí theo các mạng Wi-Fi xung quanh) rồi thử lại.')
            : tr('Không xác định được vị trí. Bật Wi-Fi hoặc GPS của máy và dịch vụ vị trí của Windows, rồi thử lại.');
        const open = btn('map-me-settings', tr('Mở cài đặt vị trí của Windows'), 'quiet', async () => {
          try { const r = await api.post('/api/location-settings'); if (!r.opened) toast(tr('Không mở được: hãy mở Cài đặt của Windows > Quyền riêng tư và bảo mật > Vị trí.'), 'bad', 8000); } catch (e) { showError(e); }
        });
        hits.append(h('div', { class: 'help', id: 'map-me-error' }, why), err.code === 3 ? null : open);
      }, { enableHighAccuracy: true, timeout: 20000, maximumAge: 30000 });
    }

    // ---- the plan's options
    function paintOpts() {
      const rtl = h('input', { type: 'checkbox', id: 'mission-rtl' }); rtl.checked = plan.rtl;
      rtl.addEventListener('change', () => { plan.rtl = rtl.checked; redraw(); });
      const rc = h('select', { id: 'mission-rclost' }, Object.entries(ENUM_LABELS.mission_rc_lost).map(([k, v]) => h('option', { value: k }, tr(v))));
      rc.value = plan.rc_lost; rc.addEventListener('change', () => { plan.rc_lost = rc.value; });
      opts.replaceChildren(
        h('label', {}, tr('Cất cánh lên (m)'), ' ', input('mission-takeoff', plan.takeoff_alt, (v) => { plan.takeoff_alt = v; redraw(); }, { min: 2, max: 120, step: 1 })),
        h('label', {}, tr('Tốc độ (m/s, 0 = như rescue)'), ' ', input('mission-speed', plan.speed, (v) => { plan.speed = v; redraw(); }, { min: 0, max: 20, step: 0.5 })),
        h('label', {}, rtl, ' ', tr('Xong điểm cuối thì về nhà hạ cánh')),
        h('label', {}, tr('Mất sóng tay điều khiển:'), ' ', rc),
        btn('mission-clear', tr('Xoá hết điểm'), 'quiet', clearAll));
    }
    function clearAll() {
      if (!plan.points.length) { toast(tr('Chưa có điểm nào: bấm lên bản đồ để thêm.')); return; }
      if (!window.confirm(tr('Xoá cả {n} điểm trên bản đồ? Kế hoạch đã nạp trong drone giữ nguyên cho tới khi bạn bấm "Nạp vào drone".', { n: plan.points.length }))) return;
      plan.points.splice(0); view.selected = -1; redraw();
    }

    // ---- route, points, distances
    function redraw() {
      layer.clearLayers();
      const home = gpsFix();
      const pts = plan.points.map((p) => [p.lat, p.lon]);
      if (home) {
        L.marker([home.lat, home.lon], { icon: L.divIcon({ className: 'home-marker', html: '<span>H</span>', iconSize: [26, 26] }), interactive: false }).addTo(layer);
      }
      const route = home ? [[home.lat, home.lon], ...pts] : pts;
      if (route.length > 1) L.polyline(route, { color: '#7fb4ff', weight: 3.5, dashArray: '9 6', interactive: false }).addTo(layer);
      if (plan.rtl && home && pts.length) L.polyline([pts[pts.length - 1], [home.lat, home.lon]], { color: '#7fb4ff', weight: 2, dashArray: '3 7', opacity: 0.7, interactive: false }).addTo(layer);
      for (let i = 1; i < route.length; i++) {
        const a = { lat: route[i - 1][0], lon: route[i - 1][1] }, b = { lat: route[i][0], lon: route[i][1] };
        L.marker([(a.lat + b.lat) / 2, (a.lon + b.lon) / 2], { icon: L.divIcon({ className: 'leg-label', html: `${num(distance(a, b))} m`, iconSize: null }), interactive: false }).addTo(layer);
      }
      plan.points.forEach((p, i) => {
        const cls = 'wp-marker' + (i === view.selected ? ' sel' : '') + (p.act === 'land' ? ' land' : p.act === 'servo' ? ' servo' : '');
        const mk = L.marker([p.lat, p.lon], { draggable: true, icon: L.divIcon({ className: cls, html: `<span data-wp="${i}">${i + 1}</span>`, iconSize: [28, 28] }) }).addTo(layer);
        mk.on('click', () => { view.selected = i; redraw(); });
        mk.on('dragend', () => { const ll = mk.getLatLng(); p.lat = Number(ll.lat.toFixed(7)); p.lon = Number(ll.lng.toFixed(7)); redraw(); });
      });
      paintEditor(); paintHud();
    }
    map.on('click', (ev) => { plan.points.push(newPoint(ev.latlng)); view.selected = plan.points.length - 1; redraw(); });

    // ---- the floating editor of the selected point
    function paintEditor() {
      const p = plan.points[view.selected];
      if (!p) { editor.hidden = true; editor.replaceChildren(); return; }
      editor.hidden = false;
      const i = view.selected;
      const act = h('select', { id: 'pt-act' }, Object.entries(acts()).map(([k, v]) => h('option', { value: k }, v)));
      act.value = p.act; act.addEventListener('change', () => { p.act = act.value; redraw(); });
      const rows = [h('label', { class: 'wide' }, tr('Việc'), act),
        h('label', {}, p.act === 'land' ? tr('Cất cánh lại lên (m)') : tr('Cao khi bay (m)'), input('pt-alt', p.alt, (v) => { p.alt = v; redraw(); }, { min: 2, max: 120, step: 1 }))];
      if (p.act === 'fly') rows.push(h('label', {}, tr('Giữ ở điểm (s)'), input('pt-hold', p.hold || 0, (v) => { p.hold = v; paintHud(); }, { min: 0, max: 3600 })));
      if (p.act === 'land') rows.push(h('label', {}, tr('Chờ dưới đất (s)'), input('pt-wait', p.wait || 0, (v) => { p.wait = v; paintHud(); }, { min: 0, max: 3600 })));
      if (p.act === 'servo') {
        p.steps = p.steps || [];
        rows.push(h('label', {}, tr('Đứng yên trước (s)'), input('pt-hover', p.hover || 0, (v) => { p.hover = v; paintHud(); }, { min: 0, max: 3600 })));
        const servos = (store.settings.servo || {}).source || [];
        const note = h('p', { class: 'help', id: 'step-note' });
        const paintNote = () => {         // the servos of these steps that the mission cannot move: said once, under the steps
          const off = [...new Set(p.steps.map((s) => s.servo))].filter((n) => servos[n - 1] !== 'MISSION').sort();
          note.textContent = off.length ? tr('Servo {list} chưa chọn kênh "Nhiệm vụ" ở tab Motor: nhiệm vụ sẽ không điều khiển được.', { list: off.join(', ') }) : '';
          note.hidden = !off.length;
        };
        p.steps.forEach((s, k) => {
          const sel = h('select', { id: `step-servo-${k}`, 'aria-label': tr('Servo') }, [1, 2, 3, 4].map((n) => h('option', { value: String(n) }, `Servo ${n}`)));
          sel.value = String(s.servo); sel.addEventListener('change', () => { s.servo = Number(sel.value); paintNote(); });
          const del = h('button', { type: 'button', class: 'btn quiet', id: `step-del-${k}`, title: tr('Bỏ bước này') }, '✕');
          del.addEventListener('click', () => { p.steps.splice(k, 1); paintEditor(); paintHud(); });
          rows.push(h('div', { class: 'step-row' }, sel, input(`step-deg-${k}`, s.deg, (v) => { s.deg = v; }, { min: -360, max: 360, title: tr('Góc (°)') }), h('span', {}, '°'),
            input(`step-wait-${k}`, s.wait, (v) => { s.wait = v; paintHud(); }, { min: 0, max: 3600, title: tr('Chờ sau đó (s)') }), h('span', {}, 's'), del));
        });
        const add = h('button', { type: 'button', class: 'btn quiet', id: 'step-add' }, tr('Thêm bước servo'));
        add.addEventListener('click', () => { p.steps.push({ servo: 1, deg: 90, wait: 2 }); paintEditor(); paintHud(); });
        paintNote();
        rows.push(note, add);
      }
      const del = h('button', { type: 'button', class: 'btn quiet', id: 'pt-delete' }, tr('Xoá điểm'));
      del.addEventListener('click', () => { plan.points.splice(i, 1); view.selected = -1; redraw(); });
      const close = h('button', { type: 'button', class: 'btn quiet close', id: 'pt-close', 'aria-label': tr('Đóng') }, '✕');
      close.addEventListener('click', () => { view.selected = -1; redraw(); });
      editor.replaceChildren(h('div', { class: 'ed-head' }, h('b', {}, tr('Điểm {n}', { n: i + 1 })), close), ...rows, h('div', { class: 'btn-row' }, del));
    }

    // ---- the bottom line: what the craft does now, and the plan's size
    function paintHud() {
      const n = store.streams.nav || {}, e = store.streams.estimate || {}, b = store.streams.battery || {};
      const st = n.mission_state || 'IDLE';
      const doing = st === 'IDLE' ? tr('Chưa chạy') : `${tr(ENUM_LABELS.mission_state[st] || st)}${n.mission_count ? ` (${(n.mission_index || 0) + 1}/${n.mission_count})` : ''}`;
      const home = gpsFix() || (plan.points[0] ? { lat: plan.points[0].lat, lon: plan.points[0].lon } : null);
      const est = home ? estimate(plan, home, speedOf()) : { distance_m: 0, time_s: 0 };
      const cmds = commandCount(plan);
      const cell = (id, label, value) => h('div', { id }, label, h('b', {}, value));
      hud.replaceChildren(
        cell('hud-state', tr('Đang làm'), doing),
        cell('hud-dist', tr('Tới điểm'), n.mission_dist ? `${num(n.mission_dist)} m` : '–'),
        cell('hud-wait', tr('Còn chờ'), n.mission_wait ? `${n.mission_wait} s` : '–'),
        cell('hud-alt', tr('Cao'), e.z !== undefined ? `${num(e.z, 1)} m` : '–'),
        cell('hud-batt', tr('Pin'), b.voltage ? `${num(b.voltage, 1)} V` : '–'),
        cell('hud-home', tr('Cách nhà'), n.home_dist !== undefined ? `${num(n.home_dist)} m` : '–'),
        cell('hud-total', tr('Cả chuyến'), tr('{d} m · ~{t} · {c}/{max} lệnh', { d: num(est.distance_m), t: fmtTime(est.time_s), c: cmds, max: MAX_COMMANDS })));
      hud.classList.toggle('over', cmds > MAX_COMMANDS);
      const g = store.streams.gps || {};
      linkPill.className = 'pill ' + (g.fix >= 3 ? 'ok' : 'warn');
      linkPill.textContent = g.fix >= 3 ? `GPS ${g.sats || 0}` : tr('GPS chưa fix');
    }

    // ---- the craft, live
    function paintDrone() {
      const g = gpsFix(); if (!g) return;
      const yaw = (store.streams.attitude || {}).yaw || 0;
      if (!drone) drone = L.marker([g.lat, g.lon], { icon: droneIcon(yaw), interactive: false, zIndexOffset: 1000 }).addTo(live);
      else { drone.setLatLng([g.lat, g.lon]); drone.setIcon(droneIcon(yaw)); }
      const armed = ((store.streams.status || {}).armed) || (store.streams.nav || {}).mission_state && store.streams.nav.mission_state !== 'IDLE';
      const last = view.trail[view.trail.length - 1];
      if (armed && (!last || distance({ lat: last[0], lon: last[1] }, g) > 1)) { view.trail.push([g.lat, g.lon]); if (view.trail.length > 3000) view.trail.shift(); trail.setLatLngs(view.trail); }
    }
    listen('stream:gps', () => { paintDrone(); paintHud(); });
    listen('stream:nav', paintHud);
    listen('stream:attitude', paintDrone);

    // ---- the drone: download, upload, start, stop
    async function download() {
      try {
        await reloadSection('mission');
        const got = decodePlan(store.settings.mission, store.settings.servo);
        Object.assign(plan, got); view.selected = -1;
        paintOpts(); redraw();
        if (plan.points.length) map.fitBounds(plan.points.map((p) => [p.lat, p.lon]), { padding: [60, 60], maxZoom: 18 });
        toast(tr('Đã tải kế hoạch từ drone.'), 'ok');
      } catch (e) { showError(e); }
    }
    async function upload() {
      let s;
      try { s = encodePlan(plan, store.settings.servo); } catch (e) { toast(tr('Kế hoạch quá dài: tối đa 32 lệnh (mỗi điểm hạ cánh 3 lệnh, mỗi bước servo 2 lệnh).'), 'bad', 8000); return; }
      if (!plan.points.length) { toast(tr('Chưa có điểm nào: bấm lên bản đồ để thêm.'), 'bad'); return; }
      try {
        const r = await api.post('/api/set', { section: 'mission', changes: s });
        store.settings.mission = r.values;
        await api.post('/api/save');
        toast(tr('Đã nạp kế hoạch vào drone và lưu.'), 'ok');
      } catch (e) { showError(e); }
    }
    async function start(on) {
      try { await command('MISSION', { action: on ? 1 : 0 }); toast(on ? tr('Đã gửi lệnh bắt đầu.') : tr('Đã dừng nhiệm vụ.'), 'ok'); } catch (e) {
        if (on) toast(tr('Không bắt đầu được: drone phải được arm bằng công tắc trước, có GPS fix và đã nạp kế hoạch.'), 'bad', 8000); else showError(e);
      }
    }

    paintOpts(); redraw(); paintDrone(); paintMe();
    setTimeout(() => map.invalidateSize(), 0);         // (the tab's slide-in animation sized the box late)
  },
};
