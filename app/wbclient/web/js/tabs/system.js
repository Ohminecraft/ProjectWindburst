// System: failsafe limits, backup / restore, defaults, firmware log.
import { store, loadSettings } from '../store.js';
import { api } from '../api.js';
import { h, block, fields, listen, toast, showError, kv } from '../ui.js';
import { tr } from '../i18n.js';

export default {
  id: 'system',
  title: tr('Hệ thống'),
  lead: tr('Failsafe, sao lưu cấu hình và nhật ký của firmware.'),
  streams: { status: 10 },
  mount(view) {
    const info = store.conn.info || {};
    // ---- backup / restore
    const backup = h('a', { class: 'btn', href: api.backupUrl, download: 'windburst_settings.json', id: 'backup' }, tr('Tải bản sao lưu (.json)'));
    const file = h('input', { type: 'file', accept: '.json,application/json', id: 'restore-file', hidden: true });
    const restore = h('button', { type: 'button', class: 'btn', id: 'restore' }, tr('Nạp bản sao lưu…'));
    const result = h('div', { id: 'restore-result' });
    restore.addEventListener('click', () => file.click());
    file.addEventListener('change', async () => {
      const f = file.files[0];
      if (!f) return;
      try {
        const doc = JSON.parse(await f.text());
        const r = await api.post('/api/restore', doc);
        await loadSettings();
        result.replaceChildren(
          h('p', { class: 'note ok' }, tr('Đã nạp {n} mục vào RAM. Kiểm tra lại rồi bấm "Lưu" ở góc trên để ghi vào flash.', { n: r.sections.length })),
          r.changed.length ? h('p', { class: 'note' }, tr('Firmware đã giới hạn: ') + r.changed.map((c) => tr('{field} ({asked} thành {stored})', { field: c.field, asked: JSON.stringify(c.asked), stored: JSON.stringify(c.stored) })).join('; ')) : null,
          r.skipped.length ? h('p', { class: 'note' }, tr('Bỏ qua vì firmware này không có: ') + r.skipped.join(', ')) : null);
      } catch (e) { if (e instanceof SyntaxError) toast(tr('File không phải JSON hợp lệ.'), 'bad'); else showError(e); }
      file.value = '';
    });
    // ---- defaults
    const sectionSel = h('select', { id: 'defaults-section' }, h('option', { value: '' }, tr('Tất cả các mục')), store.schema.sections.map((s) => h('option', { value: s.name }, s.name)));
    const defaults = h('button', { type: 'button', class: 'btn danger', id: 'defaults' }, tr('Về mặc định'));
    defaults.addEventListener('click', async () => {
      const what = sectionSel.value || tr('TOÀN BỘ cấu hình (kể cả chân và hiệu chuẩn)');
      if (!window.confirm(tr('Đưa {what} về mặc định? Thay đổi nằm trong RAM cho tới khi bạn bấm "Lưu".', { what }))) return;
      try { await api.post('/api/defaults', { section: sectionSel.value || null }); await loadSettings(); toast(tr('Đã về mặc định (chưa lưu).'), 'ok'); } catch (e) { showError(e); }
    });
    const lockArmed = () => { defaults.disabled = restore.disabled = store.armed; };
    listen('armed', lockArmed); lockArmed();
    // ---- log
    const log = h('pre', { class: 'log', id: 'log', tabindex: '0' });
    const paintLog = () => { log.textContent = store.logs.map(([lvl, text]) => (lvl >= 3 ? '! ' : '  ') + text).join('\n') || tr('Chưa có dòng nào.'); log.scrollTop = log.scrollHeight; };
    listen('log', paintLog); paintLog();
    // ---- raw serial data: whatever the port sent that was NOT a protocol frame (ESP32 boot text, a crash dump, another program's output)
    const raw = h('pre', { class: 'log', id: 'raw', tabindex: '0' });
    const paintRaw = () => { raw.textContent = store.raw || tr('Chưa có byte nào ngoài giao thức.'); raw.scrollTop = raw.scrollHeight; };
    listen('raw', paintRaw); paintRaw();

    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Failsafe và an toàn'), fields('general', ['fs_guard_ms', 'fs_recover_ms', 'fs_max_descent_s', 'fs_fallback_throttle', 'runaway_takeoff']),
          h('p', { class: 'help' }, tr('Mất sóng quá thời gian chờ thì máy tự cân bằng và hạ xuống; chạm đất thì tắt motor. Sóng trở lại và giữ ổn định thì bạn lấy lại quyền điều khiển; công tắc Failsafe thì hết ngay khi tắt công tắc.')),
          h('p', { class: 'help' }, tr('Pin dưới mức tối thiểu cũng làm máy tự hạ xuống (tab Pin và còi), nhưng roll, pitch và yaw vẫn theo cần của bạn để chọn chỗ đáp; lần hạ cánh này không huỷ được. Đang đứng trên mặt đất mà mất sóng thì motor tắt luôn.')))),
      h('div', {},
        block('Firmware', kv([
          [tr('Tên'), info.name || '–'], [tr('Phiên bản'), info.version || '–'], [tr('Giao thức'), String(info.protocol || '–')],
          [tr('Mã schema'), info.schema_hash || '–'], [tr('Tính năng'), (info.features || []).join(', ') || '–'], [tr('Cổng'), store.conn.port || '–'], ['Baud', String(store.conn.baud || '–')],
        ])),
        block(tr('Sao lưu và khôi phục'), h('div', { class: 'btn-row' }, backup, restore, file), result,
          h('p', { class: 'help' }, tr('Bản sao lưu chứa mọi mục cấu hình, kể cả bảng chân và kết quả hiệu chuẩn.'))),
        block(tr('Về mặc định'), h('div', { class: 'btn-row' }, sectionSel, defaults)),
        block(tr('Nhật ký firmware'), log),
        block(tr('Dữ liệu thô từ cổng serial'), raw,
          h('p', { class: 'help' }, tr('Những byte không thuộc khung WBP: log khởi động của ESP32, thông báo lỗi khi firmware treo ("Guru Meditation", backtrace), hoặc dữ liệu của chương trình khác. Bình thường ô này trống.'))))));
  },
};
