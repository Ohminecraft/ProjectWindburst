// System: failsafe limits, backup / restore, defaults, firmware log.
import { store, loadSettings } from '../store.js';
import { api } from '../api.js';
import { h, block, fields, listen, toast, showError, kv, note } from '../ui.js';
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
    const sectionSel = h('select', { id: 'defaults-section' }, h('option', { value: '' }, tr('Tất cả các mục')), store.schema.sections.filter((s) => (s.role || 'rx') === 'rx').map((s) => h('option', { value: s.name }, s.name)));
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
    // ---- the link: always streamed at 1 Hz by the app (it tells a firmware that restarted from one that only paused)
    const linkBox = h('div', { id: 'link-diag' });
    const paintLink = () => {
      const l = store.streams.link;
      if (!l) { linkBox.replaceChildren(h('p', { class: 'help' }, tr('Chưa có số liệu.'))); return; }
      const warn = (v, lim) => (v > lim ? ' ⚠' : '');
      linkBox.replaceChildren(kv([
        [tr('Firmware chạy được'), tr('{s} giây', { s: Math.floor(l.uptime) })],
        [tr('Khung nhận từ app / hỏng'), `${l.rx_frames} / ${l.rx_bad}`],
        [tr('Lệnh không được xử lý kịp (BUSY)'), String(l.busy)],
        [tr('Lệnh chờ lâu nhất (1 s qua)'), `${l.cmd_max_ms} ms${warn(l.cmd_max_ms, 50)}`],
        [tr('Luồng link nghỉ lâu nhất'), `${l.link_gap_ms} ms${warn(l.link_gap_ms, 20)}`],
        [tr('Luồng cảm biến nghỉ lâu nhất'), `${l.slow_gap_ms} ms${warn(l.slow_gap_ms, 20)}`],
        [tr('Khung telemetry bỏ qua / trả lời bị mất'), `${l.dropped} / ${l.tx_lost}`],
        [tr('Cổng USB bị kẹt phải kích lại'), String(l.kicks)],
        [tr('Byte đang chờ gửi'), String(l.queued)],
      ]));
    };
    // ---- the board's name on USB (feature USB_NAME: builds whose USB stack is the firmware's own)
    const hasUsbName = (info.features || []).includes('USB_NAME') && !!store.schema.sectionByName.general.fields.find((x) => x.n === 'usb_name');
    const usbDefault = 'Windburst ' + ((info.features || []).includes('STM32') ? 'STM32' : 'ESP32-S3');
    const usbNow = () => { const l = (store.conn.report || []).find((x) => x.startsWith("usb: '")); return l ? l.slice(6, l.indexOf("'", 6)) : ''; };
    listen('stream:link', paintLink); paintLink();

    view.append(h('div', { class: 'cols' },
      h('div', {},
        block(tr('Failsafe và an toàn'), fields('general', ['fs_guard_ms', 'fs_recover_ms', 'fs_max_descent_s', 'fs_throttle', 'runaway_takeoff']),
          h('p', { class: 'help' }, tr('Mất sóng quá thời gian chờ thì máy tự cân bằng và hạ xuống; chạm đất thì tắt motor. Sóng trở lại và giữ ổn định thì bạn lấy lại quyền điều khiển; công tắc Failsafe thì hết ngay khi tắt công tắc.')),
          h('p', { class: 'help' }, tr('Pin dưới mức tối thiểu cũng làm máy tự hạ xuống (tab Pin và còi), nhưng roll, pitch và yaw vẫn theo cần của bạn để chọn chỗ đáp; lần hạ cánh này không huỷ được. Đang đứng trên mặt đất mà mất sóng thì motor tắt luôn.'))),
        hasUsbName ? block(tr('Tên USB'), fields('general', ['usb_name'], { usb_name: { placeholder: usbDefault, ascii: true } }),
          h('p', { class: 'help', id: 'usb-name-now' }, tr('Tên đang dùng: {name}', { name: usbNow() || '–' })),
          note(tr('Tên máy tính thấy khi cắm USB (Linux / macOS: tên cổng; Windows: thuộc tính thiết bị, "Bus reported device description"). Để trống = tên mặc định. Chỉ chữ, số và ký hiệu ASCII, tối đa 31 ký tự. Có hiệu lực sau khi lưu và khởi động lại.'))) : null),
      h('div', {},
        block(tr('Công cụ khác qua MSP'), fields('msp', ['identity', 'writes']),
          h('p', { class: 'help' }, tr('Cùng cổng với app này. Betaflight Configurator / SpeedyBee đọc được tư thế, receiver, GPS, pin, PID, rate, mode, map kênh, tên máy bay; ghi được các mục đó khi được phép.'))),
        block('Firmware', kv([
          [tr('Tên'), info.name || '–'], [tr('Phiên bản'), info.version || '–'], [tr('Giao thức'), String(info.protocol || '–')],
          [tr('Mã schema'), info.schema_hash || '–'], [tr('Tính năng'), (info.features || []).join(', ') || '–'], [tr('Cổng'), store.conn.port || '–'], ['Baud', String(store.conn.baud || '–')],
        ])),
        block(tr('Sao lưu và khôi phục'), h('div', { class: 'btn-row' }, backup, restore, file), result,
          h('p', { class: 'help' }, tr('Bản sao lưu chứa mọi mục cấu hình, kể cả bảng chân và kết quả hiệu chuẩn.'))),
        block(tr('Về mặc định'), h('div', { class: 'btn-row' }, sectionSel, defaults)),
        block(tr('Nhật ký firmware'), log),
        block(tr('Đường truyền với app'), linkBox,
          h('p', { class: 'help' }, tr('Khi app báo "firmware không trả lời": số giây chạy tăng đều nghĩa là firmware không khởi động lại, chỉ là dữ liệu bị nghẽn. Luồng cảm biến nghỉ lâu (hàng chục ms) nghĩa là vòng lặp chiếm hết CPU của core 1: hạ tần số loop. Gửi các số này kèm log khi báo lỗi.'))),
        block(tr('Dữ liệu thô từ cổng serial'), raw,
          h('p', { class: 'help' }, tr('Những byte không thuộc khung WBP: log khởi động của ESP32, thông báo lỗi khi firmware treo ("Guru Meditation", backtrace), hoặc dữ liệu của chương trình khác. Bình thường ô này trống.'))))));
  },
};
