// The DIY transmitter, system: the firmware, its boot report, back to defaults, restart.
import { store, loadSettings } from '../store.js';
import { api } from '../api.js';
import { h, block, kv, toast, showError } from '../ui.js';
import { tr } from '../i18n.js';

export default {
  id: 'tx-system',
  title: tr('Hệ thống'),
  lead: tr('Phiên bản firmware của tay phát, báo cáo khởi động, về mặc định.'),
  streams: { tx: 5 },
  mount(view) {
    const info = store.conn.info || {};
    const report = h('pre', { class: 'log', id: 'tx-report' }, (store.conn.report || []).join('\n') || '–');
    const defaults = h('button', { type: 'button', class: 'btn danger', id: 'tx-defaults' }, tr('Về mặc định'));
    defaults.addEventListener('click', async () => {
      if (!window.confirm(tr('Đưa mọi cài đặt của tay phát về mặc định (kể cả hiệu chỉnh cần và địa chỉ bind)? Thay đổi nằm trong RAM cho tới khi bạn bấm "Lưu".'))) return;
      try { await api.post('/api/defaults', { section: null }); await loadSettings(); toast(tr('Đã về mặc định (chưa lưu).'), 'ok'); } catch (e) { showError(e); }
    });
    const reboot = h('button', { type: 'button', class: 'btn', id: 'tx-reboot' }, tr('Khởi động lại tay phát'));
    reboot.addEventListener('click', async () => { try { await api.post('/api/reboot'); toast(tr('Tay phát đã khởi động lại.'), 'ok'); } catch (e) { showError(e); } });
    view.append(h('div', { class: 'cols' },
      h('div', {},
        block('Firmware', kv([[tr('Tên'), h('span', { id: 'tx-fw' }, `${info.name || '–'} ${info.version || ''}`)], [tr('Chip'), info.mcu || '–'], [tr('Cổng'), store.conn.port || '–']])),
        block(tr('Về mặc định'), h('div', { class: 'btn-row' }, defaults, reboot))),
      h('div', {}, block(tr('Báo cáo khởi động'), report))));
  },
};
