// Pins and ports: which GPIO each module uses. Stored in the "pins" settings section (flash); the firmware loads it into
// HardwareConfig at boot, so every change here needs "Lưu và khởi động lại".
import { store, fieldDef, setFields, setField } from '../store.js';
import { h, block, fields, control, listen, note, toast } from '../ui.js';
import { tr } from '../i18n.js';
import { pinName, pinParse } from '../pinname.js';

// An STM32 board (device info feature STM32): pins are written PA0 .. PK15 (pinname.js).
const isStm32 = () => ((store.conn.info || {}).features || []).includes('STM32');
// STM32 pins that are usually not free (only a warning).
const STM32_NOTES = { 11: 'USB D-', 12: 'USB D+', 13: tr('cổng debug (SWDIO)'), 14: tr('cổng debug (SWCLK)') };

const GROUPS = [
  [tr('IMU qua SPI'), tr('Điền chân CS thì firmware dùng SPI; để trống cả bốn thì IMU chạy I2C trên bus cảm biến bên dưới.'),
    [['imu_cs', 'CS'], ['imu_miso', 'MISO'], ['imu_mosi', 'MOSI'], ['imu_sck', 'SCK'], ['imu_int', tr('INT (chưa dùng)')]]],
  [tr('IMU thứ hai'), tr('Chip IMU thứ hai trên cùng bus SPI với IMU đầu (chung SCK, MISO, MOSI), chỉ cần chân CS riêng. Chọn cách dùng ở tab Cảm biến (Dùng gyro nào). Để trống nếu mạch chỉ có một IMU.'), [['imu2_cs', 'CS']]],
  [tr('Bus I2C cảm biến'), tr('Barometer, lidar, la bàn (và IMU nếu không có chân CS).'), [['i2c_sda', 'SDA'], ['i2c_scl', 'SCL']]],
  ['GPS (UART1)', tr('RX của ESP32 nối với TX của module GPS.'), [['gps_rx', 'RX'], ['gps_tx', 'TX']]],
  [tr('Receiver nối tiếp (UART2)'), tr('CRSF / SBUS / IBUS / Spektrum. TX chỉ cần khi dùng telemetry.'), [['rx_rx', 'RX'], ['rx_tx', 'TX']]],
  [tr('Receiver PPM'), tr('Một dây mang mọi kênh. Chỉ dùng khi giao thức receiver (tab Receiver) là PPM; khi đó UART2 để trống cho việc khác.'), [['ppm', 'PPM']]],
  [tr('Pin và còi'), tr('ADC đo pin: chân nối với cầu chia áp, trên ESP32-S3 là GPIO1 đến GPIO20. ADC đo dòng: chân ra của cảm biến dòng (cũng GPIO1 đến GPIO20); để trống nếu không có cảm biến dòng. Còi: chân bật / tắt còi, hoặc phát xung cho còi thụ động; để trống nếu máy không gắn còi (motor kêu thay, xem tab Pin và còi).'), [['batt_adc', tr('ADC đo pin')], ['curr_adc', tr('ADC đo dòng')], ['buzzer', tr('Còi')]]],
  [tr('Thẻ SD (log chuyến bay)'), tr('Kiểu SPI: đủ 4 chân. Kiểu SDMMC 1 bit: bỏ trống CS, ba chân còn lại là CLK, CMD, D0. Chọn kiểu nối ở tab Blackbox.'),
    [['sd_cs', 'CS'], ['sd_clk', 'SCK / CLK'], ['sd_cmd', 'MOSI / CMD'], ['sd_d0', 'MISO / D0']]],
  ['Optical flow', tr('PMW3901: bốn chân SPI của riêng nó (không chung bus với IMU, và không dùng được cùng lúc với thẻ SD kiểu SPI). Cảm biến MSP qua UART: chỉ cần RX, nối với TX của cảm biến; firmware tự lấy cổng UART còn trống (UART1 nếu không có GPS, UART2 nếu không có receiver nối tiếp, UART0 khi app nối qua USB của chip - RX mặc định của UART0 là GPIO44). Chọn loại cảm biến ở tab Vị trí.'),
    [['flow_cs', 'CS'], ['flow_sck', 'SCK'], ['flow_miso', 'MISO'], ['flow_mosi', 'MOSI'], ['flow_rx', 'RX (UART)'], ['flow_tx', 'TX (UART)']]],
  [tr('CAN bus'), tr('Nối với bộ thu phát CAN 3,3 V: TX vào chân D (TXD), RX từ chân R (RXD). Bật ở tab Cảm biến (CAN bus).'), [['can_tx', 'TX'], ['can_rx', 'RX']]],
  [tr('Telemetry ESC'), tr('Dây telemetry chung của các ESC (tab Motor: Dây telemetry ESC). Nối vào RX của một UART; firmware lấy UART còn trống (hoặc UART có chức năng Telemetry ESC trong bảng cổng).'), [['esc_tlm', 'RX']]],
  [tr('Khác'), tr('Chân dữ liệu (DIN) của dải LED WS2812 / SK6812 (tab Pin và còi: Đèn LED).'), [['led', 'LED']]],
];
// ESP32-S3 pins that are usually not free. Only a warning: boards differ.
const S3_NOTES = {
  0: tr('chân strapping (BOOT)'), 3: tr('chân strapping'), 45: tr('chân strapping'), 46: tr('chân strapping'),
  19: 'USB D-', 20: 'USB D+', 43: tr('UART0 TX (cổng nối app nếu dùng cầu USB-UART)'), 44: tr('UART0 RX (cổng nối app nếu dùng cầu USB-UART)'),
  26: 'flash SPI', 27: 'flash SPI', 28: 'flash SPI', 29: 'flash SPI', 30: 'flash SPI', 31: 'flash SPI', 32: 'flash SPI',
  33: tr('flash trên module có flash octal (N32R8V...)'), 34: tr('flash trên module có flash octal (N32R8V...)'),
  35: tr('PSRAM của module R8 (ví dụ N16R8): không dùng được'), 36: tr('PSRAM của module R8 (ví dụ N16R8): không dùng được'), 37: tr('PSRAM của module R8 (ví dụ N16R8): không dùng được'),
};
// A pin map that works on an ESP32-S3-DevKitC-1 with an N16R8 (or N8) module: no strapping pin, no USB / UART0 pin, nothing of the
// flash or the PSRAM, the battery on an ADC1 pin. Only a starting point - the wiring of YOUR board decides.
export const S3_DEVKIT_PRESET = {
  imu_cs: 10, imu_mosi: 11, imu_sck: 12, imu_miso: 13, i2c_sda: 8, i2c_scl: 9,
  gps_rx: 18, gps_tx: 17, rx_rx: 16, rx_tx: 15, motor: [4, 5, 6, 7, 255, 255, 255, 255], batt_adc: 1,
  sd_cs: 42, sd_clk: 39, sd_cmd: 40, sd_d0: 41,
};

const FLOW_SPI = ['flow_cs', 'flow_sck', 'flow_miso', 'flow_mosi'], FLOW_UART = ['flow_rx', 'flow_tx'];

function pinBox(name, label, index) {
  if (isStm32()) return h('label', { class: 'pin' }, h('span', {}, label), stm32Box(name, label, index));
  const input = control('pins', name, { index, empty: 255, width: '92px' });
  input.min = 0; input.max = 48;
  return h('label', { class: 'pin' }, h('span', {}, label), input);
}

// A text box that shows and takes a pin name (PB3); the setting keeps the number.
function stm32Box(name, label, index) {
  const el = h('input', { type: 'text', class: 'num pin-name', placeholder: tr('trống'), spellcheck: 'false', autocomplete: 'off' });
  el.style.width = '92px';
  el.dataset.field = `pins.${name}` + (index === undefined ? '' : `[${index}]`);
  const get = () => { const v = (store.settings.pins || {})[name]; return index === undefined ? v : (v ? v[index] : undefined); };
  const sync = (force) => { if (!force && document.activeElement === el) return; el.value = pinName(get(), true); };
  el.addEventListener('change', () => {
    const v = pinParse(el.value, true);
    if (v === null) { toast(tr('{label}: "{text}" không phải tên chân STM32 (PA0 đến PK15).', { label, text: el.value.trim() }), 'warn'); sync(true); return; }
    if (v !== get()) setField('pins', name, v, index);
    sync(true);
  });
  const lock = () => { el.disabled = store.armed; };
  listen('settings:pins', () => sync()); listen('armed', lock);
  sync(); lock();
  return el;
}

function problems() {
  const p = store.settings.pins;
  if (!p) return [];
  const used = new Map(), out = [];
  // the flow sensor's pins count only for the kind of sensor that is selected (as in the firmware's own check); an MSP sensor
  // on UART0's pins is the normal wiring of a board that talks to the app over USB
  const flowType = (store.settings.flow || {}).type;
  const unused = (name) => (FLOW_SPI.includes(name) && flowType !== 'PMW3901') || (FLOW_UART.includes(name) && flowType !== 'MSP');
  const stm32 = isStm32();
  const visit = (label, v, name) => {
    if (v === 255 || v === undefined || v === null || unused(name)) return;
    if (stm32) {                                          // (which pin can be which UART / SPI / timer: the firmware checks it at boot)
      if (STM32_NOTES[v]) out.push(tr('{label}: {pin} thường là {what}.', { label, pin: pinName(v, true), what: STM32_NOTES[v] }));
      if (used.has(v)) out.push(tr('{pin} đang gán cho cả "{a}" và "{b}".', { pin: pinName(v, true), a: used.get(v), b: label }));
      else used.set(v, label);
      return;
    }
    if (v > 48) out.push(tr('{label}: GPIO{pin} không tồn tại trên ESP32-S3 (0 đến 48).', { label, pin: v }));
    if (S3_NOTES[v] && !((v === 43 || v === 44) && FLOW_UART.includes(name))) out.push(tr('{label}: GPIO{pin} thường là {what}.', { label, pin: v, what: S3_NOTES[v] }));
    if (used.has(v)) out.push(tr('GPIO{pin} đang gán cho cả "{a}" và "{b}".', { pin: v, a: used.get(v), b: label }));
    else used.set(v, label);
  };
  for (const [title, , list] of GROUPS) for (const [n, l] of list) visit(FLOW_SPI.includes(n) || FLOW_UART.includes(n) ? `${title} ${l}` : l, p[n], n);
  p.motor.forEach((v, i) => visit(`Motor slot ${i + 1}`, v));
  (p.servo || []).forEach((v, i) => visit(`Servo ${i + 1}`, v));
  if ((store.settings.receiver || {}).protocol === 'PWM') (p.rx_pwm || []).forEach((v, i) => visit(`PWM CH${i + 1}`, v));
  if ((store.settings.receiver || {}).protocol === 'PWM' && p.rx_pwm && p.rx_pwm[0] === 255) out.push(tr('Receiver PWM: chưa có chân của kênh 1.'));
  if ((store.settings.receiver || {}).protocol === 'PPM' && p.ppm === 255) out.push(tr('Receiver PPM: chưa có chân PPM.'));
  if ((p.imu_cs !== 255) && [p.imu_miso, p.imu_mosi, p.imu_sck].some((v) => v === 255)) out.push(tr('IMU SPI: đã có CS nhưng còn thiếu MISO, MOSI hoặc SCK.'));
  if (p.imu2_cs !== undefined && p.imu2_cs !== 255 && p.imu_cs === 255) out.push(tr('IMU thứ hai dùng chung bus SPI của IMU đầu: IMU đầu cũng phải nối SPI (có chân CS).'));
  if (flowType === 'PMW3901' && FLOW_SPI.some((n) => p[n] === 255)) out.push(tr('Optical flow PMW3901: cần đủ bốn chân CS, SCK, MISO, MOSI.'));
  if (flowType === 'MSP' && p.flow_rx === 255) out.push(tr('Optical flow qua UART: chưa có chân RX.'));
  if (flowType === 'PMW3901' && (store.settings.blackbox || {}).device === 'SDCARD' && (store.settings.blackbox || {}).sd_mode === 'SPI') out.push(tr('PMW3901 và thẻ SD kiểu SPI cần cùng một bus SPI: đổi thẻ sang SDMMC (tab Blackbox) hoặc dùng cảm biến optical flow qua UART.'));
  if ((p.i2c_sda === 255) !== (p.i2c_scl === 255)) out.push(tr('Bus I2C: cần cả SDA và SCL.'));
  const ports = store.settings.ports;
  if (ports && ports.assign === 'TABLE') {
    const fn = ports.function || [];
    const twice = ['GPS', 'SERIAL_RX', 'FLOW_MSP'].filter((f) => fn.filter((x) => x === f).length > 1);
    for (const f of twice) out.push(tr('Cổng nối tiếp: "{f}" được chọn cho hơn một UART; firmware dùng UART đầu tiên.', { f }));
    if ((store.settings.sensors || {}).gps_enabled && !fn.includes('GPS')) out.push(tr('Cổng nối tiếp: bảng không có UART nào cho GPS.'));
    const proto = (store.settings.receiver || {}).protocol;
    if (proto && proto !== 'PPM' && proto !== 'PWM' && proto !== 'NRF24' && !fn.includes('SERIAL_RX')) out.push(tr('Cổng nối tiếp: bảng không có UART nào cho receiver nối tiếp.'));
  }
  if (!stm32 && p.batt_adc !== 255 && (p.batt_adc < 1 || p.batt_adc > 20)) out.push(tr('ADC đo pin: GPIO{pin} không đo được điện áp trên ESP32-S3 (chỉ GPIO1 đến GPIO20).', { pin: p.batt_adc }));
  if (!stm32 && p.curr_adc !== undefined && p.curr_adc !== 255 && (p.curr_adc < 1 || p.curr_adc > 20)) out.push(tr('ADC đo dòng: GPIO{pin} không đo được điện áp trên ESP32-S3 (chỉ GPIO1 đến GPIO20).', { pin: p.curr_adc }));
  const sd = [p.sd_clk, p.sd_cmd, p.sd_d0], sdSet = sd.filter((v) => v !== 255).length;
  if ((sdSet > 0 && sdSet < 3) || (p.sd_cs !== 255 && sdSet < 3)) out.push(tr('Thẻ SD: cần đủ SCK / CLK, MOSI / CMD và MISO / D0.'));
  if (sdSet === 3 && p.sd_cs === 255 && (store.settings.blackbox || {}).sd_mode === 'SPI') out.push(tr('Thẻ SD kiểu SPI cần thêm chân CS (hoặc đổi sang kiểu SDMMC ở tab Blackbox).'));
  return out;
}

export default {
  id: 'pins',
  title: tr('Chân và cổng'),
  lead: tr('Gán GPIO cho từng module. Ô để trống nghĩa là không dùng (hoặc dùng chân mặc định ghi sẵn trong hardware_config.h).'),
  streams: { status: 10 },
  mount(view) {
    const warn = h('ul', { class: 'warnings', id: 'pin-warnings' });
    const paint = () => {
      const list = problems();
      warn.replaceChildren(...(list.length ? list.map((t) => h('li', {}, t)) : [h('li', { class: 'ok' }, tr('Không thấy chân trùng hay chân đặc biệt.'))]));
    };
    listen('settings:pins', paint); listen('settings:blackbox', paint); listen('settings:ports', paint); listen('settings:receiver', paint); paint();      // (the sensor type is chosen on another tab: coming back here paints anew)
    const motorCount = fieldDef('pins', 'motor').count;
    const preset = h('button', { type: 'button', class: 'btn', id: 'pins-preset' }, tr('Điền bộ chân gợi ý'));
    preset.addEventListener('click', () => {
      if (!window.confirm(tr('Điền bộ chân gợi ý cho mạch ESP32-S3 DevKitC (module N16R8 hoặc N8) vào các ô? Những chân bạn đã nhập sẽ bị thay; chân còi và các chân khác giữ nguyên. Chưa có gì được lưu cho tới khi bạn bấm "Lưu và khởi động lại".'))) return;
      setFields('pins', JSON.parse(JSON.stringify(S3_DEVKIT_PRESET)));
      toast(tr('Đã điền bộ chân gợi ý. Đối chiếu với dây nối thật của mạch rồi mới lưu.'), 'ok');
    });
    const lockPreset = () => { preset.disabled = store.armed; };
    listen('armed', lockPreset); lockPreset();
    view.append(
      note(tr('Chân được lưu trong flash của flight controller, không nằm trong mã nguồn. Sau khi sửa, bấm "Lưu và khởi động lại" ở góc trên: firmware chỉ đọc bảng chân lúc khởi động.')),
      h('div', { class: 'cols' },
        h('div', {},
          GROUPS.map(([title, help, list]) => block(title, h('p', { class: 'help' }, help), h('div', { class: 'pins' }, list.map(([n, l]) => pinBox(n, l))))),
          block('Motor / ESC', h('p', { class: 'help' }, tr('Theo thứ tự slot phần cứng. Motor nào dùng slot nào chỉnh ở tab Motor (thứ tự motor).')),
            h('div', { class: 'pins' }, Array.from({ length: motorCount }, (_, i) => pinBox('motor', `Slot ${i + 1}`, i)))),
          block('Servo', h('p', { class: 'help' }, tr('Chân xung của servo 1 đến 4; kênh RC chọn ở tab Motor.')),
            h('div', { class: 'pins' }, [0, 1, 2, 3].map((i) => pinBox('servo', `Servo ${i + 1}`, i)))),
          block(tr('Receiver PWM'), h('p', { class: 'help' }, tr('Mỗi kênh một dây, kênh 1 trước. Chỉ dùng khi giao thức receiver là PWM. Để trống kênh nào không nối.')),
            h('div', { class: 'pins' }, Array.from({ length: 8 }, (_, i) => pinBox('rx_pwm', `CH${i + 1}`, i))))),
        h('div', {},
          block(tr('Kiểm tra'), warn,
            h('p', { class: 'help' }, tr('Firmware cũng kiểm tra lúc khởi động: hai chức năng trùng một chân thì không cho arm, và ghi rõ trong báo cáo ở tab Tổng quan.'))),
          isStm32()
            ? block(tr('Mạch STM32'), h('p', { class: 'help', id: 'pins-stm32' }, tr('Chân ghi theo tên: PA0, PB3, PC13 ... Trên STM32 mỗi UART, SPI, I2C và timer chỉ ra được một số chân nhất định; firmware kiểm tra lúc khởi động và ghi lỗi vào báo cáo ở tab Tổng quan nếu chân không đúng chức năng.')))
            : block(tr('Bộ chân gợi ý cho ESP32-S3 DevKitC'),
              h('p', { class: 'help' }, tr('Một bộ chân không đụng chân strapping, USB, UART0, flash hay PSRAM của module N16R8: IMU 10-13, I2C 8/9, GPS 18/17, receiver 16/15, motor 4-7, ADC đo pin GPIO1, thẻ SD 39-42. Chỉ là điểm bắt đầu: dây nối trên mạch của bạn mới quyết định.')),
              h('div', { class: 'btn-row' }, preset)),
          block(tr('Cổng nối tiếp'),
            fields('ports', ['assign']),
            h('table', { class: 'grid', id: 'ports-table' },
              h('thead', {}, h('tr', {}, h('th', {}, tr('Cổng')), h('th', {}, tr('Chức năng')))),
              h('tbody', {}, [0, 1, 2].map((i) => h('tr', {}, h('th', { scope: 'row' }, 'UART' + i), h('td', {}, control('ports', 'function', { index: i })))))),
            h('p', { class: 'help' }, tr('ESP32-S3 có ba UART, mỗi cái nối được vào bất kỳ chân nào; chân của từng thiết bị đặt ở các nhóm bên trái. UART0 chỉ dùng được khi app nối qua cổng USB của chip (không qua mạch USB-UART). Tốc độ baud do từng thiết bị tự chọn (GPS tự dò, receiver theo giao thức, optical flow ở tab Vị trí).'))))));
  },
};
