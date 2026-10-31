// Pins and ports: which GPIO each module uses. Stored in the "pins" settings section (flash); the firmware loads it into
// HardwareConfig at boot, so every change here needs "Lưu và khởi động lại".
import { store, fieldDef, setFields, setField } from '../store.js';
import { h, block, fields, control, listen, note, toast } from '../ui.js';
import { tr } from '../i18n.js';
import { pinName } from '../pinname.js';

// An STM32 board (device info feature STM32): pins are written PA0 .. PK15 (pinname.js).
const isStm32 = () => ((store.conn.info || {}).features || []).includes('STM32');
// The MCU by name and the pin numbers it has (device info; firmware before 0.25 does not say: null = any pin).
export const mcuName = () => (store.conn.info || {}).mcu || (isStm32() ? 'STM32' : 'ESP32-S3');
const chipPins = () => { const p = (store.conn.info || {}).pins; return Array.isArray(p) && p.length ? new Set(p) : null; };
const isPlainEsp32 = () => mcuName() === 'ESP32';
const isS3 = () => !isStm32() && !isPlainEsp32();
const PORT_LETTERS = 'ABCDEFGHIJK';
// STM32 pins that are usually not free (only a warning).
const STM32_NOTES = { 11: 'USB D-', 12: 'USB D+', 13: tr('cổng debug (SWDIO)'), 14: tr('cổng debug (SWCLK)') };

const GROUPS = [
  [tr('IMU qua SPI'), tr('Điền chân CS thì firmware dùng SPI; để trống cả bốn thì IMU chạy I2C trên bus cảm biến bên dưới.'),
    [['imu_cs', 'CS'], ['imu_miso', 'MISO'], ['imu_mosi', 'MOSI'], ['imu_sck', 'SCK'], ['imu_int', tr('INT (chưa dùng)')]]],
  [tr('IMU thứ hai'), tr('Chip IMU thứ hai trên cùng bus SPI với IMU đầu (chung SCK, MISO, MOSI), chỉ cần chân CS riêng. Chọn cách dùng ở tab Cảm biến (Dùng gyro nào). Để trống nếu mạch chỉ có một IMU.'), [['imu2_cs', 'CS']]],
  [tr('Khí áp kế và la bàn trên bus SPI của IMU'), tr('Khí áp kế (BMP280 / BMP388 / MS5611) và la bàn RM3100 nối chung SCK, MISO, MOSI với IMU, mỗi chip một chân CS riêng. Để trống nếu chúng nằm trên bus I2C.'), [['baro_cs', tr('CS khí áp kế')], ['mag_cs', tr('CS la bàn')]]],
  [tr('Bus I2C cảm biến'), tr('Barometer, lidar, la bàn (và IMU nếu không có chân CS).'), [['i2c_sda', 'SDA'], ['i2c_scl', 'SCL']]],
  ['GPS (UART1)', tr('RX của ESP32 nối với TX của module GPS.'), [['gps_rx', 'RX'], ['gps_tx', 'TX']]],
  [tr('Receiver nối tiếp (UART2)'), tr('CRSF / SBUS / IBUS / Spektrum / MAVLink. TX chỉ cần khi dùng telemetry.'), [['rx_rx', 'RX'], ['rx_tx', 'TX']]],
  [tr('Receiver PPM'), tr('Một dây mang mọi kênh. Chỉ dùng khi giao thức receiver (tab Receiver) là PPM; khi đó UART2 để trống cho việc khác.'), [['ppm', 'PPM']]],
  [tr('Receiver nRF24'), tr('Module nRF24L01+ của tay phát DIY, trên bus SPI thứ hai (chân riêng, không chung với IMU). Chỉ dùng khi giao thức receiver là NRF24.'),
    [['nrf_sck', 'SCK'], ['nrf_miso', 'MISO'], ['nrf_mosi', 'MOSI'], ['nrf_cs', 'CSN'], ['nrf_ce', 'CE']]],
  [tr('Pin và còi'), tr('ADC đo pin: chân nối với cầu chia áp, trên ESP32-S3 là GPIO1 đến GPIO20. ADC đo dòng: chân ra của cảm biến dòng (cũng GPIO1 đến GPIO20); để trống nếu không có cảm biến dòng. Còi: chân bật / tắt còi, hoặc phát xung cho còi thụ động; để trống nếu máy không gắn còi (motor kêu thay, xem tab Pin và còi).'), [['batt_adc', tr('ADC đo pin')], ['curr_adc', tr('ADC đo dòng')], ['buzzer', tr('Còi')]]],
  [tr('Thẻ SD (log chuyến bay)'), tr('Kiểu SPI: đủ 4 chân. Kiểu SDMMC 1 bit: bỏ trống CS, ba chân còn lại là CLK, CMD, D0. Chọn kiểu nối ở tab Blackbox.'),
    [['sd_cs', 'CS'], ['sd_clk', 'SCK / CLK'], ['sd_cmd', 'MOSI / CMD'], ['sd_d0', 'MISO / D0']]],
  [tr('Optical flow và cổng MSP'), tr('PMW3901: bốn chân SPI của riêng nó (không chung bus với IMU, và không dùng được cùng lúc với thẻ SD kiểu SPI). Cổng MSP (thiết bị MSP tự gửi dữ liệu: optical flow MSP, GPS / la bàn / khí áp kế MSP, receiver MSP): chỉ cần RX, nối với TX của thiết bị; mọi thiết bị MSP dùng chung một cổng; firmware tự lấy cổng UART còn trống (UART1 nếu GPS không dùng UART, UART2 nếu không có receiver nối tiếp, UART0 khi app nối qua USB của chip - RX mặc định của UART0 là GPIO44). Baud: ô "Baud của cảm biến UART" ở tab Vị trí.'),
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
// Plain ESP32 (WROOM-32) pins that are usually not free.
const ESP32_NOTES = {
  0: tr('chân strapping (BOOT)'), 2: tr('chân strapping'), 5: tr('chân strapping'), 12: tr('chân strapping (điện áp flash: phải ở mức thấp lúc khởi động)'), 15: tr('chân strapping'),
  1: tr('UART0 TX (cổng nối app qua cầu USB-UART)'), 3: tr('UART0 RX (cổng nối app qua cầu USB-UART)'),
  16: tr('PSRAM trên module WROVER'), 17: tr('PSRAM trên module WROVER'),
};
// A pin map that works on an ESP32-S3-DevKitC-1 with an N16R8 (or N8) module: no strapping pin, no USB / UART0 pin, nothing of the
// flash or the PSRAM, the battery on an ADC1 pin. Only a starting point - the wiring of YOUR board decides.
export const S3_DEVKIT_PRESET = {
  imu_cs: 10, imu_mosi: 11, imu_sck: 12, imu_miso: 13, i2c_sda: 8, i2c_scl: 9,
  gps_rx: 18, gps_tx: 17, rx_rx: 16, rx_tx: 15, motor: [4, 5, 6, 7, 255, 255, 255, 255], batt_adc: 1,
  sd_cs: 42, sd_clk: 39, sd_cmd: 40, sd_d0: 41,
};

const FLOW_SPI = ['flow_cs', 'flow_sck', 'flow_miso', 'flow_mosi'], FLOW_UART = ['flow_rx', 'flow_tx'];
// the firmware opens the MSP port when anything is set to use it (FlightApp::msp_port_wanted)
function mspPortWanted() {
  const st = store.settings, sn = st.sensors || {};
  return (st.flow || {}).type === 'MSP' || (sn.gps_enabled && (st.gps || {}).provider === 'MSP') || (st.receiver || {}).protocol === 'MSP' ||
    (sn.mag_enabled && sn.mag_model === 'MSP') || (sn.baro_enabled && sn.baro_model === 'MSP');
}

function pinBox(name, label, index, section = 'pins') {
  if (isStm32()) return h('div', { class: 'pin' }, h('span', {}, label), stm32Box(name, label, index, section));
  const input = control(section, name, { index, empty: 255, width: '92px' });
  const pins = chipPins();
  input.min = 0; input.max = pins ? Math.max(...pins) : 48;
  return h('label', { class: 'pin' }, h('span', {}, label), input);
}

// STM32: two steps - the port (PA, PB ...), then the pin of that port - offering only what the chip's package has (the
// device info's pin list). The setting keeps the number (port * 16 + pin). Choosing a port writes nothing until its pin is chosen.
function stm32Box(name, label, index, section = 'pins') {
  const field = `${section}.${name}` + (index === undefined ? '' : `[${index}]`);
  const portSel = h('select', { 'data-part': 'port', 'aria-label': tr('{label}: cổng', { label }) });
  const pinSel = h('select', { 'data-part': 'pin', 'aria-label': tr('{label}: chân', { label }) });
  const box = h('span', { class: 'pin-pick', 'data-field': field }, portSel, pinSel);
  const get = () => { const v = (store.settings[section] || {})[name]; return index === undefined ? v : (v ? v[index] : undefined); };
  const has = (n) => { const a = chipPins(); return !a || a.has(n); };
  const ports = () => { const out = []; for (let p = 0; p < PORT_LETTERS.length; p++) { for (let k = 0; k < 16; k++) if (has(p * 16 + k)) { out.push(p); break; } } return out; };
  let shownPort = null;                                    // a port chosen in the box whose pin is not chosen yet
  const fillPins = (port, cur) => {
    const opts = [h('option', { value: '' }, '…')];
    for (let k = 0; k < 16; k++) { const n = port * 16 + k; if (has(n) || n === cur) opts.push(h('option', { value: String(k) }, has(n) ? `P${PORT_LETTERS[port]}${k}` : tr('{pin} (không có trên chip)', { pin: `P${PORT_LETTERS[port]}${k}` }))); }
    pinSel.replaceChildren(...opts);
  };
  const sync = () => {
    const v = get(), none = v === 255 || v === undefined || v === null;
    const list = ports();
    const port = none ? shownPort : Math.floor(v / 16);
    if (port !== null && !list.includes(port)) list.push(port);
    portSel.replaceChildren(h('option', { value: '' }, tr('trống')), ...list.sort((a, b) => a - b).map((p) => h('option', { value: String(p) }, 'P' + PORT_LETTERS[p])));
    portSel.value = port === null ? '' : String(port);
    if (port === null) { pinSel.replaceChildren(h('option', { value: '' }, '–')); pinSel.disabled = true; }
    else { fillPins(port, none ? -1 : v); pinSel.value = none ? '' : String(v % 16); pinSel.disabled = store.armed; }
    box.dataset.pin = none ? '' : pinName(v, true);
  };
  portSel.addEventListener('change', () => {
    if (portSel.value === '') { shownPort = null; if (get() !== 255) setField(section, name, 255, index); sync(); return; }
    shownPort = Number(portSel.value);
    const v = get();
    if (v !== 255 && Math.floor(v / 16) === shownPort) { sync(); return; }
    if (v !== 255) setField(section, name, 255, index);      // another port: the old pin is gone until a pin of this one is chosen
    sync(); pinSel.focus();
  });
  pinSel.addEventListener('change', () => {
    if (pinSel.value === '' || portSel.value === '') return;
    const v = Number(portSel.value) * 16 + Number(pinSel.value);
    shownPort = null;
    if (v !== get()) setField(section, name, v, index);
    sync();
  });
  const lock = () => { portSel.disabled = store.armed; if (store.armed) pinSel.disabled = true; else if (portSel.value !== '') pinSel.disabled = false; };
  listen('settings:' + section, () => sync()); listen('armed', lock); listen('conn', () => sync());
  sync(); lock();
  return box;
}

function problems() {
  const p = store.settings.pins;
  if (!p) return [];
  const used = new Map(), out = [];
  // the flow sensor's pins count only for the kind of sensor that is selected (as in the firmware's own check); an MSP sensor
  // on UART0's pins is the normal wiring of a board that talks to the app over USB
  const flowType = (store.settings.flow || {}).type;
  const unused = (name) => (FLOW_SPI.includes(name) && flowType !== 'PMW3901') || (FLOW_UART.includes(name) && flowType !== 'MSP');
  const stm32 = isStm32(), chip = chipPins(), mcu = mcuName(), s3 = isS3(), esp32 = isPlainEsp32();
  const OUT_ONLY = ['motor', 'servo', 'buzzer', 'led', 'gps_tx', 'rx_tx', 'imu_cs', 'imu2_cs', 'baro_cs', 'mag_cs', 'imu_mosi', 'imu_sck', 'sd_cs', 'sd_clk', 'sd_cmd', 'i2c_sda', 'i2c_scl', 'flow_cs', 'flow_sck', 'flow_mosi', 'flow_tx', 'can_tx', 'tx_pin', 'nrf_sck', 'nrf_mosi', 'nrf_cs', 'nrf_ce'];
  const visit = (label, v, name) => {
    if (v === 255 || v === undefined || v === null || unused(name)) return;
    if (chip && !chip.has(v)) out.push(tr('{label}: {pin} không có trên {mcu}.', { label, pin: stm32 ? pinName(v, true) : 'GPIO' + v, mcu }));
    if (stm32) {                                          // (which pin can be which UART / SPI / timer: the firmware checks it at boot)
      if (STM32_NOTES[v]) out.push(tr('{label}: {pin} thường là {what}.', { label, pin: pinName(v, true), what: STM32_NOTES[v] }));
      if (used.has(v)) out.push(tr('{pin} đang gán cho cả "{a}" và "{b}".', { pin: pinName(v, true), a: used.get(v), b: label }));
      else used.set(v, label);
      return;
    }
    if (!chip && s3 && v > 48) out.push(tr('{label}: GPIO{pin} không tồn tại trên ESP32-S3 (0 đến 48).', { label, pin: v }));
    if (s3 && S3_NOTES[v] && !((v === 43 || v === 44) && FLOW_UART.includes(name))) out.push(tr('{label}: GPIO{pin} thường là {what}.', { label, pin: v, what: S3_NOTES[v] }));
    if (esp32 && ESP32_NOTES[v]) out.push(tr('{label}: GPIO{pin} thường là {what}.', { label, pin: v, what: ESP32_NOTES[v] }));
    if (esp32 && v >= 34 && v <= 39 && OUT_ONLY.includes(name || '')) out.push(tr('{label}: GPIO{pin} trên ESP32 chỉ là chân vào, không xuất tín hiệu được.', { label, pin: v }));
    if (used.has(v)) out.push(tr('GPIO{pin} đang gán cho cả "{a}" và "{b}".', { pin: v, a: used.get(v), b: label }));
    else used.set(v, label);
  };
  const nrf = (store.settings.receiver || {}).protocol === 'NRF24';
  for (const [title, , list] of GROUPS) for (const [n, l] of list) if (nrf || !n.startsWith('nrf_')) visit(FLOW_SPI.includes(n) || FLOW_UART.includes(n) ? `${title} ${l}` : l, p[n], n);
  p.motor.forEach((v, i) => visit(`Motor slot ${i + 1}`, v, 'motor'));
  (p.servo || []).forEach((v, i) => visit(`Servo ${i + 1}`, v, 'servo'));
  if ((store.settings.receiver || {}).protocol === 'PWM') (p.rx_pwm || []).forEach((v, i) => visit(`PWM CH${i + 1}`, v));
  const tlm = store.settings.telemetry, rxc = store.settings.receiver || {};
  const mavRx = rxc.protocol === 'MAVLINK' && !!rxc.telemetry;           // (MAVLink back on the receiver's UART: the telemetry pins are not used)
  const mavlink = tlm && tlm.protocol === 'MAVLINK' && !mavRx;
  if (mavlink) { visit(tr('Telemetry TX'), tlm.tx_pin, 'tx_pin'); visit(tr('Telemetry RX'), tlm.rx_pin); }
  const osdc = store.settings.osd, osdOn = osdc && osdc.type !== 'NONE';
  if (osdOn) { visit('OSD TX', osdc.tx_pin, 'tx_pin'); visit('OSD RX', osdc.rx_pin); }
  if (mavlink && tlm.tx_pin === 255) out.push(tr('Telemetry MAVLink: không có chân TX - không có gì được gửi đi.'));
  if (mavRx && p.rx_tx === 255) out.push(tr('Receiver MAVLink: chưa có chân TX của receiver - không gửi được telemetry về ground station.'));
  if ((store.settings.receiver || {}).protocol === 'PWM' && p.rx_pwm && p.rx_pwm[0] === 255) out.push(tr('Receiver PWM: chưa có chân của kênh 1.'));
  if ((store.settings.receiver || {}).protocol === 'PPM' && p.ppm === 255) out.push(tr('Receiver PPM: chưa có chân PPM.'));
  if ((p.imu_cs !== 255) && [p.imu_miso, p.imu_mosi, p.imu_sck].some((v) => v === 255)) out.push(tr('IMU SPI: đã có CS nhưng còn thiếu MISO, MOSI hoặc SCK.'));
  if (p.imu2_cs !== undefined && p.imu2_cs !== 255 && p.imu_cs === 255) out.push(tr('IMU thứ hai dùng chung bus SPI của IMU đầu: IMU đầu cũng phải nối SPI (có chân CS).'));
  if (((p.baro_cs !== undefined && p.baro_cs !== 255) || (p.mag_cs !== undefined && p.mag_cs !== 255)) && p.imu_cs === 255) out.push(tr('Khí áp kế / la bàn trên SPI dùng chung bus của IMU: IMU cũng phải nối SPI (có chân CS).'));
  if (flowType === 'PMW3901' && FLOW_SPI.some((n) => p[n] === 255)) out.push(tr('Optical flow PMW3901: cần đủ bốn chân CS, SCK, MISO, MOSI.'));
  if (mspPortWanted() && p.flow_rx === 255) out.push(tr('Cổng MSP: chưa có chân RX (optical flow, GPS, la bàn, khí áp kế hoặc receiver đang chọn MSP).'));
  if (flowType === 'PMW3901' && (store.settings.blackbox || {}).device === 'SDCARD' && (store.settings.blackbox || {}).sd_mode === 'SPI') out.push(tr('PMW3901 và thẻ SD kiểu SPI cần cùng một bus SPI: đổi thẻ sang SDMMC (tab Blackbox) hoặc dùng cảm biến optical flow qua UART.'));
  if ((p.i2c_sda === 255) !== (p.i2c_scl === 255)) out.push(tr('Bus I2C: cần cả SDA và SCL.'));
  const ports = store.settings.ports;
  if (ports && ports.assign === 'TABLE') {
    const fn = ports.function || [];
    const twice = ['GPS', 'SERIAL_RX', 'FLOW_MSP'].filter((f) => fn.filter((x) => x === f).length > 1);
    for (const f of twice) out.push(tr('Cổng nối tiếp: "{f}" được chọn cho hơn một UART; firmware dùng UART đầu tiên.', { f }));
    if (mavlink && tlm.tx_pin !== 255 && !fn.includes('TELEMETRY')) out.push(tr('Cổng nối tiếp: bảng không có UART nào cho telemetry.'));
    if (osdOn && !fn.includes('DISPLAYPORT')) out.push(tr('Cổng nối tiếp: bảng không có UART nào cho OSD (DisplayPort).'));
    if ((store.settings.sensors || {}).gps_enabled && !fn.includes('GPS')) out.push(tr('Cổng nối tiếp: bảng không có UART nào cho GPS.'));
    const proto = (store.settings.receiver || {}).protocol;
    if (proto && proto !== 'PPM' && proto !== 'PWM' && proto !== 'NRF24' && !fn.includes('SERIAL_RX')) out.push(tr('Cổng nối tiếp: bảng không có UART nào cho receiver nối tiếp.'));
  }
  if (s3 && p.batt_adc !== 255 && (p.batt_adc < 1 || p.batt_adc > 20)) out.push(tr('ADC đo pin: GPIO{pin} không đo được điện áp trên ESP32-S3 (chỉ GPIO1 đến GPIO20).', { pin: p.batt_adc }));
  if (s3 && p.curr_adc !== undefined && p.curr_adc !== 255 && (p.curr_adc < 1 || p.curr_adc > 20)) out.push(tr('ADC đo dòng: GPIO{pin} không đo được điện áp trên ESP32-S3 (chỉ GPIO1 đến GPIO20).', { pin: p.curr_adc }));
  if (esp32 && p.batt_adc !== 255 && (p.batt_adc < 32 || p.batt_adc > 39)) out.push(tr('ADC đo pin: GPIO{pin} không đo được điện áp trên ESP32 (chỉ GPIO32 đến GPIO39, ADC1).', { pin: p.batt_adc }));
  if (esp32 && p.curr_adc !== undefined && p.curr_adc !== 255 && (p.curr_adc < 32 || p.curr_adc > 39)) out.push(tr('ADC đo dòng: GPIO{pin} không đo được điện áp trên ESP32 (chỉ GPIO32 đến GPIO39, ADC1).', { pin: p.curr_adc }));
  const sd = [p.sd_clk, p.sd_cmd, p.sd_d0], sdSet = sd.filter((v) => v !== 255).length;
  if ((sdSet > 0 && sdSet < 3) || (p.sd_cs !== 255 && sdSet < 3)) out.push(tr('Thẻ SD: cần đủ SCK / CLK, MOSI / CMD và MISO / D0.'));
  if (sdSet === 3 && p.sd_cs === 255 && (store.settings.blackbox || {}).sd_mode === 'SPI') out.push(tr('Thẻ SD kiểu SPI cần thêm chân CS (hoặc đổi sang kiểu SDMMC ở tab Blackbox).'));
  return out;
}

// MAVLink telemetry to a ground station (docs/MAVLINK.md): a UART (its pins live in the telemetry section).
function telemetryBlock() {
  return block(tr('Telemetry (MAVLink)'),
    h('p', { class: 'help' }, tr('Gửi trạng thái máy bay cho Mission Planner / QGroundControl qua radio telemetry (433 / 915 MHz, ELRS backpack) nối vào một UART. Chỉ gửi, chưa nhận lệnh. UART: Tự động lấy UART còn trống sau GPS, receiver, optical flow và dây telemetry ESC; Theo bảng: UART có chức năng Telemetry.')),
    h('p', { class: 'help' }, tr('Receiver chọn MAVLink (ELRS) và bật "Telemetry về tay điều khiển": telemetry đi ra chính UART của receiver, ở đây chỉ còn dùng tốc độ gửi và System ID.')),
    fields('telemetry', ['protocol', 'baud', 'sysid', 'remote_arm', 'mp_home_item']),
    h('div', { class: 'pins', id: 'tlm-pins' }, pinBox('tx_pin', 'TX', undefined, 'telemetry'), pinBox('rx_pin', 'RX', undefined, 'telemetry')),
    fields('telemetry', ['rate_attitude', 'rate_position', 'rate_hud', 'rate_status', 'rate_rc']));
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
    listen('settings:pins', paint); listen('settings:blackbox', paint); listen('settings:ports', paint); listen('settings:receiver', paint); listen('settings:telemetry', paint); listen('settings:osd', paint); paint();      // (the sensor type is chosen on another tab: coming back here paints anew)
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
            ? block(tr('Mạch {mcu}', { mcu: mcuName() }), h('p', { class: 'help', id: 'pins-stm32' }, tr('Chọn chân theo hai bước: cổng (PA, PB ...) rồi chân của cổng đó. Chỉ có những chân mà vỏ chip {mcu} này có. Trên STM32 mỗi UART, SPI, I2C và timer chỉ ra được một số chân nhất định; firmware kiểm tra lúc khởi động và ghi lỗi vào báo cáo ở tab Tổng quan nếu chân không đúng chức năng.', { mcu: mcuName() })))
            : !isS3() ? block(tr('Mạch {mcu}', { mcu: mcuName() }), h('p', { class: 'help', id: 'pins-esp32' }, tr('ESP32: GPIO 0 đến 39, trừ 6 đến 11 (flash của module). GPIO34 đến 39 chỉ là chân vào (đo pin, RX). Đo điện áp: GPIO32 đến 39. App nối qua UART0 (GPIO1 / 3) và mạch USB-UART của board.')))
            : block(tr('Bộ chân gợi ý cho ESP32-S3 DevKitC'),
              h('p', { class: 'help' }, tr('Một bộ chân không đụng chân strapping, USB, UART0, flash hay PSRAM của module N16R8: IMU 10-13, I2C 8/9, GPS 18/17, receiver 16/15, motor 4-7, ADC đo pin GPIO1, thẻ SD 39-42. Chỉ là điểm bắt đầu: dây nối trên mạch của bạn mới quyết định.')),
              h('div', { class: 'btn-row' }, preset)),
          block(tr('Cổng nối tiếp'),
            fields('ports', ['assign']),
            h('table', { class: 'grid', id: 'ports-table' },
              h('thead', {}, h('tr', {}, h('th', {}, tr('Cổng')), h('th', {}, tr('Chức năng')))),
              h('tbody', {}, [0, 1, 2].map((i) => h('tr', {}, h('th', { scope: 'row' }, 'UART' + i), h('td', {}, control('ports', 'function', { index: i })))))),
            h('p', { class: 'help' }, tr('ESP32-S3 có ba UART, mỗi cái nối được vào bất kỳ chân nào; chân của từng thiết bị đặt ở các nhóm bên trái. UART0 chỉ dùng được khi app nối qua cổng USB của chip (không qua mạch USB-UART). Tốc độ baud do từng thiết bị tự chọn (GPS tự dò, receiver theo giao thức, optical flow ở tab Vị trí).'))),
          store.schema.sectionByName.telemetry ? telemetryBlock() : null)));
  },
};
