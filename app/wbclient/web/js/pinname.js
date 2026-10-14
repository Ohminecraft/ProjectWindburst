// Pin numbers of the "pins" settings as people write them. The firmware's rule (src/port/pin_names.h):
//   ESP32: the GPIO number;  STM32 (feature STM32 in the device info): port * 16 + pin, PA0 = 0, PB3 = 19 ... PK15 = 175;  255: none.
// No DOM here: test_gui_pins.py runs it in node against the firmware's own examples.
export const NO_PIN = 255;
const PORTS = 11;                          // A .. K

export function pinName(v, stm32) {
  if (v === NO_PIN || v === null || v === undefined) return '';
  if (!stm32) return String(v);
  if (v >= PORTS * 16) return '?' + v;
  return 'P' + String.fromCharCode(65 + Math.floor(v / 16)) + (v % 16);
}

// "PB3" / "pb03" / "B3" -> 19 on an STM32; "12" / "GPIO12" -> 12 on an ESP32; '' -> 255; not a pin -> null.
export function pinParse(text, stm32) {
  const t = String(text).trim();
  if (t === '') return NO_PIN;
  if (stm32) {
    const m = /^[Pp]?([A-Ka-k])0*(\d{1,2})$/.exec(t);
    if (!m) return null;
    const n = Number(m[2]);
    return n > 15 ? null : (m[1].toUpperCase().charCodeAt(0) - 65) * 16 + n;
  }
  const m = /^(?:[Gg][Pp][Ii][Oo])?(\d{1,3})$/.exec(t);
  if (!m) return null;
  const n = Number(m[1]);
  return n > 254 ? null : n;
}
