// The OSD preview's font: what the goggles draw for the symbols of Betaflight's font (drivers/osd_symbols.h), redrawn as small
// vector glyphs of one character cell (12 x 18, the font's own cell) - white with a black outline, like the goggles show them.
// Not Betaflight's bitmaps: shapes of the same meaning and size, so a preview line is as wide as the line in the goggles.

const P = (d, extra = '') => `<path d="${d}" ${extra}/>`;
const arrow = (deg) => `<g transform="rotate(${deg} 6 9)">${P('M6 3 L10 9 H7.3 V15 H4.7 V9 H2 Z')}</g>`;

// name -> SVG body in a 12 x 18 box
export const GLYPHS = {
  batt: (lvl = 6) => P('M3.5 3.5 H8.5 V5 H10 V16 H2 V5 H3.5 Z', 'fill="none"') + (lvl > 0 ? `<rect x="3.3" y="${15 - lvl * 1.5}" width="5.4" height="${lvl * 1.5}" />` : ''),
  battCrit: () => P('M3.5 3.5 H8.5 V5 H10 V16 H2 V5 H3.5 Z', 'fill="none"') + P('M6 7 V12 M6 13.6 V14.4', 'fill="none" stroke-linecap="round"'),
  volt: () => P('M2 6 L6 15 L10 6', 'fill="none"'),
  amp: () => P('M2 15 L6 5 L10 15 M3.6 11 H8.4', 'fill="none"'),
  mah: () => P('M2 15 V8 L4 11 L6 8 V15 M7.5 15 V8 M7.5 11 H10 M10 8 V15', 'fill="none"'),
  lq: () => '<rect x="1.5" y="12" width="2.2" height="4"/><rect x="4.9" y="9" width="2.2" height="7"/><rect x="8.3" y="6" width="2.2" height="10"/>',
  rssi: () => P('M6 15 V8 M2.5 6 A5 5 0 0 1 9.5 6 M4 8.4 A2.6 2.6 0 0 1 8 8.4', 'fill="none"'),
  satL: () => '<rect x="5" y="6" width="5" height="6" transform="rotate(-30 7.5 9)"/>' + P('M1 5 L4 7 M1 13 L4 11', 'fill="none"'),
  satR: () => P('M2 9 Q6 3 10 9 M3.5 11 Q6 7.5 8.5 11', 'fill="none"'),
  home: () => P('M2 9 L6 4.5 L10 9 V15 H2 Z', 'fill="none"') + '<rect x="5" y="11" width="2" height="4"/>',
  overHome: () => P('M2 9 L6 4.5 L10 9 V15 H2 Z', 'fill="none"') + '<circle cx="6" cy="11" r="1.6"/>',
  alt: () => P('M6 3 V15 M3 6 L6 3 L9 6 M3 15 H9', 'fill="none"'),
  speed: () => P('M2 13 A5.5 5.5 0 1 1 10 13 M6 11 L8.6 7', 'fill="none"'),
  thr: () => P('M3 4 H9 M6 4 V15', 'fill="none"'),
  m: () => P('M2 15 V8 L4 11 L6 8 V15 M7.5 15 V8 L9.5 11 V15', 'fill="none"'),
  kph: () => P('M1.5 15 V5 M1.5 11 L5 8 M2.8 10 L5.5 15 M7 15 V8 M7 11 H10.5 M10.5 8 V15', 'fill="none"'),
  mps: () => P('M2 15 V8 L4 11 L6 8 V15 M7 15 L10.5 5', 'fill="none"'),
  c: () => '<circle cx="3" cy="5" r="1.4" fill="none"/>' + P('M10 7 A3 3.4 0 1 0 10 13', 'fill="none"'),
  flyM: () => P('M6 2.5 L7.4 7 L11 9 L7.4 10 L7 14 L8.6 15.5 H3.4 L5 14 L4.6 10 L1 9 L4.6 7 Z'),
  onM: () => P('M6 3 V8 M3.4 5.5 A4.6 4.6 0 1 0 8.6 5.5', 'fill="none"'),
  roll: () => P('M2 13 Q6 5 10 13 M8 10 L10 13 L7 13.6', 'fill="none"'),
  pitch: () => P('M6 3 V15 M3.5 5.5 L6 3 L8.5 5.5 M3.5 12.5 L6 15 L8.5 12.5', 'fill="none"'),
  up: () => arrow(0), down: () => arrow(180),
  arrow: (deg) => arrow(deg),
  // the artificial horizon: one of 9 levels inside the cell
  ah: (lvl) => `<rect x="0" y="${16 - lvl * 1.6}" width="12" height="1.6" />`,
  ahLine: () => '<rect x="0" y="8.2" width="12" height="1.6"/>',
  ahCenter: () => P('M0 9 H3.5 M8.5 9 H12', 'fill="none"') + '<circle cx="6" cy="9" r="2" fill="none"/>',
  ahDeco: () => '<rect x="5.2" y="8" width="1.6" height="2"/>',
  ahLeft: () => P('M10 9 L4 5.5 V12.5 Z'), ahRight: () => P('M2 9 L8 5.5 V12.5 Z'),
  hdgLine: () => '<rect x="0" y="12" width="12" height="1.4"/>',
  hdgTick: () => '<rect x="0" y="12" width="12" height="1.4"/><rect x="5.3" y="7" width="1.4" height="5"/>',
  dist: () => P('M2 14 H10 M2 11 V17 M10 11 V17 M4 6 H8 M6 3 V9', 'fill="none"'),
  lat: () => P('M1.5 9 H10.5 M6 3 A6 6 0 0 1 6 15 A6 6 0 0 1 6 3', 'fill="none"'),
  lon: () => P('M6 3 V15 M6 3 A3.4 6 0 0 0 6 15 A3.4 6 0 0 0 6 3', 'fill="none"'),
  pbStart: () => P('M10 4 H3 V14 H10', 'fill="none"'), pbClose: () => P('M2 4 H9 V14 H2', 'fill="none"'),
  pbFull: () => P('M0 4 H12 M0 14 H12', 'fill="none"') + '<rect x="0.8" y="5.6" width="10.4" height="6.8"/>',
  pbHalf: () => P('M0 4 H12 M0 14 H12', 'fill="none"') + '<rect x="0.8" y="5.6" width="5" height="6.8"/>',
  pbEmpty: () => P('M0 4 H12 M0 14 H12', 'fill="none"'),
};

// One cell of the preview: a character, or a glyph from the table above ([name, argument])
export function cellSvg(g) {
  const [name, arg] = Array.isArray(g) ? g : [g];
  const body = (GLYPHS[name] || (() => ''))(arg);
  return `<svg viewBox="0 0 12 18" aria-hidden="true"><g class="glyph">${body}</g></svg>`;
}
