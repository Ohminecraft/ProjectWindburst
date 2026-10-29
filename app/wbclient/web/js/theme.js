// Page themes: a data-theme attribute on <html> that app.css keys its tokens on. No attribute = Carbon, light or dark as the
// system says. Remembered in this browser only (index.html sets it before the first paint, so a page never flashes the default).
import { tr } from './i18n.js';

export const THEMES = {
  '': tr('Carbon (theo hệ thống)'),
  'carbon-light': tr('Carbon sáng'),
  'carbon-dark': tr('Carbon tối'),
  hud: tr('HUD buồng lái'),
  glass: tr('Kính mờ'),
  swiss: tr('Thụy Sĩ tối giản'),
  field: tr('Ngoài trời'),
};

export function getTheme() {
  try { const t = globalThis.localStorage.getItem('wb-theme') || ''; return t in THEMES ? t : ''; } catch (e) { return ''; }
}

export function setTheme(t) {
  if (!(t in THEMES)) t = '';
  try { if (t) globalThis.localStorage.setItem('wb-theme', t); else globalThis.localStorage.removeItem('wb-theme'); } catch (e) { /* this load only */ }
  if (t) document.documentElement.dataset.theme = t; else delete document.documentElement.dataset.theme;
}

// Page zoom (the desktop window has no browser menu for it): CSS zoom on <html>, remembered like the theme. Also Ctrl + / - / 0
// and Ctrl + mouse wheel (main.js). index.html applies it before the first paint.
export const ZOOMS = [50, 60, 67, 75, 80, 90, 100, 110, 125, 150];
export function getZoom() {
  try { const z = Number(globalThis.localStorage.getItem('wb-zoom')); return ZOOMS.includes(z) ? z : 100; } catch (e) { return 100; }
}
export function setZoom(z) {
  if (!ZOOMS.includes(z)) z = 100;
  try { if (z !== 100) globalThis.localStorage.setItem('wb-zoom', String(z)); else globalThis.localStorage.removeItem('wb-zoom'); } catch (e) { /* this load only */ }
  // (100vh stays the window's height under CSS zoom: the page's full-height frame divides by --zoom - app.css)
  const st = document.documentElement.style;
  if (z !== 100) { st.zoom = String(z / 100); st.setProperty('--zoom', String(z / 100)); } else { st.removeProperty('zoom'); st.removeProperty('--zoom'); }
  globalThis.dispatchEvent(new Event('resize'));          // (canvases redraw at the new size)
  return z;
}
// the next step up (+1) or down (-1) from z
export function stepZoom(z, dir) {
  const i = ZOOMS.indexOf(z);
  if (i < 0) return 100;
  return ZOOMS[Math.max(0, Math.min(ZOOMS.length - 1, i + dir))];
}
