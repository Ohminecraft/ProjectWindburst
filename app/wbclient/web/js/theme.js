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
