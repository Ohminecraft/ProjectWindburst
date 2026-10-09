// Two languages. The source text in the code is Vietnamese; lang/en.js maps each Vietnamese string to its English one.
//   tr('Đã lưu vào flash.')                       -> the string in the current language
//   tr('Đã có {n}/6 mặt.', { n: 3 })              -> with values filled in
// A string that has no English entry is shown in Vietnamese (app/tests/test_gui_i18n.py fails when one is missing).
// This module must stay free of DOM access at import time: the tests load it with node.
import { EN } from './lang/en.js';

export const LANGS = { vi: 'Tiếng Việt', en: 'English' };

function detect() {
  try {
    const saved = globalThis.localStorage && globalThis.localStorage.getItem('wb-lang');
    if (saved && LANGS[saved]) return saved;
  } catch (e) { /* storage blocked: fall through */ }
  const nav = (globalThis.navigator && globalThis.navigator.language) || 'vi';
  return nav.toLowerCase().startsWith('vi') ? 'vi' : 'en';
}

let lang = detect();
export const getLang = () => lang;

export function tr(text, values) {
  let out = text;
  if (lang === 'en' && Object.prototype.hasOwnProperty.call(EN, text)) out = EN[text];
  if (values) out = out.replace(/\{(\w+)\}/g, (m, k) => (values[k] === undefined ? m : String(values[k])));
  return out;
}

// Changing the language reloads the page: everything on it is rebuilt from the firmware's state anyway.
export function setLang(next) {
  if (!LANGS[next] || next === lang) return;
  try { globalThis.localStorage.setItem('wb-lang', next); } catch (e) { /* not stored: applies to this load only */ }
  lang = next;
  if (globalThis.location) globalThis.location.reload();
}
// For tests (node): switch without a page.
export function forceLang(next) { lang = next; }
