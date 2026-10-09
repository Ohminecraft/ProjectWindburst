// Calls to the local server (wbclient/gui.py). Every request carries the token that was embedded in this page.
import { tr } from './i18n.js';
const token = document.querySelector('meta[name="wb-token"]').content;

export class ApiError extends Error {
  constructor(code, message, extra) { super(message || code); this.code = code; this.extra = extra || {}; }
}

async function call(method, path, body) {
  let res;
  try {
    const headers = { 'X-WB-Token': token };
    if (body !== undefined) headers['Content-Type'] = 'application/json';
    res = await fetch(path, { method, headers, body: body === undefined ? undefined : JSON.stringify(body) });
  } catch (e) {
    throw new ApiError('SERVER_GONE', tr('Không gọi được chương trình wbclient. Cửa sổ lệnh chạy "python -m wbclient gui" còn mở không?'));
  }
  let doc = {};
  try { doc = await res.json(); } catch (e) { /* empty or non-JSON body */ }
  if (!res.ok) throw new ApiError(doc.error || ('HTTP_' + res.status), doc.message || res.statusText, doc);
  return doc;
}

export const api = {
  get: (path) => call('GET', path),
  post: (path, body = {}) => call('POST', path, body),
  backupUrl: '/api/backup?t=' + encodeURIComponent(token),
  // address of a file the browser downloads itself (a link cannot send the token header)
  url: (path, params = {}) => path + '?' + new URLSearchParams({ ...params, t: token }).toString(),
  query: (path, params = {}) => call('GET', path + '?' + new URLSearchParams(params).toString()),
  // Server-sent events: telemetry, firmware log lines, connection changes. The browser reconnects by itself.
  events(onMessage, onLink) {
    const es = new EventSource('/api/events?t=' + encodeURIComponent(token));
    es.onmessage = (ev) => { onLink(true); onMessage(JSON.parse(ev.data)); };
    es.onopen = () => onLink(true);
    es.onerror = () => onLink(false);
    return es;
  },
};
