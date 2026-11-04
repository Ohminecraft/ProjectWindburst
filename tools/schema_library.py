#!/usr/bin/env python3
"""The app's library of firmware schemas (app/wbclient/schemas): one file per released firmware version, so that a newer app can
still talk to an older firmware (it picks the schema whose hash the firmware reports in HELLO).

    python tools/schema_library.py               the present protocol/wbp_schema.json, as the firmware version in src/config/version.h
    python tools/schema_library.py --from-git    every version the git history holds (the last commit of each version wins)

tools/gen_proto.py runs the first form itself: while a version is being developed its entry is simply rewritten. The files carry
no descriptions (the app's texts come from labels.js; the hash does not cover them either)."""
import json, os, re, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = os.path.join(ROOT, 'app', 'wbclient', 'schemas')


def strip_desc(o):
    if isinstance(o, dict): return {k: strip_desc(v) for k, v in o.items() if k not in ('desc', 'compat')}
    if isinstance(o, list): return [strip_desc(v) for v in o]
    return o


def fw_version(header_text):
    v = [re.search(r'#define WB_FW_%s (\d+)' % k, header_text) for k in ('MAJOR', 'MINOR', 'PATCH')]
    return '.'.join(m.group(1) for m in v) if all(v) else None


def _key(v):
    return tuple(int(x) for x in v.split('.'))


def add(fw, schema):
    os.makedirs(LIB, exist_ok=True)
    idx_path = os.path.join(LIB, 'index.json')
    idx = json.load(open(idx_path, encoding='utf-8')) if os.path.exists(idx_path) else []
    name = fw + '.json'
    body = strip_desc(schema)      # (the version is in index.json: the file stays exactly what the hash covers)
    with open(os.path.join(LIB, name), 'w', encoding='utf-8', newline='\n') as f:
        json.dump(body, f, separators=(',', ':'), sort_keys=True); f.write('\n')
    idx = [e for e in idx if e['fw'] != fw and e['hash'] != schema['hash']]
    idx.append({'fw': fw, 'hash': schema['hash'], 'protocol': schema['version'], 'file': name})
    idx.sort(key=lambda e: _key(e['fw']))
    with open(idx_path, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(idx, f, indent=1); f.write('\n')
    # (a file whose version left the index - a hash moved to another version number - is removed)
    for fn in os.listdir(LIB):
        if fn.endswith('.json') and fn != 'index.json' and fn not in {e['file'] for e in idx}: os.remove(os.path.join(LIB, fn))


def from_git():
    git = lambda *a: subprocess.run(('git',) + a, cwd=ROOT, capture_output=True, text=True, encoding='utf-8').stdout
    seen = set()
    for c in git('log', '--format=%H', '--', 'protocol/wbp_schema.json').split():      # newest first: the last commit of a version wins
        v = fw_version(git('show', c + ':src/config/version.h'))
        if not v or v in seen: continue
        try: s = json.loads(git('show', c + ':protocol/wbp_schema.json'))
        except ValueError: continue
        if 'hash' not in s: continue
        seen.add(v); add(v, s); print(v, s['hash'])


if __name__ == '__main__':
    if '--from-git' in sys.argv: from_git()
    else:
        v = fw_version(open(os.path.join(ROOT, 'src', 'config', 'version.h'), encoding='utf-8').read())
        s = json.load(open(os.path.join(ROOT, 'protocol', 'wbp_schema.json'), encoding='utf-8'))
        add(v, s); print('schema library:', v, s['hash'])
