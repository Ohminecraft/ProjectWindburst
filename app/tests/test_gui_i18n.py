"""The configurator speaks Vietnamese and English. Source strings are Vietnamese (in the code); web/js/lang/en.js maps them to English.
These tests make an untranslated or half-translated page impossible to ship: every string the page can show must be in en.js
(EN, or SAME when it reads the same in both languages), with the same {placeholders}."""
import glob
import json
import os
import re
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
JS = os.path.join(os.path.dirname(HERE), "wbclient", "web", "js")
VN = re.compile("[À-ÖØ-öø-ỹ]")          # letters with diacritics (not the signs x and /)
NOT_TEXT = {"i18n.js", os.path.join("lang", "en.js"), "labels.js"}        # the dictionary, its engine, and the label tables (checked through node)


def literals(src):
    """(kind, start, end, text) of every string literal of a JavaScript source; comments are skipped."""
    i, n, out = 0, len(src), []
    while i < n:
        c = src[i]
        if src.startswith("//", i):
            j = src.find("\n", i)
            i = n if j < 0 else j
        elif src.startswith("/*", i):
            j = src.find("*/", i)
            i = n if j < 0 else j + 2
        elif c in "'\"":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            out.append((c, i, j + 1, src[i + 1:j]))
            i = j + 1
        elif c == "`":
            j, depth = i + 1, 0
            while j < n:
                if src[j] == "\\":
                    j += 2
                    continue
                if depth == 0 and src[j] == "`":
                    break
                if depth == 0 and src.startswith("${", j):
                    depth, j = 1, j + 2
                    continue
                if depth:
                    depth += {"{": 1, "}": -1}.get(src[j], 0)
                j += 1
            out.append(("`", i, j + 1, src[i + 1:j]))
            i = j + 1
        else:
            i += 1
    return out


def page_sources():
    for path in sorted(glob.glob(os.path.join(JS, "**", "*.js"), recursive=True)):
        rel = os.path.relpath(path, JS)
        if rel not in NOT_TEXT:
            with open(path, encoding="utf-8") as fp:
                yield rel, fp.read()


def unescape(text):
    return text.replace("\\'", "'").replace('\\"', '"').replace("\\\\", "\\")


@pytest.fixture(scope="module")
def tables(tmp_path_factory):
    """en.js and labels.js as data, read by node (they are plain ES modules without DOM access)."""
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    script = r"""
import { pathToFileURL } from 'node:url';
const dir = process.argv[2];
const en = await import(pathToFileURL(dir + '/lang/en.js').href);
const L = await import(pathToFileURL(dir + '/labels.js').href);
const i18n = await import(pathToFileURL(dir + '/i18n.js').href);
const labels = [];
const add = (s) => { if (typeof s === 'string' && s && !labels.includes(s)) labels.push(s); };
for (const m of Object.values(L.FIELDS)) { add(m.label); add(m.unit); add(m.help); }
for (const v of Object.values(L.BLOCKERS)) add(v);
for (const m of Object.values(L.MODES)) { add(m.label); add(m.help); }
for (const m of Object.values(L.BEEP_CONDITIONS)) { add(m.label); add(m.help); }
for (const m of Object.values(L.BB_FIELDS)) { add(m.label); add(m.help); }
for (const v of L.LOG_END_REASONS) add(v);
for (const g of Object.values(L.ENUM_LABELS)) for (const v of Object.values(g)) add(v);
for (const v of Object.values(L.ERRORS)) add(v);
i18n.forceLang('en');
const sample = [i18n.tr('Lưu'), i18n.tr('Đã có {n}/6 mặt. Xoay máy sang mặt khác rồi bấm "Lấy mặt này".', { n: 4 }), i18n.tr('Hz'), i18n.tr('chuỗi chưa dịch')];
i18n.forceLang('vi');
sample.push(i18n.tr('Lưu'), i18n.tr('{n} khung/s', { n: 150 }));
console.log(JSON.stringify({ EN: en.EN, SAME: en.SAME, labels, sample, langs: Object.keys(i18n.LANGS), blockers: Object.keys(L.BLOCKERS), modes: Object.keys(L.MODES), beep: Object.keys(L.BEEP_CONDITIONS), bb_fields: Object.keys(L.BB_FIELDS), enums: Object.fromEntries(Object.entries(L.ENUM_LABELS).map(([k, v]) => [k, Object.keys(v)])) }));
"""
    path = tmp_path_factory.mktemp("i18n") / "dump.mjs"
    path.write_text(script, encoding="utf-8")
    r = subprocess.run([node, str(path), JS], capture_output=True, text=True, encoding="utf-8")
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def used_strings(tables):
    used = {}
    for rel, src in page_sources():
        for kind, a, b, text in literals(src):
            if kind != "`" and re.search(r"\btr\($", src[:a].rstrip()):
                used.setdefault(unescape(text), rel)
    for s in tables["labels"]:
        used.setdefault(s, "labels.js")
    return used


def test_every_string_of_the_page_has_an_english_text(tables):
    en, same = tables["EN"], set(tables["SAME"])
    used = used_strings(tables)
    assert len(used) > 400
    missing = sorted("%s  (%s)" % (s, where) for s, where in used.items() if s not in en and s not in same)
    assert not missing, "add these to web/js/lang/en.js (EN, or SAME if identical in English):\n  " + "\n  ".join(missing)
    for s in same:
        assert not VN.search(s), "%r is listed as 'same in both languages' but is Vietnamese" % s
    for vi, eng in en.items():
        assert eng.strip() and not VN.search(eng), "the English text of %r still has Vietnamese letters: %r" % (vi, eng)
        assert sorted(re.findall(r"\{(\w+)\}", vi)) == sorted(re.findall(r"\{(\w+)\}", eng)), "placeholders differ: %r -> %r" % (vi, eng)
        assert (vi[:1] == " ") == (eng[:1] == " ") and (vi[-1:] == " ") == (eng[-1:] == " "), "leading / trailing space must be kept (texts are glued together): %r" % vi


def test_every_state_the_firmware_can_report_has_a_text(tables):
    """A blocker, a failsafe cause or a driver state without a text is shown to the pilot as its code name."""
    from wbclient.schema import Schema
    s = Schema()
    missing = [b for b in s.bits["arm_blocker"] if b not in tables["blockers"]]
    assert not missing, "labels.js BLOCKERS has no text for: %s" % missing
    assert set(s.bits["fs_cause"]) == set(tables["enums"]["fs_cause"])
    for name in ("failsafe", "dshot_driver", "bidir_state", "airmode", "bb_state", "batt_state"):
        assert set(s.enums[name]) == set(tables["enums"][name]), "labels.js ENUM_LABELS.%s: %s vs the schema's %s" % (name, tables["enums"][name], s.enums[name])
    # the names on the Modes tab, the beeper table and the log's field groups
    assert set(s.enums["mode"]) == set(tables["modes"]), "labels.js MODES: %s vs the schema's %s" % (tables["modes"], s.enums["mode"])
    for table, enum in (("beep", "beeper_cond"), ("bb_fields", "bb_field")):
        assert set(s.enums[enum]) == set(tables[table]), "labels.js %s: %s vs the schema's %s" % (table, tables[table], s.enums[enum])


def test_the_dictionary_has_no_dead_entries(tables):
    used = used_strings(tables)
    dead = [s for s in list(tables["EN"]) + tables["SAME"] if s not in used]
    assert not dead, "en.js entries that no longer match any string of the page (a text was edited without its translation?):\n  " + "\n  ".join(dead)
    assert not set(tables["EN"]) & set(tables["SAME"])


def test_no_text_bypasses_the_translation(tables):
    """A Vietnamese string literal that is not passed through tr() would stay Vietnamese on the English page."""
    bad = []
    for rel, src in page_sources():
        for kind, a, b, text in literals(src):
            if not VN.search(text) or text == "Ngôn ngữ / Language":
                continue
            if kind == "`":
                bad.append("%s: template string with Vietnamese text (use tr('... {x} ...', { x })): %s" % (rel, src[a:b][:90]))
            elif not re.search(r"\btr\($", src[:a].rstrip()):
                bad.append("%s: not wrapped in tr(): %s" % (rel, src[a:b][:90]))
    assert not bad, "\n".join(bad)


def test_tr_fills_values_and_falls_back(tables):
    save, faces, unit, unknown, save_vi, frames_vi = tables["sample"]
    assert save == "Save" and save_vi == "Lưu"
    assert faces.startswith("4/6 faces captured") and "{n}" not in faces
    assert unit == "Hz" and unknown == "chuỗi chưa dịch", "an unknown string is shown as it is, never dropped"
    assert frames_vi == "150 khung/s"
    assert tables["langs"] == ["vi", "en"]
