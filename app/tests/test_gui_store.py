"""js/store.js: the order of settings writes. A second edit of a section made while the first is still on its way to the firmware
must be sent AFTER it, and an answer to the older write must not wipe the newer edit off the page. Run in node with a stand-in
for js/api.js whose requests the test answers when it wants."""
import json
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
STORE = os.path.join(os.path.dirname(HERE), "wbclient", "web", "js", "store.js")

API = r"""
export const calls = []; const answers = [];
export const api = { post: (path, body) => new Promise((resolve) => { calls.push({ path, body }); answers.push(resolve); }), get: async () => ({}) };
export const answer = (i, doc) => answers[i](doc);
"""
JS = r"""
import { pathToFileURL } from 'node:url';
const dir = process.argv[2];
const { calls, answer } = await import(pathToFileURL(dir + '/api.js').href);
const st = await import(pathToFileURL(dir + '/store.js').href);
const tick = () => new Promise((r) => setTimeout(r, 5));
st.store.settings = { pid: { slider_mode: 'RPY', p: [45, 47, 45] } };
st.setField('pid', 'slider_mode', 'OFF'); const f1 = st.flush('pid'); await tick();        // the first write is on its way
const busy1 = st.hasPending();
st.setField('pid', 'p', 51, 0); const f2 = st.flush('pid'); await tick();                   // the user was faster than the round trip
const sentWhileFirstOnItsWay = calls.length;
answer(0, { values: { slider_mode: 'OFF', p: [45, 47, 45] } });                             // the firmware's answer to the FIRST write: it has not seen p yet
await f1; await tick();
const shownAfterFirstAnswer = JSON.parse(JSON.stringify(st.store.settings.pid));
const sentAfterFirstAnswer = calls.length;
answer(1, { values: { slider_mode: 'OFF', p: [51, 47, 45] } }); await f2;
console.log(JSON.stringify({ busy1, sentWhileFirstOnItsWay, sentAfterFirstAnswer, order: calls.map((c) => Object.keys(c.body.changes)), shownAfterFirstAnswer, shown: st.store.settings.pid, busyAtTheEnd: st.hasPending() }));
"""


def test_writes_of_one_section_go_out_in_order_and_old_answers_do_not_undo_new_edits(tmp_path):
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    shutil.copy(STORE, tmp_path / "store.js")
    (tmp_path / "api.js").write_text(API)
    (tmp_path / "run.mjs").write_text(JS)
    r = subprocess.run([node, str(tmp_path / "run.mjs"), str(tmp_path)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    o = json.loads(r.stdout)
    assert o["busy1"] is True, "a write on its way counts as unsaved work (Save waits for it)"
    assert o["sentWhileFirstOnItsWay"] == 1 and o["sentAfterFirstAnswer"] == 2, "the second write waits for the first to be answered: %s" % o
    assert o["order"] == [["slider_mode"], ["p"]]
    assert o["shownAfterFirstAnswer"]["p"] == [51, 47, 45], "the answer to the older write must not put the old value back on the page"
    assert o["shown"] == {"slider_mode": "OFF", "p": [51, 47, 45]} and o["busyAtTheEnd"] is False
