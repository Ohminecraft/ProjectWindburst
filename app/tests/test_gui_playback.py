"""Playing a log back like Betaflight's Blackbox Explorer (0.31): the page's own js/gfx/playback.js, run in node."""
import json
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PLAYBACK = os.path.join(os.path.dirname(HERE), "wbclient", "web", "js", "gfx", "playback.js")

JS = r"""
import { pathToFileURL } from 'node:url';
const m = await import(pathToFileURL(process.argv[2]).href);
const span = [0, 60];
// one second of 60 frames at 25 % and at 200 %
const run = (speed) => { let t = 10, ended = false; for (let i = 0; i < 60; i++) { const r = m.advance(t, speed, 1 / 60, span); t = r.t; ended = r.ended; } return [t, ended]; };
const out = {
  speeds: m.SPEEDS,
  quarter: run(0.25), double: run(2),
  end: m.advance(59.99, 4, 0.1, span),
  win_mid: m.windowAround(30, 4, span), win_start: m.windowAround(0.5, 4, span), win_end: m.windowAround(59.5, 4, span), win_all: m.windowAround(30, 100, span),
  seek: [m.seekX(0, 600, span), m.seekX(300, 600, span), m.seekX(600, 600, span), m.seekX(-5, 600, span), m.seekX(900, 600, span)],
  step: [m.stepT(10, 1, 0.004, span), m.stepT(10, -1, 0.004, span, 10), m.stepT(0, -1, 0.004, span), m.stepT(60, 1, 0.004, span)],
  faster: m.nextSpeed(1, 1), slower: m.nextSpeed(1, -1), top: m.nextSpeed(4, 1), bottom: m.nextSpeed(0.05, -1), odd: m.nextSpeed(0.3, 1),
  cov_in: m.covered(10, 14, 8, 20, span), cov_short: m.covered(10, 14, 9.5, 20, span), cov_edge: m.covered(0, 4, 0, 9, span), cov_end: m.covered(56, 60, 50, 60, span),
  pre: m.prefetch(10, 14, span), pre_end: m.prefetch(56, 60, span),
};
console.log(JSON.stringify(out));
"""


@pytest.fixture(scope="module")
def res(tmp_path_factory):
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    script = tmp_path_factory.mktemp("js") / "playback.mjs"
    script.write_text(JS)
    r = subprocess.run([node, str(script), PLAYBACK], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def test_playhead_runs_at_the_speed_and_stops_at_the_end(res):
    assert res["speeds"][0] == 0.05 and res["speeds"][-1] == 4 and 1 in res["speeds"]
    assert abs(res["quarter"][0] - 10.25) < 1e-9 and not res["quarter"][1], "one second at 25 %: a quarter of a second of the log"
    assert abs(res["double"][0] - 12.0) < 1e-9, "at 200 %: two seconds"
    assert res["end"] == {"t": 60, "ended": True}, "the end of the log stops it"


def test_window_keeps_the_playhead_in_the_middle_inside_the_log(res):
    assert res["win_mid"] == [28, 32]
    assert res["win_start"] == [0, 4] and res["win_end"] == [56, 60], "at the edges the window stays inside the log"
    assert res["win_all"] is None, "wider than the log: the whole log"


def test_overview_seek_and_steps(res):
    assert res["seek"] == [0, 30, 60, 0, 60]
    s = res["step"]
    assert abs(s[0] - 10.004) < 1e-9 and abs(s[1] - 9.96) < 1e-9 and s[2] == 0 and s[3] == 60, "one record, ten with Shift, never outside the log"


def test_speed_keys(res):
    assert res["faster"] == 2 and res["slower"] == 0.5 and res["top"] == 4 and res["bottom"] == 0.05 and res["odd"] == 2


def test_fetch_ahead_of_the_playhead(res):
    assert res["cov_in"] and not res["cov_short"], "a quarter of the window of margin behind and ahead, else fetched again"
    assert res["cov_edge"] and res["cov_end"], "at the ends of the log the margin is what the log has (no endless refetching)"
    assert res["pre"] == [8, 18] and res["pre_end"] == [54, 60], "half a window behind, a whole window ahead"
