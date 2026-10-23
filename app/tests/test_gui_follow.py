"""A live recording on the chart (0.31): the window that follows it moves smoothly whatever the rhythm of the data - the page's
own js/gfx/follow.js, run in node against records that arrive in bursts (every 20 ms, every 0.25 s, once a second)."""
import json
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FOLLOW = os.path.join(os.path.dirname(HERE), "wbclient", "web", "js", "gfx", "follow.js")

JS = r"""
import { pathToFileURL } from 'node:url';
const m = await import(pathToFileURL(process.argv[2]).href);
// records arrive in bursts every `burst` s (each brings everything up to its own moment); the page fetches every 0.2 s and
// paints at 60 frames a second
function run(burst, seconds) {
  const fps = 60, dt = 1 / fps; let disp = null, lag = 0.3, newest = 0, seen = -1, seenWall = 0, steps = [], ahead = 0, lags = [];
  for (let k = 0; k < seconds * fps; k++) {
    const t = k * dt;
    if (k % Math.round(0.2 * fps) === 0) {                       // a fetch: it sees the newest record that has arrived by now
      const avail = Math.floor(t / burst) * burst;
      if (avail > seen) { if (seenWall) lag = m.nextLag(lag, t - seenWall); seen = avail; seenWall = t; }
      newest = avail;
    }
    const before = disp; disp = m.followStep(disp, newest, lag, dt);
    if (t > 3 && before !== null) steps.push(disp - before);
    if (disp > newest + 1e-9) ahead++;
    lags.push(lag);
  }
  return { steps, ahead, lag, min_step: Math.min(...steps), max_step: Math.max(...steps) };
}
const out = { r20: run(0.02, 20), r250: run(0.25, 20), r1000: run(1.0, 20) };
out.lag_up = m.nextLag(0.1, 1.0); out.lag_down = m.nextLag(1.25, 0.02); out.lag_cap = m.nextLag(0.1, 10);
out.first = m.followStep(null, 10, 0.3, 0.016);
out.back = m.followStep(9.0, 9.5, 2.0, 0.016);
console.log(JSON.stringify(out));
"""


@pytest.fixture(scope="module")
def res(tmp_path_factory):
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    script = tmp_path_factory.mktemp("js") / "follow.mjs"
    script.write_text(JS)
    r = subprocess.run([node, str(script), FOLLOW], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


@pytest.mark.parametrize("key", ["r20", "r250", "r1000"])
def test_window_moves_smoothly_whatever_the_bursts(res, key):
    r = res[key]
    dt = 1 / 60
    # every frame the window moves on by about one frame of time: never stops, never jumps (a 1 Hz burst used to be a 1 s jump)
    assert r["ahead"] == 0, "the window never ends past the newest record"
    assert r["min_step"] > 0.5 * dt and r["max_step"] < 2.0 * dt, (key, r["min_step"], r["max_step"])


def test_lag_follows_the_rhythm(res):
    assert res["r20"]["lag"] < 0.3, "records every 20 ms: the window ends about one fetch (0.2 s) behind the newest"
    assert 1.0 <= res["r1000"]["lag"] <= 2.0, "records once a second: it runs about 1.25 s behind, so it never waits for data"
    assert res["lag_up"] == 1.25 and res["lag_cap"] == 2.0 and 1.0 < res["lag_down"] < 1.25, "the lag grows at once, shrinks slowly, at most 2 s"


def test_start_and_no_jump_back(res):
    assert abs(res["first"] - 9.7) < 1e-9, "the first frame starts lag behind the newest record"
    assert res["back"] >= 9.0, "a window ahead of its target waits, it never jerks backwards"
