"""The page's motion (js/anim.js): the frames of a value being 'decoded' left to right, and how long it takes. Pure functions,
evaluated with node."""
import json
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ANIM = os.path.join(os.path.dirname(HERE), "wbclient", "web", "js", "anim.js")

JS = r"""
import { pathToFileURL } from 'node:url';
const m = await import(pathToFileURL(process.argv[2]).href);
let seed = 7; const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
const text = '1234.5 Hz ok';
const frames = [0, 0.25, 0.5, 0.75, 1, 1.5, -1].map((p) => m.scrambleFrame(text, p, rnd));
console.log(JSON.stringify({ frames, noise: m.NOISE, ms: [0, 4, 12, 100].map(m.decodeMs), empty: m.scrambleFrame('', 0.5, rnd) }));
"""


@pytest.fixture(scope="module")
def out():
    node = shutil.which("node")
    if not node:
        pytest.skip("node not installed")
    r = subprocess.run([node, "--input-type=module", "-", ANIM], input=JS, capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def test_frames_reveal_left_to_right(out):
    text, frames = "1234.5 Hz ok", out["frames"]
    for f in frames:
        assert len(f) == len(text)                           # the noise is exactly as long as the value
        assert [i for i, c in enumerate(f) if c == " "] == [i for i, c in enumerate(text) if c == " "]   # spaces stay where they are
    assert frames[4] == text and frames[5] == text           # done (and past done): the value itself
    for f, p in zip(frames[:4], [0, 0.25, 0.5, 0.75]):
        n = round(p * len(text))
        assert f[:n] == text[:n]                             # the left part is the value already
        assert all(c in out["noise"] for c, t in zip(f[n:], text[n:]) if t != " ")   # the rest is noise
    assert frames[0] != text and all(c in out["noise"] + " " for c in frames[6])   # before the start: all noise


def test_duration(out):
    assert out["ms"] == [150, 250, 450, 500]                 # 150 ms + 25 per character, at most half a second
    assert out["empty"] == ""
