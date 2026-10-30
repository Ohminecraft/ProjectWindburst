"""The Mission tab's plan <-> the firmware's `mission` section (js/mission_plan.js), evaluated by node: what the app writes must be
what nav/mission.h flies (ArduPilot command order: a DO command runs when the NAV command before it starts)."""
import json
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PLAN_JS = os.path.join(os.path.dirname(HERE), "wbclient", "web", "js", "mission_plan.js")

JS = r"""
import { pathToFileURL } from 'node:url';
import { readFileSync } from 'node:fs';
const m = await import(pathToFileURL(process.argv[process.argv.length - 1]).href);      // (node -e: no script name in argv)
const req = JSON.parse(readFileSync(0, 'utf8'));
const out = {};
for (const [k, call] of Object.entries(req)) {
  try { out[k] = m[call.fn](...call.args); } catch (e) { out[k] = { error: String(e.message || e) }; }
}
console.log(JSON.stringify(out));
"""

SERVO = {"source": ["MISSION", "OFF", "OFF", "OFF"], "min_us": [1000, 1000, 1000, 1000], "max_us": [2000, 2000, 2000, 2000], "reverse": [0, 1, 0, 0],
         "angle_min": [0, 0, 0, 0], "angle_max": [180, 180, 180, 180], "init_us": [1500, 1500, 1500, 1500]}
HOME = {"lat": 47.3977419, "lon": 8.5455938}


def run(calls):
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    r = subprocess.run([node, "--input-type=module", "-e", JS, PLAN_JS], input=json.dumps(calls), capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def plan():
    return {"takeoff_alt": 10, "speed": 0, "rtl": True, "points": [
        {"lat": 47.398, "lon": 8.5456, "alt": 12, "act": "fly", "hold": 3},
        {"lat": 47.3985, "lon": 8.5460, "alt": 15, "act": "servo", "hover": 2, "steps": [{"servo": 1, "deg": 90, "wait": 3}, {"servo": 1, "deg": 0, "wait": 1}]},
        {"lat": 47.3990, "lon": 8.5470, "alt": 12, "act": "land", "wait": 60},
    ]}


def test_encode_the_delivery_plan():
    out = run({"s": {"fn": "encodePlan", "args": [plan(), SERVO]}})["s"]
    assert out["cmd"][:out["count"]] == ["TAKEOFF", "WAYPOINT", "WAYPOINT", "LOITER_TIME", "LOITER_TIME", "SERVO", "LOITER_TIME", "SERVO", "LAND", "DELAY", "TAKEOFF", "RTL"]
    assert len(out["cmd"]) == 32 and out["cmd"][12] == "NONE", "the whole section: 32 slots, the unused ones NONE"
    # take-off: height only; the first point: a waypoint with its hold time
    assert out["alt"][0] == 10 and out["lat"][0] == 0 and out["lon"][0] == 0
    assert out["cmd"][1] == "WAYPOINT" and out["lat"][1] == 473980000 and out["lon"][1] == 85456000 and out["alt"][1] == 12 and out["p"][1] == 3
    # the servo point: fly there, hover 2 s, then each step = LOITER_TIME (its wait) + SERVO (runs when that wait starts)
    assert out["cmd"][2] == "WAYPOINT" and out["p"][2] == 0
    assert out["cmd"][3] == "LOITER_TIME" and out["p"][3] == 2 and out["lat"][3] == 0
    assert out["cmd"][4] == "LOITER_TIME" and out["p"][4] == 3 and out["cmd"][5] == "SERVO" and out["arg"][5] == 1 and out["p"][5] == 1500
    assert out["cmd"][6] == "LOITER_TIME" and out["p"][6] == 1 and out["cmd"][7] == "SERVO" and out["p"][7] == 1000
    # the delivery stop: LAND at the point, DELAY, TAKEOFF to the point's height; the end: RTL
    assert out["cmd"][8:12] == ["LAND", "DELAY", "TAKEOFF", "RTL"]
    assert out["lat"][8] == 473990000 and out["lon"][8] == 85470000 and out["p"][9] == 60 and out["alt"][10] == 12
    assert out["count"] == 12 and out["rc_lost"] == "CONTINUE"


def test_round_trip_and_degrees():
    p = plan()
    r = run({"s": {"fn": "encodePlan", "args": [p, SERVO]}})
    back = run({"b": {"fn": "decodePlan", "args": [r["s"], SERVO]}})["b"]
    assert back["takeoff_alt"] == 10 and back["rtl"] is True and len(back["points"]) == 3
    a, b, c = back["points"]
    assert a["act"] == "fly" and a["hold"] == 3 and abs(a["lat"] - 47.398) < 1e-7 and a["alt"] == 12
    assert b["act"] == "servo" and b["hover"] == 2 and b["steps"] == [{"servo": 1, "deg": 90, "wait": 3}, {"servo": 1, "deg": 0, "wait": 1}]
    assert c["act"] == "land" and c["wait"] == 60 and c["alt"] == 12
    r = run({"a": {"fn": "degToUs", "args": [SERVO, 0, 45]}, "b": {"fn": "degToUs", "args": [SERVO, 1, 45]}, "c": {"fn": "degToUs", "args": [SERVO, 0, 400]},
             "d": {"fn": "usToDeg", "args": [SERVO, 0, 1250]}, "e": {"fn": "usToDeg", "args": [SERVO, 1, 1250]}})
    assert r["a"] == 1250 and r["b"] == 1750 and r["c"] == 2000, "45 deg of 0..180 on 1000..2000 us; servo 2 reversed; beyond the range: its end"
    assert r["d"] == 45 and r["e"] == 135


def test_limits_and_estimate():
    many = {"takeoff_alt": 10, "speed": 0, "rtl": True, "points": [{"lat": 47.398 + i * 1e-4, "lon": 8.5456, "alt": 10, "act": "land", "wait": 10} for i in range(11)]}
    r = run({"s": {"fn": "encodePlan", "args": [many, SERVO]}, "n": {"fn": "commandCount", "args": [many]}})
    assert r["n"] == 1 + 11 * 3 + 1 and "error" in r["s"] and "32" in r["s"]["error"], "34 commands do not fit: refused with the limit in the message"
    est = run({"e": {"fn": "estimate", "args": [plan(), HOME, 5.0]}})["e"]
    # legs: home -> 1 (28.7 m) -> 2 (63.3 m) -> 3 (93.7 m) -> home (175.6 m) = 361.3 m
    assert 359 <= est["distance_m"] <= 363, est
    assert est["time_s"] > est["distance_m"] / 5.0 + 60 + 3 + 2 + 4, "flying at 5 m/s plus the waits (60 s on the ground, 3 s hold, 2 + 4 s at the servo)"
