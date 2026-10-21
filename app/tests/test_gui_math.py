"""The previews in the GUI (motor diagram, rate curve, throttle curve, deadband) must show what the FIRMWARE computes.
build/ui_reference (tools/ui_reference.cpp) prints the firmware's numbers; node evaluates the page's own js/gfx/fwmath.js."""
import json
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
REF = os.path.join(ROOT, "build", "ui_reference" + (".exe" if os.name == "nt" else ""))
FWMATH = os.path.join(ROOT, "app", "wbclient", "web", "js", "gfx", "fwmath.js")

JS = r"""
import { pathToFileURL } from 'node:url';
import { readFileSync } from 'node:fs';
const m = await import(pathToFileURL(process.argv[2]).href);
const ref = JSON.parse(readFileSync(0, 'utf8'));
const out = { layouts: {}, yaw_response: {}, rate: [], throttle: [], stick: [], attitude: [] };
for (const name of Object.keys(ref.layouts)) {
  out.layouts[name] = ref.layouts[name].map((_, i) => { const r = m.mixerRow(name, i); return [r.roll, r.pitch, r.yaw]; });
  if (m.motorCount(name) !== ref.layouts[name].length) throw new Error('motor count ' + name);
}
out.yaw_response.normal = [0, 1, 2, 3].map((i) => m.mixerRow('QUAD_X', i, false).yaw);
out.yaw_response.props_out = [0, 1, 2, 3].map((i) => m.mixerRow('QUAD_X', i, true).yaw);
out.rate = ref.rate.map(([c, mx, e, s]) => m.actualRate(c, mx, e, s));
out.throttle = ref.throttle.map(([mid, e, lim, t]) => m.throttleCurve(mid, e, lim, t));
out.stick = ref.stick.map(([us, db]) => [m.stick(us, db), m.throttle01(us)]);
out.attitude = ref.attitude.map(([r, p, y]) => m.rotation(r, p, y).flat());
out.beeper = m.BEEP_PATTERNS;
out.motor_never = m.MOTOR_NEVER;
out.motor_tones = Object.fromEntries(Object.keys(m.BEEP_PATTERNS).map((n) => [n, m.motorTones(n)]));
out.beacon_slot_ms = m.BEACON_SLOT_MS;
out.led = ref.led.map((c) => m.ledColors(c, c.st, c.t));
{ let R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]; out.acro = ref.acro.map(([r, p, y, dt, n]) => { for (let k = 0; k < n; k++) R = m.acroStep(R, [r, p, y], dt); return R.flat(); }); }
out.angle = ref.angle.map(([l, c, mx, e, s]) => m.angleTarget(l, c, mx, e, s));
console.log(JSON.stringify(out));
"""


@pytest.fixture(scope="module")
def both(tmp_path_factory):
    if not os.path.exists(REF):
        pytest.skip("build/ui_reference not built (run `make apptest`)")
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed: the page's formulas can not be evaluated outside a browser")
    ref_text = subprocess.run([REF], capture_output=True, text=True, check=True).stdout
    script = tmp_path_factory.mktemp("js") / "eval.mjs"
    script.write_text(JS)
    r = subprocess.run([node, str(script), FWMATH], input=ref_text, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return json.loads(ref_text), json.loads(r.stdout)


def test_motor_layouts_match_the_mixer(both):
    ref, js = both
    assert set(js["layouts"]) == {"QUAD_X", "QUAD_PLUS", "HEX_X", "HEX_PLUS", "HEX_X_BF", "HEX_H", "Y6", "Y4", "OCTO_X8", "OCTO_FLAT_PLUS", "OCTO_FLAT_X", "VTAIL4", "ATAIL4", "QUAD_X_1234"}
    for name, rows in ref["layouts"].items():
        for i, (a, b) in enumerate(zip(rows, js["layouts"][name])):
            assert a == pytest.approx(b, abs=1e-5), "%s motor %d: firmware %s, page %s" % (name, i + 1, a, b)
    assert ref["yaw_response"]["normal"] == pytest.approx(js["yaw_response"]["normal"]), "which motors speed up for yaw right"
    assert ref["yaw_response"]["props_out"] == pytest.approx(js["yaw_response"]["props_out"]), "props-out flips every motor"
    # Betaflight's default quad X (props in): 1 rear-right CW, 2 front-right CCW, 3 rear-left CCW, 4 front-left CW; CCW props speed up for yaw right
    assert ref["yaw_response"]["normal"] == pytest.approx([-1, 1, 1, -1])


def test_rate_curve_matches(both):
    ref, js = both
    assert len(ref["rate"]) > 200
    for row, v in zip(ref["rate"], js["rate"]):
        assert v == pytest.approx(row[4], abs=0.01), "center %d max %d expo %d stick %.1f: firmware %.3f, page %.3f" % (*row, v)


def test_throttle_curve_matches(both):
    ref, js = both
    assert len(ref["throttle"]) > 150
    for row, v in zip(ref["throttle"], js["throttle"]):
        assert v == pytest.approx(row[4], abs=1e-4), "mid %d expo %d limit %d stick %.1f: firmware %.4f, page %.4f" % (*row, v)


def test_stick_deadband_matches(both):
    ref, js = both
    for row, (st, thr) in zip(ref["stick"], js["stick"]):
        assert st == pytest.approx(row[2], abs=1e-5) and thr == pytest.approx(row[3], abs=1e-5), "us %d deadband %d" % (row[0], row[1])


def test_3d_view_rotates_like_the_estimator(both):
    """roll / pitch / yaw of the telemetry -> the rotation the 3D model is drawn with == the firmware's own body->world rotation."""
    ref, js = both
    assert len(ref["attitude"]) > 30
    for row, R in zip(ref["attitude"], js["attitude"]):
        assert R == pytest.approx(row[3:], abs=2e-4), "roll %.1f pitch %.1f yaw %.1f" % tuple(row[:3])
    # the convention in words: right wing down = +roll, nose down = +pitch, nose right = +yaw
    one = next(r for r in ref["attitude"] if abs(r[0]) > 20 and abs(r[1]) > 20)
    fwd_down, right_down = one[3 + 6], one[3 + 7]
    assert (fwd_down > 0) == (one[1] > 0), "nose below the horizon <=> positive pitch"
    if abs(one[0]) < 90:
        assert (right_down > 0) == (one[0] > 0), "right wing below the horizon <=> positive roll"


def test_beep_patterns_drawn_are_the_patterns_played(both):
    """The Battery and beeper tab draws each sound as on/off bars. They come from a table in the page; it must be the firmware's."""
    ref, js = both
    assert js["beeper"] == ref["beeper"], "js/gfx/fwmath.js BEEP_PATTERNS differs from src/hal/beeper/beeper_core.h"
    assert len(ref["beeper"]) == 17 and ref["beeper"]["ARMING"] == [30, 5, 5, 5]
    from wbclient import Schema
    assert Schema().enums["beeper_cond"] == ref["beeper_order"], "the schema's condition names are the firmware table, in priority order"


def test_motor_sounder_shown_as_the_firmware_plays_it(both):
    """The tab says how many tones the motors play for each condition, and greys out what they never play."""
    ref, js = both
    assert js["motor_never"] == ref["motor_never"], "js MOTOR_NEVER differs from beep::kMotorNever"
    assert js["beacon_slot_ms"] == ref["beacon_slot_ms"] == 450
    assert js["motor_tones"] == ref["motor_tones"], "tones per pattern: page %s, firmware %s" % (js["motor_tones"], ref["motor_tones"])
    assert ref["motor_tones"]["GYRO_CALIBRATED"] == [3, 1100] and ref["motor_tones"]["DISARMING"] == [2, 500] and ref["motor_tones"]["RX_LOST"] == [1, 0]
    from wbclient import Schema
    sch = Schema()
    assert sorted(ref["motor_default"]) == sorted(sch.layout("section", "beeper").defaults()["motor_conditions"]), "schema default = firmware default"
    assert not set(ref["motor_default"]) & set(ref["motor_never"]) and "SYSTEM_INIT" not in ref["motor_default"]


def test_led_preview_shows_what_the_strip_shows(both):
    """Battery and beeper tab, LED layout MANUAL: the preview runs js/gfx/fwmath.js ledColors(); it must be src/app/led_logic.h."""
    ref, js = both
    from wbclient import Schema
    assert Schema().enums["led_function"] == ref["led_functions"], "the schema's led_function names are LedLogic::Fn, in order"
    assert len(ref["led"]) == 24 * 12
    seen = set()
    for c, got in zip(ref["led"], js["led"]):
        assert got == c["out"], "count %d speed %d warnings %s state %s t %d" % (c["count"], c["speed"], c["warnings"], c["st"], c["t"])
        seen.update(c["fn"][:c["count"]])
    assert seen == set(ref["led_functions"]), "every function was compared"


def test_stick_preview_flies_like_the_firmware(both):
    """Receiver / Rate tabs, the craft on the sticks: Acro turns the body as the firmware's attitude would with those rates
    (gyro alone through the Ahrs), Angle tilts to the AngleController's target."""
    ref, js = both
    assert len(ref["acro"]) == 6
    for row, R in zip(ref["acro"], js["acro"]):
        assert R == pytest.approx(row[5:], abs=2e-3), "after rates roll %.0f pitch %.0f yaw %.0f" % tuple(row[:3])
    # the conventions in words: +roll = right wing down, +pitch = nose down
    one = ref["acro"][0]; assert one[5 + 7] > 0.5, "200 deg/s roll for 0.5 s: the right wing points down"
    assert len(ref["angle"]) > 100
    for row, v in zip(ref["angle"], js["angle"]):
        assert v == pytest.approx(row[5], abs=1e-3), "limit %d centre %d max %d expo %d stick %.1f" % tuple(row[:5])
