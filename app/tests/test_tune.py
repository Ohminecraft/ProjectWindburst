"""PID tuning from a log (wbclient/tune.py, 0.31): the step response recovered from flown sticks must be the response the craft
really has. Here the "craft" is a known second-order system driven by random stick moves: its step response is known exactly."""
import math
import random
import struct

from wbclient import Schema, tune, wblog
from wbclient import framing as F


def fly(zeta, wn, seconds=24.0, rate=1000.0, delay_ms=4.0, noise=0.0, seed=3):
    """Setpoint: stick moves held for random times (like a pilot); gyro: the setpoint through wn^2 / (s^2 + 2 zeta wn s + wn^2), late by delay_ms."""
    rnd = random.Random(seed)
    n, dt = int(seconds * rate), 1.0 / rate
    sp, gy = [], []
    target, hold, y, v = 0.0, 0, 0.0, 0.0
    lag = [0.0] * max(1, int(delay_ms * rate / 1000.0))
    for _ in range(n):
        if hold <= 0:
            target = rnd.choice([0.0, rnd.uniform(-400, 400)])
            hold = int(rnd.uniform(0.15, 0.8) * rate)
        hold -= 1
        sp.append(target)
        lag.append(target)
        u = lag.pop(0)
        a = wn * wn * (u - y) - 2 * zeta * wn * v
        v += a * dt
        y += v * dt
        gy.append(y + (rnd.gauss(0, noise) if noise else 0.0))
    return sp, gy, rate


def true_step(zeta, wn, rate, delay_ms, m):
    y, v, out = 0.0, 0.0, []
    d = int(delay_ms * rate / 1000.0)
    for k in range(m):
        u = 1.0 if k >= d else 0.0
        a = wn * wn * (u - y) - 2 * zeta * wn * v
        v += a / rate
        y += v / rate
        out.append(y)
    return out


def test_fft_matches_a_plain_dft():
    x = [math.sin(0.3 * k) + 0.5 * math.cos(1.7 * k) for k in range(16)]
    f = tune._fft(x)
    for k in range(16):
        ref = sum(x[j] * complex(math.cos(-2 * math.pi * k * j / 16), math.sin(-2 * math.pi * k * j / 16)) for j in range(16))
        assert abs(f[k] - ref) < 1e-9
    back = tune._fft(f, inverse=True)
    assert max(abs(back[k].real - x[k]) for k in range(16)) < 1e-12


def test_step_response_is_the_systems_own():
    for zeta, wn, gyro_noise, tol in ((0.4, 90.0, 0.0, 0.04), (1.0, 60.0, 0.0, 0.04), (0.4, 90.0, 4.0, 0.06), (1.0, 60.0, 4.0, 0.1)):
        sp, gy, rate = fly(zeta, wn, noise=gyro_noise)
        r = tune.step_response(sp, gy, rate)
        assert r and r["windows"] >= 5
        ref = true_step(zeta, wn, rate, 4.0, len(r["y"]))
        err = max(abs(a - b) for a, b in zip(r["y"], ref))
        assert err < tol, "zeta %.1f, gyro noise %.0f deg/s: recovered step response within %.3f of the true one" % (zeta, gyro_noise, err)
        if gyro_noise:
            continue
        m = tune.metrics(r, rate)
        want_peak = 1 + math.exp(-math.pi * zeta / math.sqrt(1 - zeta * zeta)) if zeta < 1 else 1.0
        assert abs(m["peak"] - want_peak) < 0.06 and abs(m["steady"] - 1.0) < 0.03, (zeta, m)


def test_no_stick_moves_no_answer():
    sp = [0.0] * 8000
    gy = [random.Random(1).gauss(0, 1) for _ in sp]
    assert tune.step_response(sp, gy, 1000.0) is None


def test_suggestions_follow_the_response():
    pid = {"p": [45, 47, 45], "i": [80, 84, 80], "d": [30, 34, 0], "d_max": [40, 46, 0], "f": [120, 125, 120]}
    sp, gy, rate = fly(0.35, 90.0)                       # 30 % of overshoot
    over = tune.metrics(tune.step_response(sp, gy, rate), rate)
    s = tune.suggest({0: over}, pid, {0: 0.1}, {0: 1.0})
    d = [x for x in s if x["param"] == "d"]
    assert d and d[0]["why"] == "OVERSHOOT" and d[0]["new"] > 30 and d[0]["new"] <= 30 * 1.25 + 1, s
    s_noisy = tune.suggest({0: over}, pid, {0: 0.5}, {0: 1.0})
    assert any(x["param"] == "p" and x["why"] == "OVERSHOOT_NOISY_D" and x["new"] < 45 for x in s_noisy), "a noisy D: less P instead"
    sp, gy, rate = fly(1.2, 45.0)                         # slow, no overshoot
    slow = tune.metrics(tune.step_response(sp, gy, rate), rate)
    assert slow["latency_ms"] > 25
    s = tune.suggest({1: slow}, pid, {1: 0.1}, {1: 1.0})
    assert any(x["param"] == "p" and x["why"] == "SLOW" and x["new"] > 47 for x in s) and any(x["param"] == "f" for x in s), s
    s = tune.suggest({0: dict(over, steady=0.85)}, pid, {0: 0.1}, {0: 1.0})
    assert any(x["param"] == "i" and x["why"] == "STEADY_LOW" and 80 < x["new"] <= 100 for x in s)
    good = {"peak": 1.03, "peak_ms": 30, "latency_ms": 12, "steady": 1.0, "ringing": 1}
    assert tune.suggest({0: good, 1: good}, pid, {0: 0.1, 1: 0.1}, {0: 1, 1: 1}) == [], "a good response: nothing to change"


def test_noise_measure():
    rate = 2000.0
    calm = [100 * math.sin(2 * math.pi * 5 * k / rate) for k in range(4000)]
    buzz = [v + 10 * math.sin(2 * math.pi * 300 * k / rate) for k, v in enumerate(calm)]
    assert tune.noise(calm, rate) < 1.0 and 5.0 < tune.noise(buzz, rate) < 8.0


def tune_log_bytes(zeta=0.35, wn=90.0, seconds=12.0, rate=1000.0, pid_d=30):
    """A whole log the way the firmware writes it: setpoint and gyro of three axes, the pid section in the header (pid.d = pid_d)."""
    schema = Schema()
    lay = schema.layout("section", "pid")
    vals = lay.defaults()
    vals["d"] = [pid_d, pid_d + 4, 0]
    header = ("WBLOG 1\nsession 1\nto storage\nfirmware Test 0.31.0\nschema 0x%08X\nloop_hz 4000\nrate_div 4\nrate_hz %d\nmotors 4\n" % (schema.hash, int(rate))
              + "fast t:u32:1e-06:s setpoint[0]:i16:0.1:deg/s setpoint[1]:i16:0.1:deg/s setpoint[2]:i16:0.1:deg/s gyro[0]:i16:0.1:deg/s gyro[1]:i16:0.1:deg/s gyro[2]:i16:0.1:deg/s\n"
              + "slow t:u32:1e-06:s armed:u8:1:\nevents 1:ARM 7:END\nset pid " + lay.pack(vals).hex() + "\nend\n").encode()
    out = bytearray()
    for i in range(0, len(header), 100):
        out += F.encode(F.BB_HEADER, i // 100, header[i:i + 100])
    sp = [fly(zeta, wn, seconds, rate, seed=s)[0] for s in (3, 4, 5)]
    gy = [fly(zeta, wn, seconds, rate, seed=s)[1] for s in (3, 4, 5)]
    seq, pay = 0, bytearray()
    for k in range(len(sp[0])):
        rec = b"F" + struct.pack("<I6h", 1_000_000 + int(k * 1e6 / rate), *[int(round(sp[a][k] * 10)) for a in range(3)], *[int(round(gy[a][k] * 10)) for a in range(3)])
        if len(pay) + len(rec) > 240:
            out += F.encode(F.BB_DATA, seq & 0xFF, pay); seq += 1; pay = bytearray()
        pay += rec
    out += F.encode(F.BB_DATA, seq & 0xFF, pay)
    return bytes(out)


def test_a_whole_log():
    s = wblog.parse(tune_log_bytes())[0]
    r = tune.analyse(s, Schema())
    assert set(r["axes"]) == {"roll", "pitch", "yaw"} and r["pid"]["d"][:2] == [30, 34] and not r["notes"], r["notes"]
    m = r["axes"]["roll"]["metrics"]
    assert 1.2 < m["peak"] < 1.4 and r["axes"]["roll"]["step"]["windows"] >= 3
    d = [x for x in r["suggestions"] if x["param"] == "d" and x["axis"] in ("roll", "pitch")]
    assert len(d) == 2 and all(x["new"] > x["now"] for x in d), r["suggestions"]
    assert not [x for x in r["suggestions"] if x["axis"] == "yaw" and x["param"] == "d"], "no D on yaw"
