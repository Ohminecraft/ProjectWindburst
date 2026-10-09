"""What the log viewer is built on (no firmware needed): reading a log while it grows, the folder of logs, and the reduced
view a chart draws - which must never hide a spike."""
import os

import pytest

from wbclient import logview, wblog
from wbclient import framing as F
from test_blackbox import _log_bytes


def test_a_log_read_in_pieces_is_the_log_read_at_once():
    data = _log_bytes(t0=0xFFFFFFFF - 30_000, records=200)             # the firmware's clock wraps inside this log
    whole = wblog.parse(data)[0]
    r = wblog.Reader()
    seen = []
    for i in range(0, len(data), 37):                                  # a live stream: bytes arrive in odd pieces, the chart asks in between
        r.feed(data[i:i + 37])
        got = r.sessions()
        if got:
            seen.append(len(got[0].fast["t"]))
    s = r.sessions()[0]
    assert seen[0] < 20 and seen == sorted(seen) and seen[-1] == 200, "records appear as their frames arrive"
    assert list(s.fast["t"]) == list(whole.fast["t"]) and list(s.fast["roll"]) == list(whole.fast["roll"]) and list(s.slow["lat"]) == list(whole.slow["lat"])
    assert s.events == whole.events and s.dropped == whole.dropped == 17 and s.complete, "asking again does not count anything twice"
    assert s.fast["t"][0] == pytest.approx(0.002) and s.fast["t"][-1] == pytest.approx(0.400) and all(b > a for a, b in zip(s.fast["t"], s.fast["t"][1:]))


def test_view_keeps_every_extreme():
    s = wblog.parse(_log_bytes(records=5000))[0]
    roll, t = s.fast["roll"], s.fast["t"]
    assert len(t) == 5000 and s.duration == pytest.approx(9.998)
    # everything on a chart 100 pixels wide: 50 samples per pixel column
    v = logview.view(s, ["roll", "gyro[0]"], px=100)
    r = v["series"]["roll"]
    assert v["span"] == [0.0, pytest.approx(10.0005)] and v["t0"] == 0.0 and v["t1"] == pytest.approx(10.0005), "the chart spans the events too: ARM just before the first record, END after the last"
    assert "v" not in r and len(r["t"]) == len(r["lo"]) == len(r["hi"]) == 100 and r["unit"] == "deg" and r["dt"] == pytest.approx(0.002)
    assert min(r["lo"]) == pytest.approx(min(roll)) and max(r["hi"]) == pytest.approx(max(roll)), "the lowest and the highest sample are on the chart"
    for k in (0, 37, 99):
        part = roll[k * 50:(k + 1) * 50]
        assert r["lo"][k] == pytest.approx(min(part)) and r["hi"][k] == pytest.approx(max(part)) and r["t"][k] == pytest.approx(t[k * 50])
    assert v["series"]["gyro[0]"]["unit"] == "deg/s"
    # a spike one sample wide survives any zoom
    roll[2345] = 9999.0
    wide = logview.view(s, ["roll"], px=64)["series"]["roll"]
    assert max(wide["hi"]) == 9999.0 and wide["hi"].index(9999.0) == 2345 * 64 // 5000
    # zoomed in far enough, the samples themselves are sent (with one beyond each edge so the line reaches the border)
    z = logview.view(s, ["roll"], t0=4.0, t1=4.1, px=400)
    r = z["series"]["roll"]
    assert "lo" not in r and len(r["t"]) == len(r["v"]) == 53 and r["t"][0] == pytest.approx(3.998) and r["t"][-1] == pytest.approx(4.102)
    assert r["v"][1] == pytest.approx(roll[1999]) and z["t0"] == 4.0 and z["t1"] == 4.1
    # slow fields have their own time base; their precision is the field's (latitude needs 7 digits)
    lat = logview.view(s, ["lat", "vbat"], px=1000)["series"]
    # (dt is the spacing the records really have - this handmade log has a slow record every 20 ms - not a nominal 10 Hz)
    assert len(lat["lat"]["v"]) == 500 and lat["lat"]["dt"] == pytest.approx(0.02) and lat["lat"]["v"][3] == pytest.approx(47.3977458, abs=1e-9)
    assert lat["vbat"]["v"][0] == pytest.approx(15.861) and lat["vbat"]["unit"] == "V"
    with pytest.raises(logview.LogError) as e:
        logview.view(s, ["nonsense"])
    assert e.value.code == "NO_SUCH_FIELD"
    assert logview.view(s, ["roll"], t0=50, t1=40, px=10)["series"]["roll"]["t"] != [], "a senseless range does not crash"


def test_library(tmp_path):
    lib = logview.Library(str(tmp_path / "not yet there"))
    assert lib.list() == []
    a = lib.save("my flight.wblog", _log_bytes(records=100))
    b = lib.save("my flight", _log_bytes(records=300) + _log_bytes(records=50))            # same name asked again; two sessions in one file
    c = lib.save("../../etc/passwd", _log_bytes())
    assert a == "my flight.wblog" and b == "my flight (2).wblog" and c == "passwd.wblog"
    assert sorted(os.listdir(lib.root)) == sorted([a, b, c]), "everything lands inside the folder, nothing half-written is left"
    rows = {r["file"]: r for r in lib.list()}
    assert rows[a]["sessions"] == 1 and rows[a]["records"] == 100 and rows[a]["firmware"] == "Handmade 9.9.9" and rows[a]["duration"] == pytest.approx(0.2, abs=0.01)
    assert rows[b]["sessions"] == 2 and rows[b]["records"] == 350 and rows[b]["size"] == os.path.getsize(os.path.join(lib.root, b))
    assert len(lib.session(b, 1).fast["t"]) == 50 and lib.sessions(b) is lib.sessions(b), "a parsed log is kept"
    for bad, code in ((("nope.wblog", 0), "NO_SUCH_LOG"), ((b, 2), "NO_SUCH_SESSION"), ((b, -1), "NO_SUCH_SESSION"), (("../" + a, 0), "BAD_NAME"),
                      ((os.path.join(lib.root, a), 0), "BAD_NAME"), ((".hidden.wblog", 0), "BAD_NAME"), (("x.txt", 0), "BAD_NAME")):
        with pytest.raises(logview.LogError) as e:
            lib.session(*bad)
        assert e.value.code == code, bad
    with pytest.raises(logview.LogError) as e:
        lib.save("holiday.wblog", b"\x89PNG not a log at all" * 50)
    assert e.value.code == "NOT_A_LOG" and "holiday.wblog" not in os.listdir(lib.root)
    # a file replaced on disk is read again
    assert len(lib.session(a, 0).fast["t"]) == 100
    with open(os.path.join(lib.root, a), "wb") as fp:
        fp.write(_log_bytes(records=20))
    os.utime(os.path.join(lib.root, a), (1, 1))
    assert len(lib.session(a, 0).fast["t"]) == 20 and {r["file"]: r for r in lib.list()}[a]["records"] == 20
    lib.delete(a)
    assert a not in os.listdir(lib.root) and sorted(r["file"] for r in lib.list()) == sorted([b, c])
    with pytest.raises(logview.LogError):
        lib.delete(a)


def test_recording_is_file_and_chart_at_once(tmp_path):
    data = _log_bytes(records=400)
    rec = logview.Recording(str(tmp_path / "sub" / "live.wblog"), "live.wblog")
    assert rec.session() is None and rec.status()["on"] and rec.status()["records"] == 0
    frames = list(F.Parser().feed(data))
    for k, (t, q, p) in enumerate(frames):
        rec.frame(t, q, p, F.encode(t, q, p))
        if k == len(frames) // 2:
            mid = rec.status()
            v = logview.view(rec.session(), ["roll"], px=50)
            assert 100 < mid["records"] < 300 and not mid["complete"] and v["span"][1] == pytest.approx(rec.session().fast["t"][-1])
    st = rec.status()
    assert st["records"] == 400 and st["complete"] and st["bytes"] == len(data) and st["duration"] == pytest.approx(0.798, abs=0.01)
    rec.close()
    rec.frame(*frames[-1], b"late")                                     # a frame after the stop is dropped, not written to a closed file
    assert not rec.status()["on"] and (tmp_path / "sub" / "live.wblog").read_bytes() == data


def test_holes_are_judged_by_the_spacing_the_log_really_has():
    """Firmware 0.5 on a real board: the loop ran at 1300 Hz instead of the configured 4000, so a "1000 records/s" log had a
    record every 3.08 ms. The chart broke its line at 3 x the NOMINAL step (3 ms): every sample stood alone - dots, no line."""
    slow_loop = wblog.parse(_log_bytes(records=2000, step_us=3080))[0]              # header says rate_hz 500 (2 ms), records are 3.08 ms apart
    assert slow_loop.rate_hz == 500 and slow_loop.sample_dt() == pytest.approx(0.00308, abs=1e-6)
    assert slow_loop.summary()["real_rate_hz"] == pytest.approx(324.7, abs=0.2) and slow_loop.summary()["rate_hz"] == 500
    v = logview.view(slow_loop, ["roll"], t0=1.0, t1=1.3, px=600)["series"]["roll"]
    steps = [b - a for a, b in zip(v["t"], v["t"][1:])]
    assert "v" in v and v["dt"] == pytest.approx(0.00308, abs=1e-6) and max(steps) < 3 * v["dt"], "no step counts as a hole: the chart draws a line"
    # a real hole (frames lost) is still far above 3 x the real spacing
    data = _log_bytes(records=2000, step_us=3080)
    holed = wblog.parse(data[:len(data) // 2] + data[len(data) // 2 + 4000:])[0]
    w = logview.view(holed, ["roll"], px=4000)["series"]["roll"]
    big = [b - a for a, b in zip(w["t"], w["t"][1:]) if b - a > 3 * w["dt"]]
    assert holed.gaps >= 1 and len(big) == 1 and w["dt"] == pytest.approx(0.00308, abs=1e-5), "one hole, where the frames are missing"
    # a log at its nominal rate is unchanged; a few records are too few to measure: the header's rate is used
    ok = wblog.parse(_log_bytes(records=2000))[0]
    assert ok.sample_dt() == pytest.approx(0.002) and ok.summary()["real_rate_hz"] == pytest.approx(500)
    tiny = wblog.parse(_log_bytes(records=5, step_us=7000))[0]
    assert tiny.sample_dt() == pytest.approx(0.002) and tiny.sample_dt(slow=True) == pytest.approx(0.1)
    # a recording that grows: the spacing is measured again as it does
    r = wblog.Reader(); data = _log_bytes(records=3000, step_us=4000)
    r.feed(data[:len(data) // 3]); a = r.sessions()[0].sample_dt()
    r.feed(data[len(data) // 3:]); b = r.sessions()[0].sample_dt()
    assert a == pytest.approx(0.004) and b == pytest.approx(0.004)
