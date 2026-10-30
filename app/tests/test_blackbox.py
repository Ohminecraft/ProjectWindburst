"""The flight log (blackbox): the Python reader against a log built by hand, then the whole chain with the real firmware in the
simulator - fly, store to (simulated) flash, download through the link, decode, compare with what the craft really did."""
import os
import struct
import subprocess
import sys
import time

import pytest

from wbclient import WbClient, WbError, Schema, wblog
from wbclient import framing as F

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.dirname(HERE)


# ---------------------------------------------------------------------------------------------------------------- reader
def _log_bytes(t0=1_000_000, records=50, damage=None, cut_tail=0, step_us=2000):
    """A log written here, byte by byte, from the format description - not by the firmware."""
    header = ("WBLOG 1\nsession 3\nto storage\nfirmware Handmade 9.9.9\nschema 0xDEADBEEF\nloop_hz 4000\nrate_div 8\nrate_hz 500\nmotors 2\n"
              "fast t:u32:1e-06:s gyro[0]:i16:0.1:deg/s motor[0]:u16:0.0001: motor[1]:u16:0.0001: roll:i16:0.01:deg\n"
              "slow t:u32:1e-06:s armed:u8:1: lat:i32:1e-07:deg vbat:u16:0.001:V\n"
              "events 1:ARM 2:DISARM 3:FAILSAFE 4:MODES 5:DROPPED 6:BATTERY 7:END 8:AH_STATE\n"
              "set pid " + "00" * 7 + "\nend\n").encode()
    out = bytearray()
    for i in range(0, len(header), 100):                       # the header in pieces, like the firmware cuts it
        out += F.encode(F.BB_HEADER, i // 100, header[i:i + 100])
    seq = 0
    pay = bytearray(b"E" + struct.pack("<IBB", t0 & 0xFFFFFFFF, 1, 0))                                    # ARM
    for k in range(records):
        t = (t0 + step_us * (k + 1)) & 0xFFFFFFFF
        rec = b"F" + struct.pack("<IhHHh", t, -1234 + k, 5000 + k, 10000 - k, (100 * k - 2000 + 32768) % 65536 - 32768)      # roll: a sawtooth in long logs
        if len(pay) + len(rec) > 240:
            out += F.encode(F.BB_DATA, seq, pay); seq += 1; pay = bytearray()
        pay += rec
        if k % 10 == 9:
            pay += b"S" + struct.pack("<IBiH", t, 1, 473977419 + k, 15870 - k)
        if k == 20:
            pay += b"E" + struct.pack("<IBBI", t, 5, 4, 17)                                              # DROPPED 17
    pay += b"E" + struct.pack("<IBBB", (t0 + step_us * records + 500) & 0xFFFFFFFF, 7, 1, 0)                 # END, trigger off
    out += F.encode(F.BB_DATA, seq, pay)
    if damage is not None:
        out[damage] ^= 0x40
    return bytes(out[:len(out) - cut_tail]) if cut_tail else bytes(out)


def test_reader_decodes_a_handmade_log():
    sessions = wblog.parse(_log_bytes())
    assert len(sessions) == 1
    s = sessions[0]
    assert s.meta["firmware"] == "Handmade 9.9.9" and s.meta["session"] == "3" and s.meta["to"] == "storage" and s.rate_hz == 500
    assert [f.name for f in s.fast_fields] == ["t", "gyro[0]", "motor[0]", "motor[1]", "roll"] and s.fast_fields[1].unit == "deg/s" and s.fast_fields[2].unit == ""
    assert s.names() == (["gyro[0]", "motor[0]", "motor[1]", "roll"], ["armed", "lat", "vbat"])
    assert len(s.fast["t"]) == 50 and s.fast["t"][0] == pytest.approx(0.002) and s.fast["t"][49] == pytest.approx(0.100) and s.duration == pytest.approx(0.098)
    assert s.fast["gyro[0]"][0] == pytest.approx(-123.4) and s.fast["gyro[0]"][49] == pytest.approx(-118.5)
    assert s.fast["motor[0]"][10] == pytest.approx(0.5010) and s.fast["motor[1]"][10] == pytest.approx(0.9990) and s.fast["roll"][30] == pytest.approx(10.0)
    assert len(s.slow["t"]) == 5 and s.slow["lat"][0] == pytest.approx(47.3977428, abs=1e-9) and s.slow["vbat"][4] == pytest.approx(15.821) and s.slow["armed"][0] == 1
    assert [e[1] for e in s.events] == ["ARM", "DROPPED", "END"] and s.events[0][0] == 0.0 and s.events[2][0] == pytest.approx(0.1005)
    assert s.dropped == 17 and s.complete and s.gaps == 0 and s.bad_records == 0
    col, t, f = s.field("vbat")
    assert f.unit == "V" and len(col) == len(t) == 5
    with pytest.raises(KeyError):
        s.field("nope")
    summ = s.summary()
    assert summ["records"] == 50 and summ["end"] == "trigger off" and summ["fields"][0] == "gyro[0]" and summ["complete"] is True
    # settings: only sections whose size matches the reader's schema are decoded (this log claims another firmware)
    assert s.settings(Schema()) == {} and not s.schema_matches(Schema())


def test_reader_survives_wrap_damage_and_a_cut_file():
    # the microsecond counter wraps in the middle of the session (71.6 minutes after power-on)
    s = wblog.parse(_log_bytes(t0=0xFFFFFFFF - 30_000))[0]
    t = s.fast["t"]
    assert len(t) == 50 and all(b - a == pytest.approx(0.002) for a, b in zip(t, t[1:])), "time keeps counting through the wrap"
    assert s.events[-1][0] == pytest.approx(0.1005) and s.slow["t"][-1] == pytest.approx(0.100)
    # one damaged byte in a data frame: that frame is lost and counted, the rest is read
    clean = _log_bytes(records=120)
    first_data = clean.index(F.encode(F.BB_DATA, 0, b"")[:2], 600)        # somewhere inside the data frames
    hurt = wblog.parse(_log_bytes(records=120, damage=first_data + 300))[0]
    assert hurt.gaps == 1 and 80 < len(hurt.fast["t"]) < 120 and hurt.complete, "one frame missing: %d of 120 records" % len(hurt.fast["t"])
    # a file cut in the middle of the last frame (power lost): everything before it is there, the session is marked as not closed
    cut = wblog.parse(_log_bytes(records=120, cut_tail=30))[0]
    assert not cut.complete and 80 < len(cut.fast["t"]) < 120 and cut.gaps == 0
    # two sessions in one file, with flash padding (0xFF) and noise between them
    both = wblog.parse(_log_bytes(records=10) + b"\xFF" * 3000 + b"\xA5\x5A\x00junk" + _log_bytes(records=20))
    assert [len(x.fast["t"]) for x in both] == [10, 20]
    assert wblog.parse(b"") == [] and wblog.parse(b"\xFF" * 5000) == []
    # an incremental reader (live stream) gives the same result as reading at once
    r = wblog.Reader()
    data = _log_bytes(records=60)
    for i in range(0, len(data), 37):
        r.feed(data[i:i + 37])
    assert len(r.sessions()[0].fast["t"]) == 60 and r.sessions()[0].complete


def test_reader_csv(tmp_path):
    s = wblog.parse(_log_bytes(records=3))[0]
    p = tmp_path / "x.csv"
    with open(p, "w") as fp:
        s.to_csv(fp)
    lines = p.read_text().splitlines()
    assert lines[0] == "t (s),gyro[0] (deg/s),motor[0],motor[1],roll (deg)" and len(lines) == 4
    assert lines[1].split(",")[0] == "0.002000" and float(lines[1].split(",")[1]) == pytest.approx(-123.4)


def test_csv_keeps_the_resolution():
    """0.9.0: "%g" cut a latitude to 6 digits (20.8634 = 11 m steps) - every value now has the decimals of its field's scale."""
    assert wblog._num(20.86341234 + 1e-9, 7) == "20.8634123" and wblog._num(105.8012345, 7) == "105.8012345"
    assert wblog._num(-123.4, 1) == "-123.4" and wblog._num(1500.0, 0) == "1500" and wblog._num(-0.00001, 2) == "0" and wblog._num(0.5, 3) == "0.5"


# ---------------------------------------------------------------------------------------------------------------- firmware
_ARMED_S = []                                             # how long the last _fly() kept the craft armed, by the simulator's clock


def _fly(sim, fc, seconds=4.0):
    """Arm, take off in Altitude Hold, a roll input, land by disarming. Returns the largest roll angle seen (from telemetry).
    How long it was armed (simulated seconds) goes to _ARMED_S: the telemetry waits below take wall-clock time while the simulator
    runs on, so the flight is longer on a slow computer."""
    fc.subscribe("attitude", 50)
    sim.do("rc 6 2000"); sim.do("rc 7 2000"); sim.sleep(0.3)
    t_arm = int(sim.state()["t_ms"])
    sim.do("rc 5 2000"); sim.sleep(0.5)
    sim.do("rc 3 1700"); sim.sleep(seconds * 0.5)
    sim.do("rc 3 1500"); sim.do("rc 1 1800")
    peak = 0.0
    for _ in range(25):                                   # the craft overshoots the commanded tilt before it settles: follow it
        sim.sleep(0.04); peak = max(peak, fc.wait_stream("attitude")["roll"])
    sim.do("rc 1 1500"); sim.sleep(seconds * 0.5 - 1.0)
    z = float(sim.state()["z"])
    sim.do("rc 3 1000"); sim.do("rc 5 1000")
    _ARMED_S[:] = [(int(sim.state()["t_ms"]) - t_arm) / 1000.0]
    sim.sleep(1.0)
    return peak, z


def test_flight_logged_to_flash_downloaded_and_decoded(sim, fc, tmp_path):
    st = fc.blackbox_status()
    assert st["state"] == "OFF" and st["capacity"] == 1024 * 1024 and st["sessions"] == 0
    assert fc.set("blackbox", device="FLASH", trigger="ARMED", rate_hz=500)["rate_hz"] == 500
    sim.sleep(0.5)
    st = fc.blackbox_status()
    assert st["state"] == "IDLE" and st["record_bytes"] == 74 and fc.blackbox_sessions() == []
    fc.subscribe("status", 10)
    assert "REBOOT_REQUIRED" not in fc.wait_stream("status")["settings_dirty"], "blackbox settings apply without a restart"
    peak_roll, z_top = _fly(sim, fc, 6.0)
    assert peak_roll > 8 and z_top > 0.5, "the simulated craft really flew (roll %.1f deg, %.2f m)" % (peak_roll, z_top)
    state = sim.state()
    assert int(state["bad_writes"]) == 0 and int(state["flash_writes"]) > 300 and int(state["stall_ms"]) > 100, "flash was written page by page, the loop stalled meanwhile: %s" % state
    st = fc.blackbox_status()
    assert st["state"] == "IDLE" and st["sessions"] == 1 and st["used"] > 100_000 and st["dropped"] == 0
    sessions = fc.blackbox_sessions()
    assert len(sessions) == 1 and sessions[0][0] == 0 and sessions[0][1] == st["used"]

    seen = []
    data = fc.blackbox_download(0, progress=lambda done, total: seen.append((done, total)))
    assert len(data) == sessions[0][1] and seen and seen[-1] == (len(data), len(data)) and all(a[0] <= b[0] for a, b in zip(seen, seen[1:]))
    (tmp_path / "flight.wblog").write_bytes(data)
    logs = wblog.load(str(tmp_path / "flight.wblog"))
    assert len(logs) == 1
    s = logs[0]
    assert s.complete and s.gaps == 0 and s.bad_records == 0 and s.dropped == 0
    assert s.meta["firmware"].startswith("Windburst RX") and s.meta["to"] == "storage" and s.meta["session"] == "1" and s.rate_hz == 500 and s.schema_matches(fc.schema)
    # the settings of that flight are in the log
    stored = s.settings(fc.schema)
    assert stored["pid"] == fc.get("pid") and stored["rates"] == fc.get("rates") and stored["blackbox"]["rate_hz"] == 500 and len(stored) == len(fc.schema.sections) - 2 and "wifi" not in stored and "osd" not in stored     # (every section but the WiFi one: its password stays out of the log)
    # what the records say
    t = s.fast["t"]
    assert 6.0 < s.duration and abs(s.duration - _ARMED_S[0]) < 0.3 and len(t) > 0.85 * 500 * s.duration, "%.2f s logged of %.2f s armed, %d records" % (s.duration, _ARMED_S[0], len(t))
    assert max(s.fast["roll"]) == pytest.approx(peak_roll, abs=4.0) and max(s.fast["stick[0]"]) == pytest.approx((0.6 - 0.02) / 0.98, abs=0.03)
    assert max(s.fast["setpoint[0]"]) > 20 and max(s.fast["gyro[0]"]) > 10 and abs(sum(s.fast["gyro[2]"]) / len(t)) < 3
    hover = [m for m, tt in zip(s.fast["motor[0]"], t) if 3.0 < tt < 4.0]
    assert 0.2 < sum(hover) / len(hover) < 0.8 and max(s.fast["throttle"]) > 0.3
    assert min(s.fast["gap_us"]) == 250 and 900 <= max(s.fast["gap_us"][100:]) <= 1400, "the stalls caused by flash writes are in the log itself"
    assert max(abs(v) for v in s.fast["pid_p[0]"]) > 20 and max(abs(v) for v in s.fast["pid_i[2]"]) < 400, "PID terms in PID sum units (a roll input makes tens of them), not clipped at 3.28"
    assert {"t_lock", "t_imu", "t_core", "t_est", "t_motor"} <= set(s.fast) and max(s.fast["t_core"]) == 0, "the stages of a cycle are logged (nothing takes time on the simulator)"
    assert max(s.slow["est_z"]) == pytest.approx(z_top, abs=1.0) and max(s.slow["armed"]) == 1 and s.slow["vbat"][5] == pytest.approx(15.7, abs=0.4)
    assert s.slow["lat"][-1] == pytest.approx(47.3977419, abs=3e-5) and len(s.slow["t"]) == pytest.approx(s.duration * 10, abs=4)
    names = [e[1] for e in s.events]
    assert names.count("ARM") == 1 and names.count("DISARM") == 1 and names[-1] == "END" and s.summary()["end"] == "trigger off"

    # a frame lost on the way is asked for again
    orig, dropped = fc._dispatch, []

    def lossy(msg_type, seq, payload):
        if msg_type == F.BB_BLOB and len(dropped) < 1 and struct.unpack_from("<I", payload)[0] > 20000:
            dropped.append(seq)
            return
        orig(msg_type, seq, payload)
    fc._dispatch = lossy
    again = fc.blackbox_read(0, len(data), stall_timeout=0.5)
    fc._dispatch = orig
    assert dropped and again == data, "the download recovered from a lost frame"

    # refused while armed; the log is still there after a power cycle
    sim.do("rc 3 1000"); sim.do("rc 5 2000"); sim.sleep(0.6)
    for cmd, args in (("BLACKBOX_ERASE", {}), ("BLACKBOX_READ", dict(offset=0, length=100))):
        with pytest.raises(WbError) as e:
            fc.command(cmd, **args)
        assert e.value.name == "ARMED"
    sim.do("rc 5 1000"); sim.sleep(0.8)
    assert fc.blackbox_status()["sessions"] == 2
    fc.save()
    fc.close()
    sim.power_cycle()
    with WbClient(sim.url, timeout=2.0) as fc2:
        sim.sleep(1.0)
        again = fc2.blackbox_download(0)                       # now followed by a second session: the size runs up to that one's sector, padded with 0xFF
        assert len(fc2.blackbox_sessions()) == 2 and again[:len(data)] == data and set(again[len(data):]) <= {0xFF}, "after a power cycle the same bytes come back"
        assert len(wblog.parse(again)) == 1 and len(wblog.parse(fc2.blackbox_download())) == 2, "one session by index, both when everything is fetched"
        assert any("blackbox: partition" in line and "2 sessions" in line for line in fc2.report())
        # erase
        pct = []
        st = fc2.blackbox_erase(progress=pct.append)
        assert st["state"] == "IDLE" and st["sessions"] == 0 and st["used"] == 0 and fc2.blackbox_sessions() == [] and fc2.blackbox_download() == b""
        assert int(sim.state()["bad_writes"]) == 0


def test_flight_logged_to_the_sd_card(sim, fc, tmp_path):
    """The card is chosen in the app (no restart), a flight goes into a file, the client downloads it like a flash log."""
    from wbclient import wblog
    assert fc.blackbox_status()["device"] == "NONE" and int(sim.state()["sd_writes"]) == 0
    fc.set("blackbox", device="SDCARD", trigger="ARMED", rate_hz=1000)
    sim.sleep(0.5)
    st = fc.blackbox_status()
    assert st["state"] == "IDLE" and st["device"] == "SDCARD" and st["capacity_kb"] == 64 * 1024 and st["sessions"] == 0, st
    fc.subscribe("status", 10)
    assert "REBOOT_REQUIRED" not in fc.wait_stream("status")["settings_dirty"]
    sim.do("sdcard slow 25")                                   # a card that freezes for 180 ms now and then
    peak_roll, z_top = _fly(sim, fc, 6.0)
    assert peak_roll > 8 and z_top > 0.5
    state = sim.state()
    assert int(state["stall_ms"]) == 0 and int(state["flash_writes"]) == 0 and int(state["sd_writes"]) > 50 and int(state["sd_files"]) == 1, \
        "the card took the log; the flight loop was never held up and the flash was not touched: %s" % state
    st = fc.blackbox_status()
    assert st["state"] == "IDLE" and st["sessions"] == 1 and st["dropped"] == 0 and st["used"] > 300_000 and st["used_kb"] == (st["used"] + 1023) // 1024
    sessions = fc.blackbox_sessions()
    assert sessions == [(0, st["used"])]
    raw = fc.blackbox_download(0)
    assert len(raw) == st["used"]
    ses = wblog.parse(raw)
    assert len(ses) == 1 and ses[0].complete and ses[0].dropped == 0 and not ses[0].gaps
    t = ses[0].fast["t"]
    steps = {round((b - a) * 1e6) for a, b in zip(t, t[1:])}
    assert steps == {1000} and len(t) > 5500, "1000 records a second, every one exactly 1 ms after the last (%d records, steps %s)" % (len(t), sorted(steps)[:5])
    assert max(ses[0].fast["roll"]) == pytest.approx(peak_roll, abs=4.0)

    # the card is pulled out: said in the status; pushed back: found again without a restart, the next flight is file 2
    sim.do("sdcard pull")
    _wait(lambda: fc.blackbox_status()["state"] == "NO_STORAGE", sim, 5.0, "a missing card is noticed")
    assert "NO_SENSOR" in _err(fc.blackbox_sessions) and "NO_SENSOR" in _err(lambda: fc.blackbox_download(0)), "no card: the firmware says there is no storage"
    sim.do("sdcard push")
    _wait(lambda: fc.blackbox_status()["state"] == "IDLE", sim, 5.0, "the card is found again")
    assert fc.blackbox_status()["sessions"] == 1
    sim.do("rc 5 2000"); sim.sleep(1.0); sim.do("rc 5 1000"); sim.sleep(1.0)
    assert fc.blackbox_status()["sessions"] == 2 and int(sim.state()["sd_files"]) == 2 and len(fc.blackbox_sessions()) == 2

    # erase removes the log files (in a moment: nothing is wiped)
    fc.blackbox_erase()
    st = fc.blackbox_status()
    assert st["state"] == "IDLE" and st["sessions"] == 0 and int(sim.state()["sd_files"]) == 0

    # the choice survives a power cycle when saved; the boot report then names the card
    fc.save()
    fc.close(); sim.power_cycle()
    from wbclient import WbClient
    with WbClient(sim.url, timeout=2.0) as fc2:
        sim.sleep(1.5)
        assert any("blackbox: SD card 'sim-sd' 64 MB" in line for line in fc2.report()), fc2.report()
        assert fc2.blackbox_status()["device"] == "SDCARD"


def _wait(cond, sim, seconds, what):
    for _ in range(int(seconds / 0.2)):
        if cond():
            return
        sim.sleep(0.2)
    raise AssertionError("timed out: " + what)


def _err(fn):
    try:
        fn()
    except Exception as e:  # noqa: BLE001
        return str(e)
    return ""


def test_live_stream_recording(sim, fc):
    sim.do("pose 15 -8 0")
    sim.sleep(0.5)
    data = fc.blackbox_record(0.75, rate_hz=200)              # 0.75 s of wall time = 3 s for the simulated craft (it runs 4x)
    logs = wblog.parse(data)
    assert len(logs) == 1
    s = logs[0]
    assert s.meta["to"] == "stream" and s.meta["session"] == "0" and s.rate_hz == 200 and s.complete and s.summary()["end"] == "stream stopped"
    assert 400 < len(s.fast["t"]) < 700 and s.bad_records == 0
    assert s.fast["roll"][-1] == pytest.approx(15, abs=1.5) and s.fast["pitch"][-1] == pytest.approx(-8, abs=1.5) and max(s.fast["motor[0]"]) == 0
    assert int(sim.state()["flash_writes"]) == 0 and fc.blackbox_status()["state"] == "OFF", "nothing was stored; with the device set to NONE the state is OFF again"
    # the stream stops by itself when the app goes away without saying so
    got = []
    fc.on_blackbox = lambda t, q, p: got.append(t)
    fc.blackbox_stream(True)
    sim.sleep(1.0)
    assert fc.blackbox_status()["state"] == "STREAMING" and got
    fc.keepalive = 0
    fc.unsubscribe()
    sim.sleep(4.0)
    n = len(got)
    sim.sleep(1.0)
    assert len(got) == n, "no more frames 3 s after the app fell silent"


def test_foreign_data_in_the_partition(sim, fc):
    fc.set("blackbox", device="FLASH", trigger="ARMED")
    fc.save()
    sim.do("logmem garbage")
    fc.close()
    sim.power_cycle()
    with WbClient(sim.url, timeout=2.0) as fc2:
        sim.sleep(1.5)
        assert fc2.blackbox_status()["state"] == "NEEDS_ERASE" and any("holds other data" in line for line in fc2.report())
        w0 = int(sim.state()["flash_writes"])
        sim.do("rc 5 2000"); sim.sleep(1.0); sim.do("rc 5 1000"); sim.sleep(0.5)
        assert int(sim.state()["flash_writes"]) == w0 and fc2.blackbox_status()["sessions"] == 0, "nothing is logged over foreign data"
        fc2.blackbox_erase()
        # the erase stalls the firmware long enough for the receiver link to count as lost for a moment; an arm switch
        # flipped at that very moment is refused (it has to be seen OFF on a live link first) - a pilot is not that quick
        sim.sleep(0.3)
        sim.do("rc 5 2000"); sim.sleep(1.0)
        assert fc2.blackbox_status()["state"] == "LOGGING"
        sim.do("rc 5 1000"); sim.sleep(0.6)
        assert fc2.blackbox_status()["sessions"] == 1 and int(sim.state()["bad_writes"]) == 0


def test_blackbox_cli(sim, fc, tmp_path):
    fc.set("blackbox", device="FLASH", trigger="ARMED", rate_hz=250)
    sim.do("rc 5 2000"); sim.sleep(2.0); sim.do("rc 5 1000"); sim.sleep(0.6)
    fc.close()

    def cli(*args):
        r = subprocess.run([sys.executable, "-m", "wbclient", "--port", sim.url] + list(args), cwd=APP, capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, r.stdout + r.stderr
        return r.stdout
    assert "state IDLE, 1 sessions" in cli("blackbox", "status")
    assert cli("blackbox", "list").split()[0] == "0"
    out = cli("blackbox", "download", "-o", str(tmp_path))
    files = [f for f in os.listdir(tmp_path) if f.endswith(".wblog")]
    assert len(files) == 1 and "session 0:" in out
    path = str(tmp_path / files[0])
    r = subprocess.run([sys.executable, "-m", "wbclient", "blackbox", "info", path], cwd=APP, capture_output=True, text=True, timeout=30)
    assert r.returncode == 0 and "session 0: Windburst RX" in r.stdout and "at 250 Hz" in r.stdout and " ARM" in r.stdout and " END" in r.stdout and "gyro[0]" in r.stdout, r.stdout + r.stderr
    r = subprocess.run([sys.executable, "-m", "wbclient", "blackbox", "csv", path, "-o", str(tmp_path / "f.csv")], cwd=APP, capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    rows = (tmp_path / "f.csv").read_text().splitlines()
    assert rows[0].startswith("t (s),gyro[0] (deg/s)") and 400 < len(rows) < 560
    rec = str(tmp_path / "live.wblog")
    out = cli("--seconds", "0.5", "--hz", "100", "blackbox", "record", rec)
    assert "session 0:" in out and "at 100 Hz" in out and os.path.getsize(rec) > 2000
    assert "erased" in cli("blackbox", "erase") and "0 sessions" in cli("blackbox", "status")
