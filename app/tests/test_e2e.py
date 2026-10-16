"""The Python client against the real firmware code (wb_sim): what the configurator app will do, done for real."""
import json
import os
import socket
import subprocess
import sys
import time

import pytest

from wbclient import WbClient, WbError, WbTimeout, SchemaMismatch, Schema
from wbclient import framing as F

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_hello_report_and_schema_guard(sim, fc):
    assert fc.info.role == "RX" and fc.info.protocol == 1 and fc.info.schema_hash == fc.schema.hash
    assert "CALIBRATION" in fc.info.features and "MOTOR_TEST" in fc.info.features
    rep = fc.report()
    assert any("imu: SPI ICM42688P" in l for l in rep) and any("rx: CRSF" in l for l in rep) and any("range: FAILED" in l for l in rep)
    # an app built for another schema must refuse to talk instead of mis-reading settings
    fc.close()                                            # one app at a time, like one USB cable
    other = Schema(); other.hash ^= 1
    c = WbClient(sim.url, schema=other)
    try:
        with pytest.raises(SchemaMismatch):
            c.open()
    finally:
        c.close()


def test_defaults_agree_with_the_firmware(tmp_path):
    """Firmware defaults (generated C++) == the defaults the client derives from the json, for every field of every section."""
    from conftest import Sim
    s = Sim(tmp_path, "--fresh")
    try:
        with WbClient(s.url, timeout=2.0) as fc:
            for name in fc.schema.sections:
                got, want = fc.get(name), fc.schema.layout("section", name).defaults()
                for k in want:
                    a, b = got[k], want[k]
                    if isinstance(b, list):
                        assert all(x == pytest.approx(y, rel=1e-6, abs=1e-9) if isinstance(y, float) else x == y for x, y in zip(a, b)), (name, k, a, b)
                    else:
                        assert a == pytest.approx(b, rel=1e-6, abs=1e-9) if isinstance(b, float) else a == b, (name, k, a, b)
            assert any("imu: FAILED" in l for l in fc.report())           # fresh board: no pins yet
    finally:
        s.stop()


def test_two_edits_of_one_section_at_the_same_time_both_stick(sim, fc):
    """The page sends a second change of a section while the first is still on its way (another thread of the local server).
    set() reads the section, changes it and writes it back: without doing that as one step, the second write carries the old
    value of the first field and silently undoes it."""
    import threading
    for k in range(12):
        ta = threading.Thread(target=lambda: fc.set("rates", throttle_mid=30 + k))
        tb = threading.Thread(target=lambda: fc.set("rates", throttle_expo=40 + k))
        ta.start(); tb.start(); ta.join(); tb.join()
        r = fc.get("rates")
        assert (r["throttle_mid"], r["throttle_expo"]) == (30 + k, 40 + k), "round %d: one of the two edits was lost: %s" % (k, r)


def test_settings_set_clamp_save_reboot(sim, fc):
    stored = fc.set("pid", slider_mode="OFF", p=[50, 60, 999], tpa_rate=40, anti_gravity=False, stab_p=[3.5, 4.0, 99.0])
    assert stored["p"] == [50, 60, 250] and stored["stab_p"][2] == pytest.approx(12.0), "out of range values are clamped and reported back"
    assert stored["tpa_rate"] == 40 and stored["anti_gravity"] is False and stored["slider_mode"] == "OFF"
    # the tuning sliders: the firmware computes the gains and answers with what is in effect
    sl = fc.set("pid", slider_mode="RP", slider_master=150, p=[1, 2, 3])
    assert sl["p"] == [67, 70, 3] and sl["i"][:2] == [120, 126] and sl["f"][:2] == [180, 187], "roll / pitch from the slider (1.5 x the defaults), yaw as typed"
    fl = fc.set("filters", slider_gyro_mult=80, gyro_lpf2_hz=123)
    assert fl["gyro_lpf1_dyn_min_hz"] == 200 and fl["gyro_lpf1_dyn_max_hz"] == 400 and fl["gyro_lpf2_hz"] == 400, "filter slider: cutoffs computed, a typed one is replaced"
    assert fc.set("filters", slider_gyro=False, gyro_lpf2_hz=123)["gyro_lpf2_hz"] == 123, "slider off: typed values stand"
    fc.subscribe("status", 20)
    assert fc.wait_stream("status")["settings_dirty"] == ["DIRTY"], "pid is a live section: unsaved, no reboot needed"
    assert fc.set("motors", protocol="dshot600", idle_x10=70)["protocol"] == "DSHOT600"
    sim.sleep(0.2)
    assert fc.wait_stream("status")["settings_dirty"] == ["DIRTY", "REBOOT_REQUIRED"]
    with pytest.raises(KeyError):
        fc.set("pid", no_such_field=1)
    with pytest.raises(ValueError):
        fc.set("motors", protocol="WARP")
    fc.set("modes", mode=["ARM", "ANGLE", "ALTHOLD", "BEEPER"] + [None] * 12, aux=[0, 1, 2, 3] + [0] * 12, lo=[1700] * 4 + [1000] * 12, hi=[2100] * 4 + [1000] * 12)
    fc.save()
    info = fc.reboot()
    assert info.role == "RX"
    sim.sleep(1.0)
    assert fc.get("pid")["p"] == [67, 70, 3] and fc.get("pid")["tpa_rate"] == 40 and fc.get("filters")["gyro_lpf2_hz"] == 123
    assert fc.get("motors")["protocol"] == "DSHOT600" and fc.get("motors")["idle_x10"] == 70
    assert fc.get("modes")["mode"][:5] == ["ARM", "ANGLE", "ALTHOLD", "BEEPER", None]
    fc.subscribe("status", 20)
    assert fc.wait_stream("status")["settings_dirty"] == []
    # really on the flash image: survives killing the process
    sim.power_cycle()
    fc.close(); fc.open()
    assert fc.get("pid")["tpa_rate"] == 40 and fc.get("pid")["slider_master"] == 150 and int(sim.state()["boots"]) == 1
    fc.defaults("pid")
    assert fc.get("pid")["tpa_rate"] == fc.schema.layout("section", "pid").defaults()["tpa_rate"] == 65 and fc.get("pid")["p"] == [45, 47, 45]
    assert fc.get("filters")["gyro_lpf2_hz"] == 123, "the defaults of one section leave the others alone"


def test_streams_show_the_craft(sim, fc):
    sim.do("pose 20 -10 0")                               # right side down 20, nose UP 10
    sim.sleep(3.0)
    fc.subscribe("attitude", 50); fc.subscribe("sensors", 50); fc.subscribe("rc", 20)
    a = fc.wait_stream("attitude")
    assert a["roll"] == pytest.approx(20, abs=0.6) and a["pitch"] == pytest.approx(-10, abs=0.6), "pilot convention: nose up is negative pitch"
    s = fc.wait_stream("sensors")
    assert 101200 < s["baro_pa"] < 101450 and s["temp"] == pytest.approx(25, abs=0.5)
    assert sum(x * x for x in s["acc"]) ** 0.5 == pytest.approx(1.0, abs=0.02)
    sim.do("rc 4 1234")
    sim.sleep(0.3)
    rc = fc.wait_stream("rc")
    assert abs(rc["ch"][3] - 1234) <= 1 and rc["link"] == 1 and rc["rssi"] == 100 and 120 < rc["rate"] < 160
    # stream rate, measured on the firmware's own clock
    stamps = []
    fc.subscribe("attitude", 50, callback=lambda v: stamps.append(v["t_ms"]))
    sim.sleep(2.5)
    fc.subscribe("attitude", 50, callback=lambda v: None)
    span = stamps[-1] - stamps[0]
    assert span >= 2000 and (len(stamps) - 1) * 1000.0 / span == pytest.approx(50, abs=1.0), "50 Hz stream: %d frames in %d firmware ms" % (len(stamps), span)
    # The frames keep to a 20 ms grid that does not drift. One frame may leave a millisecond or two late (at most four streams are sent
    # per link poll, and other streams are running), the next one is then back on the grid; a frame is dropped rather than queued.
    late = [(t - stamps[1]) % 20 for t in stamps[1:]]
    late = [x - 20 if x > 10 else x for x in late]                 # the reference frame itself may have been the late one
    assert max(late) - min(late) <= 2, "always within 2 ms of the firmware's 20 ms grid: offsets %s" % sorted(set(late))
    gaps = [b - a for a, b in zip(stamps[1:], stamps[2:])]
    assert sum(1 for g in gaps if g > 30) <= 2, "the firmware drops a frame rather than wait when the link is busy; on a quiet link that is rare: %s" % [g for g in gaps if g > 30]
    fc.unsubscribe()
    sim.sleep(0.3)
    with pytest.raises(WbTimeout):
        fc.wait_stream("attitude", 0.3)


def test_tuning_stream_shows_what_the_filters_do(sim, fc):
    fc.subscribe("tuning", 20)
    t = fc.wait_stream("tuning")
    assert t["gyro_lpf1_hz"] == 250 and t["dterm_lpf1_hz"] == 75 and t["tpa"] == 80 and t["air_state"] == "GROUND" and t["hover"] == 37, t
    assert len(t["dyn_notch_hz"]) == 9 and all(100 <= v <= 600 for v in t["dyn_notch_hz"]) and t["rpm_hz"] == [0] * 8 and 130 < t["rc_rate"] < 160, t
    # a frame resonance on pitch: one of the three pitch notches goes there, and stays when it is gone from roll's point of view
    sim.do("vib 2 280 15"); sim.sleep(1.5)
    t = fc.wait_stream("tuning")
    assert any(abs(v - 280) <= 10 for v in t["dyn_notch_hz"][3:6]), "pitch notches %s" % t["dyn_notch_hz"][3:6]
    sim.do("vib 2 0 0")
    # bidirectional DShot (a reboot setting): the ESCs' motor speeds become RPM notches
    fc.set("motors", bidirectional=True); fc.save(); fc.reboot(); sim.sleep(1.5)
    fc.subscribe("tuning", 20); fc.subscribe("status", 10)
    assert "ESC_TELEMETRY" in fc.wait_stream("status")["health"]
    sim.do("rc 3 1000"); sim.do("rc 6 2000"); sim.do("rc 5 2000"); sim.sleep(0.5); sim.do("rc 3 1550"); sim.sleep(2.0)
    t = fc.wait_stream("tuning")
    assert all(150 < v < 400 for v in t["rpm_hz"][:4]) and t["rpm_hz"][4:] == [0] * 4 and t["air_state"] == "AIR", t
    assert t["gyro_lpf1_hz"] > 300 and t["tpa"] < 100, "the dynamic low-pass and TPA follow the throttle: %s" % t
    sim.do("esc 3 0"); sim.sleep(0.5)
    t = fc.wait_stream("tuning")
    assert t["rpm_hz"][2] == 0 and all(v > 150 for v in (t["rpm_hz"][0], t["rpm_hz"][1], t["rpm_hz"][3])), "one ESC silent: only its notch goes (%s)" % t["rpm_hz"]
    assert "ESC_TELEMETRY" not in fc.wait_stream("status")["health"]
    sim.do("rc 5 1000")


def test_arming_locks_configuration_and_failsafe(sim, fc):
    fc.subscribe("status", 20)
    sim.do("rc 6 2000"); sim.do("rc 5 2000")              # ANGLE, ARM (the simulator's default mode setup)
    sim.sleep(0.6)
    st = fc.wait_stream("status")
    assert st["armed"] == 1 and "ARM" in st["modes"] and "ANGLE" in st["modes"]
    for refused in (lambda: fc.set("pid", tpa_rate=80), lambda: fc.set("filters", yaw_lowpass_hz=80), fc.save, lambda: fc.reboot(wait=False), lambda: fc.command("MOTOR_TEST", motor=0, value=100),
                    lambda: fc.command("GYRO_CAL"), lambda: fc.command("ACC_CAL_POINT"), lambda: fc.defaults()):
        with pytest.raises(WbError) as e:
            refused()
        assert e.value.name == "ARMED"
    assert fc.get("pid")["tpa_rate"] != 80 and fc.get("filters")["yaw_lowpass_hz"] != 80, "reading is always allowed"
    sim.do("tx off")
    sim.sleep(1.5)
    st = fc.wait_stream("status")
    assert "RX" not in st["health"] and st["failsafe"] in ("GUARD", "DESCENT", "LANDED")
    sim.do("tx on"); sim.do("rc 5 1000")
    sim.sleep(3.0)
    st = fc.wait_stream("status")
    assert st["armed"] == 0 and "RX" in st["health"]


def test_failsafe_says_why_and_hands_control_back(sim, fc):
    fc.subscribe("status", 20)
    sim.do("rc 6 2000"); sim.do("rc 7 2000"); sim.do("rc 5 2000"); sim.sleep(0.5)          # ANGLE, ALTHOLD, ARM
    sim.do("rc 3 1750"); sim.sleep(4.0); sim.do("rc 3 1500"); sim.sleep(2.0)
    z_top = float(sim.state()["z"])
    assert fc.wait_stream("status")["fs_cause"] == [] and z_top > 1.5
    sim.do("tx off"); sim.sleep(2.0)
    st = fc.wait_stream("status")
    assert st["failsafe"] == "DESCENT" and st["fs_cause"] == ["LINK"] and st["armed"] == 1, st
    sim.do("tx on"); sim.sleep(2.0)                        # back, and it stays: a second later the pilot has the craft again
    st = fc.wait_stream("status")
    z_back = float(sim.state()["z"])
    assert st["failsafe"] == "NONE" and st["fs_cause"] == [] and st["armed"] == 1 and 0.2 < z_back < z_top, (st, z_back)
    sim.sleep(1.5)
    assert abs(float(sim.state()["z"]) - z_back) < 0.3, "Altitude Hold holds where the descent had got to"
    sim.do("rc 3 1000"); sim.sleep(6.0); sim.do("rc 5 1000")


def test_runaway_takeoff_is_stopped_and_reported(sim, fc):
    assert fc.get("general")["runaway_takeoff"] is True
    sim.do("swap 1 2")                                     # two ESC plugs in each other's place
    sim.do("weak 3 0.94")                                  # no craft is perfectly even
    fc.subscribe("status", 20)
    sim.do("rc 3 1000"); sim.do("rc 5 2000"); sim.sleep(0.5)
    assert fc.wait_stream("status")["armed"] == 1
    sim.do("rc 3 1650"); sim.sleep(1.5)
    st = fc.wait_stream("status")
    assert st["armed"] == 0 and "RUNAWAY_TAKEOFF" in st["blockers"] and "ARM_SWITCH_ON" in st["blockers"], st
    assert all(float(sim.state()["m%d" % i]) == 0.0 for i in range(4)), "motors off"
    sim.do("rc 3 1000"); sim.do("rc 5 1000"); sim.do("pose 0 0 0"); sim.sleep(0.5)
    assert "RUNAWAY_TAKEOFF" not in fc.wait_stream("status")["blockers"], "cleared with the arm switch"


def test_motor_test(sim, fc):
    fc.subscribe("motors", 50); fc.subscribe("status", 20)
    assert fc.command("MOTOR_TEST", motor=2, value=150) == {}
    sim.sleep(0.2)
    m = fc.wait_stream("motors")
    assert m["cmd"][2] == pytest.approx(0.15, abs=0.002) and m["cmd"][0] == 0 and m["cmd"][1] == 0 and m["cmd"][3] == 0
    assert "CALIBRATING" in fc.wait_stream("status")["blockers"], "no arming while the app spins motors"
    sim.sleep(0.8)
    assert fc.wait_stream("motors")["cmd"][2] == 0, "stops by itself when the app stops refreshing"
    for bad in (dict(motor=4, value=100), dict(motor=0, value=1001)):
        with pytest.raises(WbError) as e:
            fc.command("MOTOR_TEST", **bad)
        assert e.value.name == "BAD_ARG"
    with pytest.raises(KeyError):
        fc.command("MOTOR_TEST", motor=0)                 # missing argument caught by the client
    with pytest.raises(WbError) as e:
        fc.command("MAG_CAL_START")
    assert e.value.name == "NO_SENSOR"


def test_battery(sim, fc):
    """The simulated pack is 16.0 V behind an 11:1 divider on the ADC pin."""
    fc.subscribe("battery", 10); fc.subscribe("status", 10)
    sim.sleep(1.0)
    b = fc.wait_stream("battery")
    assert b["voltage"] == pytest.approx(16.0, abs=0.06) and b["cells"] == 4 and b["cell"] == pytest.approx(4.0, abs=0.02)
    assert b["state"] == "OK" and b["source"] == "ADC" and b["pin_mv"] == pytest.approx(1455, abs=4)
    st = fc.wait_stream("status")
    assert st["cell_mv"] == pytest.approx(4000, abs=20) and "BATTERY" in st["health"] and "BATT_LOW" not in st["blockers"]
    # the divider calibration: what the user does when the reading is off (here: he had typed 10.0, the board has 11.0)
    assert fc.set("battery", divider=10.0)["divider"] == 10.0
    sim.sleep(3.0)
    assert fc.wait_stream("battery")["voltage"] == pytest.approx(16.0 * 10 / 11, abs=0.06)
    measured, shown = 16.0, fc.wait_stream("battery")["voltage"]
    fc.set("battery", divider=round(10.0 * measured / shown, 2))
    sim.sleep(3.0)
    assert fc.wait_stream("battery")["voltage"] == pytest.approx(16.0, abs=0.08) and "REBOOT_REQUIRED" not in fc.wait_stream("status")["settings_dirty"]
    # a pack at 3.45 V per cell: warning, no take-off
    sim.do("batt 13.8"); sim.sleep(3.5)
    assert fc.wait_stream("battery")["state"] == "WARNING" and "BATT_LOW" in fc.wait_stream("status")["blockers"]
    sim.do("rc 5 2000"); sim.sleep(0.5)
    assert fc.wait_stream("status")["armed"] == 0
    sim.do("rc 5 1000"); sim.do("batt 16.0"); sim.sleep(3.0)
    assert fc.wait_stream("battery")["state"] == "OK" and "BATT_LOW" not in fc.wait_stream("status")["blockers"]
    # USB power only
    sim.do("batt 0"); sim.sleep(5.0)
    b = fc.wait_stream("battery")
    assert b["state"] == "NOT_PRESENT" and b["cells"] == 0 and b["cell"] == 0 and fc.wait_stream("status")["cell_mv"] == 0
    assert fc.get("battery")["cell_warning_mv"] == 3500 and fc.set("battery", cell_warning_mv=9999)["cell_warning_mv"] == 4300, "clamped like every setting"


def test_beeper_and_the_bench_silence(sim, fc):
    def buzz():
        return int(sim.state()["buzz_ms"])
    fc.subscribe("battery", 10)
    sim.sleep(6.0)                                      # boot chirp, "gyro calibrated" and the "GPS ready" tune (fix after ~4 s) are over
    # the test button: one named sound, exactly as long as its pattern (ARMING = 300 + 50 ms on)
    t0 = buzz()
    assert fc.command("BEEP", condition="ARMING") == {}
    sim.sleep(0.15)
    assert fc.wait_stream("battery")["beeping"] == fc.schema.enums["beeper_cond"].index("ARMING") + 1, "telemetry names the sound that plays"
    sim.sleep(0.6)
    assert buzz() - t0 == pytest.approx(350, abs=8) and fc.wait_stream("battery")["beeping"] == 0
    # a sound that repeats is cut by "silence"
    fc.command("BEEP", condition="RX_LOST"); sim.sleep(0.2); fc.command("BEEP", condition=None); sim.sleep(0.1)
    t0 = buzz(); sim.sleep(1.0)
    assert buzz() == t0
    for bad in ("USB", 40):
        with pytest.raises(WbError) as e:
            fc.command("BEEP", condition=bad)
        assert e.value.name == "BAD_ARG"
    # with a battery connected the craft beeps when it is armed and disarmed, app or not
    t0 = buzz(); sim.do("rc 5 2000"); sim.sleep(1.2)
    assert buzz() - t0 > 300, "arming beep"
    sim.do("rc 5 1000"); sim.sleep(1.0)
    # on the bench (USB power, the app connected) it keeps quiet ...
    sim.do("batt 0"); sim.sleep(5.0)
    assert fc.wait_stream("battery")["state"] == "NOT_PRESENT"
    t0 = buzz(); sim.do("rc 5 2000"); sim.sleep(1.2); sim.do("rc 5 1000"); sim.sleep(1.0)
    assert buzz() == t0, "bench silence: the client's keepalive tells the firmware the app is there"
    # ... unless the user wants to hear it there too
    cond = fc.get("beeper")["conditions"]
    fc.set("beeper", conditions=cond + ["USB"])
    t0 = buzz(); sim.do("rc 5 2000"); sim.sleep(1.2)
    assert buzz() - t0 > 300
    sim.do("rc 5 1000"); sim.sleep(1.0)
    fc.set("beeper", conditions=cond)
    # the app goes away without closing the port (crashed, laptop asleep): after 3 s the board is on its own and beeps again
    fc.keepalive = 0
    sim.sleep(4.0)
    t0 = buzz(); sim.do("rc 5 2000"); sim.sleep(1.2)
    assert buzz() - t0 > 300, "no keepalive for 3 s: not a bench any more"
    sim.do("rc 5 1000")


def test_motors_as_the_beeper(sim, fc):
    """A craft without a buzzer: the ESCs beep the motors. `tones` counts the DShot beacon commands the simulated ESCs received."""
    def tones():
        return int(sim.state()["tones"])
    cond = fc.schema.enums["beeper_cond"]
    b = fc.get("beeper")
    assert "RX_LOST" in b["motor_conditions"] and "ARMING" not in b["motor_conditions"] and b["dshot_beacon_tone"] == 1
    fc.subscribe("battery", 20)
    sim.sleep(6.0)                                      # boot is over: "GPS ready" = 7 tones. ("Gyro calibrated", 3 tones, comes one second after
    assert tones() in (7, 10), "boot (%d tones)" % tones()   #  power-on; since 0.8.1 the bench rule no longer mutes the motors)
    t0 = tones()
    fc.command("BEEP", condition="GYRO_CALIBRATED")
    sim.sleep(0.2)
    assert fc.wait_stream("battery")["motor_beeping"] == cond.index("GYRO_CALIBRATED") + 1, "telemetry names what the motors play"
    sim.sleep(1.8)
    assert tones() - t0 == 3 and fc.wait_stream("battery")["motor_beeping"] == 0, "the test button: 3 tones"
    # a condition switched off for the motors only: the buzzer still beeps, the motors do not
    fc.set("beeper", motor_conditions=[c for c in b["motor_conditions"] if c != "ARM_REFUSED"])
    buzz0 = int(sim.state()["buzz_ms"]); t0 = tones()
    sim.do("rc 3 1600"); sim.sleep(0.3); sim.do("rc 5 2000"); sim.sleep(2.0); sim.do("rc 5 1000"); sim.do("rc 3 1000"); sim.sleep(0.5)
    assert tones() == t0 and int(sim.state()["buzz_ms"]) - buzz0 > 500, "arming refused: buzzer yes, motors no"
    # conditions a motor can never play are refused by the firmware even if a client sets them
    back = fc.set("beeper", motor_conditions=b["motor_conditions"] + ["ARMING", "ARMED"])
    t0 = tones(); sim.do("rc 5 2000"); sim.sleep(3.0)
    assert fc.wait_stream("battery")["motor_beeping"] == 0 and tones() == t0, "armed: never a beacon command"
    assert float(sim.state()["m0"]) > 0.03, "the motors are simply running"
    sim.do("rc 5 1000"); sim.sleep(3.0)
    assert tones() - t0 == 2, "after the disarm (and the 1.2 s guard): 2 tones (%d)" % (tones() - t0)
    assert "ARMING" in back["motor_conditions"], "the setting itself is stored as sent; the firmware masks it when it plays"
    # the CPU / RAM numbers of the status stream are there
    fc.subscribe("status", 5)
    st = fc.wait_stream("status")
    assert 0 <= st["cpu"] <= 100 and st["ram"] == 25 and st["heap_free"] == 240 and st["psram_total"] == 8192 and st["psram_free"] == 7680, st


def test_accelerometer_calibration_and_level_trim(sim, fc):
    sim.do("accerr 0.045 -0.030 0.080 1.012 0.991 1.020")
    sim.do("mount 3.0 -2.0")
    fc.subscribe("calibration", 20); fc.subscribe("sensors", 20); fc.subscribe("attitude", 20); fc.subscribe("status", 10)
    with pytest.raises(WbError) as e:
        fc.command("ACC_CAL_FINISH")
    assert e.value.name == "BAD_STATE"
    mask = 0
    for roll, nose in ((4, -3), (176, 5), (93, 2), (-87, -4), (3, 86), (-5, -94)):          # faces a few degrees off square
        sim.do("pose %d %d 0" % (roll, nose)); sim.sleep(0.3)
        fc.command("ACC_CAL_POINT")
        end = time.monotonic() + 10
        while True:
            c = fc.wait_stream("calibration")
            if c["kind"] == "ACCEL" and c["state"] in ("DONE", "FAILED"):
                break
            assert time.monotonic() < end
        assert c["state"] == "DONE" and c["mask"] != mask
        mask = c["mask"]
    assert mask == 0x3F
    sim.do("shake 1"); sim.sleep(0.2)
    fc.command("ACC_CAL_POINT"); sim.sleep(1.5)
    assert fc.wait_stream("calibration")["state"] == "FAILED", "a capture while the craft is moved is rejected"
    sim.do("shake 0")
    fc.command("ACC_CAL_FINISH")
    cal = fc.get("calibration")
    assert cal["acc_valid"] is True
    assert cal["acc_offset"] == pytest.approx([0.045, -0.030, 0.080], abs=0.004) and cal["acc_scale"] == pytest.approx([1.012, 0.991, 1.020], abs=0.004)
    st = fc.wait_stream("status")
    assert "DIRTY" in st["settings_dirty"] and "ACC_CAL" in st["health"]
    # level trim removes the crooked mounting
    sim.do("pose 0 0 0"); sim.sleep(4.0)
    a = fc.wait_stream("attitude")
    assert abs(a["roll"]) > 1.5 and abs(a["pitch"]) > 1.0
    fc.command("LEVEL_TRIM"); sim.sleep(4.0)
    a = fc.wait_stream("attitude")
    assert abs(a["roll"]) < 0.2 and abs(a["pitch"]) < 0.2
    sim.do("pose 30 0 0"); sim.sleep(3.0)
    with pytest.raises(WbError) as e:
        fc.command("LEVEL_TRIM")
    assert e.value.name == "FAILED"
    # both survive save + reboot
    sim.do("pose 0 0 0")
    fc.save(); fc.reboot(); sim.sleep(4.0)
    fc.subscribe("attitude", 20); fc.subscribe("status", 10)
    a = fc.wait_stream("attitude")
    assert abs(a["roll"]) < 0.2 and abs(a["pitch"]) < 0.2 and "ACC_CAL" in fc.wait_stream("status")["health"]


def test_compass_calibration(sim, fc):
    sim.do("magerr 0.12 -0.08 0.05")
    fc.set("sensors", mag_enabled=True)
    fc.save(); fc.reboot(); sim.sleep(1.5)
    assert any("mag: QMC5883L (not calibrated)" in l for l in fc.report())
    fc.subscribe("status", 10); fc.subscribe("calibration", 20)
    assert "MAG_NOT_CALIBRATED" in fc.wait_stream("status")["blockers"]
    fc.command("MAG_CAL_START")
    sim.sleep(0.3)
    with pytest.raises(WbError) as e:
        fc.command("MAG_CAL_FINISH")
    assert e.value.name == "FAILED" and e.value.result["reason"] in ("TOO_FEW_SAMPLES", "POOR_COVERAGE")
    for yaw in range(0, 360, 30):
        for roll in range(-180, 180, 30):
            for pitch in (-60, 0, 60):
                sim.do("pose %d %d %d" % (roll, pitch, yaw))
                sim.sleep(0.03)
    assert fc.wait_stream("calibration")["progress"] >= 80
    assert fc.command("MAG_CAL_FINISH", timeout=5.0) == {"reason": "OK"}
    cal = fc.get("calibration")
    assert cal["mag_valid"] is True and cal["mag_offset"] == pytest.approx([0.12, -0.08, 0.05], abs=0.02)
    sim.do("pose 0 0 135"); sim.sleep(8.0)
    fc.subscribe("attitude", 20); fc.subscribe("estimate", 10)
    assert fc.wait_stream("attitude")["yaw"] == pytest.approx(135, abs=3.0)
    assert "YAW_FROM_MAG" in fc.wait_stream("estimate")["valid"]
    st = fc.wait_stream("status")
    assert "MAG_CAL" in st["health"] and "MAG_NOT_CALIBRATED" not in st["blockers"]


def test_automatic_compass_calibration(sim, fc):
    sim.do("magerr 0.10 0.06 -0.04")
    fc.set("sensors", mag_enabled=True)
    fc.save(); fc.reboot(); sim.sleep(1.5)
    fc.subscribe("calibration", 20); fc.subscribe("status", 10)
    fc.command("MAG_AUTOCAL")
    c = fc.wait_stream("calibration")
    assert c["kind"] == "MAG_AUTO" and c["state"] == "RUNNING" and "CALIBRATING" in fc.wait_stream("status")["blockers"]
    grips = [(1.2, 0.9, 1.7), (-1.4, 1.1, 0.6), (0.8, -1.6, -1.0), (1.5, 0.4, -1.3)]
    for i in range(16):                                        # turned by hand, a new grip every 1.5 s, until it is done
        sim.do("tumble %g %g %g" % grips[i % 4]); sim.sleep(1.5)
        if fc.wait_stream("calibration")["state"] != "RUNNING":
            break
    sim.do("tumble off"); sim.sleep(0.5)
    c = fc.wait_stream("calibration")
    assert c["state"] == "DONE" and c["result"] == "OK" and c["progress"] == 100
    cal = fc.get("calibration")
    assert cal["mag_valid"] is True and cal["mag_offset"] == pytest.approx([0.10, 0.06, -0.04], abs=0.02)
    sim.sleep(0.5); assert "DIRTY" not in fc.wait_stream("status")["settings_dirty"], "saved to flash by the firmware itself"


def test_line_noise_and_a_stalled_app(sim, fc):
    # garbage and half a frame on the wire (a second program poking the port, a glitching cable)
    raw = fc._ser
    raw.write(b"\x00\xff garbage \xA5\x5A\x40\x00\x30\x07 half a frame")
    time.sleep(0.2)
    assert fc.hello().role == "RX"
    assert fc.get("rates")["max_rate"][0] > 0
    # an app that subscribes to everything and then stops reading must not hurt the firmware
    fc.subscribe("gyro_fast", 1000); fc.subscribe("attitude", 200); fc.subscribe("sensors", 200)
    sim.sleep(1.0)
    assert fc.stats["streams"] > 500 and fc.stats["errors"] == 0
    fc.unsubscribe()
    assert int(sim.state()["boots"]) == 1


def test_gps_and_home(sim, fc):
    fc.subscribe("gps", 5)
    with pytest.raises(WbError) as e:
        fc.command("SET_HOME")
    assert e.value.name == "NO_SENSOR"
    sim.sleep(6.0)
    g = fc.wait_stream("gps")
    assert g["fix"] == 3 and g["sats"] == 14 and g["state"] == "RECEIVING_DATA" and g["lat"] == pytest.approx(47.3977419, abs=1e-6) and g["lon"] == pytest.approx(8.5455938, abs=1e-6)
    assert fc.command("SET_HOME") == {}


def _wait(sim, cond, sim_seconds, step=0.25):
    """Let simulated time pass until cond() is true (returns its value) or the time is up."""
    t = 0.0
    while t < sim_seconds:
        v = cond()
        if v:
            return v
        sim.sleep(step)
        t += step
    return cond()


def _dist(sim, n0=0.0, e0=0.0):
    st = sim.state()
    return ((float(st["pn"]) - n0) ** 2 + (float(st["pe"]) - e0) ** 2) ** 0.5


def test_optical_flow_sensor_and_position_hold(sim, fc):
    """An MSP flow sensor (with its lidar) from "not set up" to a craft that holds its position on it."""
    fc.subscribe("nav", 20); fc.subscribe("estimate", 20); fc.subscribe("status", 20)
    assert fc.get("flow")["type"] == "NONE" and not any(l.startswith("flow:") for l in fc.report())
    n = fc.wait_stream("nav")
    assert n["flow_rate"] == 0 and n["flow_reject"] == "NO_SAMPLE" and n["flags"] == [] and n["rescue_phase"] == "IDLE"
    assert fc.set("flow", type="MSP")["type"] == "MSP"
    sim.sleep(0.3)
    assert "REBOOT_REQUIRED" in fc.wait_stream("status")["settings_dirty"], "the sensor type is read at power-up"
    m = fc.get("modes"); m["mode"][3] = "POSHOLD"; m["aux"][3] = 3; m["lo"][3] = 1700; m["hi"][3] = 2100
    fc.set("modes", m)
    fc.save(); fc.reboot(); sim.sleep(2.5)
    rep = fc.report()
    assert any("flow: MSP flow on UART0" in l for l in rep) and any("range: MSP lidar (flow sensor)" in l for l in rep), rep
    fc.subscribe("nav", 20); fc.subscribe("estimate", 20); fc.subscribe("status", 20)
    n = _wait(sim, lambda: (lambda v: v if v["flow_rate"] >= 95 else None)(fc.wait_stream("nav")), 3.0)
    assert n and 95 <= n["flow_rate"] <= 101 and n["flow_quality"] == 78 and n["flow_reject"] == "HEIGHT", n
    st = fc.wait_stream("status")
    assert "FLOW" in st["health"] and "RANGE" in st["health"] and st["blockers"] == []
    # carried by hand: the speed the app shows for checking how the sensor is mounted
    sim.do("carry 1.0 0.6 0"); sim.sleep(2.0)
    n, e = fc.wait_stream("nav"), fc.wait_stream("estimate")
    assert n["flow_reject"] == "OK" and n["flow_fwd"] == pytest.approx(0.6, abs=0.08) and n["flow_right"] == pytest.approx(0.0, abs=0.08), n
    assert "VEL" in e["valid"] and "FLOW" in e["valid"] and "POS" not in e["valid"] and e["vel_n"] == pytest.approx(0.6, abs=0.12), e
    # the sensor sits turned a quarter to the right: wrong until the (live) nav section says so
    sim.do("flow mount 90 0 1"); sim.sleep(1.5)
    n = fc.wait_stream("nav")
    assert abs(n["flow_fwd"]) < 0.1 and abs(n["flow_right"]) == pytest.approx(0.6, abs=0.08), n
    assert fc.set("nav", flow_align="CW90")["flow_align"] == "CW90"
    sim.sleep(1.5)
    n = fc.wait_stream("nav")
    assert n["flow_fwd"] == pytest.approx(0.6, abs=0.08) and abs(n["flow_right"]) < 0.08, n
    assert "REBOOT_REQUIRED" not in fc.wait_stream("status")["settings_dirty"]
    sim.do("carry off"); sim.sleep(1.5)
    # flight: Angle + Altitude Hold + Position Hold, then a wind
    sim.do("rc 6 2000"); sim.do("rc 7 2000"); sim.do("rc 8 2000"); sim.sleep(0.3); sim.do("rc 5 2000"); sim.sleep(0.5)
    assert fc.wait_stream("status")["armed"] == 1, fc.wait_stream("status")
    sim.do("rc 3 1700"); sim.sleep(3.0); sim.do("rc 3 1500"); sim.sleep(3.0)
    n, st, e = fc.wait_stream("nav"), fc.wait_stream("status"), fc.wait_stream("estimate")
    assert "POSHOLD" in n["flags"] and "NAV_LEAN" in n["flags"] and n["ph_roll"] == "LOITER" and n["ph_pitch"] == "LOITER" and "POSHOLD" in st["modes"], (n, st)
    assert "TILT_AIDED" in e["valid"] and float(sim.state()["z"]) > 0.6
    n0, e0 = float(sim.state()["pn"]), float(sim.state()["pe"])
    sim.do("wind 0 4"); far = 0.0
    for _ in range(24):
        sim.sleep(0.5); far = max(far, _dist(sim, n0, e0))
    n = fc.wait_stream("nav")
    assert 0.1 < far < 1.5 and _dist(sim, n0, e0) < 0.4 and n["lean_roll"] < -4 and n["wind_e"] < -0.8, "blown %.2f m at most, %.2f m off after 12 s, leaning %.1f deg into the wind" % (far, _dist(sim, n0, e0), n["lean_roll"])
    with pytest.raises(WbError) as err:
        fc.set("nav", ph_angle_max=20)
    assert err.value.name == "ARMED"
    # the floor gives the camera nothing
    sim.do("wind 0 0"); sim.sleep(4.0); sim.do("flow quality 20"); sim.sleep(1.0)
    n = fc.wait_stream("nav")
    assert n["flow_reject"] == "QUALITY" and "POSHOLD_UNAVAILABLE" in n["flags"] and "POSHOLD" not in n["flags"], n
    sim.do("flow quality 200"); sim.sleep(2.0)
    assert "POSHOLD" in fc.wait_stream("nav")["flags"]
    sim.do("rc 3 1300"); sim.sleep(6.0); sim.do("rc 3 1000"); sim.sleep(1.0); sim.do("rc 5 1000"); sim.sleep(0.5)
    assert fc.wait_stream("status")["armed"] == 0


def test_gps_rescue_brings_the_craft_home(sim, fc):
    """Failsafe procedure RESCUE with GPS + compass: arming rules, the flight home after a lost link, what the nav stream says."""
    fc.set("sensors", mag_enabled=True)
    fc.set("calibration", mag_valid=1, mag_radius=0.48)
    nav = fc.set("nav", fs_procedure="RESCUE", rescue_initial_climb=5, rescue_min_start_dist=5, rescue_descent_dist=10)
    assert nav["fs_procedure"] == "RESCUE" and nav["rescue_ground_speed"] == pytest.approx(7.5)
    fc.save(); fc.reboot(); sim.sleep(1.5)
    fc.subscribe("nav", 20); fc.subscribe("status", 20); fc.subscribe("estimate", 10)
    st = fc.wait_stream("status")
    assert "RESCUE_NO_FIX" in st["blockers"], "a rescue is set up and the GPS has no fix yet: %s" % st["blockers"]
    st = _wait(sim, lambda: (lambda v: v if v["blockers"] == [] else None)(fc.wait_stream("status")), 8.0)
    assert st and "GPS" in st["health"] and "MAG_READY" in st["health"], fc.wait_stream("status")
    # out: take off in Altitude Hold, fly north
    sim.do("rc 6 2000"); sim.do("rc 7 2000"); sim.sleep(0.3); sim.do("rc 5 2000"); sim.sleep(0.5)
    sim.do("rc 3 1800"); sim.sleep(6.0); sim.do("rc 3 1500"); sim.sleep(2.0)
    sim.do("rc 2 1800"); sim.sleep(6.0); sim.do("rc 2 1500"); sim.sleep(3.0)
    out, z0 = _dist(sim), float(sim.state()["z"])
    n, e = fc.wait_stream("nav"), fc.wait_stream("estimate")
    assert out > 30 and z0 > 3 and "RESCUE_AVAILABLE" in n["flags"] and "RESCUE" not in n["flags"] and n["home_dist"] == pytest.approx(out, abs=2.5), (out, z0, n)
    assert "POS" in e["valid"] and "YAW_FROM_MAG" in e["valid"] and (e["pos_n"] ** 2 + e["pos_e"] ** 2) ** 0.5 == pytest.approx(out, abs=2.5)
    sim.do("tx off"); sim.sleep(1.5)
    n, st = fc.wait_stream("nav"), fc.wait_stream("status")
    assert st["failsafe"] == "DESCENT" and st["fs_cause"] == ["LINK"] and "RESCUE" in n["flags"] and n["rescue_phase"] == "CLIMB" and n["return_alt"] == pytest.approx(z0 + 5, abs=1.0), (st, n)
    seen = []
    def phase():
        p = fc.wait_stream("nav")["rescue_phase"]
        if not seen or seen[-1] != p:
            seen.append(p)
        return fc.wait_stream("status")["armed"] == 0
    assert _wait(sim, phase, 60.0, step=0.2), "still armed; phases %s, %s" % (seen, sim.state())
    assert seen[:5] == ["CLIMB", "ROTATE", "FLY_HOME", "DESCENT", "LANDING"], seen
    assert _dist(sim) < 2.0 and float(sim.state()["z"]) < 0.05, "landed %.1f m from the take-off point" % _dist(sim)
    sim.do("tx on"); sim.do("rc 5 1000"); sim.sleep(1.0)
    # back to LAND, with the craft switched on: the rescue arming rules go with it
    sim.do("gps fix 0"); sim.sleep(2.5)
    assert "RESCUE_NO_FIX" in fc.wait_stream("status")["blockers"]
    fc.set("nav", fs_procedure="LAND"); sim.sleep(0.5)
    st = fc.wait_stream("status")
    assert "RESCUE_NO_FIX" not in st["blockers"] and "REBOOT_REQUIRED" not in st["settings_dirty"], st


def test_cli(sim, fc, tmp_path):
    fc.close()
    env = dict(os.environ, PYTHONPATH=APP)
    run = lambda *a: subprocess.run([sys.executable, "-m", "wbclient", "--port", sim.url] + list(a), capture_output=True, text=True, env=env, cwd=APP, timeout=60)
    r = run("info"); assert r.returncode == 0 and "Windburst RX" in r.stdout and "imu: SPI ICM42688P" in r.stdout
    r = run("set", "pid", "slider_mode=OFF", "p=50,50,999", "tpa_rate=55"); assert r.returncode == 0 and "the firmware limited it" in r.stdout and "pid.tpa_rate = 55" in r.stdout
    r = run("get", "pid"); assert json.loads(r.stdout)["tpa_rate"] == 55 and json.loads(r.stdout)["p"] == [50, 50, 250]
    bk = str(tmp_path / "bk.json")
    assert run("backup", bk).returncode == 0 and json.load(open(bk))["settings"]["pid"]["tpa_rate"] == 55
    assert run("defaults", "pid").returncode == 0 and json.loads(run("get", "pid").stdout)["tpa_rate"] != 55
    assert run("restore", bk).returncode == 0 and json.loads(run("get", "pid").stdout)["tpa_rate"] == 55
    r = run("cmd", "MOTOR_TEST", "motor=9", "value=10"); assert r.returncode == 1 and "BAD_ARG" in r.stderr
    r = run("motor", "0", "10"); assert r.returncode != 0 and "props-off" in (r.stderr + r.stdout)
    r = run("monitor", "attitude,status", "--seconds", "0.6"); assert r.returncode == 0 and "attitude: roll=" in r.stdout and "status: armed=0" in r.stdout
    r = run("monitor", "nav,estimate", "--seconds", "0.6"); assert r.returncode == 0 and "nav: flags=" in r.stdout and "rescue_phase=IDLE" in r.stdout and "estimate: z=" in r.stdout, r.stdout
    r = run("calibrate", "gyro"); assert r.returncode == 0 and "DONE" in r.stdout
    assert run("save").returncode == 0 and "restarted" in run("reboot").stdout
    r = subprocess.run([sys.executable, "-m", "wbclient", "schema"], capture_output=True, text=True, env=env, cwd=APP)
    assert r.returncode == 0 and "MOTOR_TEST" in r.stdout and "calibration" in r.stdout
