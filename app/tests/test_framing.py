import random
import re
import os

from wbclient import framing as F
from wbclient.schema import Schema

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_crc_known_vector():
    assert F.crc16(b"123456789") == 0x29B1           # CRC-16/CCITT-FALSE check value


def test_round_trip_all_sizes():
    p = F.Parser()
    for n in (0, 1, 2, 17, 255, 256, 511, 512):
        payload = bytes((i * 7 + n) & 0xFF for i in range(n))
        assert p.feed(F.encode(0x21, n & 0xFF, payload)) == [(0x21, n & 0xFF, payload)]
    assert p.errors == 0


def test_byte_by_byte_and_noise():
    p = F.Parser()
    f = F.encode(0x11, 9, b"\xA5\x5A\xA5\x5A hello")            # sync bytes inside the payload
    got = []
    for b in b"boot: rst:0x1 (POWERON)\r\n\xA5" + f + b"\xA5\xA5" + f:
        got += p.feed(bytes([b]))
    assert got == [(0x11, 9, b"\xA5\x5A\xA5\x5A hello")] * 2


def test_false_header_does_not_hide_real_frames():
    p = F.Parser()
    f1, f2 = F.encode(1, 1, b"a"), F.encode(1, 2, b"bb")
    assert p.feed(b"\xA5\x5A\x20\x00\x01\x00" + f1 + f2) == []   # header announces 32 payload bytes that never come
    assert p.mid_frame
    assert p.idle() == [(1, 1, b"a"), (1, 2, b"bb")]            # ...released by the idle timeout


def test_corruption_is_detected():
    rng = random.Random(1)
    good = F.encode(0x12, 3, bytes(range(40)))
    for _ in range(300):
        bad = bytearray(good)
        bad[rng.randrange(2, len(bad))] ^= 1 << rng.randrange(8)
        p = F.Parser()
        assert p.feed(bytes(bad)) + p.idle() + p.idle() == [] or bad == good


def test_random_garbage_never_yields_frames():
    rng = random.Random(2)
    p = F.Parser()
    got = p.feed(bytes(rng.randrange(256) for _ in range(200000)))
    assert got == []
    assert p.feed(F.encode(1, 1, b"x")) + p.idle() + p.idle() + p.idle() in ([(1, 1, b"x")], [])   # at most one frame lost after garbage
    assert p.feed(F.encode(1, 2, b"y")) == [(1, 2, b"y")]


def test_too_long_payload_refused():
    try:
        F.encode(1, 1, bytes(513))
        assert False
    except ValueError:
        pass


def test_schema_matches_generated_header():
    """The json the client reads and the C++ header the firmware is built from describe the same bytes."""
    s = Schema()
    hdr = open(os.path.join(ROOT, "src", "proto", "wbp_schema.h"), encoding="utf-8").read()
    assert int(re.search(r"kSchemaHash = (0x[0-9A-Fa-f]+)u", hdr).group(1), 16) == s.hash
    assert s.computed_hash() == s.hash, "protocol/wbp_schema.json was edited without running tools/gen_proto.py"
    sizes = dict(re.findall(r"static_assert\(sizeof\(([ST]_\w+)\) == (\d+)", hdr))
    for name in s.sections:
        assert int(sizes["S_" + name]) == s.layout("section", name).size
    for name in s.streams:
        assert int(sizes["T_" + name]) == s.layout("stream", name).size
    for msg, val in re.findall(r"MSG_(\w+) = (0x[0-9A-Fa-f]+)", open(os.path.join(ROOT, "src", "proto", "wbp_codec.h"), encoding="utf-8").read()):
        assert getattr(F, {"SECTION_GET": "SECTION_GET"}.get(msg, msg)) == int(val, 16)


def test_layout_pack_unpack():
    s = Schema()
    lay = s.layout("section", "modes")
    d = lay.defaults()
    assert d["mode"] == [None] * 16
    d["mode"][0] = "ARM"; d["mode"][1] = "althold"; d["aux"][1] = 2; d["lo"][1] = 1700; d["hi"][1] = 2100
    back = lay.unpack(lay.pack(d))
    assert back["mode"][:3] == ["ARM", "ALTHOLD", None] and back["hi"][1] == 2100
    g = s.layout("section", "general")
    v = g.defaults(); v["board_trim_roll"] = -2.5; v["board_align"] = "CW90_FLIP"; v["airmode"] = "OFF"
    raw = g.pack(v)
    assert g.unpack(raw)["board_trim_roll"] == -2.5 and g.unpack(raw)["board_align"] == "CW90_FLIP" and g.unpack(raw)["airmode"] == "OFF"
    try:
        v["board_align"] = "SIDEWAYS"; g.pack(v); assert False
    except ValueError:
        pass
    st = s.layout("stream", "status")
    import struct
    blob = struct.pack(st.fmt, 1, 0xB004, 2, 0x0B, 0x83, 4, 80, 120, 310, 3987, 3900, 3, 456, 37, 25, 240, 7680, 8192, 31, 62, 13, 9, 412, 17, 4000, 0x05, 1995, 2000)
    u = st.unpack(blob)
    assert u["cpu"] == 37 and u["ram"] == 25 and u["heap_free"] == 240 and u["psram_free"] == 7680 and u["psram_total"] == 8192
    assert (u["t_imu"], u["t_core"], u["t_motor"], u["t_rest"], u["t_slow_max"], u["overruns"], u["loop_target"]) == (31, 62, 13, 9, 412, 17, 4000)
    assert (u["pid_hz"], u["pid_target"]) == (1995, 2000)
    assert "LOOP_RATE" in s.bits["arm_blocker"] and "dshot_driver" not in s.enums and s.enums["bidir_state"] == ["OFF", "ACTIVE", "UNAVAILABLE"]
    assert u["blockers"] == ["RX_LOST", "MAG_NOT_CALIBRATED", "PIN_CONFLICT", "RUNAWAY_TAKEOFF"] and u["failsafe"] == "DESCENT" and u["modes"] == ["ARM", "ANGLE", "ALTHOLD"]
    assert u["fs_cause"] == ["LINK", "BATTERY"] and s.bits["fs_cause"] == ["LINK", "SWITCH", "BATTERY"]
    assert u["health"] == ["IMU", "GYRO_CAL", "RX"] and u["ah_state"] == "FLYING" and u["settings_dirty"] == ["DIRTY", "REBOOT_REQUIRED"] and abs(u["throttle"] - 0.456) < 1e-9
    assert u["loop_us"] == 80 and u["loop_max_us"] == 120 and u["gap_max_us"] == 310 and u["loop_hz"] == 3987 and u["cell_mv"] == 3900
    # a bit field stored in the settings, an optional enum as a command argument, scaled settings
    bp = s.layout("section", "beeper")
    d = bp.defaults()
    assert "ARMING" in d["conditions"] and "USB" not in d["conditions"] and len(d["conditions"]) == 17 and d["type"] == "ACTIVE"
    assert "RX_LOST" in d["motor_conditions"] and "ARMING" not in d["motor_conditions"] and "SYSTEM_INIT" not in d["motor_conditions"] and len(d["motor_conditions"]) == 11
    d["conditions"] = ["BAT_LOW", "usb"]
    assert bp.unpack(bp.pack(d))["conditions"] == ["BAT_LOW", "USB"]
    args = s.layout("args", "BEEP")
    assert args.pack({"condition": "ARMING"}) == bytes([4]) and args.pack({"condition": None}) == bytes([255]) and args.unpack(bytes([255]))["condition"] is None
    bt = s.layout("section", "battery")
    d = bt.defaults()
    assert d["divider"] == 11.0 and d["warning_delay"] == 0.5 and d["critical_delay"] == 2.0 and d["source"] == "AUTO" and d["cells"] == 0
    d["divider"] = 21.37
    assert abs(bt.unpack(bt.pack(d))["divider"] - 21.37) < 1e-9
