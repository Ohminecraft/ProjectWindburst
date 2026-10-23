"""The GUI's local server (wbclient/gui.py) against the real firmware in wb_sim: what the browser page relies on."""
import http.client
import json
import threading
import time

import pytest

from wbclient.gui import GuiServer


class Api:
    def __init__(self, srv):
        self.srv = srv
        self.port = srv.server_address[1]
        self.host = "127.0.0.1:%d" % self.port

    def raw(self, method, path, body=None, headers=None, token=True, timeout=20):
        h = {"Host": self.host}
        if token:
            h["X-WB-Token"] = self.srv.token
        data = None
        if body is not None:
            data = json.dumps(body).encode()
            h["Content-Type"] = "application/json"
        h.update(headers or {})
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=timeout)
        c.request(method, path, body=data, headers=h)
        r = c.getresponse()
        raw = r.read()
        c.close()
        try:
            doc = json.loads(raw.decode("utf-8"))
        except ValueError:
            doc = raw
        return r.status, doc

    def get(self, path):
        st, doc = self.raw("GET", path)
        assert st == 200, (path, st, doc)
        return doc

    def post(self, path, body=None):
        st, doc = self.raw("POST", path, body or {})
        assert st == 200, (path, st, doc)
        return doc

    def events(self, seconds):
        """Collect the server-sent events of `seconds` of wall time."""
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=seconds + 5)
        c.request("GET", "/api/events?t=" + self.srv.token, headers={"Host": self.host})
        r = c.getresponse()
        assert r.status == 200 and r.getheader("Content-Type").startswith("text/event-stream")
        out, end = [], time.monotonic() + seconds
        while time.monotonic() < end:
            line = r.fp.readline()
            if not line:
                break
            if line.startswith(b"data: "):
                out.append(json.loads(line[6:].decode("utf-8")))
        c.close()
        return out


@pytest.fixture
def gui():
    srv = GuiServer(http_port=0).start()
    yield Api(srv)
    srv.stop()


@pytest.fixture
def linked(gui, sim):
    st = gui.post("/api/connect", {"port": sim.url})
    assert st["connected"]
    sim.sleep(1.5)                       # boot gyro calibration
    return gui


def test_only_the_served_page_may_use_the_api(gui):
    st, page = gui.raw("GET", "/", token=False)
    assert st == 200 and gui.srv.token.encode() in page, "the page carries the token"
    assert gui.raw("GET", "/api/state", token=False)[0] == 403, "no token"
    assert gui.raw("GET", "/api/state", headers={"X-WB-Token": "x" * 24}, token=False)[0] == 403, "wrong token"
    assert gui.raw("GET", "/api/state", headers={"Host": "evil.example:%d" % gui.port})[0] == 403, "DNS rebinding: foreign Host"
    assert gui.raw("GET", "/api/state", headers={"Origin": "https://evil.example"})[0] == 403, "cross-site request"
    assert gui.raw("GET", "/", headers={"Host": "evil.example"}, token=False)[0] == 403, "the page (and its token) is not served to a foreign Host"
    assert gui.raw("POST", "/api/command", {"name": "MOTOR_TEST"}, token=False)[0] == 403
    st, _ = gui.raw("POST", "/api/save", None, headers={"Content-Type": "text/plain", "Content-Length": "0"})
    assert st == 415, "a form post (no CORS preflight) is refused"
    assert gui.raw("GET", "/../client.py", token=False)[0] == 404 and gui.raw("GET", "/%2e%2e/client.py", token=False)[0] == 404, "only files of web/ are served"
    assert gui.raw("GET", "/api/state")[0] == 200 and gui.raw("GET", "/api/state", headers={"Origin": "http://" + gui.host})[0] == 200


def test_not_connected_is_an_answer_not_a_crash(gui):
    st = gui.get("/api/state")
    assert st["connected"] is False and st["info"] is None
    code, doc = gui.raw("GET", "/api/settings")
    assert code == 409 and doc["error"] == "NOT_CONNECTED"
    code, doc = gui.raw("POST", "/api/connect", {"port": "socket://127.0.0.1:1"})
    assert code == 409 and doc["error"] == "OPEN_FAILED" and doc["port"] == "socket://127.0.0.1:1" and doc["detail"], "a code plus values: the page words it in the user's language"
    assert gui.get("/api/schema")["hash"].startswith("0x")
    ports = gui.get("/api/ports")
    assert isinstance(ports["ports"], list) and 230400 in ports["bauds"] and ports["default_baud"] == 230400
    for p in ports["ports"]:
        assert set(p) >= {"port", "description", "likely", "sim", "vid", "pid"}
    assert [p for p in ports["ports"] if p.get("wifi")] == [p for p in ports["ports"] if p["port"] == "socket://192.168.4.1:5760"] != [], "the board's own access point is always offered (never probed)"
    assert gui.raw("POST", "/api/connect", {"port": "socket://127.0.0.1:1", "baud": "fast"})[1]["error"] == "BAD_REQUEST"
    assert gui.raw("POST", "/api/connect", {"port": "socket://127.0.0.1:1", "baud": 50})[1]["error"] == "BAD_REQUEST"


def test_connect_settings_save_reboot(linked, sim):
    gui = linked
    st = gui.get("/api/state")
    assert st["info"]["role"] == "RX" and st["info"]["schema_hash"] == gui.get("/api/schema")["hash"].upper().replace("0X", "0x")
    assert any(l.startswith("imu:") for l in st["report"]), st["report"]
    s = gui.get("/api/settings")
    assert set(s) == set(x["name"] for x in gui.get("/api/schema")["sections"])
    assert s["motors"]["protocol"] == "DSHOT300" or isinstance(s["motors"]["protocol"], str)
    stored = gui.post("/api/set", {"section": "rates", "changes": {"angle_limit": 500, "max_rate": [700, 710, 420]}})["values"]
    assert stored["angle_limit"] == 80 and stored["max_rate"] == [700, 710, 420], "the firmware's clamped value comes back"
    code, doc = gui.raw("POST", "/api/set", {"section": "rates", "changes": {"nope": 1}})
    assert code == 400 and doc["error"] == "BAD_VALUE"
    code, doc = gui.raw("POST", "/api/set", {"section": "motors", "changes": {"protocol": "WARP"}})
    assert code == 400 and "WARP" in doc["message"]
    gui.post("/api/streams", {"rates": {"status": 20}})
    ev = gui.events(0.6)
    flags = [e["s"]["status"]["settings_dirty"] for e in ev if "s" in e and "status" in e["s"]]
    assert flags and flags[-1] == ["DIRTY"], flags
    gui.post("/api/save")
    st = gui.post("/api/reboot")
    assert st["connected"] and st["boots"] == 1
    assert gui.get("/api/settings")["rates"]["max_rate"] == [700, 710, 420], "saved value survived the restart"
    ev = gui.events(0.8)
    assert any("s" in e and "status" in e["s"] for e in ev), "the status stream is subscribed again after the restart"


def test_streams_and_rates(linked, sim):
    gui = linked
    r = gui.post("/api/streams", {"rates": {"status": 10, "attitude": 5000, "gyro_fast": 500}})["rates"]
    assert r == {"status": 10, "attitude": 200, "gyro_fast": 500}, "rates are limited to the stream's maximum"
    sim.do("pose 20 0 0")
    sim.sleep(1.0)
    ev = gui.events(1.0)
    assert "conn" in ev[0] and ev[0]["conn"]["connected"], "the first event is the connection state"
    att = [e["s"]["attitude"] for e in ev if "s" in e and "attitude" in e["s"]]
    assert att and abs(att[-1]["roll"] - 20) < 2.0, att[-1]
    fast = [row for e in ev if "f" in e for row in e["f"]]
    assert len(fast) > 100 and len(fast[0]) == 7, "every gyro_fast sample is forwarded (%d rows)" % len(fast)
    ts = [row[0] for row in fast]
    assert ts == sorted(ts) and len(set(ts)) == len(ts), "in order, none twice"
    r = gui.post("/api/streams", {"rates": {"status": 10}})["rates"]
    assert r == {"status": 10}
    time.sleep(0.3)
    ev = gui.events(0.6)
    assert not any("f" in e for e in ev) and not any("attitude" in e.get("s", {}) for e in ev), "switched-off streams stop"
    code, doc = gui.raw("POST", "/api/streams", {"rates": {"nope": 5}})
    assert code == 400


def test_commands_and_refusals(linked, sim):
    gui = linked
    assert gui.post("/api/command", {"name": "MOTOR_TEST", "args": {"motor": 0, "value": 100}})["result"] == {}
    sim.sleep(0.1)
    m = sim.state()
    assert 0.08 < float(m["m0"]) < 0.12 and float(m["m1"]) == 0.0, "motor 0 really got 10 %% at the ESC, the others nothing: %s" % m
    gui.post("/api/command", {"name": "MOTOR_STOP"})
    sim.sleep(0.1)
    assert float(sim.state()["m0"]) == 0.0
    code, doc = gui.raw("POST", "/api/command", {"name": "MOTOR_TEST", "args": {"motor": 0}})
    assert code == 400 and doc["error"] == "BAD_VALUE", doc
    code, doc = gui.raw("POST", "/api/command", {"name": "WARP_DRIVE"})
    assert code == 400
    code, doc = gui.raw("POST", "/api/command", {"name": "ACC_CAL_FINISH"})
    assert code == 409 and doc["error"] == "BAD_STATE", "the firmware's refusal reaches the page by name"
    # arm, then try to change something
    gui.post("/api/set", {"section": "modes", "changes": {"mode": ["ARM"] + [None] * 15, "aux": [0] * 16, "lo": [1700] + [1000] * 15, "hi": [2100] + [1000] * 15}})
    gui.post("/api/save")
    sim.do("rc 3 1000"); sim.do("rc 5 2000"); sim.sleep(0.5)
    gui.post("/api/streams", {"rates": {"status": 20}})
    ev = gui.events(0.5)
    assert [e["s"]["status"]["armed"] for e in ev if "status" in e.get("s", {})][-1] == 1
    code, doc = gui.raw("POST", "/api/set", {"section": "pid", "changes": {"tpa_rate": 80}})
    assert code == 409 and doc["error"] == "ARMED"
    code, doc = gui.raw("POST", "/api/command", {"name": "MOTOR_TEST", "args": {"motor": 0, "value": 100}})
    assert code == 409 and doc["error"] == "ARMED"
    sim.do("rc 5 1000")


def test_a_restart_behind_our_back_is_noticed(linked, sim):
    gui = linked
    hub = gui.srv.hub
    gui.post("/api/streams", {"rates": {"status": 10}})
    assert gui.events(0.5)
    hub.fc.reboot(wait=False)             # like pressing the reset button: nobody tells the GUI server
    end = time.monotonic() + 8
    while hub.boots == 0 and time.monotonic() < end:
        time.sleep(0.1)
    assert hub.boots == 1, "the silence was noticed and the firmware asked again (stalls %d, note %s, link %s)" % (hub.stalls, hub.note, hub.link)
    ev = gui.events(1.0)
    assert ev[0]["conn"]["boots"] == 1 and ev[0]["conn"]["note"] == {"code": "REBOOTED"}
    assert any("status" in e.get("s", {}) for e in ev), "streams run again without the page doing anything"


def test_a_pause_of_the_streams_is_not_called_a_restart(linked, sim):
    """0.8.8: streams that stop for a while from a firmware that still answers were reported as a restart, and the boot report read
    again then showed the reset reason of the last REAL start ("software restart" after a Save). The link stream's uptime decides."""
    gui = linked
    hub = gui.srv.hub
    gui.post("/api/streams", {"rates": {"status": 10}})
    end = time.monotonic() + 4
    while hub.link is None and time.monotonic() < end:
        time.sleep(0.1)
    assert hub.link is not None, "the link stream runs on its own (1 Hz)"
    time.sleep(1.2)
    gui.events(0.2)
    sim.do("linkpause %g" % (3.0 * sim.speed))      # (simulated seconds: 3 s on the wall clock)
    end = time.monotonic() + 10
    while hub.stalls == 0 and hub.boots == 0 and time.monotonic() < end:
        time.sleep(0.1)
    assert hub.stalls == 1 and hub.boots == 0, "a pause, not a restart (stalls %d, boots %d)" % (hub.stalls, hub.boots)
    assert hub.note["code"] == "STALLED" and hub.note["seconds"] >= 1.5
    assert any("did NOT restart" in t for _, _, t in hub.logs), "the app's log says so, with the link numbers"
    end = time.monotonic() + 5
    while time.monotonic() < end and not any("status" in e.get("s", {}) for e in gui.events(0.5)):
        pass
    assert any("status" in e.get("s", {}) for e in gui.events(1.0)), "streams run again"


def test_losing_the_port(linked, sim):
    gui = linked
    gui.post("/api/streams", {"rates": {"status": 10}})
    got = []
    t = threading.Thread(target=lambda: got.extend(gui.events(4.0)))
    t.start()
    time.sleep(0.8)
    sim.stop()                            # the cable is pulled
    t.join()
    conns = [e["conn"] for e in got if "conn" in e]
    # (the app waits a while for the port to come back - a board on native USB leaves the bus while it restarts)
    assert conns[0]["connected"] and conns[-1]["connected"] is False and conns[-1]["note"] == {"code": "PORT_LOST", "port": sim.url, "waiting": True}, conns
    code, doc = gui.raw("POST", "/api/save", {})
    assert code == 409 and doc["error"] == "NOT_CONNECTED"


def test_backup_restore(linked, sim):
    gui = linked
    gui.post("/api/set", {"section": "pid", "changes": {"tpa_rate": 30}})
    st, raw = gui.raw("GET", "/api/backup")
    assert st == 200
    doc = raw if isinstance(raw, dict) else json.loads(raw)
    assert doc["settings"]["pid"]["tpa_rate"] == 30 and doc["windburst_backup"] == 1
    gui.post("/api/defaults", {"section": "pid"})
    assert gui.get("/api/settings")["pid"]["tpa_rate"] == 65
    doc["settings"]["pid"]["from_the_future"] = 7
    doc["settings"]["rates"]["angle_limit"] = 200
    doc["settings"]["warp"] = {"x": 1}
    res = gui.post("/api/restore", doc)
    assert "pid" in res["sections"] and "pid.from_the_future" in res["skipped"] and "warp" in res["skipped"]
    assert {"field": "rates.angle_limit", "asked": 200, "stored": 80} in res["changed"], res["changed"]
    assert gui.get("/api/settings")["pid"]["tpa_rate"] == 30


def test_gui_command_starts_serves_and_quits(sim, tmp_path):
    """`python -m wbclient --port ... gui`: connects, serves the page, and Ctrl+C leaves cleanly (the port is released)."""
    import os
    import re
    import signal
    import subprocess
    import sys
    import urllib.request

    if os.name == "nt":
        pytest.skip("needs a POSIX SIGINT to stand in for Ctrl+C")
    env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
    app_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # (a test run that was started in the background of a script - `make apptest &` - has SIGINT set to "ignore", every child
    #  inherits that, and Python then never raises KeyboardInterrupt: give the child the disposition a terminal gives it)
    p = subprocess.Popen([sys.executable, "-m", "wbclient", "--port", sim.url, "gui", "--http", "0", "--no-browser", "--logs", str(tmp_path / "my logs")], cwd=app_dir, env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                         preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
    try:
        first = p.stdout.readline()
        m = re.search(r"http://127\.0\.0\.1:(\d+)/", first)
        assert m, first
        logs = p.stdout.readline()
        assert "flight logs:" in logs and logs.rstrip().endswith("my logs"), logs
        second = p.stdout.readline()
        assert "connected:" in second and "Windburst RX" in second, second
        page = urllib.request.urlopen(m.group(0), timeout=5).read().decode("utf-8")
        token = re.search(r'name="wb-token" content="([^"]+)"', page).group(1)
        assert len(token) >= 20 and "__WB_TOKEN__" not in page
        req = urllib.request.Request(m.group(0) + "api/state", headers={"X-WB-Token": token})
        st = json.loads(urllib.request.urlopen(req, timeout=5).read().decode("utf-8"))
        assert st["connected"] and st["port"] == sim.url
        js = urllib.request.urlopen(m.group(0) + "js/main.js", timeout=5)
        assert js.headers["Content-Type"].startswith("text/javascript"), "browsers refuse a module script served as text/plain"
    finally:
        p.send_signal(signal.SIGINT)
        try:
            assert p.wait(timeout=8) == 0
        except subprocess.TimeoutExpired:
            p.kill()
            raise AssertionError("the GUI server did not stop on Ctrl+C")
    # the simulator's single connection is free again
    from wbclient import WbClient
    with WbClient(sim.url, timeout=2.0) as fc:
        assert fc.info.role == "RX"


# ---------------------------------------------------------------------------------------------------------------------
# Serial: the same things through a real serial device (a pseudo-terminal in front of the simulator), not socket://
# ---------------------------------------------------------------------------------------------------------------------
def test_serial_port_end_to_end(gui, sim, serial_port):
    st = gui.post("/api/connect", {"port": serial_port.path, "baud": 460800})
    assert st["connected"] and st["port"] == serial_port.path and st["baud"] == 460800 and st["info"]["role"] == "RX"
    assert gui.srv.hub.fc._ser.baudrate == 460800 and gui.srv.hub.fc._ser.port == serial_port.path, "the serial device itself was opened at the chosen baud rate"
    sim.sleep(1.5)
    assert gui.get("/api/settings")["motors"]["layout"] == "QUAD_X"
    stored = gui.post("/api/set", {"section": "pid", "changes": {"tpa_rate": 33}})["values"]
    assert stored["tpa_rate"] == 33
    gui.post("/api/streams", {"rates": {"status": 20, "attitude": 50}})
    ev = gui.events(1.0)
    n_att = sum(1 for e in ev if "attitude" in e.get("s", {}))
    assert n_att >= 15, "telemetry flows over the serial device (%d attitude updates in 1 s)" % n_att
    # text that is not a protocol frame (a task printing, a crash message) is kept apart and does not disturb the link
    serial_port.inject(b"\r\nGuru Meditation Error: Core  1 panic'ed (LoadProhibited)\r\nBacktrace: 0x4200cf80\r\n")
    ev = gui.events(0.8)
    raw = "".join(e.get("raw", "") for e in ev)
    assert "Guru Meditation Error" in raw and "Backtrace: 0x4200cf80" in raw and "\r" not in raw, repr(raw)
    assert any("status" in e.get("s", {}) for e in ev) and gui.get("/api/settings")["pid"]["tpa_rate"] == 33
    # the cable is pulled
    got = []
    t = threading.Thread(target=lambda: got.extend(gui.events(4.0)))
    t.start()
    time.sleep(0.6)
    serial_port.unplug()
    t.join()
    conns = [e["conn"] for e in got if "conn" in e]
    assert conns and conns[-1]["connected"] is False and conns[-1]["note"] == {"code": "PORT_LOST", "port": serial_port.path}, conns


def test_serial_port_that_is_not_a_windburst_board(gui):
    """The port opens, bytes arrive, but nothing speaks WBP (boot loop, other firmware, wrong baud): say so and show what was heard."""
    from conftest import PtyPort
    boot = b"ESP-ROM:esp32s3-20210327\r\nrst:0x3 (RTC_SW_SYS_RST),boot:0x8 (SPI_FAST_FLASH_BOOT)\r\n\xff\x00\x81"
    port = PtyPort(None, chatter=boot)
    try:
        code, doc = gui.raw("POST", "/api/connect", {"port": port.path, "baud": 115200}, timeout=30)
        assert code == 504 and doc["error"] == "NO_ANSWER" and doc["port"] == port.path and doc["baud"] == 115200, doc
        assert "ESP-ROM:esp32s3-20210327" in doc["raw"] and "rst:0x3" in doc["raw"], "what the port did send is handed to the page: %r" % doc["raw"]
        assert all(32 <= ord(c) < 127 or c == "\n" for c in doc["raw"]), "binary rubbish is shown as dots, never as control characters"
        assert gui.get("/api/state")["connected"] is False
        # and the port is free again: for the next attempt, and for the flashing tool (on Windows a handle left open blocks both)
        assert not [t for t in threading.enumerate() if t.name == "wbclient-reader"], "nothing keeps reading the port"
        code2, doc2 = gui.raw("POST", "/api/connect", {"port": port.path}, timeout=30)
        assert code2 == 504 and doc2["error"] == "NO_ANSWER", "not OPEN_FAILED: the first attempt let go of the port (%r)" % (doc2,)
    finally:
        port.close()


def test_a_port_held_by_another_program(gui, sim):
    """A serial monitor (or a second configurator) has the port: say that it is busy, do not share it and mis-read half of the answers."""
    import serial
    from conftest import PtyPort
    port = PtyPort(sim)
    other = serial.serial_for_url(port.path, do_not_open=True)
    other.exclusive = True
    other.open()
    try:
        code, doc = gui.raw("POST", "/api/connect", {"port": port.path}, timeout=30)
        assert code == 409 and doc["error"] == "OPEN_FAILED" and doc["port"] == port.path and doc["detail"], doc
        assert gui.get("/api/state")["connected"] is False
        other.close()
        st = gui.post("/api/connect", {"port": port.path})
        assert st["connected"] and st["port"] == port.path, "free again once the other program closed it"
        gui.post("/api/disconnect", {})
    finally:
        other.close()
        port.close()


def test_a_silent_port(gui):
    from conftest import PtyPort
    port = PtyPort(None)
    try:
        code, doc = gui.raw("POST", "/api/connect", {"port": port.path}, timeout=30)
        assert code == 504 and doc["error"] == "NO_ANSWER" and doc["raw"] == ""
    finally:
        port.close()


# ---------------------------------------------------------------------------------------------------------------- flight logs
def _download(gui, index, during=None, seconds=30):
    """Start a log download while a page is listening; returns (the POST's reply, the last job message, every job message)."""
    seen, listening = [], threading.Event()

    def listen():
        c = http.client.HTTPConnection("127.0.0.1", gui.port, timeout=seconds)
        c.request("GET", "/api/events?t=" + gui.srv.token, headers={"Host": gui.host})
        r = c.getresponse()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            line = r.fp.readline()
            listening.set()
            if line.startswith(b"data: "):
                msg = json.loads(line[6:].decode("utf-8"))
                if "job" in msg and msg["job"]["index"] == index:
                    seen.append(msg["job"])
                    if msg["job"]["state"] != "running":
                        break
        c.close()
    th = threading.Thread(target=listen, daemon=True)
    th.start()
    assert listening.wait(5)
    first = gui.post("/api/blackbox/download", {"index": index})["job"]
    if during:
        during()
    th.join(seconds)
    assert seen and seen[-1]["state"] != "running", "the job did not finish: %s" % (seen[-3:],)
    return first, seen[-1], seen


def _short_flight(sim, seconds=3.0):
    sim.do("rc 6 2000"); sim.do("rc 7 2000"); sim.sleep(0.3)             # Angle + Altitude Hold
    sim.do("rc 5 2000"); sim.sleep(0.5)
    sim.do("rc 3 1700"); sim.sleep(seconds / 2)
    sim.do("rc 3 1500"); sim.do("rc 1 1800"); sim.sleep(seconds / 2)
    sim.do("rc 1 1500"); sim.do("rc 3 1000"); sim.do("rc 5 1000"); sim.sleep(1.0)


def test_flight_log_from_the_craft_to_the_chart(linked, sim, log_dir):
    gui = linked
    assert gui.get("/api/logs") == {"dir": str(log_dir), "logs": []} and not log_dir.exists(), "looking at an empty library creates nothing"
    assert gui.get("/api/blackbox/sessions") == {"sessions": []}
    st, doc = gui.raw("POST", "/api/blackbox/download", {"index": None})
    assert st == 409 and doc["error"] == "NO_LOG"
    gui.post("/api/set", {"section": "blackbox", "changes": {"device": "FLASH", "trigger": "ARMED", "rate_hz": 250}})
    sim.sleep(0.5)
    _short_flight(sim)
    _short_flight(sim, 2.0)
    stored = gui.get("/api/blackbox/sessions")["sessions"]
    assert [s["index"] for s in stored] == [0, 1] and stored[0]["start"] == 0 and stored[1]["start"] >= stored[0]["size"] > 50_000
    for bad in (2, -1, "0", True, 1.0):
        st, doc = gui.raw("POST", "/api/blackbox/download", {"index": bad})
        assert st == 400 and doc["error"] == "BAD_VALUE", bad

    # ---- one session into the library, with progress on the way
    fc = gui.srv.hub.fc
    fast_link = fc._dispatch

    def serial_speed(msg_type, seq, payload):               # the simulator's socket delivers a log in milliseconds; a serial cable does not
        if msg_type == 0x62:
            time.sleep(0.004)
        fast_link(msg_type, seq, payload)
    fc._dispatch = serial_speed
    refused = []

    def meanwhile():
        refused.append(gui.raw("POST", "/api/blackbox/download", {"index": 0}))
        refused.append(gui.raw("POST", "/api/reboot", {}))
        refused.append(gui.raw("POST", "/api/blackbox/record", {"on": True}))
    job, last, seen = _download(gui, 1, meanwhile)
    fc._dispatch = fast_link
    assert job["state"] == "running" and job["total"] == stored[1]["size"] and job["done"] == 0
    assert [(st, doc["error"]) for st, doc in refused] == [(409, "BUSY")] * 3, "one download at a time; no reboot and no recording under it"
    assert last["state"] == "done" and last["done"] == last["total"] and last["sessions"] == 1 and last["error"] is None
    done = [j["done"] for j in seen]
    assert done == sorted(done) and len(set(done)) > 3, "progress was reported on the way: %s" % done[:8]
    name = last["file"]
    assert name.startswith("flight_") and name.endswith("_s2.wblog") and (log_dir / name).stat().st_size == stored[1]["size"]
    logs = gui.get("/api/logs")["logs"]
    assert len(logs) == 1 and logs[0]["file"] == name and logs[0]["sessions"] == 1 and logs[0]["records"] > 500 and logs[0]["firmware"].startswith("Windburst RX")
    assert gui.get("/api/state")["connected"] and gui.get("/api/state")["responding"], "the link survived the download"

    # ---- everything at once: two sessions in one file
    _, last, _ = _download(gui, None)
    both = last["file"]
    assert last["state"] == "done" and last["sessions"] == 2 and both.startswith("flights_") and both != name
    meta = gui.get("/api/log/meta?file=" + both.replace(" ", "%20"))
    assert [s["session"] for s in meta["sessions"]] == ["1", "2"] and all(s["complete"] and s["end"] == "trigger off" and s["gaps"] == 0 for s in meta["sessions"])
    m = meta["sessions"][0]
    assert m["rate_hz"] == 250 and 3.0 < m["duration"] < 5.0 and m["records"] > 0.85 * 250 * m["duration"] and m["to"] == "storage"
    assert {"gyro[0]", "setpoint[0]", "motor[3]", "roll", "pid_p[1]", "loop_us"} <= set(m["fields"]) and {"vbat", "est_z", "lat", "armed"} <= set(m["slow_fields"])
    assert m["units"]["gyro[0]"] == "deg/s" and m["units"]["vbat"] == "V" and m["units"]["motor[0]"] == ""
    assert [e[1] for e in m["events"]].count("ARM") == 1 and m["events"][-1][1] == "END"

    # ---- the chart's view of it
    v = gui.get("/api/log/view?file=%s&session=0&fields=roll,setpoint[0],gyro[0],motor[0],vbat&px=300" % both)
    assert v["records"] == m["records"] and v["complete"] and v["span"][1] - v["span"][0] == pytest.approx(m["duration"], abs=0.2)
    roll = v["series"]["roll"]
    assert len(roll["lo"]) == len(roll["hi"]) == len(roll["t"]) == 300 and 8 < max(roll["hi"]) < 35 and roll["unit"] == "deg" and roll["dt"] == pytest.approx(0.004)
    assert max(v["series"]["setpoint[0]"]["hi"]) > 20 and 0.2 < max(v["series"]["motor[0]"]["hi"]) <= 1.0
    assert "v" in v["series"]["vbat"] and v["series"]["vbat"]["v"][5] == pytest.approx(15.7, abs=0.5), "slow fields are few: sent as they are"
    t_mid = (v["span"][0] + v["span"][1]) / 2
    z = gui.get("/api/log/view?file=%s&session=0&fields=roll&t0=%.3f&t1=%.3f&px=600" % (both, t_mid, t_mid + 0.5))
    assert "v" in z["series"]["roll"] and 120 <= len(z["series"]["roll"]["v"]) <= 130 and z["t0"] == pytest.approx(t_mid, abs=1e-3)
    for path, code, status in (("/api/log/view?file=%s&session=7&fields=roll" % both, "NO_SUCH_SESSION", 404), ("/api/log/view?file=%s&fields=wings" % both, "NO_SUCH_FIELD", 400),
                               ("/api/log/meta?file=nothing.wblog", "NO_SUCH_LOG", 404), ("/api/log/meta?file=../flash.bin", "BAD_NAME", 400),
                               ("/api/log/file?file=..%2F..%2Fflash.bin", "BAD_NAME", 400), ("/api/log/view?file=%s&fields=roll&px=abc" % both, "BAD_REQUEST", 400),
                               ("/api/log/meta?file=@live", "NO_SUCH_LOG", 404)):
        st, doc = gui.raw("GET", path)
        assert st == status and doc["error"] == code, (path, st, doc)

    # ---- the file itself, CSV, and the settings the flight was flown with
    st, raw = gui.raw("GET", "/api/log/file?file=" + name)
    assert st == 200 and raw == (log_dir / name).read_bytes()
    st, csv = gui.raw("GET", "/api/log/csv?file=%s&session=1" % both)
    lines = csv.decode().splitlines()
    assert st == 200 and lines[0].startswith("t (s),gyro[0] (deg/s),") and len(lines) == meta["sessions"][1]["records"] + 1
    st, csv = gui.raw("GET", "/api/log/csv?file=%s&session=1&slow=1" % both)
    assert st == 200 and "vbat (V)" in csv.decode().splitlines()[0] and len(csv.decode().splitlines()) == meta["sessions"][1]["slow_records"] + 1
    flown = gui.get("/api/log/settings?file=%s&session=0" % both)
    assert flown["schema_matches"] and flown["settings"]["pid"] == gui.get("/api/settings")["pid"] and flown["settings"]["blackbox"]["rate_hz"] == 250
    assert gui.post("/api/restore", flown)["changed"] == [], "the settings of a logged flight restore like a backup"

    # ---- delete; import a file from somewhere else
    gui.post("/api/log/delete", {"file": name})
    assert [x["file"] for x in gui.get("/api/logs")["logs"]] == [both] and not (log_dir / name).exists()
    st, doc = gui.raw("POST", "/api/log/delete", {"file": name})
    assert st == 404 and doc["error"] == "NO_SUCH_LOG"
    import base64
    up = gui.post("/api/log/upload", {"name": "C:\\Users\\me\\Desktop\\buổi sáng.wblog", "data": base64.b64encode(raw).decode()})["file"]
    assert up == "bu_i s_ng.wblog" and (log_dir / up).read_bytes() == raw
    big = base64.b64encode(b"this is not a flight log" * 100000).decode()                    # 3 MB: a log may be that large, a settings request may not
    for body, code in (({"name": "x.wblog", "data": big}, "NOT_A_LOG"), ({"name": "x.wblog", "data": "%%%"}, "BAD_REQUEST")):
        st, doc = gui.raw("POST", "/api/log/upload", body)
        assert st == 400 and doc["error"] == code
    st, doc = gui.raw("POST", "/api/log/delete", {"file": big})
    assert st == 413, "only the import route takes a large body"
    assert sorted(x["file"] for x in gui.get("/api/logs")["logs"]) == sorted([both, up])


def test_live_recording_from_the_page(linked, sim, log_dir):
    gui = linked
    st, doc = gui.raw("POST", "/api/blackbox/record", {"on": True, "rate_hz": 99999})
    assert st == 400 and doc["error"] == "BAD_VALUE"
    sim.do("pose 12 -6 0")
    rec = gui.post("/api/blackbox/record", {"on": True, "rate_hz": 200})["rec"]
    assert rec["on"] and rec["file"].startswith("live_") and rec["file"].endswith(".wblog")
    assert gui.post("/api/blackbox/record", {"on": True, "rate_hz": 200})["rec"]["file"] == rec["file"], "asking twice does not start a second file"
    msgs = [m["rec"] for m in gui.events(1.0) if "rec" in m]
    # (the very first message may still be the header alone: no record yet)
    assert len(msgs) >= 3 and all(m["on"] for m in msgs) and msgs[-1]["records"] > msgs[0]["records"] >= 0 and msgs[1]["records"] > 0, "the page hears how the recording grows"
    # the chart follows the recording while it runs
    a = gui.get("/api/log/view?file=@live&fields=roll,pitch&px=200")
    time.sleep(0.3)
    b = gui.get("/api/log/view?file=@live&fields=roll,pitch&px=200")
    assert b["records"] > a["records"] > 100 and b["span"][1] > a["span"][1] and not b["complete"]
    tail = gui.get("/api/log/view?file=@live&fields=roll,pitch&t0=%.3f&px=200" % (b["span"][1] - 0.2))["series"]
    assert tail["roll"]["v"][-1] == pytest.approx(12, abs=1.5) and tail["pitch"]["v"][-1] == pytest.approx(-6, abs=1.5)
    assert gui.get("/api/log/meta?file=@live")["sessions"][0]["to"] == "stream"
    fc = gui.srv.hub.fc
    fast_link = fc._dispatch

    def serial_speed(msg_type, seq, payload):               # on a cable the tail of the stream arrives after the firmware's "stopped" answer
        if msg_type == 0x61:
            time.sleep(0.08)
        fast_link(msg_type, seq, payload)
    fc._dispatch = serial_speed
    done = gui.post("/api/blackbox/record", {"on": False})["rec"]
    fc._dispatch = fast_link
    assert not done["on"] and done["complete"] and done["file"] == rec["file"] and done["records"] >= b["records"] and done["gaps"] == 0
    logs = gui.get("/api/logs")["logs"]
    assert [x["file"] for x in logs] == [rec["file"]] and logs[0]["records"] == done["records"] and logs[0]["size"] == done["bytes"]
    assert gui.get("/api/log/meta?file=" + rec["file"])["sessions"][0]["end"] == "stream stopped"
    assert gui.post("/api/blackbox/record", {"on": False})["rec"]["on"] is False, "stopping twice is harmless"
    # a recording that is cut by pulling the plug keeps what it got
    rec2 = gui.post("/api/blackbox/record", {"on": True})["rec"]
    time.sleep(0.5)
    gui.post("/api/disconnect")
    logs = {x["file"]: x for x in gui.get("/api/logs")["logs"]}
    assert rec2["file"] in logs and logs[rec2["file"]]["records"] > 50 and len(logs) == 2
    st, doc = gui.raw("POST", "/api/blackbox/record", {"on": True})
    assert st == 409 and doc["error"] == "NOT_CONNECTED"


def test_download_ends_a_live_recording_instead_of_refusing(linked, sim, log_dir):
    """The page shows the live chart (a recording runs) and the user presses Download: the recording is closed and kept, the
    stored flight is fetched. Refusing with BUSY here made the download look broken."""
    gui = linked
    gui.post("/api/set", {"section": "blackbox", "changes": {"device": "FLASH", "trigger": "ARMED", "rate_hz": 250}})
    sim.sleep(0.5)
    _short_flight(sim)
    rec = gui.post("/api/blackbox/record", {"on": True, "rate_hz": 200})["rec"]
    assert rec["on"]
    time.sleep(0.5)
    _, last, _ = _download(gui, 0)
    assert last["state"] == "done" and last["error"] is None and last["sessions"] == 1 and last["file"].startswith("flight_")
    assert not gui.srv.hub.rec_status()["on"], "the recording was ended by the download"
    logs = {x["file"]: x for x in gui.get("/api/logs")["logs"]}
    assert rec["file"] in logs and logs[rec["file"]]["records"] > 30 and last["file"] in logs and len(logs) == 2, "both files are kept: the live recording and the downloaded flight"
    assert gui.get("/api/log/meta?file=" + rec["file"])["sessions"][0]["end"] == "stream stopped"


def test_download_with_trigger_always(linked, sim, log_dir):
    """Trigger ALWAYS: a session is being written whenever the craft has power. The firmware closes it for the download (on the
    ground), holds the logging back while the app reads, and resumes by itself afterwards."""
    gui = linked
    gui.post("/api/set", {"section": "blackbox", "changes": {"device": "FLASH", "trigger": "ALWAYS", "rate_hz": 250}})
    sim.sleep(1.5)
    assert gui.srv.hub.fc.blackbox_status()["state"] == "LOGGING"
    stored = gui.get("/api/blackbox/sessions")["sessions"]
    assert len(stored) == 1 and stored[0]["size"] > 20_000, "the running session is listed (0.8.8: and keeps running)"
    assert gui.srv.hub.fc.blackbox_status()["state"] == "LOGGING", "a look at the list does not stop it"
    _, last, _ = _download(gui, 0)
    assert last["state"] == "done" and last["sessions"] == 1 and last["error"] is None
    meta = gui.get("/api/log/meta?file=" + last["file"])["sessions"][0]
    assert meta["complete"] and meta["end"] == "app took over" and meta["gaps"] == 0
    assert gui.srv.hub.fc.blackbox_status()["state"] == "IDLE", "held back while the app is at the memory"
    sim.sleep(4.0)
    assert gui.srv.hub.fc.blackbox_status()["state"] == "LOGGING", "and logging again a few seconds later, by itself"
    # in the air the flight log is not cut: the app is told so
    sim.do("rc 5 2000"); sim.sleep(0.8)
    st, doc = gui.raw("POST", "/api/blackbox/download", {"index": 0})
    assert st == 409 and doc["error"] == "BUSY"
    sim.do("rc 5 1000"); sim.sleep(0.5)


def test_recording_refused_in_flight_leaves_no_file(linked, sim, log_dir):
    gui = linked
    gui.post("/api/set", {"section": "blackbox", "changes": {"device": "FLASH", "trigger": "ARMED"}})
    sim.sleep(0.5)
    sim.do("rc 5 2000"); sim.sleep(0.8)
    st, doc = gui.raw("POST", "/api/blackbox/record", {"on": True})
    assert st == 409 and doc["error"] == "BUSY", "the log goes to flash while armed: the app may not take it over in flight"
    sim.do("rc 5 1000"); sim.sleep(0.8)
    assert gui.get("/api/logs")["logs"] == [] and (not log_dir.exists() or list(log_dir.iterdir()) == [])
    events = gui.events(0.5)
    assert [m["rec"] for m in events if "rec" in m][0] == {"on": False, "file": None}
