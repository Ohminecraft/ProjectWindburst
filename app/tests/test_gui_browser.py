"""The configurator page in a real (headless) browser, talking to the real firmware in wb_sim.
Needs the optional `playwright` package with a Chromium (pip install playwright && playwright install chromium); skipped otherwise."""
import json
import re
import time

import pytest

pw = pytest.importorskip("playwright.sync_api")

from wbclient.gui import GuiServer  # noqa: E402


@pytest.fixture(scope="module")
def browser():
    with pw.sync_playwright() as p:
        try:
            b = p.chromium.launch()
        except Exception as e:  # noqa: BLE001
            pytest.skip("no Chromium for playwright: %s" % e)
        yield b
        b.close()


@pytest.fixture
def sim(tmp_path):
    """Real-time simulator: the page's timers (motor-test refresh every 0.2 s against the firmware's 0.6 s watchdog) are part of what is tested."""
    from conftest import Sim
    s = Sim(tmp_path, speed=1.0)
    yield s
    s.stop()


class Gui:
    def __init__(self, page, srv, sim, errors):
        self.page, self.srv, self.sim, self.errors = page, srv, sim, errors
        self.hub = srv.hub

    def choose_port(self, port, baud=None):
        """Type a port name by hand (the "another port" entry) and optionally pick a baud rate."""
        pg = self.page
        pg.wait_for_selector("#port option", state="attached")
        pg.once("dialog", lambda d: d.accept(port))
        pg.select_option("#port", "__custom__")
        pg.wait_for_function("p => document.getElementById('port').value === p", arg=port)
        if baud:
            pg.select_option("#baud", str(baud))

    def connect(self, port=None, baud=None):
        pg = self.page
        pg.goto(self.srv.url)
        self.choose_port(port or self.sim.url, baud)
        pg.click("#connect")
        pg.wait_for_selector("#arm-pill.ok, #arm-pill.warn", timeout=15000)
        self.sim.sleep(1.5)                                   # boot gyro calibration
        pg.wait_for_selector("#arm-pill.ok", timeout=8000)
        return self

    def tab(self, name):
        self.page.evaluate("id => { location.hash = id; }", name)
        self.page.wait_for_selector("#tab-%s.active" % name)
        self.page.wait_for_selector("#view h1")

    def field(self, name):
        return self.page.locator('[data-field="%s"]' % name)

    def type_into(self, name, text):
        f = self.field(name)
        f.fill(str(text))
        f.press("Tab")

    def setting(self, section, name):
        return self.hub.fc.get(section)[name]

    def wait(self, fn, timeout=6.0, what=""):
        end = time.monotonic() + timeout
        while True:
            v = fn()
            if v:
                return v
            if time.monotonic() > end:
                raise AssertionError("timed out: " + what)
            time.sleep(0.05)


def _gui(browser, sim, locale):
    srv = GuiServer(http_port=0).start()
    page = browser.new_page(viewport={"width": 1440, "height": 900}, locale=locale)       # the page follows the browser's language
    errors = []
    # a firmware refusal (HTTP 409: "not all faces yet", "armed"...) or a failed connection attempt (504) is an answer the page shows to the user;
    # the browser still notes the HTTP status in its console
    page.on("console", lambda m: errors.append(m.text) if m.type == "error" and "status of 409" not in m.text and "status of 504" not in m.text else None)
    page.on("pageerror", lambda e: errors.append("pageerror: %s" % e))
    g = Gui(page, srv, sim, errors)
    yield g
    page.close()
    srv.stop()
    assert not errors, "the page logged errors: %s" % errors


@pytest.fixture
def sim_stm32(tmp_path):
    """The simulator saying it is an STM32 board (device feature STM32): the page writes pins as PA0 .. PK15."""
    from conftest import Sim
    s = Sim(tmp_path, "--stm32", speed=1.0)
    yield s
    s.stop()


@pytest.fixture
def gui_stm32(browser, sim_stm32):
    yield from _gui(browser, sim_stm32, "vi-VN")


@pytest.fixture
def gui(browser, sim):
    """Vietnamese page (the tests below read the Vietnamese texts)."""
    yield from _gui(browser, sim, "vi-VN")


@pytest.fixture
def gui_en(browser, sim):
    yield from _gui(browser, sim, "en-US")


def test_connect_and_overview(gui):
    pg = gui.page
    pg.goto(gui.srv.url)
    pg.wait_for_selector(".welcome")
    assert "Nối flight controller" in pg.inner_text("#view") and pg.inner_text("#arm-pill") == "Chưa kết nối"
    assert pg.locator("#save").is_hidden()
    gui.connect()
    assert pg.inner_text("#arm-pill") == "Sẵn sàng arm" and "Windburst RX" in pg.inner_text("#fw")
    assert pg.inner_text("#connect") == "Ngắt kết nối" and pg.locator("#port").is_disabled()
    rows = {r.get_attribute("data-module"): r.inner_text() for r in pg.locator("table.modules tbody tr").all()}
    assert "Có" in rows["imu"] and "ICM42688P" in rows["imu"] and "Có" in rows["rx"] and "CRSF" in rows["rx"]
    assert "Tắt" in rows["mag"], "a module switched off in the settings is shown as off, not as broken"
    assert "Không còn gì chặn" in pg.inner_text("#blockers")
    # load and memory as the firmware reports them (simulated module: N16R8, 320 kB of heap, 8 MB of PSRAM)
    pg.wait_for_function("/^\\d+ %$/.test(document.getElementById('cpu-load').textContent)", timeout=6000)
    assert pg.inner_text("#ram-use") == "25 % đang dùng, còn 240 kB" and pg.inner_text("#psram-use") == "còn 7680 trên 8192 kB"
    assert 0 <= int(pg.inner_text("#cpu-load").split()[0]) <= 100
    gui.sim.do("pose 20 -10 0")
    pg.wait_for_function("() => { const t = document.querySelector('.big-read').innerText; return /20\\.\\d°/.test(t) && /-10\\.\\d°|-9\\.\\d°/.test(t); }", timeout=6000)
    gui.sim.do("pose 60 0 0")                                  # steeper than max_arm_angle
    pg.wait_for_selector('#blockers [data-blocker="ANGLE_TOO_HIGH"]')
    assert pg.inner_text("#arm-pill") == "Chưa arm được" and "nghiêng quá góc" in pg.inner_text("#ready")
    # the 3D view draws something that changes with the attitude
    a = pg.evaluate("document.querySelector('canvas.craft').toDataURL()")
    gui.sim.do("pose -40 20 0")
    time.sleep(0.5)
    assert a != pg.evaluate("document.querySelector('canvas.craft').toDataURL()") and len(a) > 5000


def test_edit_clamp_save_and_reboot(gui):
    pg = gui.connect().page
    gui.tab("rates")
    assert gui.field("rates.max_rate[0]").input_value() == "670"
    gui.type_into("rates.max_rate[0]", "5000")
    pg.wait_for_function("document.querySelector('[data-field=\"rates.max_rate[0]\"]').value === '1998'")       # the firmware's limit comes back
    gui.tab("tuning")
    gui.type_into("pid.stab_p[2]", "99")
    pg.wait_for_function("document.querySelector('[data-field=\"pid.stab_p[2]\"]').value === '12'")              # and here: Stabilize yaw P stops at 12
    gui.type_into("pid.ff_boost", "22")
    pg.wait_for_selector("#save:not([hidden])")
    assert pg.inner_text("#save") == "Lưu" and pg.locator("#revert").is_visible()
    # (the page sends an edit 0.2 s after the last key: wait for the firmware to have both)
    gui.wait(lambda: gui.setting("pid", "stab_p")[2] == pytest.approx(12.0) and gui.setting("pid", "ff_boost") == 22, what="both edits of the section reach the firmware")
    assert gui.setting("rates", "max_rate")[0] == 1998
    pg.click("#save")
    pg.wait_for_selector("#save", state="hidden")
    gui.sim.power_cycle()                                       # pull the plug: only what is in flash survives
    pg.wait_for_selector(".welcome")
    gui.connect(); gui.tab("rates")
    assert gui.field("rates.max_rate[0]").input_value() == "1998"
    gui.tab("tuning")
    assert gui.field("pid.stab_p[2]").input_value() == "12" and gui.field("pid.ff_boost").input_value() == "22"
    # a section that needs a restart
    gui.tab("motors")
    pg.select_option('[data-field="motors.protocol"]', "DSHOT600")
    pg.wait_for_function("document.getElementById('save').textContent === 'Lưu và khởi động lại'")
    boots = int(gui.sim.state()["boots"])
    pg.click("#save")
    gui.wait(lambda: int(gui.sim.state()["boots"]) == boots + 1, 12, "firmware restart")
    pg.wait_for_selector("#save", state="hidden", timeout=12000)
    pg.wait_for_function("(document.querySelector('[data-field=\"motors.protocol\"]') || {}).value === 'DSHOT600'", timeout=8000)
    assert gui.setting("motors", "protocol") == "DSHOT600"
    assert any("DSHOT600" in l or "host" in l for l in gui.hub.report)
    assert not gui.field("motors.ext_pullup").is_checked(), "external pull-up resistors: off by default"
    # throwing changes away
    gui.tab("tuning")
    gui.type_into("pid.tpa_rate", "40")
    pg.wait_for_selector("#revert:not([hidden])")
    pg.once("dialog", lambda d: d.accept())
    pg.click("#revert")
    pg.wait_for_selector("#revert", state="hidden", timeout=12000)
    pg.wait_for_function("(document.querySelector('[data-field=\"pid.tpa_rate\"]') || {}).value === '65'", timeout=8000)   # the page re-read the settings
    assert gui.setting("pid", "tpa_rate") == 65


def test_modes_tab(gui):
    pg = gui.connect().page
    gui.tab("modes")
    stab = pg.locator('[data-mode="STABILIZE"]')
    assert stab.locator(".mode-range").count() == 0
    stab.locator("[data-add]").click()
    stab.locator(".mode-range").wait_for()
    slot = int(stab.locator(".mode-range").get_attribute("data-slot"))
    stab.locator("select").select_option("6")                  # AUX7
    lo = stab.locator('[data-h="lo"]')
    lo.focus()
    for _ in range(4):
        lo.press("ArrowLeft")                                   # 1700 -> 1600
    gui.wait(lambda: gui.setting("modes", "lo")[slot] == 1600, what="range written")
    m = gui.hub.fc.get("modes")
    assert m["mode"][slot] == "STABILIZE" and m["aux"][slot] == 6 and m["hi"][slot] == 2100
    assert "1600" in stab.locator(".range-text").inner_text()
    assert stab.locator("[data-mode-state]").inner_text() == "Tắt"
    gui.sim.do("rc 11 1650")                                    # AUX7 = channel 11
    pg.wait_for_function("document.querySelector('[data-mode=\"STABILIZE\"] [data-mode-state]').textContent === 'Đang bật'")
    pg.wait_for_function("document.querySelector('[data-mode=\"STABILIZE\"] .mode-range').classList.contains('inside')")   # follows the rc stream, a few ms after the status stream
    gui.sim.do("rc 11 1000")
    pg.wait_for_function("document.querySelector('[data-mode=\"STABILIZE\"] [data-mode-state]').textContent === 'Tắt'")
    stab.locator("button", has_text="Xóa").click()
    gui.wait(lambda: gui.setting("modes", "mode")[slot] is None, what="range removed")
    assert stab.locator(".mode-range").count() == 0


def test_motor_tab_and_test_sliders(gui):
    pg = gui.connect().page
    gui.tab("motors")
    spins = [pg.locator('#frame [data-motor="%d"]' % i).get_attribute("data-spin") for i in range(4)]
    assert spins == ["cw", "ccw", "ccw", "cw"], "Betaflight props-in: 1 rear-right CW, 2 front-right CCW, 3 rear-left CCW, 4 front-left CW"
    assert pg.locator('[data-test="1"]').is_disabled(), "sliders are locked until the propellers are declared off"
    pg.check("#props-off")
    pg.evaluate("() => { const s = document.querySelector('[data-test=\"1\"]'); s.value = 15; s.dispatchEvent(new Event('input', { bubbles: true })); }")
    gui.wait(lambda: 0.13 < float(gui.sim.state()["m1"]) < 0.17, what="motor 2 at 15 %")
    assert float(gui.sim.state()["m0"]) == 0.0 and float(gui.sim.state()["m2"]) == 0.0
    time.sleep(1.5)                                             # far longer than the firmware's 0.5 s watchdog: the page keeps refreshing
    assert 0.13 < float(gui.sim.state()["m1"]) < 0.17
    assert "Chưa arm được" == pg.inner_text("#arm-pill") and "test motor" in pg.inner_text("#ready")
    pg.click("#test-stop")
    gui.wait(lambda: float(gui.sim.state()["m1"]) == 0.0, what="stop")
    assert pg.locator('[data-test="1"]').input_value() == "0"
    # all motors, then leave the tab: nothing may keep spinning
    pg.evaluate("() => { const s = document.getElementById('test-all'); s.value = 8; s.dispatchEvent(new Event('input', { bubbles: true })); }")
    gui.wait(lambda: all(0.06 < float(gui.sim.state()["m%d" % i]) < 0.10 for i in range(4)), what="all motors at 8 %")
    sent = []
    real = gui.hub.command
    gui.hub.command = lambda name, args: (sent.append(name), real(name, args))[1]
    gui.tab("overview")
    gui.wait(lambda: all(float(gui.sim.state()["m%d" % i]) == 0.0 for i in range(4)), what="stopped when the tab was left")
    assert "MOTOR_STOP" in sent, "the page stopped the motors itself instead of leaving it to the firmware's watchdog (%s)" % sent
    gui.hub.command = real
    # props out flips the picture; direction switch reaches the settings
    gui.tab("motors")
    pg.check('[data-field="motors.props_out"]')
    pg.wait_for_function("document.querySelector('#frame [data-motor=\"0\"]').dataset.spin === 'ccw'")
    pg.check('[data-rev="2"]')
    gui.wait(lambda: gui.setting("motors", "reversed_mask") == 4, what="reversed bit")
    pg.select_option('[data-order="0"]', "3")
    gui.wait(lambda: gui.setting("motors", "order")[0] == 3, what="order")


def test_vibration_graph(gui):
    pg = gui.connect().page
    gui.tab("motors")
    assert "gyro_fast" not in gui.hub.rates
    pg.check("#vib-on")
    gui.wait(lambda: gui.hub.rates.get("gyro_fast") == 500, what="fast stream on")
    gui.sim.do("shake 1")
    pg.wait_for_function("/RMS/.test(document.getElementById('vib-stats').textContent)", timeout=8000)
    pg.wait_for_function("(document.getElementById('vib-stats').textContent.match(/RMS/g) || []).length === 6", timeout=8000)
    text = pg.inner_text("#vib-stats")
    vals = [float(x) for x in re.findall(r"([\d.]+) °/s RMS", text)]
    assert len(vals) == 6 and all(vals[2 * i] > 1.0 and vals[2 * i + 1] < vals[2 * i] for i in range(3)), "every axis shakes, raw more than filtered: %s" % text
    assert pg.locator(".legend-item").filter(has_text="Roll").count() == 1 and pg.locator(".legend-item").filter(has_text="Yaw").count() == 1, "three lines, one per axis"
    pg.select_option("#vib-signal", "filt")
    pg.wait_for_function("(document.getElementById('vib-stats').textContent.match(/RMS/g) || []).length === 6", timeout=4000)
    gui.tab("overview")
    gui.wait(lambda: "gyro_fast" not in gui.hub.rates, what="fast stream off again when the tab is left")


def test_accelerometer_calibration_flow(gui):
    pg = gui.connect().page
    gui.sim.do("accerr 0.03 -0.02 0.04 1.01 0.99 1.02")
    gui.tab("sensors")
    assert "Chưa có hiệu chuẩn" in pg.inner_text("#acc-stored")
    for n, (roll, nose) in enumerate([(0, 0), (180, 0), (90, 0), (-90, 0), (0, 90), (0, -90)], 1):
        gui.sim.do("pose %d %d 0" % (roll, nose))
        gui.sim.sleep(0.4)
        pg.click('[data-cmd="ACC_CAL_POINT"]')
        pg.wait_for_function("n => document.querySelectorAll('#faces .face.done').length === n", arg=n, timeout=10000)
    assert "Đủ 6 mặt" in pg.inner_text("#acc-state")
    gui.sim.do("pose 0 0 0")
    pg.click("#acc-finish")
    pg.wait_for_function("document.getElementById('acc-stored').textContent.startsWith('Đang dùng hiệu chuẩn')", timeout=6000)
    cal = gui.hub.fc.get("calibration")
    assert cal["acc_valid"] is True and cal["acc_offset"] == pytest.approx([0.03, -0.02, 0.04], abs=0.006)
    assert "Đã tính xong" in pg.inner_text("#acc-state")
    pg.wait_for_selector("#save:not([hidden])")               # a calibration is a setting: it wants to be saved
    # level trim on a slightly tilted bench
    gui.sim.do("pose 2 -1 0")
    gui.sim.sleep(1.5)
    pg.click('[data-cmd="LEVEL_TRIM"]')
    gui.wait(lambda: abs(gui.hub.fc.get("calibration")["acc_trim_roll"]) > 1.0, what="level trim stored")
    pg.wait_for_function("Math.abs(Number(document.querySelector('[data-field=\"calibration.acc_trim_roll\"]').value)) > 1")


def test_pins_tab(gui):
    pg = gui.connect().page
    gui.tab("pins")
    assert gui.field("pins.imu_cs").input_value() == "10" and gui.field("pins.buzzer").input_value() == "21" and gui.field("pins.batt_adc").input_value() == "2"
    assert gui.field("pins.ppm").input_value() == "" and "Không thấy chân trùng" in pg.inner_text("#pin-warnings")
    gui.type_into("pins.batt_adc", "38")                        # not an ADC pin on the ESP32-S3
    pg.wait_for_function("/GPIO38 không đo được điện áp/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.batt_adc", "2")
    pg.wait_for_function("!/không đo được điện áp/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.buzzer", "10")
    pg.wait_for_function("/GPIO10 đang gán cho cả/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.led", "19")
    pg.wait_for_function("/USB D-/.test(document.getElementById('pin-warnings').textContent)")
    gui.wait(lambda: gui.setting("pins", "buzzer") == 10 and gui.setting("pins", "led") == 19, what="both pins reached the firmware")
    gui.type_into("pins.buzzer", "")                            # emptied box = pin not used
    gui.wait(lambda: gui.setting("pins", "buzzer") == 255, what="pin cleared")
    assert gui.field("pins.buzzer").input_value() == "" and gui.setting("pins", "led") == 19
    gui.type_into("pins.motor[4]", "21")
    gui.wait(lambda: gui.setting("pins", "motor")[4] == 21, what="motor slot pin")
    pg.wait_for_function("document.getElementById('save').textContent === 'Lưu và khởi động lại'")
    # ---- SD card pins: all of CLK / CMD / D0, and CS as well for SPI wiring
    gui.type_into("pins.sd_clk", "39")
    pg.wait_for_function("/Thẻ SD: cần đủ SCK/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.sd_cmd", "40"); gui.type_into("pins.sd_d0", "41")
    pg.wait_for_function("/Thẻ SD kiểu SPI cần thêm chân CS/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.sd_cs", "36")                            # a pin of the N16R8's PSRAM
    pg.wait_for_function("/GPIO36 thường là PSRAM của module R8/.test(document.getElementById('pin-warnings').textContent)")
    assert "Thẻ SD kiểu SPI" not in pg.inner_text("#pin-warnings")
    # ---- a conflict that is saved: the firmware finds it at boot, refuses to arm, and the Overview names the pin
    gui.type_into("pins.buzzer", "2")                            # the battery's ADC pin
    pg.wait_for_function("/GPIO2 đang gán cho cả/.test(document.getElementById('pin-warnings').textContent)")
    boots = int(gui.sim.state()["boots"])
    pg.click("#save")
    gui.wait(lambda: int(gui.sim.state()["boots"]) == boots + 1, 12, "firmware restart")
    pg.wait_for_selector("#save", state="hidden", timeout=12000)
    gui.tab("overview")
    pg.wait_for_selector('[data-module="pins"]', timeout=8000)
    row = pg.inner_text('[data-module="pins"]')
    assert "Lỗi" in row and "GPIO2 is set for both batt_adc and buzzer" in row and "FAILED" not in row, row
    pg.wait_for_selector('#blockers [data-blocker="PIN_CONFLICT"]')
    assert "dùng chung một chân GPIO" in pg.inner_text("#blockers") and pg.inner_text("#arm-pill") == "Chưa arm được"
    gui.tab("pins")
    gui.type_into("pins.buzzer", "")
    gui.wait(lambda: gui.setting("pins", "buzzer") == 255, what="pin cleared again")
    # ---- the suggested pin map: fills the boxes (nothing is saved), keeps what it does not know about, and is itself free of warnings
    asked = []
    pg.once("dialog", lambda d: (asked.append(d.message), d.dismiss()))
    pg.click("#pins-preset")
    gui.sim.sleep(0.3)
    assert len(asked) == 1 and "ESP32-S3 DevKitC" in asked[0] and gui.setting("pins", "batt_adc") == 2 and gui.setting("pins", "sd_cs") == 36, "saying no changes nothing"
    pg.once("dialog", lambda d: d.accept())
    pg.click("#pins-preset")
    gui.wait(lambda: gui.setting("pins", "batt_adc") == 1 and gui.setting("pins", "sd_cs") == 42 and gui.setting("pins", "gps_rx") == 18, what="preset reached the firmware (RAM)")
    p = gui.hub.fc.get("pins")
    assert p["motor"] == [4, 5, 6, 7, 255, 255, 255, 255] and p["imu_cs"] == 10 and p["sd_clk"] == 39 and p["led"] == 19 and p["buzzer"] == 255
    assert gui.field("pins.batt_adc").input_value() == "1" and gui.field("pins.motor[4]").input_value() == ""
    gui.type_into("pins.led", "")
    pg.wait_for_function("/Không thấy chân trùng/.test(document.getElementById('pin-warnings').textContent)")


def test_pid_tab_sliders_compute_the_gains(gui):
    pg = gui.connect().page
    gui.tab("tuning")
    val = lambda f: gui.field(f).input_value()
    out = lambda f: pg.locator('.slider:has([data-field="%s"]) output' % f).inner_text()
    # as shipped: the sliders own all three axes - the boxes show Betaflight's defaults and cannot be typed into
    assert [val("pid.p[%d]" % a) for a in range(3)] == ["45", "47", "45"] and val("pid.d_max[1]") == "46" and val("pid.f[1]") == "125"
    assert all(gui.field(f).is_disabled() for f in ("pid.p[0]", "pid.i[1]", "pid.d_max[1]", "pid.f[2]")) and "Cả ba trục" in pg.inner_text("#slider-state")
    assert out("pid.slider_pi") == "1.00" and out("pid.slider_master") == "1.00"
    # every column of the gain table holds the gain its heading names
    heads = pg.evaluate("() => [...document.querySelectorAll('#pid-gains thead th')].slice(1).map((e) => e.textContent)")
    cells = pg.evaluate("() => [...document.querySelectorAll('#pid-gains tbody tr:first-child td [data-field]')].map((e) => e.dataset.field)")
    assert dict(zip(heads, cells)) == {"P": "pid.p[0]", "I": "pid.i[0]", "D": "pid.d[0]", "D Max": "pid.d_max[0]", "Feedforward": "pid.f[0]"}, (heads, cells)
    # while the thumb is dragged only the number next to it moves; the firmware gets ONE write when it is let go
    pg.evaluate("() => { const e = document.querySelector('[data-field=\"pid.slider_pi\"]'); e.value = 150; e.dispatchEvent(new Event('input', { bubbles: true })); }")
    assert out("pid.slider_pi") == "1.50" and gui.setting("pid", "slider_pi") == 100 and val("pid.p[0]") == "45"
    # ... also when something else in the section changes meanwhile (the page hears about it): the thumb stays where it is held
    pg.select_option('[data-field="pid.iterm_relax"]', "RPY")
    gui.wait(lambda: gui.setting("pid", "iterm_relax") == "RPY", what="another PID setting changed while the slider is held")
    pg.wait_for_timeout(400)
    assert val("pid.slider_pi") == "150" and out("pid.slider_pi") == "1.50", "the thumb snapped back while it was held"
    assert gui.setting("pid", "slider_pi") == 100 and val("pid.p[0]") == "45", "nothing was written for the slider before it was let go"
    pg.select_option('[data-field="pid.iterm_relax"]', "RP")
    gui.wait(lambda: gui.setting("pid", "iterm_relax") == "RP", what="iterm_relax back")
    pg.evaluate("() => document.querySelector('[data-field=\"pid.slider_pi\"]').dispatchEvent(new Event('change', { bubbles: true }))")
    pg.wait_for_function("document.querySelector('[data-field=\"pid.p[0]\"]').value === '67'")           # Tracking 1.5: P and I, computed by the firmware
    assert [val("pid.p[%d]" % a) for a in range(3)] == ["67", "70", "67"] and [val("pid.i[%d]" % a) for a in range(3)] == ["120", "126", "120"] and val("pid.d[0]") == "30" and val("pid.f[0]") == "120"
    assert gui.setting("pid", "p") == [67, 70, 67] and gui.setting("pid", "slider_pi") == 150
    gui.field("pid.slider_d").fill("50")                                                                    # Damping 0.5
    pg.wait_for_function("document.querySelector('[data-field=\"pid.d[1]\"]').value === '17'")
    assert val("pid.d[0]") == "15" and val("pid.d_max[0]") == "20" and out("pid.slider_d") == "0.50"
    # sliders for roll + pitch only: the yaw row opens
    pg.select_option('[data-field="pid.slider_mode"]', "RP")
    pg.wait_for_function("!document.querySelector('[data-field=\"pid.p[2]\"]').disabled")
    assert gui.field("pid.p[0]").is_disabled() and gui.field("pid.p[1]").is_disabled() and "yaw nhập tay" in pg.inner_text("#slider-state")
    gui.type_into("pid.p[2]", "90")
    gui.wait(lambda: gui.setting("pid", "p") == [67, 70, 90], what="yaw P typed by hand, roll / pitch still from the slider")
    # sliders off: every box is the user's, the sliders are greyed out
    pg.select_option('[data-field="pid.slider_mode"]', "OFF")
    pg.wait_for_function("!document.querySelector('[data-field=\"pid.p[0]\"]').disabled && document.querySelector('[data-field=\"pid.slider_pi\"]').disabled")
    assert pg.is_disabled("#sliders-reset") and "đang tắt" in pg.inner_text("#slider-state")
    gui.type_into("pid.p[0]", "51"); gui.type_into("pid.f[1]", "2000")
    assert gui.field("pid.f[1]").input_value() == "1000"                                                    # the box knows the limit
    gui.wait(lambda: gui.setting("pid", "slider_mode") == "OFF" and gui.setting("pid", "p")[0] == 51 and gui.setting("pid", "f")[1] == 1000, what="typed gains reach the firmware")
    # back on: what was typed is replaced by what the sliders give; one button brings them all back to 1.00
    pg.select_option('[data-field="pid.slider_mode"]', "RPY")
    pg.wait_for_function("document.querySelector('[data-field=\"pid.p[0]\"]').value === '67' && document.querySelector('[data-field=\"pid.p[2]\"]').value === '67'")
    pg.click("#sliders-reset")
    pg.wait_for_function("document.querySelector('[data-field=\"pid.p[0]\"]').value === '45' && document.querySelector('[data-field=\"pid.d[1]\"]').value === '34'")
    assert out("pid.slider_pi") == "1.00" and out("pid.slider_d") == "1.00" and gui.setting("pid", "slider_d") == 100
    # the other groups of the tab are there and write
    pg.select_option('[data-field="pid.iterm_relax"]', "RPY"); pg.select_option('[data-field="pid.ff_averaging"]', "3_POINT"); gui.type_into("pid.stab_p[0]", "6.5"); gui.type_into("pid.angle_strength", "70")
    pg.uncheck('[data-field="pid.anti_gravity"]')
    gui.wait(lambda: gui.setting("pid", "iterm_relax") == "RPY" and gui.setting("pid", "ff_averaging") == "3_POINT" and gui.setting("pid", "stab_p")[0] == pytest.approx(6.5) and gui.setting("pid", "angle_strength") == 70 and
             gui.setting("pid", "anti_gravity") is False, what="PID controller options")
    assert gui.field("rates.angle_limit").input_value() == "45"
    # live values of the firmware
    pg.wait_for_function("/80 %/.test(document.querySelector('#view dl.kv').textContent)")                  # TPA at zero throttle (the low-throttle part)
    assert pg.inner_text("#air-state") == "trên mặt đất: motor ở idle"                                       # disarmed: the motors are not flown by the rate loop
    assert pg.inner_text("#hover-read") == "37 %", "the hover throttle before the first flight is the firmware's guess"


def test_filters_tab_sliders_map_and_live_values(gui):
    pg = gui.connect().page
    gui.tab("filters")
    val = lambda f: gui.field(f).input_value()
    assert val("filters.gyro_lpf1_dyn_min_hz") == "250" and val("filters.gyro_lpf1_dyn_max_hz") == "500" and val("filters.gyro_lpf2_hz") == "500" and val("filters.dterm_lpf1_dyn_min_hz") == "75" and val("filters.dterm_lpf2_hz") == "150"
    assert all(gui.field(f).is_disabled() for f in ("filters.gyro_lpf1_dyn_min_hz", "filters.gyro_lpf1_static_hz", "filters.gyro_lpf2_hz", "filters.dterm_lpf1_dyn_max_hz", "filters.dterm_lpf2_hz")), "the sliders own the low-pass cutoffs"
    assert not gui.field("filters.gyro_lpf1_type").is_disabled() and not gui.field("filters.gyro_notch1_hz").is_disabled() and not gui.field("filters.dyn_notch_count").is_disabled() and not gui.field("filters.yaw_lowpass_hz").is_disabled()
    pg.wait_for_function("/Gyro lowpass 1: 250 Hz/.test(document.getElementById('filter-read').textContent) && /D-term lowpass 1: 75 Hz/.test(document.getElementById('filter-read').textContent)")
    ink0 = pg.evaluate("() => { const cv = document.getElementById('filter-map'); const d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data; let n = 0; for (let i = 3; i < d.length; i += 4) if (d[i] > 30) n++; return n; }")
    assert ink0 > 3000, "the filter map is drawn (%d px)" % ink0
    # the gyro lane (upper half) and the D-term lane (lower half) each carry the shade of their two low-passes
    LANES = "() => { const cv = document.getElementById('filter-map'); const d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data; const n = [0, 0]; for (let y = 0; y < cv.height; y++) for (let x = 0; x < cv.width; x++) if (d[(y * cv.width + x) * 4 + 3] > 30) n[y < cv.height / 2 ? 0 : 1]++; return n; }"
    gyro0, dterm0 = pg.evaluate(LANES)
    assert gyro0 > 0.5 * dterm0 and dterm0 > gyro0, "both lanes are shaded from their cutoffs on; the D-term's start lower (%d, %d px)" % (gyro0, dterm0)
    # gyro slider to 0.60: more filtering - the firmware computes the cutoffs, the running filter moves at once
    gui.field("filters.slider_gyro_mult").fill("60")
    pg.wait_for_function("document.querySelector('[data-field=\"filters.gyro_lpf1_dyn_min_hz\"]').value === '150'")
    gui.wait(lambda: pg.evaluate(LANES)[0] > gyro0 * 1.1 and abs(pg.evaluate(LANES)[1] - dterm0) < 0.02 * dterm0, what="the gyro low-passes moved down on the map (more of their lane is shaded), the D-term lane did not change")
    assert val("filters.gyro_lpf1_dyn_max_hz") == "300" and val("filters.gyro_lpf2_hz") == "300" and val("filters.dterm_lpf2_hz") == "150" and pg.locator('.slider:has([data-field="filters.slider_gyro_mult"]) output').inner_text() == "0.60"
    pg.wait_for_function("/Gyro lowpass 1: 150 Hz/.test(document.getElementById('filter-read').textContent)")
    gui.field("filters.slider_dterm_mult").fill("120")
    pg.wait_for_function("document.querySelector('[data-field=\"filters.dterm_lpf2_hz\"]').value === '180'")
    assert gui.setting("filters", "dterm_lpf1_dyn_min_hz") == 90 and gui.setting("filters", "gyro_lpf2_hz") == 300
    # slider off: the boxes open and what is typed stands
    pg.uncheck('[data-field="filters.slider_gyro"]')
    pg.wait_for_function("!document.querySelector('[data-field=\"filters.gyro_lpf2_hz\"]').disabled && document.querySelector('[data-field=\"filters.slider_gyro_mult\"]').disabled")
    assert gui.field("filters.dterm_lpf2_hz").is_disabled(), "the D-term slider is still on"
    gui.type_into("filters.gyro_lpf1_dyn_min_hz", "333"); gui.type_into("filters.gyro_lpf2_hz", "5000")
    pg.wait_for_function("document.querySelector('[data-field=\"filters.gyro_lpf2_hz\"]').value === '1000'")
    pg.wait_for_function("/Gyro lowpass 1: 333 Hz/.test(document.getElementById('filter-read').textContent)")
    # notches, dynamic notch, RPM filter
    gui.type_into("filters.gyro_notch1_hz", "240"); gui.type_into("filters.gyro_notch1_cutoff", "200"); gui.type_into("filters.dyn_notch_count", "5"); gui.type_into("filters.rpm_weights[2]", "0"); pg.select_option('[data-field="filters.dterm_lpf1_type"]', "PT2")
    gui.wait(lambda: gui.setting("filters", "gyro_notch1_hz") == 240 and gui.setting("filters", "dyn_notch_count") == 5 and gui.setting("filters", "rpm_weights") == [100, 100, 0] and gui.setting("filters", "dterm_lpf1_type") == "PT2", what="filter settings")
    assert pg.evaluate(INK, ["#filter-map", YELLOW]) > 40, "the static notch is marked on the map"
    kv = pg.locator("#view dl.kv").inner_text()
    assert "333 Hz" in kv and "Dynamic notch roll" in kv and kv.count(" Hz") >= 5 and "không có" in kv, kv
    assert "ok" not in pg.get_attribute("#rpm-note", "class") and "cần DShot hai chiều" in pg.inner_text("#rpm-note")
    gui.tab("motors"); pg.check('[data-field="motors.bidirectional"]'); gui.wait(lambda: gui.setting("motors", "bidirectional") is True, what="bidirectional DShot on"); gui.tab("filters")
    assert "ok" in pg.get_attribute("#rpm-note", "class") and "đang bật" in pg.inner_text("#rpm-note")
    # armed: everything is locked
    gui.sim.do("rc 3 1000"); gui.sim.do("rc 5 2000")
    pg.wait_for_selector("#arm-pill.bad")
    assert gui.field("filters.gyro_notch1_hz").is_disabled() and gui.field("filters.slider_dterm_mult").is_disabled() and gui.field("filters.gyro_lpf2_hz").is_disabled()
    # throttle up: the dynamic D-term low-pass (90 .. 180 Hz with the slider at 1.20) opens, and the map follows the running filter
    assert "D-term lowpass 1: 90 Hz" in pg.inner_text("#filter-read")
    gui.sim.do("rc 3 1700")
    pg.wait_for_function("(() => { const m = /D-term lowpass 1: (\\d+) Hz/.exec(document.getElementById('filter-read').textContent); return m && Number(m[1]) > 140; })()")
    # (the gyro low-pass 1 stands at 333 Hz: a readout that shows the wrong filter is not between 140 and 180)
    pg.wait_for_function("(() => { const m = /D-term lowpass 1\\s*(\\d+) Hz/.exec(document.querySelector('#view dl.kv').innerText); return m && Number(m[1]) > 140 && Number(m[1]) <= 180; })()")
    gui.sim.do("rc 3 1000"); gui.sim.do("rc 5 1000")


def test_inflight_level_trim_is_shown_and_saves_itself(gui):
    pg = gui.connect().page
    fc = gui.hub.fc
    m = fc.get("modes"); m["mode"][6] = "CALIB"; m["aux"][6] = 5; m["lo"][6] = 1700; m["hi"][6] = 2100
    fc.set("modes", **m); fc.save()
    gui.tab("sensors")
    pg.wait_for_function("/chưa đo/.test(document.getElementById('inflight-trim').textContent)")
    # one IMU on the simulated board: the second one is "none", its settings are there
    pg.wait_for_function("document.getElementById('imu2-state').textContent === 'không có'")
    assert gui.field("sensors.gyro_use").is_visible() and gui.field("sensors.imu2_align").is_visible()
    # the CAN bus is off on the simulated board: its settings are there, its state says so
    pg.wait_for_function("(document.getElementById('can-state') || { textContent: '' }).textContent === 'tắt'", timeout=6000)
    assert gui.field("can.mode").is_visible() and gui.field("can.node_id").input_value() == "10"
    assert gui.field("sensors.baro_model").is_visible() and "DroneCAN" in pg.locator('[data-field="sensors.baro_model"] option').all_inner_texts()
    assert gui.field("calibration.acc_trim_roll").input_value() == "0.0"
    sim = gui.sim
    # The board sits 2.2 deg crooked in the frame: with the sticks centred the craft would lean and drift away. The pilot holds
    # the roll stick where it stands still (0.2 to the right) from the take-off on.
    sim.do("mount -2.2 0"); time.sleep(0.5)
    sim.do("rc 6 2000"); sim.do("rc 7 2000"); time.sleep(0.3); sim.do("rc 5 2000"); time.sleep(0.5)       # ANGLE + ALTHOLD, arm
    sim.do("rc 1 1600"); sim.do("rc 3 1700"); time.sleep(3.0); sim.do("rc 3 1500"); time.sleep(1.5)       # climb, hover
    sim.do("rc 10 2000")                                                                                   # CALIB on
    pg.wait_for_function("/đang đo: \\d+ %|đã đo xong/.test(document.getElementById('inflight-trim').textContent)")
    pg.wait_for_function("/đã đo xong/.test(document.getElementById('inflight-trim').textContent)", timeout=6000)      # 2 s of steady hover (the simulator runs 4 x real time)
    sim.do("rc 10 1000"); time.sleep(0.3)
    assert "đã đo xong" in pg.inner_text("#inflight-trim")
    assert gui.setting("calibration", "acc_trim_roll") == 0, "nothing changes in the air"
    sim.do("rc 1 1500"); sim.do("rc 3 1000")                                                               # down, land
    gui.wait(lambda: float(sim.state()["z"]) < 0.02, 20, "landed")
    time.sleep(1.2); sim.do("rc 5 1000")
    pg.wait_for_function("/Firmware vừa tự lưu/.test(document.getElementById('toasts').textContent)", timeout=8000)
    # hovering still, its accelerometer showed where the thrust points relative to the board: 2.2 deg off. That is the trim
    gui.wait(lambda: -3.0 < gui.setting("calibration", "acc_trim_roll") < -1.5, what="the trim the pilot held")
    pg.wait_for_function("Math.abs(Number(document.querySelector('[data-field=\"calibration.acc_trim_roll\"]').value) + 2.2) < 0.6")    # the page re-read the section
    pg.wait_for_function("/chưa đo/.test(document.getElementById('inflight-trim').textContent)")
    assert pg.is_hidden("#save"), "written to flash by the firmware itself: nothing left to save"
    sim.power_cycle(); pg.wait_for_selector(".welcome"); gui.connect()                                    # pull the plug: only what is in flash survives
    assert -3.0 < gui.setting("calibration", "acc_trim_roll") < -1.5, "it is in the flash"


def test_armed_locks_the_page_and_port_loss_is_shown(gui):
    pg = gui.connect().page
    gui.tab("tuning")
    # (sliders for roll + pitch only: the yaw gains are typed by hand, so what locks them below is the arming, not a slider)
    pg.select_option('[data-field="pid.slider_mode"]', "RP")
    pg.wait_for_function("!document.querySelector('[data-field=\"pid.p[2]\"]').disabled")
    gui.sim.do("rc 3 1000"); gui.sim.do("rc 5 2000")
    pg.wait_for_selector("#arm-pill.bad")
    assert pg.inner_text("#arm-pill") == "ARMED" and "Đang ARM" in pg.inner_text("#banner")
    assert gui.field("pid.p[0]").is_disabled() and gui.field("pid.iterm_relax").is_disabled()
    assert gui.field("pid.p[2]").is_disabled() and gui.field("pid.d_max[2]").is_disabled(), "a gain box that no slider computes is locked while armed as well"
    gui.tab("pins")
    assert pg.is_disabled("#pins-preset") and gui.field("pins.sd_cs").is_disabled()
    gui.tab("motors")
    pg.check("#props-off")
    assert pg.locator('[data-test="0"]').is_disabled() and pg.locator('[data-cmd="MOTOR_BEACON"]').first.is_disabled()
    gui.sim.do("rc 5 1000")
    pg.wait_for_selector("#arm-pill.ok, #arm-pill.warn")
    assert pg.locator("#banner").is_hidden() and not pg.locator('[data-test="0"]').is_disabled()
    gui.sim.stop()                                              # the cable is pulled
    pg.wait_for_selector(".welcome", timeout=8000)
    assert "Mất kết nối" in pg.inner_text("#banner") and pg.inner_text("#connect") == "Kết nối" and not pg.locator("#port").is_disabled()


def test_loop_timing_dshot_state_and_failsafe_cause(gui):
    """What the first flights on a real board asked for: where the cycle time goes, whether the ESCs answer, why a failsafe is on."""
    pg = gui.connect().page
    pg.wait_for_function("/đọc IMU \\d+ · tính toán \\d+ · gửi motor \\d+ · còn lại \\d+ µs/.test(document.getElementById('stage-times').textContent)", timeout=6000)
    pg.wait_for_function("/^\\d+ Hz \\(đặt 4000 Hz\\), bỏ lỡ \\d+ chu kỳ trong giây vừa rồi$/.test(document.getElementById('loop-rate').textContent)", timeout=6000)
    pg.wait_for_function("/^\\d+ Hz \\(đặt 4000 Hz = gyro \\/ 1\\)$/.test(document.getElementById('pid-rate').textContent)", timeout=6000)
    assert pg.inner_text("#fs-state") == "không" and re.search(r"\d+ µs$", pg.inner_text("#slow-max"))
    assert re.search(r"\d+ / 4000 Hz", pg.inner_text("#chips")), pg.inner_text("#chips")
    # ---- motors: whether the ESCs answer (0.12: no choice of DShot driver any more; motor idle only with DShot)
    gui.tab("motors")
    assert gui.field("motors.dshot_driver").count() == 0 and pg.locator("#idle-field").is_visible()
    proto = gui.field("motors.protocol").input_value()
    pg.select_option('[data-field="motors.protocol"]', "ONESHOT125")
    pg.wait_for_function("document.getElementById('idle-field').hidden && !document.getElementById('pulse-fields').hidden")
    pg.select_option('[data-field="motors.protocol"]', proto)
    pg.wait_for_function("!document.getElementById('idle-field').hidden")
    pg.wait_for_function("(document.getElementById('dshot-state') || { innerText: '' }).innerText.includes('DShot hai chiều: tắt')")
    pg.check('[data-field="motors.bidirectional"]')
    pg.wait_for_function("document.getElementById('save').textContent === 'Lưu và khởi động lại'")
    pg.click("#save")
    pg.wait_for_selector("#save", state="hidden", timeout=12000)
    pg.wait_for_function("(document.getElementById('dshot-state') || { innerText: '' }).innerText.includes('DShot hai chiều: đang chạy, mọi ESC đều trả lời')", timeout=8000)
    gui.sim.do("esc 2 0")
    pg.wait_for_function("(document.getElementById('dshot-state') || { innerText: '' }).innerText.includes('ESC không trả lời: 2')", timeout=6000)
    gui.sim.do("esc 2 1")
    pg.wait_for_function("(document.getElementById('dshot-state') || { innerText: '' }).innerText.includes('mọi ESC đều trả lời')", timeout=6000)
    assert "Khung DShot bỏ qua trong giây vừa rồi: 0" in pg.inner_text("#dshot-state")
    # ---- safety settings
    gui.tab("system")
    assert gui.field("general.runaway_takeoff").is_checked() and gui.field("general.fs_recover_ms").input_value() == "1000"
    # ---- a failsafe says why
    gui.tab("overview")
    gui.sim.do("rc 3 1000"); gui.sim.do("rc 6 2000"); gui.sim.do("rc 7 2000")
    # (the board has just rebooted: the gyro calibrates, the barometer settles. An arm switch that goes on while something
    #  still blocks is refused and has to be cycled - so the pilot waits for "nothing blocks", and so does this test)
    gui.wait(lambda: "ALTHOLD" in _status(gui).get("modes", []) and _status(gui).get("blockers") == [], timeout=15.0, what="the mode switches have arrived and nothing blocks arming")
    pg.wait_for_selector("#blockers li.ok", timeout=5000)
    gui.sim.do("rc 5 2000")
    pg.wait_for_selector("#arm-pill.bad")
    gui.sim.do("rc 3 1750"); time.sleep(3.0); gui.sim.do("rc 3 1500")
    gui.sim.do("tx off")
    pg.wait_for_function("document.getElementById('ready').textContent === 'Failsafe: đang tự hạ cánh (mất sóng)'", timeout=8000)
    assert pg.inner_text("#fs-state") == "đang tự hạ cánh (mất sóng)"
    gui.sim.do("tx on")
    pg.wait_for_function("!document.getElementById('ready').textContent.startsWith('Failsafe')", timeout=8000)
    assert pg.inner_text("#arm-pill") == "ARMED" and pg.inner_text("#fs-state") == "không"
    gui.sim.do("rc 3 1000"); time.sleep(5.0); gui.sim.do("rc 5 1000")
    pg.wait_for_selector("#arm-pill.ok, #arm-pill.warn", timeout=10000)


def _status(gui):
    v = gui.hub.latest.get("status")
    return next((x for x in v if isinstance(x, dict)), {}) if isinstance(v, tuple) else (v or {})


# Pixels of the flow dial's arrow (the page's first series colour, as its CSS says), by where they lie around the middle: [up, down, left, right]
DIAL = """() => { const cv = document.getElementById('flow-dial'); const d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data;
  const hex = getComputedStyle(document.documentElement).getPropertyValue('--series-1').trim().slice(1);
  const c = [0, 2, 4].map((k) => parseInt(hex.slice(k, k + 2), 16));
  const n = [0, 0, 0, 0], cx = cv.width / 2, cy = cv.height / 2;
  for (let y = 0; y < cv.height; y++) for (let x = 0; x < cv.width; x++) { const i = (y * cv.width + x) * 4;
    if (d[i + 3] > 200 && Math.abs(d[i] - c[0]) < 30 && Math.abs(d[i + 1] - c[1]) < 30 && Math.abs(d[i + 2] - c[2]) < 30) { const dx = x - cx, dy = y - cy; if (Math.abs(dy) > Math.abs(dx)) n[dy < 0 ? 0 : 1]++; else n[dx < 0 ? 2 : 3]++; } }
  return n; }"""


def _arrow(pg):
    """Which way the dial's arrow points ('up', 'down', 'left', 'right'), or None when there is none / it is not clear."""
    n = pg.evaluate(DIAL)
    k = max(range(4), key=lambda i: n[i])
    return ("up", "down", "left", "right")[k] if n[k] > 150 and n[k] > 6 * (sum(n) - n[k]) else None


def test_position_tab(gui):
    """Optical flow from "not set up" to a craft that holds its position: what the firmware says about the sensor, the live dial
    for checking how it is mounted (settings that act at once), the states of Position Hold, and a rescue that cannot work."""
    pg = gui.connect().page
    txt = lambda sel: pg.inner_text(sel)
    gui.tab("nav")
    pg.wait_for_function("document.getElementById('flow-rate').textContent === '0 mẫu/giây'", timeout=6000)        # (the first frame of the nav stream)
    assert txt("#flow-report") == "chưa chọn cảm biến" and txt("#flow-use") == "cảm biến không gửi mẫu nào" and txt("#ph-state") == "tắt" and txt("#rescue-ready") == "chưa" and txt("#rescue-phase") == "không chạy"
    pg.wait_for_function("document.getElementById('est-source').textContent === 'chưa có'", timeout=6000)
    assert sum(pg.evaluate(DIAL)) == 0, "no arrow while nothing is measured"
    # the dial's scale: the outer ring is 1 m/s, or the next whole m/s above a faster speed (forward = up, right = right)
    geo = pg.evaluate("async () => { const m = await import(new URL('js/tabs/nav.js', document.baseURI).href); return [m.dialGeometry(200, 200, 2.5, 0), m.dialGeometry(200, 200, 0, 0.5)]; }")
    assert geo[0]["full"] == 3 and abs(geo[0]["y"] - (100 - 2.5 / 3 * 76)) < 1e-6 and geo[0]["x"] == 100, geo
    assert geo[1]["full"] == 1 and geo[1]["x"] == 138 and geo[1]["y"] == 100, geo
    assert not gui.field("nav.ph_angle_max").is_disabled() and gui.field("nav.ph_angle_max").input_value() == "30" and gui.field("nav.rescue_ground_speed").input_value() == "7.5"
    assert gui.field("nav.gps_alt").input_value() == "FALLBACK" and gui.field("nav.gps_vel_z").is_checked(), "GPS height: by default only without a barometer; the GPS vertical velocity on"
    assert txt("#est-gps-alt") == "không dùng lúc này" and txt("#est-gps-vz") == "không dùng lúc này", "disarmed there is no home point, so no GPS height either"
    # ---- choose the sensor: read at power-up
    pg.select_option('[data-field="flow.type"]', "MSP")
    pg.wait_for_function("document.getElementById('flow-report').textContent === 'firmware chưa khởi động lại từ khi chọn cảm biến'")
    pg.wait_for_function("document.getElementById('save').textContent === 'Lưu và khởi động lại'")
    boots = int(gui.sim.state()["boots"])
    pg.click("#save")
    gui.wait(lambda: int(gui.sim.state()["boots"]) == boots + 1, 12, "firmware restart")
    pg.wait_for_selector("#save", state="hidden", timeout=12000)
    pg.wait_for_function("(document.getElementById('flow-report') || {}).textContent === 'MSP flow on UART0'", timeout=8000)
    pg.wait_for_function("/^(9\\d|10[01]) mẫu\\/giây$/.test((document.getElementById('flow-rate') || {}).textContent)", timeout=8000)
    assert txt("#flow-quality") == "chất lượng 78 / 100" and "quá gần sàn" in txt("#flow-use") and txt("#flow-speed") == "–" and _arrow(pg) is None
    # ---- carried by hand: the dial shows where the craft goes, nose up
    gui.sim.do("carry 1.0 0.6 0")
    pg.wait_for_function("/^tiến 0\\.(5[5-9]|6\\d) m\\/s, sang phải -?0\\.0\\d m\\/s$/.test(document.getElementById('flow-speed').textContent)", timeout=8000)
    assert txt("#flow-use") == "đang dùng" and txt("#est-source") == "optical flow" and re.match(r"^0\.9\d m$", txt("#flow-range")), txt("#flow-range")
    gui.wait(lambda: _arrow(pg) == "up", what="carried forwards: the arrow points up")
    gui.sim.do("carry 1.0 0 0.6")
    gui.wait(lambda: _arrow(pg) == "right", what="carried to the right (east, nose north): the arrow points right")
    # ---- a sensor that sits a quarter turn off and sees a mirrored picture: set right on the page, no restart
    gui.sim.do("carry 1.0 0.6 0"); gui.sim.do("flow mount 90 1 1")
    gui.wait(lambda: _arrow(pg) in ("left", "right"), what="carried forwards, the arrow points sideways: mounted a quarter turn off")
    pg.select_option('[data-field="nav.flow_align"]', "CW90")
    gui.wait(lambda: _arrow(pg) == "down", what="mounting direction 90: forwards now reads backwards - the picture is mirrored")
    pg.check('[data-field="nav.flow_flip"]')
    gui.wait(lambda: _arrow(pg) == "up", what="mirrored picture set: the arrow points up")
    pg.wait_for_function("/^tiến 0\\.(5[5-9]|6\\d) m\\/s/.test(document.getElementById('flow-speed').textContent)", timeout=6000)
    assert gui.setting("nav", "flow_align") == "CW90" and gui.setting("nav", "flow_flip") is True and txt("#save") == "Lưu", "the nav section is live: nothing asks for a restart"
    gui.sim.do("carry off")
    # ---- Position Hold on a switch (Modes tab), then a flight
    gui.tab("modes")
    ph = pg.locator('[data-mode="POSHOLD"]')
    assert "todo" not in ph.get_attribute("class") and "tab Vị trí" in ph.locator(".help").inner_text() and pg.locator('[data-mode="RESCUE"] h3').inner_text() == "GPS Rescue"
    ph.locator("[data-add]").click()
    ph.locator(".mode-range").wait_for()
    ph.locator("select").select_option("3")                     # AUX4 = channel 8
    gui.wait(lambda: "POSHOLD" in gui.setting("modes", "mode") and gui.setting("modes", "aux")[gui.setting("modes", "mode").index("POSHOLD")] == 3, what="Position Hold on AUX4")
    gui.tab("nav")
    gui.sim.do("rc 3 1000"); gui.sim.do("rc 6 2000"); gui.sim.do("rc 7 2000"); gui.sim.do("rc 8 2000")
    gui.wait(lambda: "POSHOLD" in _status(gui).get("modes", []) and _status(gui).get("blockers") == [], timeout=15.0, what="the switches have arrived and nothing blocks arming (a flow sensor is something to hold with)")
    gui.sim.do("rc 5 2000")
    pg.wait_for_selector("#arm-pill.bad")
    assert gui.field("nav.ph_angle_max").is_disabled() and gui.field("nav.flow_align").is_disabled() and gui.field("flow.type").is_disabled(), "armed: the settings are locked"
    gui.sim.do("rc 3 1700"); time.sleep(3.0); gui.sim.do("rc 3 1500")
    pg.wait_for_function("document.getElementById('ph-state').textContent === 'đang giữ vị trí' && document.getElementById('ph-state').classList.contains('ok')", timeout=8000)
    pg.wait_for_function("document.getElementById('ph-axes').textContent === 'đang giữ vị trí / đang giữ vị trí'", timeout=10000)
    pg.wait_for_function("document.getElementById('est-tilt').textContent === 'đang được chỉnh theo gia tốc đo từ GPS / optical flow'", timeout=6000)
    pg.wait_for_function("document.getElementById('est-gps-vz').textContent === 'đang dùng trong ước lượng độ cao'", timeout=6000)
    assert txt("#est-gps-alt") == "không dùng lúc này", "with a barometer the GPS height stays out (FALLBACK), its vertical velocity is in"
    assert "POSHOLD" in txt("#ready") and re.match(r"^-?\d+\.\d / -?\d+\.\d m$", txt("#ph-target")), (txt("#ready"), txt("#ph-target"))
    n0, e0 = float(gui.sim.state()["pn"]), float(gui.sim.state()["pe"])
    gui.sim.do("wind 0 4")                                       # the air moves east: the craft leans left (west) against it
    pg.wait_for_function("(() => { const m = /^(-?\\d+\\.\\d) \\/ (-?\\d+\\.\\d) °$/.exec(document.getElementById('ph-lean').textContent); return !!m && Number(m[1]) < -5; })()", timeout=12000)
    pg.wait_for_function("(() => { const m = /^(-?\\d+\\.\\d+) \\/ (-?\\d+\\.\\d+) m\\/s²$/.exec(document.getElementById('ph-wind').textContent); return !!m && Number(m[2]) < -0.8; })()", timeout=15000)
    st = gui.sim.state()
    assert ((float(st["pn"]) - n0) ** 2 + (float(st["pe"]) - e0) ** 2) ** 0.5 < 1.5, "it stays where it was in a 4 m/s wind: %s" % st
    # ---- the floor gives the camera nothing: Position Hold says it cannot
    gui.sim.do("wind 0 0"); time.sleep(3.0); gui.sim.do("flow quality 20")
    pg.wait_for_function("document.getElementById('ph-state').classList.contains('warn')", timeout=6000)
    assert "chưa có gì để giữ vị trí" in txt("#ph-state") and "bề mặt quá mờ" in txt("#flow-use") and txt("#flow-quality") == "chất lượng 7 / 100" and txt("#ph-axes") == "–"
    gui.sim.do("flow quality 200")
    pg.wait_for_function("document.getElementById('ph-state').classList.contains('ok')", timeout=6000)
    gui.sim.do("rc 3 1250"); time.sleep(4.0); gui.sim.do("rc 3 1000"); time.sleep(1.0); gui.sim.do("rc 5 1000")
    pg.wait_for_selector("#arm-pill.ok, #arm-pill.warn", timeout=10000)
    pg.wait_for_function("document.getElementById('ph-state').textContent === 'tắt' || document.getElementById('ph-state').classList.contains('warn')", timeout=6000)
    gui.sim.do("rc 8 1000")
    # ---- a rescue that cannot work (no compass) is refused at arming, and the bar says why
    pg.select_option('[data-field="nav.fs_procedure"]', "RESCUE")
    pg.wait_for_function("/la bàn đang tắt hoặc không có/.test(document.getElementById('ready').textContent)", timeout=8000)
    assert txt("#arm-pill") == "Chưa arm được" and txt("#rescue-ready") == "chưa"
    gui.tab("overview")
    pg.wait_for_selector('#blockers [data-blocker="RESCUE_NO_COMPASS"]')
    row = pg.inner_text('[data-module="flow"]')
    assert "Có" in row and "MSP flow on UART0" in row and "MSP lidar (flow sensor)" in pg.inner_text('[data-module="range"]'), row
    rows = [r.get_attribute("data-module") for r in pg.locator("table.modules tbody tr").all()]
    assert rows.index("flow") == rows.index("gps") + 1, rows
    gui.tab("pins")
    assert gui.field("pins.flow_rx").input_value() == "44" and gui.field("pins.flow_cs").input_value() == "38" and gui.field("pins.flow_tx").input_value() == ""
    assert "Không thấy chân trùng" in pg.inner_text("#pin-warnings"), "an MSP sensor on UART0's RX pin is the normal wiring, and the PMW3901's pins do not count while it is not the selected sensor"
    gui.type_into("pins.flow_rx", "16")                          # the receiver's RX pin
    pg.wait_for_function("/GPIO16 đang gán cho cả/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.flow_rx", "")
    pg.wait_for_function("/Optical flow qua UART: chưa có chân RX/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.flow_rx", "44")
    pg.wait_for_function("/Không thấy chân trùng/.test(document.getElementById('pin-warnings').textContent)")
    # ---- the pins of the sensor that is NOT selected do not count; choosing the other sensor redoes the check at once
    gui.type_into("pins.flow_cs", "16")                          # the receiver's RX pin, on the PMW3901's chip select
    gui.wait(lambda: gui.setting("pins", "flow_cs") == 16, what="flow_cs in the firmware")
    assert "Không thấy chân trùng" in pg.inner_text("#pin-warnings"), "an MSP sensor is selected: the PMW3901's pins are not in use"
    gui.tab("nav"); pg.select_option('[data-field="flow.type"]', "PMW3901"); gui.tab("pins")
    pg.wait_for_function("/GPIO16 đang gán cho cả/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.flow_cs", "38"); gui.type_into("pins.flow_miso", "")
    pg.wait_for_function("/Optical flow PMW3901: cần đủ bốn chân/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.flow_miso", "48")
    pg.wait_for_function("!/PMW3901/.test(document.getElementById('pin-warnings').textContent)")
    gui.tab("blackbox"); pg.select_option('[data-field="blackbox.device"]', "SDCARD"); pg.select_option('[data-field="blackbox.sd_mode"]', "SPI"); gui.tab("pins")
    pg.wait_for_function("/PMW3901 và thẻ SD kiểu SPI cần cùng một bus SPI/.test(document.getElementById('pin-warnings').textContent)")


def test_restore_and_defaults(gui, tmp_path):
    pg = gui.connect().page
    gui.tab("system")
    backup = gui.hub.backup()
    backup["settings"]["rates"]["angle_limit"] = 200
    backup["settings"]["pid"]["tpa_rate"] = 25
    backup["settings"]["pid"]["from_the_future"] = 1
    f = tmp_path / "b.json"
    f.write_text(json.dumps(backup))
    pg.set_input_files("#restore-file", str(f))
    pg.wait_for_selector("#restore-result .note.ok")
    text = pg.inner_text("#restore-result")
    assert "rates.angle_limit" in text and "pid.from_the_future" in text
    assert gui.setting("pid", "tpa_rate") == 25 and gui.setting("rates", "angle_limit") == 80
    assert "imu: SPI ICM42688P" in pg.inner_text("#log"), "the firmware's boot report is in the log"
    pg.select_option("#defaults-section", "pid")
    pg.once("dialog", lambda d: d.accept())
    pg.click("#defaults")
    gui.wait(lambda: gui.setting("pid", "tpa_rate") == 65, what="defaults")
    gui.tab("receiver")
    gui.sim.do("rc 1 1800")
    pg.wait_for_function("document.querySelector('#chans .chan').dataset.us === '1800'")
    assert pg.inner_text("#rc-errors").count(": 0") == 3, "a clean simulated link: no bad frames, no UART overflows, no line errors: " + pg.inner_text("#rc-errors")
    pg.select_option("#chmap", "TAER")
    gui.wait(lambda: gui.setting("receiver", "ch_throttle") == 0 and gui.setting("receiver", "ch_roll") == 1, what="channel map preset")


def test_compass_calibration_flow(gui):
    pg = gui.connect().page
    gui.sim.do("magerr 0.12 -0.08 0.05")
    gui.tab("sensors")
    pg.check('[data-field="sensors.mag_enabled"]')
    pg.wait_for_function("document.getElementById('save').textContent === 'Lưu và khởi động lại'")
    pg.click("#save")
    pg.wait_for_selector("#save", state="hidden", timeout=15000)
    gui.sim.sleep(1.5)
    pg.wait_for_function("/la bàn chưa hiệu chuẩn/.test(document.getElementById('ready').textContent)", timeout=8000)
    gui.tab("overview")
    assert "QMC5883L" in pg.locator('[data-module="mag"]').inner_text() and "Có" in pg.locator('[data-module="mag"]').inner_text()
    gui.tab("sensors")
    assert "chưa hiệu chuẩn" in pg.inner_text("#mag-stored")
    empty = pg.evaluate("document.querySelector('canvas.cloud').toDataURL()")
    pg.click("#mag-start")
    pg.wait_for_function("/Đang thu mẫu/.test(document.getElementById('mag-state').textContent)")
    pg.click("#mag-finish")                                    # far too early
    pg.wait_for_function("/^Chưa được: (quá ít mẫu|chưa xoay đủ)/.test(document.getElementById('mag-state').textContent)")
    for yaw in range(0, 360, 30):
        for roll in range(-180, 180, 30):
            for pitch in (-60, 0, 60):
                gui.sim.do("pose %d %d %d" % (roll, pitch, yaw))
                gui.sim.sleep(0.03)
    assert pg.evaluate("document.querySelector('canvas.cloud').toDataURL()") != empty, "the point cloud was drawn"
    pg.click("#mag-finish")
    pg.wait_for_function("/thành công/.test(document.getElementById('mag-state').textContent)", timeout=8000)
    pg.wait_for_function("/Đang dùng hiệu chuẩn la bàn/.test(document.getElementById('mag-stored').textContent)")
    cal = gui.hub.fc.get("calibration")
    assert cal["mag_valid"] is True and cal["mag_offset"] == pytest.approx([0.12, -0.08, 0.05], abs=0.02)
    gui.sim.do("pose 0 0 0")
    pg.wait_for_function("!/la bàn chưa hiệu chuẩn/.test(document.getElementById('ready').textContent)", timeout=8000)


# ---------------------------------------------------------------------------------------------------------------------
# English
# ---------------------------------------------------------------------------------------------------------------------
VIETNAMESE = "[\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u1EF9]"
FIND_VIETNAMESE = """(re) => {
  const rx = new RegExp(re), found = [];
  const skip = (el) => el.closest('#lang, noscript');             // the language names themselves; the two-language no-script line
  const walk = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
  for (let n = walk.nextNode(); n; n = walk.nextNode()) if (rx.test(n.nodeValue) && !skip(n.parentElement)) found.push(n.nodeValue.trim().slice(0, 80));
  for (const el of document.body.querySelectorAll('*')) {
    if (skip(el)) continue;
    for (const a of ['title', 'aria-label', 'placeholder']) { const v = el.getAttribute(a); if (v && rx.test(v) && el.id !== 'lang') found.push(a + '=' + v.slice(0, 80)); }
  }
  return found;
}"""


def no_vietnamese(gui, where):
    found = gui.page.evaluate(FIND_VIETNAMESE, VIETNAMESE)
    assert not found, "Vietnamese text on the English page (%s): %s" % (where, found)


def test_ppm_and_spektrum_receivers_on_the_page(gui):
    pg = gui.connect().page
    gui.tab("receiver")
    pg.select_option('[data-field="receiver.protocol"]', "PPM")
    pg.wait_for_function("/PPM: một dây tín hiệu/.test(document.getElementById('rx-proto-help').textContent)")
    opt = lambda v: pg.inner_text('[data-field="receiver.protocol"] option[value="%s"]' % v)
    assert opt("PWM") == "PWM (mỗi kênh một chân)" and opt("SPEKTRUM1024") == "Spektrum 1024 (DSM2 22 ms)" and "chưa hỗ trợ" in opt("NRF24") and "chưa hỗ trợ" not in opt("PPM")
    gui.tab("pins")
    pg.wait_for_function("/Receiver PPM: chưa có chân PPM/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.ppm", "40")
    pg.wait_for_function("!/chưa có chân PPM/.test(document.getElementById('pin-warnings').textContent)")
    gui.wait(lambda: gui.setting("pins", "ppm") == 40 and gui.setting("receiver", "protocol") == "PPM", what="PPM settings reached the firmware")
    gui.sim.do("rclink ppm")
    boots = int(gui.sim.state()["boots"])
    pg.click("#save")
    gui.wait(lambda: int(gui.sim.state()["boots"]) == boots + 1, 12, "firmware restart")
    pg.wait_for_selector("#save", state="hidden", timeout=12000)
    gui.tab("receiver")
    gui.sim.do("rc 1 1234"); gui.sim.do("rc 8 1876")
    pg.wait_for_function("(document.querySelector('#chans .chan') || {dataset: {}}).dataset.us === '1234'", timeout=8000)
    pg.wait_for_function("/Tín hiệu: có/.test(document.getElementById('view').textContent) && /4[3-5] khung\\/s/.test(document.getElementById('view').textContent)", timeout=8000)
    gui.tab("overview")
    rows = {r.get_attribute("data-module"): r.inner_text() for r in pg.locator("table.modules tbody tr").all()}
    assert "PPM on GPIO40" in rows["rx"], rows
    # ---- Spektrum: back on UART2, channel order TAER from the quick pick
    gui.tab("receiver")
    pg.select_option('[data-field="receiver.protocol"]', "SPEKTRUM")
    pg.wait_for_function("/thứ tự TAER/.test(document.getElementById('rx-proto-help').textContent)")
    pg.select_option("#chmap", "TAER")
    gui.wait(lambda: gui.setting("receiver", "ch_throttle") == 0 and gui.setting("receiver", "ch_roll") == 1, what="TAER stored")
    gui.sim.do("rclink spektrum2048")
    boots = int(gui.sim.state()["boots"])
    pg.click("#save")
    gui.wait(lambda: int(gui.sim.state()["boots"]) == boots + 1, 12, "firmware restart")
    pg.wait_for_selector("#save", state="hidden", timeout=12000)
    gui.sim.do("rc 1 1100"); gui.sim.do("rc 2 1700")
    pg.wait_for_function("(c => c.length > 2 && c[2].dataset.us === '1100' && c[0].dataset.us === '1700')(document.querySelectorAll('#chans .chan'))", timeout=8000)   # throttle = channel 1, roll = channel 2
    pg.wait_for_function("/(89|9[0-2]) khung\\/s/.test(document.getElementById('view').textContent)", timeout=8000)


def test_serial_ports_table(gui):
    pg = gui.connect().page
    gui.tab("pins")
    assert pg.locator("#ports-table th").all_inner_texts() == ["Cổng", "Chức năng", "UART0", "UART1", "UART2"], "no baud column any more"
    assert "chưa hỗ trợ" in pg.inner_text('[data-field="ports.function[0]"] option[value="TELEMETRY"]')
    pg.select_option('[data-field="ports.assign"]', "TABLE")
    pg.wait_for_function("/bảng không có UART nào cho receiver nối tiếp/.test(document.getElementById('pin-warnings').textContent)")
    pg.select_option('[data-field="ports.function[2]"]', "SERIAL_RX")
    pg.wait_for_function("!/bảng không có UART nào/.test(document.getElementById('pin-warnings').textContent)")
    pg.select_option('[data-field="ports.function[0]"]', "GPS")
    pg.wait_for_function("/\"GPS\" được chọn cho hơn một UART/.test(document.getElementById('pin-warnings').textContent)")
    pg.select_option('[data-field="ports.function[0]"]', "NONE")
    gui.wait(lambda: gui.setting("ports", "assign") == "TABLE" and gui.setting("ports", "function") == ["NONE", "GPS", "SERIAL_RX"], what="table stored: %s" % gui.setting("ports", "function"))
    boots = int(gui.sim.state()["boots"])
    pg.click("#save")
    gui.wait(lambda: int(gui.sim.state()["boots"]) == boots + 1, 12, "firmware restart")
    pg.wait_for_selector("#save", state="hidden", timeout=12000)
    gui.tab("overview")
    pg.wait_for_selector('[data-module="rx"]', timeout=8000)
    rows = {r.get_attribute("data-module"): r.inner_text() for r in pg.locator("table.modules tbody tr").all()}
    assert "CRSF on UART2" in rows["rx"] and "UART1" in rows["gps"], rows


def test_gps_baud_is_a_list(gui):
    pg = gui.connect().page
    gui.tab("sensors")
    sel = pg.locator('[data-field="gps.baud"]')
    assert sel.evaluate("e => e.tagName") == "SELECT" and sel.locator("option").all_inner_texts() == ["9600", "19200", "38400", "57600", "115200", "230400"]
    assert sel.input_value() == "115200", "115200 by default"
    sel.select_option("57600")
    gui.wait(lambda: gui.setting("gps", "baud") == 57600, what="baud stored as a number")
    pg.wait_for_function("document.getElementById('save').textContent === 'Lưu và khởi động lại'")


def test_betaflight_frames_and_protocols_on_the_motor_tab(gui):
    pg = gui.connect().page
    gui.tab("motors")
    opts = pg.locator('[data-field="motors.layout"] option').all_inner_texts()
    assert "Octo X8 (đồng trục)" in opts and "Hexa H" in opts and "Quad X (thứ tự 1234)" in opts and len(opts) == 14, opts
    assert "ProShot1000" in pg.locator('[data-field="motors.protocol"] option').all_inner_texts()
    pg.select_option('[data-field="motors.layout"]', "OCTO_X8")
    pg.wait_for_function("document.querySelectorAll('#view [data-motor]').length === 8")
    assert pg.locator('#view [data-motor][data-under="1"]').count() == 4, "the lower motor of each coaxial pair is marked"
    assert pg.is_visible("#dshot-count") and "8 motor" in pg.inner_text("#dshot-count"), "DShot cannot drive 8 motors: said before saving"
    pg.select_option('[data-field="motors.protocol"]', "ONESHOT125")
    pg.wait_for_function("document.getElementById('dshot-count').hidden")
    assert pg.is_visible("#pulse-fields") and not pg.is_visible("#dshot-fields")
    pg.select_option('[data-field="motors.protocol"]', "PROSHOT1000")
    pg.wait_for_function("!document.getElementById('dshot-count').hidden && !document.getElementById('dshot-fields').hidden")
    gui.wait(lambda: gui.setting("motors", "layout") == "OCTO_X8" and gui.setting("motors", "protocol") == "PROSHOT1000", what="stored")
    pg.select_option('[data-field="motors.layout"]', "Y4")
    pg.wait_for_function("document.querySelectorAll('#view [data-motor]').length === 4 && document.getElementById('dshot-count').hidden")
    # ESCs on the CAN bus: no pulse or DShot settings, the motor idle stays, a note says how it works
    pg.select_option('[data-field="motors.protocol"]', "DRONECAN")
    pg.wait_for_function("!document.getElementById('can-esc-note').hidden")
    assert not pg.is_visible("#pulse-fields") and not pg.is_visible("#dshot-fields") and pg.is_visible("#idle-field") and "esc.RawCommand" in pg.inner_text("#can-esc-note")
    assert "DroneCAN (ESC trên bus CAN)" in pg.locator('[data-field="motors.protocol"] option').all_inner_texts()
    pg.select_option('[data-field="motors.protocol"]', "DSHOT300")
    pg.wait_for_function("document.getElementById('can-esc-note').hidden")


def test_current_sensor_and_servos_on_the_page(gui):
    pg = gui.connect().page
    # ---- the pins: a current sensor on GPIO3, a servo on GPIO1 (both need a restart)
    gui.tab("pins")
    gui.type_into("pins.curr_adc", "41")
    pg.wait_for_function("/ADC đo dòng: GPIO41 không đo được điện áp/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.curr_adc", "3")
    gui.type_into("pins.servo[0]", "2")                          # the battery's pin
    pg.wait_for_function("/GPIO2 đang gán cho cả \"ADC đo pin\" và \"Servo 1\"/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.servo[0]", "1")
    pg.wait_for_function("!/GPIO2 đang gán|không đo được/.test(document.getElementById('pin-warnings').textContent)")
    gui.wait(lambda: gui.setting("pins", "curr_adc") == 3 and gui.setting("pins", "servo")[0] == 1, what="pins reached the firmware")
    boots = int(gui.sim.state()["boots"])
    pg.click("#save")
    gui.wait(lambda: int(gui.sim.state()["boots"]) == boots + 1, 12, "firmware restart")
    pg.wait_for_selector("#save", state="hidden", timeout=12000)
    pg.wait_for_selector("#arm-pill.ok", timeout=12000)
    gui.sim.do("adc 3 400")                                     # 400 mV at the default 40 mV/A: 10 A
    # ---- the Battery tab: current, mAh used, what is left
    gui.tab("power")
    pg.wait_for_function("document.getElementById('batt-amps').textContent === '10.0 A'", timeout=8000)
    assert "Nguồn đo dòng: cảm biến dòng" in pg.inner_text("#batt-source") and pg.inner_text("#batt-mah").endswith("mAh")
    left_v = int(pg.inner_text("#batt-left").split()[0])
    assert 60 <= left_v <= 80, "no capacity yet: from the voltage (16.0 V on 4S between 3.3 and 4.3 V per cell = 70 %%): %d" % left_v
    gui.type_into("battery.capacity_mah", "100")                 # a tiny pack: 10 A empty it at 1 % every 0.36 s
    gui.wait(lambda: gui.setting("battery", "capacity_mah") == 100, what="capacity stored")
    gui.sim.sleep(1.0)
    pg.wait_for_function("parseInt(document.getElementById('batt-left').textContent) < 99", timeout=6000)
    # ---- calibration: the clamp meter shows 20 A while the pin reads 400 mV -> 20 mV/A = scale 200
    pg.wait_for_function("/400 mV/.test(document.getElementById('view').textContent)")
    pg.click("#curr-calibrate")
    pg.wait_for_selector(".toast.bad")                          # nothing typed yet
    pg.fill("#curr-measured", "20")
    pg.click("#curr-calibrate")
    gui.wait(lambda: gui.setting("battery", "current_scale") == 200, what="scale worked out: %s" % gui.setting("battery", "current_scale"))
    pg.wait_for_function("document.getElementById('batt-amps').textContent === '20.0 A'", timeout=8000)
    assert pg.inner_text("#save") == "Lưu", "the current sensor's numbers need no restart"
    # ---- the Motors tab: servo 1 follows channel 10
    gui.tab("motors")
    pg.select_option('[data-field="servo.source[0]"]', "CH10")
    gui.wait(lambda: gui.setting("servo", "source")[0] == "CH10", what="servo source stored")
    gui.sim.do("rc 10 1250")
    pg.wait_for_function("/^1250 µs$/.test(document.getElementById('servo-us-0').textContent.trim())", timeout=6000)
    gui.type_into("servo.max_us[0]", "1800")
    pg.wait_for_function("/^1200 µs$/.test(document.getElementById('servo-us-0').textContent.trim())", timeout=6000)   # 1000 + 0.25 x 800
    assert pg.inner_text("#servo-us-1").strip() == "tắt"


def test_battery_and_beeper_tab(gui):
    pg = gui.page
    gui.connect()
    gui.tab("overview")
    rows = {r.get_attribute("data-module"): r.inner_text() for r in pg.locator("table.modules tbody tr").all()}
    assert "Có" in rows["battery"] and "ADC" in rows["battery"] and "Có" in rows["beeper"] and "active buzzer + motors (DShot beacon" in rows["beeper"], rows
    assert "Có" in rows["blackbox"] and "partition 'sim'" in rows["blackbox"] and "pins" not in rows, rows
    gui.tab("power")
    # ---- the pack as the firmware measures it (simulated: 16.0 V, 11:1 divider on the ADC pin)
    pg.wait_for_function("document.getElementById('batt-volts').textContent.startsWith('16.0')", timeout=6000)
    assert pg.inner_text("#batt-cells") == "4S" and pg.inner_text("#batt-cell").startswith("4.0") and pg.inner_text("#batt-state") == "Tốt"
    assert pg.get_attribute("#batt-gauge", "data-cell") in ("3.99", "4.00", "4.01") and "chân ADC" in pg.inner_text("#batt-source")
    assert "4.00 V/cell" in pg.inner_text("#chips") or "3.99 V/cell" in pg.inner_text("#chips") or "4.01 V/cell" in pg.inner_text("#chips")
    # ---- a low pack: warning, and the top bar says why it will not arm
    gui.sim.do("batt 13.8")
    pg.wait_for_function("document.getElementById('batt-state').dataset.state === 'WARNING'", timeout=8000)
    pg.wait_for_function("/pin dưới mức cảnh báo/.test(document.getElementById('ready').textContent)")
    assert pg.inner_text("#arm-pill") == "Chưa arm được" and pg.inner_text("#batt-state") == "Cảnh báo"
    # the limits act at once: with the warning level lowered below 3.45 V the same pack is fine
    gui.type_into("battery.cell_warning_mv", "3400")
    pg.wait_for_function("document.getElementById('batt-state').dataset.state === 'OK'", timeout=6000)
    pg.wait_for_selector("#arm-pill.ok", timeout=6000)
    assert pg.inner_text("#save") == "Lưu", "battery settings need no restart"
    gui.type_into("battery.cell_warning_mv", "3500")
    gui.sim.do("batt 16.0")
    # ---- divider calibration: the wrong ratio is in the settings, the user types what his multimeter shows
    gui.type_into("battery.divider", "10")
    pg.wait_for_function("document.getElementById('batt-volts').textContent.startsWith('14.5')", timeout=8000)
    pg.click("#batt-calibrate")
    pg.wait_for_selector(".toast.bad")                         # nothing typed yet
    pg.fill("#batt-measured", "16.0")
    pg.click("#batt-calibrate")
    gui.wait(lambda: abs(gui.setting("battery", "divider") - 11.0) <= 0.03, what="divider worked out from the pin voltage: %s" % gui.setting("battery", "divider"))
    pg.wait_for_function("document.getElementById('batt-volts').textContent.startsWith('16.0') || document.getElementById('batt-volts').textContent.startsWith('15.9')", timeout=8000)
    assert abs(float(gui.field("battery.divider").input_value()) - 11.0) <= 0.03

    # ---- beeper: the list is the firmware's, each sound is drawn as it is played
    assert pg.locator("#beeps tbody tr").count() == 18
    assert pg.locator('tr[data-cond="GYRO_CALIBRATED"] .bp-on').count() == 3 and pg.locator('tr[data-cond="RX_LOST_LANDING"] .bp-on').count() == 9
    assert pg.locator('tr[data-cond="USB"] .beep-try').count() == 0 and not pg.is_checked("#cond-USB") and pg.is_checked("#cond-ARMING")
    pg.uncheck("#cond-DISARMING")
    pg.check("#cond-USB")
    gui.wait(lambda: "DISARMING" not in gui.setting("beeper", "conditions") and "USB" in gui.setting("beeper", "conditions"), what="conditions stored")
    assert "ARMING" in gui.setting("beeper", "conditions") and len(gui.setting("beeper", "conditions")) == 17

    def buzz():
        return int(gui.sim.state()["buzz_ms"])
    gui.sim.sleep(1.0)
    t0 = buzz()
    pg.click('tr[data-cond="DISARMING"] .beep-try')              # switched off a moment ago: the test button plays it anyway
    pg.wait_for_function("document.getElementById('beep-now').textContent === 'Disarm'", timeout=4000)
    pg.wait_for_function("document.getElementById('beep-now').textContent === 'im lặng'", timeout=4000)
    assert buzz() - t0 == pytest.approx(300, abs=12), "DISARMING is two 150 ms beeps on the buzzer pin"
    t0 = buzz()
    pg.click('tr[data-cond="RX_LOST_LANDING"] .beep-try')        # SOS lasts 4 s ...
    pg.wait_for_function("document.getElementById('beep-now').textContent === 'Đang hạ cánh failsafe'", timeout=4000)
    pg.click("#beep-silence")                                    # ... unless it is silenced
    pg.wait_for_function("document.getElementById('beep-now').textContent === 'im lặng'", timeout=1500)
    gui.sim.sleep(0.5)
    assert buzz() - t0 < 900
    # ---- the motors as the second sounder: their own column, what they cannot play is not offered, the count of tones is shown
    assert pg.get_attribute("#sounders", "data-buzzer") == "1" and pg.get_attribute("#sounders", "data-motors") == "1" and "có còi và motor kêu được" in pg.inner_text("#sounders")
    assert pg.is_checked("#mcond-RX_LOST") and pg.is_checked("#mcond-GYRO_CALIBRATED") and not pg.is_checked("#mcond-SYSTEM_INIT")
    assert pg.locator("#mcond-ARMING").count() == 0 and pg.locator("#mcond-ARMED").count() == 0 and pg.locator("#mcond-USB").count() == 0 and pg.locator("#beeps .cond-on[id^=mcond-]").count() == 12
    assert pg.inner_text('tr[data-cond="GYRO_CALIBRATED"] .motor-tones').strip() == "motor: 3 tiếng" and pg.inner_text('tr[data-cond="DISARMING"] .motor-tones').strip() == "motor: 2 tiếng"
    pg.uncheck("#mcond-BAT_LOW")
    gui.wait(lambda: "BAT_LOW" not in gui.setting("beeper", "motor_conditions") and "BAT_LOW" in gui.setting("beeper", "conditions"), what="BAT_LOW off for the motors only")
    assert len(gui.setting("beeper", "motor_conditions")) == 10
    # a low pack now beeps on the buzzer only: the two "playing now" readouts are separate things
    gui.sim.do("batt 13.8")
    pg.wait_for_function("document.getElementById('beep-now').textContent === 'Pin dưới mức cảnh báo'", timeout=8000)
    assert pg.inner_text("#motor-beep-now") == "im lặng"
    gui.sim.do("batt 16.0")
    pg.wait_for_function("document.getElementById('batt-state').dataset.state === 'OK' && document.getElementById('beep-now').textContent === 'im lặng'", timeout=8000)
    gui.type_into("beeper.dshot_beacon_tone", "4")
    gui.wait(lambda: gui.setting("beeper", "dshot_beacon_tone") == 4, what="tone stored")
    gui.sim.sleep(1.0)
    n0 = int(gui.sim.state()["tones"])
    pg.click('tr[data-cond="DISARMING"] .beep-try')
    pg.wait_for_function("document.getElementById('motor-beep-now').textContent === 'Disarm'", timeout=4000)
    pg.wait_for_function("document.getElementById('motor-beep-now').textContent === 'im lặng'", timeout=4000)
    assert int(gui.sim.state()["tones"]) - n0 == 2, "the Try button: the ESCs got two beacon commands"
    # ---- buzzer hardware: ordinary settings
    pg.select_option('[data-field="beeper.type"]', "PASSIVE")
    gui.type_into("beeper.frequency", "3200")
    gui.wait(lambda: gui.setting("beeper", "type") == "PASSIVE" and gui.setting("beeper", "frequency") == 3200, what="buzzer settings stored")
    # ---- armed: nothing here can be changed or played
    pg.select_option('[data-field="beeper.type"]', "ACTIVE")
    gui.wait(lambda: gui.setting("beeper", "type") == "ACTIVE")
    gui.sim.do("rc 3 1000"); gui.sim.do("rc 5 2000")
    pg.wait_for_selector("#arm-pill.bad")
    assert pg.is_disabled("#cond-ARMING") and pg.is_disabled("#mcond-RX_LOST") and pg.is_disabled('tr[data-cond="ARMING"] .beep-try') and pg.is_disabled("#batt-calibrate") and pg.is_disabled("#beep-silence")
    gui.sim.do("rc 5 1000")


def test_english_page_every_tab(gui_en):
    gui = gui_en
    pg = gui.page
    pg.goto(gui.srv.url)
    pg.wait_for_selector(".welcome")
    assert pg.evaluate("document.documentElement.lang") == "en" and pg.input_value("#lang") == "en"
    assert pg.inner_text("#arm-pill") == "Not connected" and pg.inner_text("#connect") == "Connect" and "Connect the flight controller" in pg.inner_text("#view")
    no_vietnamese(gui, "welcome")
    gui.connect()
    assert pg.inner_text("#arm-pill") == "Ready to arm" and pg.inner_text("#connect") == "Disconnect"
    titles = {"overview": "Overview", "receiver": "Receiver", "modes": "Modes", "sensors": "Sensors", "motors": "Motors", "tuning": "PID", "rates": "Rate", "filters": "Filters",
              "nav": "Position", "power": "Battery and beeper", "blackbox": "Blackbox", "pins": "Pins and ports", "system": "System"}
    assert [a.inner_text() for a in pg.locator("#rail a").all()] == list(titles.values())
    for tab, title in titles.items():
        gui.tab(tab)
        pg.wait_for_function("t => document.querySelector('#view h1').textContent === t", arg=title)
        time.sleep(0.4)                                          # first telemetry of the tab
        no_vietnamese(gui, tab)
    # texts that only appear in certain states
    gui.tab("overview")
    assert "Found" in pg.locator('[data-module="imu"]').inner_text() and "Off" in pg.locator('[data-module="mag"]').inner_text()
    gui.sim.do("pose 60 0 0")
    pg.wait_for_selector('#blockers [data-blocker="ANGLE_TOO_HIGH"]')
    assert pg.inner_text("#arm-pill") == "Cannot arm" and "tilted more than allowed" in pg.inner_text("#ready")
    no_vietnamese(gui, "blockers")
    gui.sim.do("pose 0 0 0")
    pg.wait_for_selector("#arm-pill.ok", timeout=8000)
    gui.tab("power")
    gui.sim.do("batt 13.8")
    pg.wait_for_function("document.getElementById('batt-state').textContent === 'Warning'", timeout=8000)
    pg.wait_for_function("/battery below the warning level/.test(document.getElementById('ready').textContent)")
    pg.click('tr[data-cond="RX_LOST_LANDING"] .beep-try')
    pg.wait_for_function("document.getElementById('beep-now').textContent === 'Failsafe landing'", timeout=4000)
    pg.click("#batt-calibrate")
    pg.wait_for_selector(".toast.bad")
    assert "Enter the battery voltage measured with a multimeter" in pg.inner_text("#toasts")
    no_vietnamese(gui, "battery warning, beeper playing, calibration hint")
    pg.click("#beep-silence")
    gui.sim.do("batt 16.0")
    pg.wait_for_function("document.getElementById('batt-state').textContent === 'Good'", timeout=8000)
    gui.tab("pins")
    gui.type_into("pins.buzzer", "10")
    pg.wait_for_function("/GPIO10 is assigned to both/.test(document.getElementById('pin-warnings').textContent)")
    assert pg.get_attribute('[data-field="pins.led"]', "placeholder") == "empty"
    no_vietnamese(gui, "pin warnings")
    pg.wait_for_function("document.getElementById('save').textContent === 'Save and restart'")
    gui.tab("sensors")
    pg.click('[data-cmd="ACC_CAL_POINT"]')
    pg.wait_for_function("/1\\/6 faces captured/.test(document.getElementById('acc-state').textContent)", timeout=8000)
    pg.click("#acc-finish")
    pg.wait_for_function("/Not all 6 faces yet\\./.test(document.getElementById('toasts').textContent)", timeout=6000)
    no_vietnamese(gui, "calibration")
    gui.tab("rates")
    assert "Roll" in pg.inner_text("#rate-read") and "Sticks now" in pg.inner_text("#rate-read")
    gui.tab("modes")
    assert pg.locator('[data-mode="ANGLE"] .help').inner_text().startswith("Self-levelling")
    # armed
    gui.tab("motors")
    pg.click('[data-cmd="ACC_CAL_CANCEL"]') if pg.locator('[data-cmd="ACC_CAL_CANCEL"]').count() else gui.hub.command("ACC_CAL_CANCEL", {})
    pg.once("dialog", lambda d: d.accept())
    pg.click("#revert")                                          # throw the pin edit away so that nothing blocks arming
    pg.wait_for_selector("#revert", state="hidden", timeout=12000)
    gui.sim.sleep(1.5)
    pg.wait_for_selector("#arm-pill.ok", timeout=8000)
    gui.sim.do("rc 3 1000"); gui.sim.do("rc 5 2000")
    pg.wait_for_selector("#arm-pill.bad")
    assert "ARMED. The firmware refuses every settings change" in pg.inner_text("#banner")
    no_vietnamese(gui, "armed")
    gui.sim.do("rc 5 1000")


def test_language_switch_is_remembered(gui_en):
    gui = gui_en
    pg = gui.connect().page
    gui.tab("motors")
    assert pg.inner_text("#view h1") == "Motors"
    with pg.expect_navigation():
        pg.select_option("#lang", "vi")                         # reloads the page in the other language, on the same tab, still connected
    pg.wait_for_function("(document.querySelector('#view h1') || {}).textContent === 'Motor'")
    assert pg.inner_text("#connect") == "Ngắt kết nối" and pg.inner_text("#tab-overview") == "Tổng quan" and pg.evaluate("document.documentElement.lang") == "vi"
    assert gui.hub.fc is not None, "changing the language does not drop the connection"
    assert len(pg.evaluate(FIND_VIETNAMESE, VIETNAMESE)) > 20, "the detector used by the English tests does see Vietnamese text"
    pg.reload()
    pg.wait_for_function("(document.querySelector('#view h1') || {}).textContent === 'Motor'")
    assert pg.input_value("#lang") == "vi", "the choice is remembered over the browser's own language"
    with pg.expect_navigation():
        pg.select_option("#lang", "en")
    pg.wait_for_function("(document.querySelector('#view h1') || {}).textContent === 'Motors'")


# ---------------------------------------------------------------------------------------------------------------------
# Serial port in the page
# ---------------------------------------------------------------------------------------------------------------------
def test_serial_connection_from_the_page(gui, serial_port):
    pg = gui.page
    pg.goto(gui.srv.url)
    pg.wait_for_selector("#baud option", state="attached")
    assert pg.input_value("#baud") == "230400" and [o.get_attribute("value") for o in pg.locator("#baud option").all()] == ["115200", "230400", "460800", "921600"]
    gui.connect(serial_port.path, baud=460800)
    assert gui.hub.port == serial_port.path and gui.hub.baud == 460800
    assert pg.locator("#baud").is_disabled() and pg.input_value("#port") == serial_port.path
    gui.tab("system")
    assert "460800" in pg.inner_text("#view") and "Chưa có byte nào ngoài giao thức." in pg.inner_text("#raw")
    serial_port.inject(b"\r\nGuru Meditation Error: Core  1 panic'ed (StoreProhibited)\r\n")
    pg.wait_for_function("/Guru Meditation Error: Core  1 panic'ed/.test(document.getElementById('raw').textContent)", timeout=6000)
    assert pg.inner_text("#arm-pill") == "Sẵn sàng arm", "the stray text did not disturb the link"
    serial_port.unplug()
    pg.wait_for_selector(".welcome", timeout=8000)
    assert "Mất kết nối với cổng " + serial_port.path in pg.inner_text("#banner")
    pg.reload()
    pg.wait_for_selector("#baud option", state="attached")
    pg.wait_for_function("document.getElementById('baud').value === '460800'")      # the baud rate chosen last time is offered again


def test_a_port_that_does_not_answer_is_explained(gui_en):
    from conftest import PtyPort
    gui = gui_en
    pg = gui.page
    port = PtyPort(None, chatter=b"ESP-ROM:esp32s3-20210327\r\nrst:0x3 (RTC_SW_SYS_RST),boot:0x8 (SPI_FAST_FLASH_BOOT)\r\n")
    try:
        pg.goto(gui.srv.url)
        gui.choose_port(port.path, baud=115200)
        pg.click("#connect")
        pg.wait_for_selector("#failure", timeout=20000)
        text = pg.inner_text("#failure")
        assert "The last connection attempt failed" in text and "opened at 115200 baud, but no Windburst firmware answered" in text
        assert "baud rate must match the firmware" in text and "ESP-ROM:esp32s3-20210327" in pg.inner_text("#failure-raw")
        assert pg.inner_text("#connect") == "Connect" and not pg.locator("#port").is_disabled()
        no_vietnamese(gui, "failure help")
        # a port that can not be opened at all
        gui.choose_port("/dev/ttyDOES_NOT_EXIST")
        pg.click("#connect")
        pg.wait_for_function("/Cannot open the port \\/dev\\/ttyDOES_NOT_EXIST/.test((document.getElementById('failure') || {}).textContent || '')", timeout=8000)
        assert "another program holding the port" in pg.inner_text("#failure") and pg.locator("#failure-raw").count() == 0
        # then the right port: the explanation goes away
        gui.connect()
        assert pg.locator("#failure").count() == 0
    finally:
        port.close()


# ---------------------------------------------------------------------------------------------------------------- blackbox
PLOT_L, PLOT_R = 58, 10                 # the plot area inside a chart canvas (gfx/logplot.js)
INK = "([sel, rgb]) => { const cv = document.querySelector(sel); const d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data; let n = 0;" \
      " for (let i = 0; i < d.length; i += 4) if (d[i + 3] > 200 && Math.abs(d[i] - rgb[0]) < 40 && Math.abs(d[i + 1] - rgb[1]) < 40 && Math.abs(d[i + 2] - rgb[2]) < 40) n++; return n; }"
COLUMNS = "([sel, rgb]) => { const cv = document.querySelector(sel); const w = cv.width, h = cv.height, d = cv.getContext('2d').getImageData(0, 0, w, h).data; const out = [];" \
          " for (let x = 0; x < w; x++) { let n = 0; for (let y = 0; y < h; y++) { const i = 4 * (y * w + x); if (d[i + 3] > 200 && Math.abs(d[i] - rgb[0]) < 40 && Math.abs(d[i + 1] - rgb[1]) < 40 && Math.abs(d[i + 2] - rgb[2]) < 40) n++; }" \
          " out.push(n); } return out; }"                 # ink of one colour in every pixel column (device pixel ratio 1 in the tests): a tall count is a vertical edge
BLUE, ORANGE, YELLOW = [42, 120, 214], [235, 104, 52], [237, 161, 0]          # --series-1, -2, -4 of the light theme (four motors that agree lie under the fourth line)


class Plot:
    """The log charts as a pilot uses them: point, drag, read."""

    def __init__(self, gui, panel="rate-0"):
        self.gui, self.pg = gui, gui.page
        self.cv = self.pg.locator('canvas.plot-cv[data-panel="%s"]' % panel)

    def loaded(self):
        before = self.pg.get_attribute("#logplot", "data-loaded")
        self.gui.wait(lambda: self.pg.get_attribute("#logplot", "data-loaded") not in (None, before) or (before and time.sleep(0.35) is None), what="chart data")
        return self

    def range(self):
        el = self.pg.locator("#logplot")
        return float(el.get_attribute("data-t0")), float(el.get_attribute("data-t1"))

    def x_of(self, t):
        self.cv.scroll_into_view_if_needed()
        box, (t0, t1) = self.cv.bounding_box(), self.range()
        return box["x"] + PLOT_L + (t - t0) / (t1 - t0) * (box["width"] - PLOT_L - PLOT_R), box["y"] + 50

    def point_at(self, t):
        self.pg.mouse.move(*self.x_of(t))
        self.gui.wait(lambda: self.pg.get_attribute("#logplot", "data-cursor"), what="crosshair")
        return float(self.pg.get_attribute("#logplot", "data-cursor"))

    def drag(self, ta, tb, shift=False):
        (xa, y), (xb, _) = self.x_of(ta), self.x_of(tb)
        if shift:
            self.pg.keyboard.down("Shift")
        self.pg.mouse.move(xa, y)
        self.pg.mouse.down()
        self.pg.mouse.move((xa + xb) / 2, y)
        self.pg.mouse.move(xb, y)
        self.pg.mouse.up()
        if shift:
            self.pg.keyboard.up("Shift")

    def legend(self, field):
        return self.pg.locator('.legend-item[data-field="%s"] .plot-val' % field).first.inner_text()


def _span(text):
    """A legend value: '12.30 °/s' -> (12.3, 12.3);  '-4.10 … 2.50 °/s' -> (-4.1, 2.5)"""
    n = [float(x) for x in re.findall(r"-?\d+\.?\d*", text.split("°")[0])]
    return (n[0], n[-1]) if n else (None, None)


def _fly_and_land(sim):
    sim.do("rc 6 2000"); sim.do("rc 7 2000"); sim.sleep(0.3)             # Angle + Altitude Hold
    sim.do("rc 5 2000"); sim.sleep(0.5)
    sim.do("rc 3 1700"); sim.sleep(1.5)
    sim.do("rc 3 1500"); sim.do("rc 1 1800"); sim.sleep(0.7)
    sim.do("rc 1 1500"); sim.sleep(0.8)


def test_blackbox_record_download_and_read_the_charts(gui, log_dir):
    from wbclient import wblog
    pg = gui.connect().page
    gui.tab("blackbox")
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'OFF'")
    assert pg.inner_text("#bb-state") == "Tắt" and "chọn \"Flash trên mạch\"" in pg.inner_text("#bb-why") and "1.00 MB" in pg.inner_text("#bb-used")
    assert pg.is_visible("#bb-none") and pg.is_hidden("#bb-sessions") and pg.is_disabled("#bb-download-all")
    assert pg.is_visible("#logs-none") and pg.is_hidden("#viewer") and pg.is_visible("#viewer-idle") and str(log_dir) in pg.inner_text("#logs-dir")
    assert not log_dir.exists(), "opening the tab writes nothing to the disk"
    assert pg.locator("#bb-groups input").count() == 15 and pg.is_checked("#bbf-GYRO") and not pg.is_checked("#bbf-GYRO_RAW") and pg.is_checked("#bbf-NAV") and not pg.is_checked("#bbf-MAG")

    # ---- what to record: the cost follows the choice, the firmware gets it
    pg.select_option('[data-field="blackbox.device"]', "FLASH")
    gui.type_into("blackbox.rate_hz", 1000)                    # more samples than the chart has pixels: the reduced view is what gets tested
    gui.wait(lambda: gui.setting("blackbox", "device") == "FLASH" and gui.setting("blackbox", "rate_hz") == 1000, what="blackbox settings in the firmware")
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'IDLE'")
    pg.wait_for_function("/74 byte/.test(document.getElementById('bb-cost').textContent)")
    assert pg.inner_text("#bb-state") == "Sẵn sàng" and pg.is_hidden("#bb-why")
    full = int(pg.get_attribute("#bb-cost", "data-rate"))
    assert 75000 < full < 80000 and "khoảng 13 giây" in pg.inner_text("#bb-cost"), "1000 x 74 bytes a second; 1 MB lasts a quarter of a minute: %s" % pg.inner_text("#bb-cost")
    pg.uncheck("#bbf-PID")
    gui.wait(lambda: "PID" not in gui.setting("blackbox", "fields"), what="PID group off in the firmware")
    pg.wait_for_function("/50 byte/.test(document.getElementById('bb-cost').textContent)")
    assert int(pg.get_attribute("#bb-cost", "data-rate")) < full * 0.7
    pg.check("#bbf-PID")
    gui.wait(lambda: "PID" in gui.setting("blackbox", "fields"), what="PID group on again")

    # ---- a flight is logged; nothing may be downloaded or erased meanwhile
    _fly_and_land(gui.sim)
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'LOGGING'")
    assert pg.is_hidden("#bb-buffer"), "the write-buffer line belongs to the SD card: the flash has no such buffer"
    assert pg.inner_text("#bb-state") == "Đang ghi" and pg.is_disabled("#bb-erase") and pg.is_disabled("#bb-download-all")
    gui.sim.do("rc 3 1000"); gui.sim.do("rc 5 1000")
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'IDLE'", timeout=8000)
    pg.wait_for_selector('#bb-sessions tbody tr[data-index="0"]')
    used = gui.hub.fc.blackbox_status()["used"]
    # (rounded as the page does it: half a kB goes up - Python's "%.0f" would round 212.5 down to the even 212)
    gui.wait(lambda: pg.inner_text('#bb-sessions tr[data-index="0"] td') == "%d kB" % int(used / 1024 + 0.5), what="the session's final size in the list (%d bytes)" % used)
    assert pg.locator("#bb-sessions tbody tr").count() == 1 and pg.is_enabled("#bb-download-all") and pg.is_enabled("#bb-erase")

    # ---- download: it lands in the library and opens
    pg.click("#bb-sessions .bb-get")
    pg.wait_for_selector('#viewer[data-file^="flight_"]', timeout=20000)
    name = pg.get_attribute("#viewer", "data-file")
    assert name.endswith("_s1.wblog") and (log_dir / name).stat().st_size == used and "Đã tải log về: " + name in pg.inner_text("#toasts")
    row = pg.locator('#logs tbody tr[data-file="%s"]' % name)
    assert row.count() == 1 and "open" in row.get_attribute("class") and pg.locator("#logs tbody tr").count() == 1 and pg.is_hidden("#logs-none")
    log = wblog.load(str(log_dir / name))[0]
    t, gyro, setp = log.fast["t"], log.fast["gyro[0]"], log.fast["setpoint[0]"]
    info = pg.inner_text("#log-info")
    assert "Windburst RX" in info and "flash trên mạch, phiên thứ 1" in info and "1000 bản ghi/giây" in info and "%d bản ghi" % len(t) in info
    assert pg.inner_text("#viewer-title") == "Đồ thị: " + name and pg.locator("#log-warn .note").count() == 0 and pg.is_hidden("#log-session") and pg.is_hidden("#log-follow")

    # ---- the charts: default groups, drawn lines, the whole log on screen
    plot = Plot(gui).loaded()
    assert [c.get_attribute("data-panel") for c in pg.locator("canvas.plot-cv").all()] == ["rate-0", "rate-1", "rate-2", "motors"]
    assert pg.is_checked("#preset-rate") and pg.is_checked("#preset-motors") and not pg.is_checked("#preset-pid")
    t0, t1 = plot.range()
    assert pg.get_attribute("#logplot", "data-whole") == "1" and t0 == pytest.approx(0.0, abs=0.01) and t1 == pytest.approx(log.events[-1][0], abs=0.01)
    assert pg.evaluate(INK, ['canvas[data-panel="rate-0"]', BLUE]) > 300 and pg.evaluate(INK, ['canvas[data-panel="rate-0"]', ORANGE]) > 300, "setpoint and gyro are drawn"
    assert pg.evaluate(INK, ['canvas[data-panel="rate-2"]', ORANGE]) > 300 and pg.evaluate(INK, ['canvas[data-panel="motors"]', YELLOW]) > 100

    step_i = max(range(5, len(t) - 5), key=lambda i: abs(setp[i + 5] - setp[i - 5]))         # where the setpoint moves fastest (the stick input): the blue line is steepest there
    ink = pg.evaluate(COLUMNS, ['canvas[data-panel="rate-0"]', BLUE])
    want_x = round(plot.x_of(t[step_i])[0] - plot.cv.bounding_box()["x"])
    there, elsewhere = max(ink[want_x - 4:want_x + 5]), sorted(ink)[len(ink) // 2]
    assert there >= 8 and there >= 0.5 * max(ink) and there > 4 * max(elsewhere, 2), \
        "the stick input at %.3f s must show as the steepest piece of the line at x = %d: %d px of ink in that column, %d at most anywhere, %d typically" % (t[step_i], want_x, there, max(ink), elsewhere)

    # ---- reading values: zoomed out the legend gives the span of the samples under the pointer, zoomed in the sample itself
    peak_i = max(range(len(t)), key=lambda i: setp[i])
    peak_t = t[peak_i]
    assert setp[peak_i] > 30, "the roll input is in the log"
    cur = plot.point_at(peak_t)
    assert cur == pytest.approx(peak_t, abs=(t1 - t0) / 900) and "con trỏ" in pg.inner_text("#plot-range")
    lo, hi = _span(plot.legend("setpoint[0]"))
    near = [setp[i] for i in range(len(t)) if abs(t[i] - cur) <= 0.007]
    assert min(near) - 0.01 <= lo <= hi <= max(near) + 0.01 and hi > 0.8 * setp[peak_i], (lo, hi, setp[peak_i])
    steep = max(range(2, len(t) - 2), key=lambda i: abs(gyro[i + 2] - gyro[i - 2]))          # where the gyro moves fastest, one pixel column holds different values
    cur = plot.point_at(t[steep])
    lo, hi = _span(plot.legend("gyro[0]"))
    near = [gyro[i] for i in range(len(t)) if abs(t[i] - cur) <= 0.007]
    assert len(t) > 2400 and "…" in plot.legend("gyro[0]") and lo < hi and min(near) - 0.01 <= lo and hi <= max(near) + 0.01, (plot.legend("gyro[0]"), min(near), max(near))
    plot.drag(peak_t - 0.20, peak_t + 0.30)
    plot.loaded()
    z0, z1 = plot.range()
    assert z0 == pytest.approx(peak_t - 0.20, abs=0.012) and z1 == pytest.approx(peak_t + 0.30, abs=0.012) and pg.get_attribute("#logplot", "data-whole") == "0"
    for at in (peak_t + 0.004, peak_t + 0.1, peak_t - 0.1):
        cur = plot.point_at(at)
        i = min(range(len(t)), key=lambda k: abs(t[k] - cur))
        got_s, got_g = float(plot.legend("setpoint[0]").split()[0]), float(plot.legend("gyro[0]").split()[0])
        assert "…" not in plot.legend("setpoint[0]") and plot.legend("gyro[0]").endswith("°/s")
        assert any(abs(got_s - setp[k]) < 0.06 and abs(got_g - gyro[k]) < 0.06 for k in (i - 1, i, i + 1)), \
            "the legend shows the logged sample at %.4f s: %s / %s, log %.2f / %.2f" % (cur, got_s, got_g, setp[i], gyro[i])
    pg.mouse.move(5, 5)
    assert plot.legend("setpoint[0]") == "" and pg.get_attribute("#logplot", "data-cursor") == ""

    # ---- moving around: buttons, shift-drag, double-click
    pg.click("#plot-out"); plot.loaded()
    a0, a1 = plot.range()
    assert a1 - a0 == pytest.approx(1.0, abs=0.03) and (a0 + a1) / 2 == pytest.approx((z0 + z1) / 2, abs=0.03)
    pg.click("#plot-in"); plot.loaded()
    assert plot.range()[1] - plot.range()[0] == pytest.approx(0.5, abs=0.02)
    pg.click("#plot-left"); plot.loaded()
    b0, b1 = plot.range()
    assert b0 == pytest.approx(z0 - 0.25, abs=0.03) and b1 - b0 == pytest.approx(0.5, abs=0.02)
    plot.drag(b0 + 0.1, b0 + 0.3, shift=True); plot.loaded()                # grab the chart at 0.1 s and pull it right: earlier data comes in
    assert plot.range()[0] == pytest.approx(b0 - 0.2, abs=0.02) and plot.range()[1] - plot.range()[0] == pytest.approx(0.5, abs=0.02)
    plot.cv.dblclick(position={"x": 300, "y": 60}); plot.loaded()
    assert pg.get_attribute("#logplot", "data-whole") == "1" and plot.range() == (t0, t1)
    for _ in range(12):
        pg.click("#plot-right")                                             # the view cannot leave the log
    plot.loaded()
    assert plot.range() == (t0, t1)

    # ---- chart groups
    pg.check("#preset-attitude"); pg.uncheck("#preset-motors")
    pg.wait_for_selector('canvas.plot-cv[data-panel="att"]')
    Plot(gui, "att").loaded()
    assert [c.get_attribute("data-panel") for c in pg.locator("canvas.plot-cv").all()] == ["rate-0", "rate-1", "rate-2", "att", "yaw"]
    att = Plot(gui, "att")
    cur = att.point_at(peak_t + 0.5)
    rl, rh = _span(att.legend("roll"))
    near = [log.fast["roll"][i] for i in range(len(t)) if abs(t[i] - cur) <= 0.012]
    assert min(near) - 0.01 <= rl <= rh <= max(near) + 0.01 and rh > 5, "roll angle after the stick input: %s, log %s" % ((rl, rh), (min(near), max(near)))
    # the PID terms are in PID sum units (a roll input makes tens of them; until 0.6.1 the log clipped them at 3.28), and the
    # loop group shows the stages of a cycle next to its length
    pg.check("#preset-pid"); pg.check("#preset-timing")
    pg.wait_for_selector('canvas.plot-cv[data-panel="stages"]')
    Plot(gui, "pid-0").loaded()
    assert [c.get_attribute("data-panel") for c in pg.locator("canvas.plot-cv").all()] == ["rate-0", "rate-1", "rate-2", "pid-0", "pid-1", "pid-2", "att", "yaw", "timing", "stages"]
    pid = Plot(gui, "pid-0")
    p_peak = max(abs(v) for v in log.fast["pid_p[0]"])
    assert p_peak > 20 and {"t_lock", "t_imu", "t_core", "t_est", "t_motor"} <= set(log.fast), "roll P term in the log: %.1f" % p_peak
    k = max(range(len(t)), key=lambda i: abs(log.fast["pid_p[0]"][i]))
    cur = pid.point_at(t[k])
    pl, ph = _span(pid.legend("pid_p[0]"))
    near = [log.fast["pid_p[0]"][i] for i in range(len(t)) if abs(t[i] - cur) <= 0.012]
    assert min(near) - 0.06 <= pl <= ph <= max(near) + 0.06 and max(abs(pl), abs(ph)) > 10, "P term under the crosshair: %s, log %s" % ((pl, ph), (min(near), max(near)))
    pg.uncheck("#preset-pid"); pg.uncheck("#preset-timing")
    # the navigation group is logged by default: the position estimate, optical flow, what Position Hold / Rescue asked for
    assert {"vel_n", "pos_e", "est_valid", "nav_flags", "flow_fwd", "flow_quality", "nav_roll", "nav_target_n", "rescue_phase"} <= set(log.slow), sorted(log.slow)
    pg.check("#preset-nav")
    pg.wait_for_selector('canvas.plot-cv[data-panel="navlean"]')
    assert [c.get_attribute("data-panel") for c in pg.locator("canvas.plot-cv").all()] == ["rate-0", "rate-1", "rate-2", "att", "yaw", "navvel", "navpos", "navlean", "flowq"]
    assert "Vị trí" in pg.locator('label[for="preset-nav"], #log-presets label:has(#preset-nav)').first.inner_text()
    pg.uncheck("#preset-nav")
    for box in ("rate", "attitude"):
        pg.uncheck("#preset-" + box)
    assert pg.locator("canvas.plot-cv").count() == 0 and "Chọn ít nhất một nhóm" in pg.inner_text("#logplot") and pg.is_hidden("#plot-all")
    pg.check("#preset-rate"); pg.check("#preset-motors")
    pg.wait_for_selector('canvas.plot-cv[data-panel="motors"]')

    # ---- events: listed, and a click goes there
    pg.click("#log-events summary")
    rows = [r.inner_text() for r in pg.locator("#log-events tbody tr").all()]
    assert any("ARM" in r and "DISARM" not in r for r in rows) and any("DISARM" in r for r in rows) and "Kết thúc: hết điều kiện ghi" in rows[-1] and any("Mode: ARM, ANGLE, ALTHOLD" in r for r in rows)
    disarm_t = [e[0] for e in log.events if e[1] == "DISARM"][-1]
    pg.locator('#log-events tr[data-event="DISARM"] .ev-go').last.click()
    Plot(gui).loaded()
    c0, c1 = Plot(gui).range()
    assert c1 - c0 == pytest.approx((t1 - t0) / 3, abs=0.02) and c0 < disarm_t <= c1 and c1 == pytest.approx(t1, abs=0.01), "a third of the log around the event, as far as the log reaches"

    # ---- exports carry the token and give real data
    csv = pg.evaluate("fetch(document.getElementById('log-csv').href).then(r => r.text())")
    assert csv.splitlines()[0].startswith("t (s),gyro[0] (deg/s)") and len(csv.splitlines()) == len(t) + 1
    flown = pg.evaluate("fetch(document.getElementById('log-settings').href).then(r => r.json())")
    assert flown["settings"]["blackbox"]["rate_hz"] == 1000 and flown["schema_matches"]
    assert pg.evaluate("fetch(document.querySelector('#logs tbody a').href).then(r => r.arrayBuffer()).then(b => b.byteLength)") == used

    # ---- delete on the computer, add the file again from somewhere else
    copy = log_dir.parent / "kept elsewhere.wblog"
    copy.write_bytes((log_dir / name).read_bytes())
    pg.once("dialog", lambda d: d.dismiss())
    pg.click("#logs .log-delete")
    assert (log_dir / name).exists(), "'Cancel' in the question keeps the file"
    pg.once("dialog", lambda d: d.accept())
    pg.click("#logs .log-delete")
    pg.wait_for_selector("#logs-none")
    assert not (log_dir / name).exists() and pg.is_hidden("#viewer") and pg.is_visible("#viewer-idle") and pg.locator("canvas.plot-cv").count() == 0
    pg.set_input_files("#log-file", str(copy))
    pg.wait_for_selector('#viewer[data-file="kept elsewhere.wblog"]')
    Plot(gui).loaded()
    assert (log_dir / "kept elsewhere.wblog").read_bytes() == copy.read_bytes() and pg.evaluate(INK, ['canvas[data-panel="rate-0"]', BLUE]) > 300
    junk = log_dir.parent / "holiday.wblog"
    junk.write_bytes(b"not a flight log at all " * 100)
    pg.set_input_files("#log-file", str(junk))
    pg.wait_for_selector(".toast.bad")
    assert pg.locator("#logs tbody tr").count() == 1 and not (log_dir / "holiday.wblog").exists()
    assert len(gui.errors) == 1 and "status of 400" in gui.errors[0], "the refusal of that file is the only thing the browser noted: %s" % gui.errors
    gui.errors.clear()

    # ---- erase the board's memory: asked first
    pg.once("dialog", lambda d: d.dismiss())
    pg.click("#bb-erase")
    time.sleep(0.6)
    assert pg.get_attribute("#bb-state", "data-state") == "IDLE" and pg.locator("#bb-sessions tbody tr").count() == 1 and gui.hub.fc.blackbox_status()["used"] == used
    pg.once("dialog", lambda d: d.accept())
    pg.click("#bb-erase")
    pg.wait_for_function("/Đã xoá xong bộ nhớ log/.test(document.getElementById('toasts').textContent)", timeout=30000)
    pg.wait_for_selector("#bb-none")
    assert pg.get_attribute("#bb-state", "data-state") == "IDLE" and gui.hub.fc.blackbox_status()["used"] == 0 and pg.is_disabled("#bb-download-all")
    assert pg.locator("#logs tbody tr").count() == 1, "erasing the board does not touch the logs on the computer"


def test_blackbox_on_an_sd_card_from_the_page(gui, log_dir):
    """Choosing the card, logging a flight to it, the card being pulled, download and erase - all from the Blackbox tab."""
    from wbclient import wblog
    pg = gui.connect().page
    gui.tab("blackbox")
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'OFF'")
    assert pg.is_hidden("#bb-sd") and pg.inner_text("#bb-store-title") == "Bộ nhớ log trên mạch"
    pg.select_option('[data-field="blackbox.device"]', "SDCARD")
    pg.wait_for_selector("#bb-sd:not([hidden])")
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'IDLE'", timeout=8000)
    pg.wait_for_function("document.getElementById('bb-store-title').textContent === 'Thẻ SD'")
    assert "64.0 MB" in pg.inner_text("#bb-used") and pg.inner_text("#save") == "Lưu", "choosing the card needs no restart"
    assert pg.input_value('[data-field="blackbox.sd_mode"]') == "SPI" and gui.field("blackbox.sd_mhz").input_value() == "20"
    gui.type_into("blackbox.rate_hz", 1000)
    # 1000 x 74 bytes x 1.04 + 900 = 77860 bytes a second into 64 MB = 862 s
    pg.wait_for_function("/đủ cho khoảng 14 phút 2\\d giây/.test(document.getElementById('bb-cost').textContent)")
    gui.sim.do("sdcard slow 25")
    _fly_and_land(gui.sim)
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'LOGGING'")
    pg.wait_for_selector("#bb-buffer:not([hidden])")
    pg.wait_for_function("Number(document.getElementById('bb-buffer').dataset.pct) >= 1 && Number(document.getElementById('bb-buffer').dataset.pct) < 30", timeout=6000)
    assert "Bộ đệm ghi thẻ: cao nhất" in pg.inner_text("#bb-buffer")
    gui.sim.do("rc 3 1000"); gui.sim.do("rc 5 1000")
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'IDLE'", timeout=8000)
    pg.wait_for_selector('#bb-sessions tbody tr[data-index="0"]')
    assert pg.is_hidden("#bb-buffer") and int(gui.sim.state()["stall_ms"]) == 0 and int(gui.sim.state()["sd_files"]) == 1
    pg.click("#bb-sessions .bb-get")
    pg.wait_for_selector('#viewer[data-file^="flight_"]', timeout=20000)
    name = pg.get_attribute("#viewer", "data-file")
    log = wblog.load(str(log_dir / name))[0]
    assert log.complete and log.dropped == 0 and len(log.fast["t"]) > 3000 and (log_dir / name).stat().st_size == gui.hub.fc.blackbox_status()["used"]
    # ---- the card is pulled out
    gui.sim.do("sdcard pull")
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'NO_STORAGE'", timeout=8000)
    pg.wait_for_selector("#bb-sessions", state="hidden")            # (the list goes once the page has asked for the sessions again)
    assert "Firmware không thấy thẻ SD" in pg.inner_text("#bb-why") and pg.is_disabled("#bb-erase")
    gui.sim.do("sdcard push")
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'IDLE'", timeout=8000)
    pg.wait_for_selector('#bb-sessions tbody tr[data-index="0"]')
    # ---- erase: the question names the files, not "the whole memory"
    asked = []
    pg.once("dialog", lambda d: (asked.append(d.message), d.accept()))
    pg.click("#bb-erase")
    gui.wait(lambda: int(gui.sim.state()["sd_files"]) == 0, what="log files deleted")
    assert "wb_*.wblog" in asked[0] and "không bị đụng tới" in asked[0]
    pg.wait_for_selector("#bb-none:not([hidden])")
    # ---- the card is the stored choice and it is not there at boot: the Overview shows the module as a warning (the flash is still there)
    pg.click("#save")
    pg.wait_for_selector("#save", state="hidden")
    gui.sim.do("sdcard pull")
    gui.hub.reboot()                                             # the firmware starts again (the simulated card stays out)
    gui.tab("overview")
    pg.wait_for_function("/Cảnh báo/.test((document.querySelector('[data-module=\"blackbox\"]') || {}).innerText || '')", timeout=8000)
    row = pg.inner_text('[data-module="blackbox"]')
    assert "Cảnh báo" in row and "partition 'sim'" in row and "SD card FAILED - no SD card" in row, row
    assert "Có" in pg.inner_text('[data-module="imu"]') and pg.locator('[data-module="pins"]').count() == 0


def test_motors_are_the_only_sounder(gui):
    """A craft without a buzzer: the Power tab says what it has to make noise with, from the pins and the ESC protocol."""
    pg = gui.connect().page
    gui.tab("power")
    note = lambda: (pg.get_attribute("#sounders", "data-buzzer"), pg.get_attribute("#sounders", "data-motors"), pg.get_attribute("#sounders", "class"))
    assert note() == ("1", "1", "note ok")
    gui.tab("pins"); gui.type_into("pins.buzzer", "")
    gui.tab("power")
    assert note() == ("0", "1", "note ok") and "Chưa gán chân còi: tiếng báo do motor phát" in pg.inner_text("#sounders")
    # throwing the change away re-reads the settings and rebuilds the tab: the note follows
    pg.once("dialog", lambda d: d.accept())
    pg.click("#revert")
    pg.wait_for_function("(document.getElementById('sounders') || {dataset: {}}).dataset.buzzer === '1'", timeout=12000)
    assert "có còi và motor kêu được" in pg.inner_text("#sounders")
    gui.tab("pins"); gui.type_into("pins.buzzer", "")
    gui.tab("power")
    assert note() == ("0", "1", "note ok")
    gui.tab("motors")
    pg.select_option('[data-field="motors.protocol"]', "ONESHOT125")
    gui.tab("power")
    assert note() == ("0", "0", "note todo") and "chưa có gì để kêu" in pg.inner_text("#sounders")
    gui.tab("pins"); gui.type_into("pins.buzzer", "21")
    gui.tab("power")
    assert note() == ("1", "0", "note ok") and "Chỉ có còi: ESC đang dùng giao thức ONESHOT125" in pg.inner_text("#sounders")
    # saved and restarted: the firmware agrees (boot report), and with neither sounder it says so
    gui.tab("pins"); gui.type_into("pins.buzzer", "")
    boots = int(gui.sim.state()["boots"])
    pg.click("#save")
    gui.wait(lambda: int(gui.sim.state()["boots"]) == boots + 1, 12, "firmware restart")
    pg.wait_for_selector("#save", state="hidden", timeout=12000)
    gui.tab("overview")
    pg.wait_for_function("/Không có/.test((document.querySelector('[data-module=\"beeper\"]') || {}).innerText || '')", timeout=8000)
    assert "no buzzer pin set, and the motors can only beep with a DShot protocol" in pg.inner_text('[data-module="beeper"]')


def test_blackbox_live_recording_on_the_page(gui, log_dir):
    pg = gui.connect().page
    gui.tab("blackbox")
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'OFF'")
    assert pg.inner_text("#rec-toggle") == "Bắt đầu ghi" and "239 bản ghi mỗi giây" in pg.inner_text("#rec-limit") and pg.input_value("#rec-rate") == "200"
    pg.fill("#rec-rate", "100")
    gui.sim.do("pose 20 -10 0")
    pg.click("#rec-toggle")
    pg.wait_for_selector('#viewer[data-file="@live"]', timeout=8000)
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'STREAMING'")
    assert pg.inner_text("#rec-toggle") == "Dừng ghi" and pg.is_disabled("#rec-rate") and pg.is_disabled("#bb-erase") and pg.inner_text("#viewer-title") == "Đồ thị: đang ghi trực tiếp"
    assert pg.is_visible("#log-follow") and pg.is_checked("#log-follow input") and "app, qua cáp" in pg.inner_text("#log-info") and "100 bản ghi/giây" in pg.inner_text("#log-info")
    n0 = int(pg.get_attribute("#rec-status", "data-records"))
    pg.check("#preset-attitude")
    pg.wait_for_selector('canvas.plot-cv[data-panel="att"]')
    att = Plot(gui, "att").loaded()
    first = att.range()
    time.sleep(1.2)
    later = att.range()
    assert later[1] > first[1] + 0.8 and int(pg.get_attribute("#rec-status", "data-records")) > n0 + 80, "the chart and the counter follow the recording"
    assert re.search(r"\d+ bản ghi, \d", pg.inner_text("#log-length")) and pg.inner_text("#log-length") == pg.inner_text("#rec-status")
    att.point_at(later[1] - 0.05)
    roll, pitch = float(att.legend("roll").split()[0]), float(att.legend("pitch").split()[0])
    assert roll == pytest.approx(20, abs=2) and pitch == pytest.approx(-10, abs=2), "the live chart shows the attitude of this moment"
    pg.mouse.move(5, 5)
    # looking back stops the following; the switch brings it back
    now = att.range()                                         # (not `later`: the chart has moved on while the legend was read)
    att.drag(now[1] - 1.0, now[1] - 0.5); att.loaded()
    held = att.range()
    assert not pg.is_checked("#log-follow input") or gui.wait(lambda: not pg.is_checked("#log-follow input"), what="follow switch off")
    time.sleep(0.8)
    # (the width is only roughly 0.5 s: while the chart still follows, its scale moves on between the two ends of the drag - by
    # more on a loaded PC, where the drag itself takes longer)
    assert att.range() == held and 0.15 < held[1] - held[0] < 0.85, held
    pg.check("#log-follow input")
    gui.wait(lambda: att.range()[1] > held[1] + 0.5, what="following again")
    # stop: the recording becomes a file in the library and stays on screen
    gui.sim.do("pose 0 0 0")
    pg.click("#rec-toggle")
    pg.wait_for_selector('#viewer[data-file^="live_"]', timeout=8000)
    name = pg.get_attribute("#viewer", "data-file")
    assert pg.inner_text("#rec-toggle") == "Bắt đầu ghi" and pg.is_enabled("#rec-rate") and pg.is_hidden("#log-follow") and pg.inner_text("#rec-status") == ""
    assert pg.locator('#logs tbody tr[data-file="%s"]' % name).count() == 1 and (log_dir / name).stat().st_size > 5000
    assert "app, qua cáp" in pg.inner_text("#log-info") and pg.locator("#log-warn .note").count() == 0
    rows = [r.text_content() for r in pg.locator("#log-events tbody tr").all()]            # the list is folded: read it without opening
    assert "Kết thúc: app dừng ghi" in rows[-1]
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'OFF'")
    pg.uncheck("#preset-attitude")


def test_logs_are_viewable_without_a_flight_controller_and_in_english(gui_en, log_dir):
    from wbclient import logview
    from test_blackbox import _log_bytes
    gui = gui_en
    lib = logview.Library(str(log_dir))
    lib.save("old flight.wblog", _log_bytes(records=3000) + _log_bytes(records=500)[:-40])      # a second session whose end is missing
    pg = gui.page
    pg.goto(gui.srv.url)
    pg.wait_for_selector(".welcome")
    gui.tab("blackbox")
    assert "No flight controller connected" in pg.inner_text("#view") and pg.locator("#bb-state").count() == 0 and pg.locator("#rec-toggle").count() == 0
    row = pg.locator('#logs tbody tr[data-file="old flight.wblog"]')
    assert "2" in row.locator("td").nth(1).inner_text() and "7.0 s" in row.inner_text()
    row.locator(".log-open").click()
    pg.wait_for_selector('#viewer[data-file="old flight.wblog"]')
    plot = Plot(gui).loaded()
    # a log of some other firmware: only the charts it has fields for, and a note about its settings
    assert [c.get_attribute("data-panel") for c in pg.locator("canvas.plot-cv").all()] == ["rate-0", "motors"]
    assert [b.inner_text() for b in pg.locator("#log-presets label").all()] == ["Gyro and setpoint", "Motor", "Attitude", "Battery"]
    assert "Handmade 9.9.9" in pg.inner_text("#log-info") and "3000 records" in pg.inner_text("#log-info") and "different settings layout" in pg.inner_text("#log-warn")
    assert pg.is_visible("#log-session") and pg.locator("#log-session option").count() == 2
    assert pg.evaluate(INK, ['canvas[data-panel="rate-0"]', BLUE]) > 300, "the gyro is drawn in the first colour: this log has no setpoint"
    assert "LOST 17" in pg.inner_text("#log-events") or pg.locator('#log-events tr[data-event="DROPPED"]').count() == 1
    assert "dropped 17 records" in pg.inner_text("#log-warn")
    cur = plot.point_at(3.0)
    assert cur == pytest.approx(3.0, abs=0.01) and "pointer 3.0" in pg.inner_text("#plot-range")
    no_vietnamese(gui, "log viewer, first session")
    pg.select_option("#log-session", "1")
    pg.wait_for_function("/500 records|4\\d\\d records/.test(document.getElementById('log-info').textContent)")
    Plot(gui).loaded()
    assert "no end marker" in pg.inner_text("#log-warn") and Plot(gui).range()[1] < 1.1
    no_vietnamese(gui, "log viewer, cut session")
    # connecting later keeps the tab usable and adds the board's part
    gui.choose_port(gui.sim.url)
    pg.click("#connect")
    pg.wait_for_selector("#bb-state", timeout=15000)
    pg.wait_for_function("document.getElementById('bb-state').dataset.state === 'OFF'")
    assert pg.inner_text("#bb-state") == "Off" and pg.inner_text("#rec-toggle") == "Start recording" and pg.locator('#logs tbody tr').count() == 1
    pg.wait_for_selector('#viewer[data-file="old flight.wblog"]')
    assert pg.input_value("#log-session") == "1" and "no end marker" in pg.inner_text("#log-warn"), "the log that was on screen is still on screen after connecting"
    no_vietnamese(gui, "blackbox tab, connected")


def test_pins_tab_stm32(gui_stm32):
    gui = gui_stm32
    pg = gui.connect().page
    gui.tab("pins")
    assert gui.field("pins.imu_cs").input_value() == "PA10" and gui.field("pins.batt_adc").input_value() == "PA2", "pin numbers shown as STM32 names"
    assert gui.field("pins.ppm").input_value() == "" and pg.locator("#pins-stm32").is_visible() and pg.locator("#pins-preset").count() == 0
    gui.type_into("pins.buzzer", "pc13")
    gui.wait(lambda: gui.setting("pins", "buzzer") == 45, what="PC13 = port C (2) * 16 + 13")
    assert gui.field("pins.buzzer").input_value() == "PC13"
    gui.type_into("pins.motor[4]", "PB3")
    gui.wait(lambda: gui.setting("pins", "motor")[4] == 19, what="a motor slot by name")
    gui.type_into("pins.led", "PZ1")                                   # not a pin: refused, the box shows the old value again
    pg.wait_for_selector(".toast:has-text('không phải tên chân STM32')")
    assert gui.setting("pins", "led") == 255 and gui.field("pins.led").input_value() == ""
    gui.type_into("pins.led", "PA11")
    pg.wait_for_function("/PA11 thường là USB D-/.test(document.getElementById('pin-warnings').textContent)")
    gui.type_into("pins.buzzer", "PA2")                                # the battery's ADC pin
    pg.wait_for_function("/PA2 đang gán cho cả/.test(document.getElementById('pin-warnings').textContent)")
    assert "không đo được điện áp" not in pg.inner_text("#pin-warnings"), "the ESP32-S3 ADC rule does not apply"
    gui.type_into("pins.buzzer", "")
    gui.wait(lambda: gui.setting("pins", "buzzer") == 255, what="pin cleared")


def test_wifi_settings_text_fields(gui):
    pg = gui.connect().page
    gui.tab("system")
    # (the simulated board says it has a radio): the texts are 0-padded UTF-8 bytes in the section, the password is hidden
    pg.wait_for_function("(document.getElementById('wifi-state') || { innerText: '' }).innerText.includes('cổng serial (USB / UART)')", timeout=6000)
    assert gui.field("wifi.pass").get_attribute("type") == "password" and gui.field("wifi.ssid").input_value() == ""
    gui.field("wifi.ssid").fill("Nhà Nagumo"); gui.field("wifi.ssid").press("Tab")
    gui.wait(lambda: bytes(gui.hub.fc.get("wifi")["ssid"]).rstrip(b"\0") == "Nhà Nagumo".encode(), timeout=5.0, what="the network name reached the firmware as UTF-8")
    gui.field("wifi.pass").fill("x" * 80); gui.field("wifi.pass").press("Tab")
    gui.wait(lambda: bytes(gui.hub.fc.get("wifi")["pass"]) == b"x" * 63 + b"\0", timeout=5.0, what="a password longer than the field: cut to 63 bytes, the end mark kept")
    gui.field("wifi.ssid").fill(""); gui.field("wifi.ssid").press("Tab"); gui.field("wifi.pass").fill(""); gui.field("wifi.pass").press("Tab")
    gui.wait(lambda: not any(gui.hub.fc.get("wifi")["ssid"]) and not any(gui.hub.fc.get("wifi")["pass"]), timeout=5.0, what="emptied again")
    if pg.locator("#revert").is_visible():
        pg.click("#revert")
