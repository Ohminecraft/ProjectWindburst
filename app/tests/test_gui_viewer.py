"""The log viewer in a real browser, with no flight controller (0.31): playback like Betaflight's Blackbox Explorer - the overview
bar, play / pause, the playhead running through the log, the sticks and motors at the playhead.
Needs the optional `playwright` package and a Chromium (or Microsoft Edge); skipped otherwise. No simulator needed."""
import os
import sys
import time

import pytest

pw = pytest.importorskip("playwright.sync_api")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_blackbox import _log_bytes  # noqa: E402
from wbclient.gui import GuiServer  # noqa: E402


@pytest.fixture(scope="module")
def browser():
    with pw.sync_playwright() as p:
        b = None
        for launch in (lambda: p.chromium.launch(), lambda: p.chromium.launch(channel="msedge")):
            try:
                b = launch()
                break
            except Exception:  # noqa: BLE001
                continue
        if b is None:
            pytest.skip("no Chromium / Edge for playwright")
        yield b
        b.close()


@pytest.fixture
def page(browser, tmp_path):
    logs = tmp_path / "logs"
    logs.mkdir()
    (logs / "flight.wblog").write_bytes(_log_bytes(records=5000))          # 10 s at 500 records/s
    srv = GuiServer(http_port=0, log_dir=str(logs)).start()
    pg = browser.new_page(viewport={"width": 1440, "height": 1000}, locale="vi-VN")
    errors = []
    # (the browser asks for /favicon.ico by itself; the page has none)
    pg.on("console", lambda m: errors.append("%s @ %s" % (m.text, m.location.get("url"))) if m.type == "error" and "favicon" not in str(m.location.get("url")) else None)
    pg.on("pageerror", lambda e: errors.append("pageerror: %s" % e))
    pg.goto(srv.url)
    pg.evaluate("() => { location.hash = 'blackbox'; }")
    pg.wait_for_selector(".log-open")
    pg.click(".log-open")
    pg.wait_for_function("() => document.getElementById('logplot') && Number(document.getElementById('logplot').dataset.loaded || 0) > 0", timeout=10000)
    yield pg
    pg.close()
    srv.stop()
    assert not errors, "the page logged errors: %s" % errors


def ds(pg, sel, key):
    return pg.evaluate("([s, k]) => document.querySelector(s).dataset[k]", [sel, key])


def test_overview_click_moves_playhead_and_window(page):
    pg = page
    assert pg.locator("#plot-overview").is_visible() and pg.locator("#plot-play").is_visible()
    box = pg.locator("#plot-overview").bounding_box()
    pg.mouse.click(box["x"] + box["width"] * 0.5, box["y"] + box["height"] / 2)
    pg.wait_for_function("() => document.getElementById('logplot').dataset.playhead !== ''")
    t = float(ds(pg, "#logplot", "playhead"))
    t0, t1 = float(ds(pg, "#logplot", "t0")), float(ds(pg, "#logplot", "t1"))
    assert 4.5 < t < 5.5, "the middle of the bar is the middle of the 10 s log (%.2f)" % t
    assert t0 < t < t1 and t1 - t0 < 5, "the chart shows a window around it (%.2f .. %.2f)" % (t0, t1)


def test_play_runs_the_playhead_and_the_hud_follows(page):
    pg = page
    box = pg.locator("#plot-overview").bounding_box()
    pg.mouse.click(box["x"] + box["width"] * 0.2, box["y"] + box["height"] / 2)
    pg.wait_for_function("() => document.getElementById('logplot').dataset.playhead !== ''")
    start = float(ds(pg, "#logplot", "playhead"))
    pg.click("#plot-play")
    assert ds(pg, "#logplot", "playing") == "1"
    time.sleep(1.0)
    pg.click("#plot-play")
    moved = float(ds(pg, "#logplot", "playhead")) - start
    assert ds(pg, "#logplot", "playing") == "0"
    assert 0.6 < moved < 1.4, "one second at 100 %%: the playhead moved %.2f s" % moved
    t0, t1 = float(ds(pg, "#logplot", "t0")), float(ds(pg, "#logplot", "t1"))
    p = float(ds(pg, "#logplot", "playhead"))
    assert abs((t0 + t1) / 2 - p) < 0.05, "the window stays centred on the playhead"
    pg.wait_for_function("p => document.getElementById('log-hud').dataset.t && Math.abs(Number(document.getElementById('log-hud').dataset.t) - p) < 0.01", arg=p)
    h = pg.evaluate("() => document.querySelector('#hud-motors .hud-motor i').style.height")
    assert h.endswith("%") and 0 < float(h[:-1]) <= 100, "the motor bars show the motors at the playhead (%s)" % h


def test_speed_and_keys(page):
    pg = page
    pg.select_option("#plot-speed", "0.25")
    box = pg.locator("#plot-overview").bounding_box()
    pg.mouse.click(box["x"] + box["width"] * 0.3, box["y"] + box["height"] / 2)
    pg.wait_for_function("() => document.getElementById('logplot').dataset.playhead !== ''")
    pg.focus("#logplot")
    start = float(ds(pg, "#logplot", "playhead"))
    pg.keyboard.press("Space")
    time.sleep(1.0)
    pg.keyboard.press("Space")
    moved = float(ds(pg, "#logplot", "playhead")) - start
    assert 0.1 < moved < 0.45, "one second at 25 %%: %.2f s of the log" % moved
    p = float(ds(pg, "#logplot", "playhead"))
    pg.keyboard.press("ArrowRight")
    assert abs(float(ds(pg, "#logplot", "playhead")) - (p + 0.002)) < 1e-3, "-> = one record (2 ms in this log)"
    pg.keyboard.press("Shift+ArrowLeft")
    assert abs(float(ds(pg, "#logplot", "playhead")) - (p + 0.002 - 0.02)) < 1e-3, "Shift + <- = ten back"
    pg.keyboard.press("End")
    assert float(ds(pg, "#logplot", "playhead")) > 9.9, "End: the end of the log"


@pytest.fixture
def tune_page(browser, tmp_path):
    from test_tune import tune_log_bytes
    logs = tmp_path / "logs"
    logs.mkdir()
    (logs / "tune.wblog").write_bytes(tune_log_bytes())
    srv = GuiServer(http_port=0, log_dir=str(logs)).start()
    pg = browser.new_page(viewport={"width": 1440, "height": 1000}, locale="vi-VN")
    errors = []
    pg.on("pageerror", lambda e: errors.append("pageerror: %s" % e))
    pg.goto(srv.url)
    pg.evaluate("() => { location.hash = 'blackbox'; }")
    pg.wait_for_selector(".log-open")
    pg.click(".log-open")
    pg.wait_for_selector("#log-tune")
    yield pg
    pg.close()
    srv.stop()
    assert not errors, errors


def test_pid_tuning_panel(tune_page):
    pg = tune_page
    pg.click("#log-tune")
    pg.wait_for_function("() => document.getElementById('tune-out').dataset.done === '1'", timeout=30000)
    assert pg.locator(".tune-axis").count() == 3 and pg.locator("canvas.tune-step").count() == 3
    rows = pg.locator("#tune-suggestions tbody tr")
    params = [rows.nth(i).get_attribute("data-param") for i in range(rows.count())]
    assert "roll.d" in params and "pitch.d" in params, params
    assert "30 → " in pg.inner_text('#tune-suggestions tr[data-param="roll.d"]'), "now -> suggested"
    assert pg.locator("#tune-apply").count() == 0, "no flight controller connected: nothing to apply to"
