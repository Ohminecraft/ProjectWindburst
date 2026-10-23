"""The app never raises DTR or RTS when it opens a port: on a USB-serial bridge they reset the board or hold GPIO0, and on the
ESP32-S3's USB Serial/JTAG port RTS with DTR low resets the chip."""
import inspect

from wbclient.client import WbClient


def test_open_keeps_dtr_and_rts_low():
    src = inspect.getsource(WbClient.open)
    assert "ser.dtr = False" in src and "ser.rts = False" in src
    assert "dtr = True" not in src and "rts = True" not in src


class _Port:
    def __init__(self, device, description, product=None, vid=0x303A, pid=0x1001):
        self.device, self.description, self.product, self.vid, self.pid, self.manufacturer = device, description, product, vid, pid, None


def _hub_ports(monkeypatch, comports, names):
    import socket

    import pytest
    list_ports = pytest.importorskip("serial.tools.list_ports")

    from wbclient import gui, usbnames
    monkeypatch.setattr(list_ports, "comports", lambda: comports)
    monkeypatch.setattr(usbnames, "product_names", lambda: names)
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: (_ for _ in ()).throw(OSError()))
    return {p["port"]: p for p in gui.Hub(log_dir=None).ports()}


def test_port_list_has_the_device_own_name(monkeypatch):
    """Windows calls every CDC port "USB Serial Device (COMx)": the list carries the device's iProduct (usbnames) as "product"."""
    out = _hub_ports(monkeypatch, [_Port("COM5", "USB Serial Device (COM5)"), _Port("COM7", "USB Serial Device (COM7)"),
                                   _Port("/dev/ttyACM0", "Windburst ESP32-S3", product="Windburst ESP32-S3")],
                     {"COM5": "Windburst-ESP32-S3"})
    assert out["COM5"]["product"] == "Windburst-ESP32-S3" and out["COM5"]["description"] == "USB Serial Device (COM5)"
    assert out["COM7"]["product"] == "", "no name read: the page falls back to the description"
    assert out["/dev/ttyACM0"]["product"] == "Windburst ESP32-S3", "Linux / macOS: pyserial's own product string"


def test_port_page_shows_the_product_first():
    import os
    src = open(os.path.join(os.path.dirname(__file__), "..", "wbclient", "web", "js", "main.js"), encoding="utf-8").read()
    assert "const name = p.product || p.description;" in src


def test_product_names_never_raises(monkeypatch):
    import sys

    from wbclient import usbnames
    names = usbnames.product_names()                     # the real lookup on this computer (Windows: SetupAPI)
    assert isinstance(names, dict) and all(isinstance(k, str) and isinstance(v, str) and v for k, v in names.items())
    monkeypatch.setattr(usbnames, "_windows", lambda: 1 / 0)
    monkeypatch.setattr(sys, "platform", "win32")
    assert usbnames.product_names() == {}
    monkeypatch.setattr(sys, "platform", "linux")
    assert usbnames.product_names() == {}
