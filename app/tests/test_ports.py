"""The app never raises DTR or RTS when it opens a port: on a USB-serial bridge they reset the board or hold GPIO0, and on the
ESP32-S3's USB Serial/JTAG port RTS with DTR low resets the chip."""
import inspect

from wbclient.client import WbClient


def test_open_keeps_dtr_and_rts_low():
    src = inspect.getsource(WbClient.open)
    assert "ser.dtr = False" in src and "ser.rts = False" in src
    assert "dtr = True" not in src and "rts = True" not in src
