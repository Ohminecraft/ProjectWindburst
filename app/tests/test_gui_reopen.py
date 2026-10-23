"""A board on native USB (ESP32-S3 / STM32 with TinyUSB) leaves the bus while it restarts and comes back as the same port a few
seconds later: the app opens it again by itself - after its own reboot (save + restart) and after a restart it did not ask for.
A fake client stands for the port here (the simulator never leaves)."""
import threading
import time

import pytest

from wbclient import gui
from wbclient.client import WbTimeout


class Board:
    """The USB device: present or not, and the clients that have it open."""
    def __init__(self):
        self.present = True
        self.answers = True                 # the firmware answers HELLO (False: USB is up, the firmware not yet)
        self.opens = 0
        self.leave_on_reboot = True         # native USB: the port goes away while the chip restarts
        self.clients = []

    def leave(self, back_after):
        self.present = False
        for c in list(self.clients):
            c.die()

        def back():
            time.sleep(back_after)
            self.present = True
        threading.Thread(target=back, daemon=True).start()


class FakeClient:
    board = None

    def __init__(self, port, baud, schema=None, timeout=1.0):
        self.port_name, self.baud, self.info, self.lost = port, baud, None, False
        self.on_log = self.on_disconnect = self.on_raw = None

    def open(self, hello=True):
        if not self.board.present:
            raise OSError("could not open port %s: FileNotFoundError" % self.port_name)
        self.board.opens += 1
        self.board.clients.append(self)
        return self

    def hello(self):
        if self.lost or not self.board.answers:
            raise WbTimeout("no answer")
        import types
        self.info = types.SimpleNamespace(name="Windburst", version="0.32.0", role="RX", protocol=1, schema_hash=0, features=[], mcu="ESP32S3", pins=None)

    def die(self):
        self.lost = True
        if self in self.board.clients:
            self.board.clients.remove(self)
        if self.on_disconnect:
            self.on_disconnect()

    def reboot(self, wait=True, timeout=10.0):
        if self.board.leave_on_reboot:
            threading.Timer(0.2, self.board.leave, args=(1.5,)).start()

    def close(self):
        if self in self.board.clients:
            self.board.clients.remove(self)

    def subscribe(self, *a, **k):
        if self.lost:
            raise WbTimeout("gone")

    def unsubscribe(self, *a, **k):
        pass

    def report(self):
        return []


@pytest.fixture
def hub(monkeypatch):
    b = Board()
    FakeClient.board = b
    monkeypatch.setattr(gui, "WbClient", FakeClient)
    monkeypatch.setattr(gui.Hub, "REOPEN_S", 6.0)
    h = gui.Hub(log_dir=None)
    h.connect("COM9")
    yield h, b
    h._stop = True


def _wait(cond, s=8.0):
    end = time.monotonic() + s
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_reboot_takes_the_port_again(hub):
    h, b = hub
    old, boots = h.fc, h.boots
    st = h.reboot()
    assert st["connected"] and h.fc is not old and not h.fc.lost and b.opens == 2, "the port came back: opened again"
    assert h.boots == boots + 1 and h.note is None and h.port == "COM9"
    time.sleep(0.3)
    assert h.fc is not None, "the old client's loss does not drop the new one"


def test_reboot_on_a_port_that_stays(hub):
    h, b = hub
    b.leave_on_reboot = False
    old = h.fc
    st = h.reboot()
    assert st["connected"] and h.fc is old and b.opens == 1 and h.boots == 1


def test_reboot_waits_for_the_firmware_after_usb(hub):
    h, b = hub
    b.answers = False
    threading.Timer(2.5, lambda: setattr(b, "answers", True)).start()
    assert h.reboot()["connected"] and b.opens >= 3, "USB back before the firmware answered: opened again until it did"


def test_reboot_on_a_port_that_never_comes_back(hub, monkeypatch):
    h, b = hub
    monkeypatch.setattr(gui.Hub, "REOPEN_S", 2.5)
    b.leave = lambda back_after: [setattr(b, "present", False)] + [c.die() for c in list(b.clients)]
    with pytest.raises(gui.ApiError) as e:
        h.reboot()
    assert e.value.code == "NO_ANSWER" and h.fc is None and h.note == {"code": "PORT_LOST", "port": "COM9"}


def test_a_restart_behind_our_back_is_followed(hub):
    h, b = hub
    b.leave(1.0)
    assert _wait(lambda: h.fc is None) and h.note == {"code": "PORT_LOST", "port": "COM9", "waiting": True}
    assert _wait(lambda: h.fc is not None) and h.note == {"code": "RECONNECTED", "port": "COM9"} and h.boots == 1


def test_the_user_ends_the_wait(hub):
    h, b = hub
    b.leave(1.5)
    assert _wait(lambda: h.fc is None)
    h.disconnect()
    time.sleep(2.5)
    assert h.fc is None, "disconnected by the user while waiting: not taken again"


def test_a_port_gone_for_good(hub, monkeypatch):
    h, b = hub
    monkeypatch.setattr(gui.Hub, "REOPEN_S", 1.5)
    b.present = False
    for c in list(b.clients):
        c.die()
    assert _wait(lambda: h.note == {"code": "PORT_LOST", "port": "COM9"}) and h.fc is None
