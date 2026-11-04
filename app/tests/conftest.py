"""Fixtures: a real firmware (build/wb_sim: FlightApp + drivers + flight core + WBP server on a virtual board) per test."""
import os
import socket
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SIM = os.path.join(ROOT, "build", "wb_sim" + (".exe" if os.name == "nt" else ""))
sys.path.insert(0, os.path.dirname(HERE))
os.environ["WBCLIENT_TILES_OFFLINE"] = "1"                  # the Mission tab's map: never the network in a test (wbclient/maptiles.py)


def _free_port_pair():
    for _ in range(50):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        p = s.getsockname()[1]
        s.close()
        ok = True
        for q in (p + 1, p + 2):                # the tester's control port, the simulated transmitter's app port
            t = socket.socket()
            try:
                t.bind(("127.0.0.1", q))
            except OSError:
                ok = False
            t.close()
        if ok:
            return p
    raise RuntimeError("no free port pair")


@pytest.fixture(autouse=True)
def log_dir(tmp_path, monkeypatch):
    """The GUI's folder of flight logs: a fresh one per test, never the user's ~/Windburst/logs."""
    d = tmp_path / "logs"
    monkeypatch.setenv("WBCLIENT_LOG_DIR", str(d))
    return d


class Sim:
    """A running simulator + its control channel (the tester's hand: sticks, poses, shaking)."""

    def __init__(self, tmp_path, *extra, speed=4.0, exe=None):
        self.exe = exe or SIM                      # (another build: an older firmware's simulator, test_gui_browser.py gui_old)
        if not os.path.exists(self.exe):
            pytest.skip("%s not built (run `make sim`)" % self.exe)
        self.port = _free_port_pair()
        self.flash = str(tmp_path / "flash.bin")
        self.speed = speed
        self.extra = list(extra)
        self.proc = None
        self.ctl = None
        self.start()

    @property
    def url(self):
        return "socket://127.0.0.1:%d" % self.port

    @property
    def tx_url(self):
        """The simulated DIY transmitter (0.38): its own app link, two ports above the board's."""
        return "socket://127.0.0.1:%d" % (self.port + 2)

    def start(self):
        self.proc = subprocess.Popen([self.exe, "--port", str(self.port), "--flash", self.flash, "--speed", str(self.speed)] + self.extra,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        end = time.monotonic() + 5
        while True:
            try:
                self.ctl = socket.create_connection(("127.0.0.1", self.port + 1), timeout=1.0)
                if self.ctl.getsockname() == self.ctl.getpeername():     # a TCP self-connect (Windows, before the simulator listens): it echoes
                    self.ctl.close()
                    raise OSError("self-connect")
                break
            except OSError:
                if time.monotonic() > end or self.proc.poll() is not None:
                    raise RuntimeError("wb_sim did not start: " + (self.proc.stdout.read() if self.proc.poll() is not None else "timeout"))
                time.sleep(0.05)
        self.ctlf = self.ctl.makefile("rw")

    def do(self, line):
        self.ctlf.write(line + "\n")
        self.ctlf.flush()
        rep = self.ctlf.readline().strip()
        assert rep.startswith("ok"), "%r -> %r" % (line, rep)
        return rep

    def state(self):
        return dict(kv.split("=") for kv in self.do("state").split()[1:])

    def sleep(self, sim_seconds):
        """Wait until this much SIMULATED time has passed."""
        t0 = int(self.state()["t_ms"])
        while int(self.state()["t_ms"]) - t0 < sim_seconds * 1000:
            time.sleep(0.01)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                self.do("quit")
                self.proc.wait(timeout=3)
            except Exception:
                self.proc.kill()
        if self.ctl:
            self.ctl.close()
        self.proc = None

    def power_cycle(self):
        """Stop the process and start it again on the same flash file."""
        self.stop()
        self.start()


@pytest.fixture
def sim(tmp_path):
    s = Sim(tmp_path)
    yield s
    s.stop()


@pytest.fixture
def fc(sim):
    from wbclient import WbClient
    c = WbClient(sim.url, timeout=2.0)
    c.open()
    sim.sleep(1.5)                      # boot gyro calibration
    yield c
    c.close()


class PtyPort:
    """A real serial device (a pseudo-terminal) as the flight controller's port: pyserial opens it like a COM port / ttyACM,
    so the serial code path is exercised, not only socket://. With a simulator behind it, bytes are bridged both ways;
    without one it is a board that only prints text (a boot loop, or another firmware)."""

    def __init__(self, sim=None, chatter=b"", chatter_period=0.2):
        import os
        import threading
        import tty
        if os.name == "nt":
            pytest.skip("pseudo-terminals are POSIX only")
        self.master, slave = os.openpty()
        tty.setraw(self.master)
        tty.setraw(slave)
        self.path = os.ttyname(slave)
        self._slave = slave                 # kept open so the master never sees a hang-up before the client opens the port
        self.sock = socket.create_connection(("127.0.0.1", sim.port), timeout=2.0) if sim else None
        self.chatter, self.period = chatter, chatter_period
        self.alive = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        import os
        import select
        last = 0.0
        while self.alive and self.master is not None:
            rd = [self.master] + ([self.sock] if self.sock else [])
            try:
                ready, _, _ = select.select(rd, [], [], 0.02)
            except (OSError, ValueError):
                return
            try:
                if self.master in ready:
                    data = os.read(self.master, 4096)
                    if self.sock and data:
                        self.sock.sendall(data)
                if self.sock and self.sock in ready:
                    data = self.sock.recv(4096)
                    if not data:
                        return
                    os.write(self.master, data)
                if self.chatter and time.monotonic() - last > self.period:
                    last = time.monotonic()
                    os.write(self.master, self.chatter)
            except (OSError, TypeError):                         # (TypeError: the port was closed under us - master is None now)
                if not self.alive or self.master is None:
                    return
                time.sleep(0.02)

    def inject(self, data):
        """Bytes that appear on the port between the firmware's frames (a crash message, another task printing)."""
        import os
        os.write(self.master, data)

    def unplug(self):
        """The cable is pulled: the device disappears under the client."""
        import os
        self.alive = False
        fds, self.master, self._slave = (self.master, self._slave), None, None      # closed once: a second close could hit a descriptor reused by someone else
        for fd in fds:
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        if self.sock:
            self.sock.close()
            self.sock = None

    close = unplug


@pytest.fixture
def serial_port(sim):
    p = PtyPort(sim)
    yield p
    p.close()
