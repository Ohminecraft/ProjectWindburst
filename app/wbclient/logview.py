"""Flight logs for the GUI: the folder of .wblog files on this computer, and the reduced view of a log that a chart can draw.

A log holds far more samples than a screen has pixels (ten minutes at 250 records/s = 150 000 per field). view() answers "what
does this field look like between t0 and t1 on a chart `px` pixels wide": the samples themselves when they fit, otherwise the
lowest and the highest value of every pixel column - so a one-sample spike is still on the chart at any zoom.

No GUI code here: gui.py serves these as JSON, the tests call them directly.
"""
import bisect
import math
import os
import re
import threading
import time

from . import wblog

EXT = ".wblog"
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 ._()+-]{0,120}\.wblog$")


class LogError(Exception):
    """code: NO_SUCH_LOG | BAD_NAME | NOT_A_LOG | NO_SUCH_SESSION | NO_SUCH_FIELD"""

    def __init__(self, code, detail=""):
        super().__init__("%s %s" % (code, detail))
        self.code, self.detail = code, detail


def default_dir():
    return os.environ.get("WBCLIENT_LOG_DIR") or os.path.join(os.path.expanduser("~"), "Windburst", "logs")


def clean_name(name):
    """A file name that is safe inside the log folder: no path, no leading dot, the .wblog extension."""
    base = os.path.basename(str(name or "").replace("\\", "/")).strip()
    if base.lower().endswith(EXT):
        base = base[:-len(EXT)]
    base = re.sub(r"[^A-Za-z0-9 ._()+-]", "_", base).strip(" .")[:100] or "log"
    return base + EXT


class Library:
    """The folder where downloaded, recorded and imported logs are kept. Files are parsed once and remembered until they change."""

    KEEP = 3                               # parsed logs held in memory (a long log is tens of MB of columns)

    def __init__(self, root=None):
        self.root = os.path.abspath(root or default_dir())
        self._lock = threading.Lock()
        self._parsed = {}                  # name -> (size, mtime, [Session])
        self._brief = {}                   # name -> (size, mtime, summary for the list)
        self._order = []

    # ---- files
    def path(self, name):
        if not isinstance(name, str) or not _NAME.match(name):
            raise LogError("BAD_NAME", str(name))
        full = os.path.join(self.root, name)
        if not os.path.isfile(full):
            raise LogError("NO_SUCH_LOG", name)
        return full

    def free_name(self, wanted):
        """`wanted` made safe, with (2), (3) ... added when a file of that name exists."""
        name = clean_name(wanted)
        stem, n = name[:-len(EXT)], 1
        while os.path.exists(os.path.join(self.root, name)):
            n += 1
            name = "%s (%d)%s" % (stem, n, EXT)
        return name

    def save(self, wanted, data):
        """Store log bytes under a free name; returns the name. Refuses data that holds no session."""
        if not wblog.parse(bytes(data[:200000])):
            raise LogError("NOT_A_LOG", clean_name(wanted))
        os.makedirs(self.root, exist_ok=True)
        name = self.free_name(wanted)
        tmp = os.path.join(self.root, name + ".part")
        with open(tmp, "wb") as fp:
            fp.write(data)
        os.replace(tmp, os.path.join(self.root, name))
        return name

    def delete(self, name):
        os.remove(self.path(name))
        with self._lock:
            self._parsed.pop(name, None)
            self._brief.pop(name, None)

    def read(self, name):
        with open(self.path(name), "rb") as fp:
            return fp.read()

    def list(self):
        """Every log in the folder, newest first: {file, size, mtime, sessions, duration, firmware}."""
        try:
            names = [n for n in os.listdir(self.root) if _NAME.match(n) and os.path.isfile(os.path.join(self.root, n))]
        except OSError:
            return []
        out = []
        for name in names:
            try:
                st = os.stat(os.path.join(self.root, name))
            except OSError:
                continue
            brief = self._brief.get(name)
            if not brief or brief[:2] != (st.st_size, st.st_mtime):
                try:
                    sessions = wblog.load(os.path.join(self.root, name))
                except OSError:
                    continue
                brief = (st.st_size, st.st_mtime, {"sessions": len(sessions), "duration": round(sum(s.duration for s in sessions), 2),
                                                   "firmware": sessions[0].meta.get("firmware", "") if sessions else "",
                                                   "records": sum(len(s.fast["t"]) for s in sessions)})
                self._brief[name] = brief
            out.append(dict({"file": name, "size": st.st_size, "mtime": int(st.st_mtime)}, **brief[2]))
        out.sort(key=lambda x: (-x["mtime"], x["file"]))
        return out

    # ---- parsed logs
    def sessions(self, name):
        full = self.path(name)
        st = os.stat(full)
        with self._lock:
            hit = self._parsed.get(name)
            if hit and hit[:2] == (st.st_size, st.st_mtime):
                return hit[2]
        sessions = wblog.load(full)
        if not sessions:
            raise LogError("NOT_A_LOG", name)
        with self._lock:
            self._parsed[name] = (st.st_size, st.st_mtime, sessions)
            self._order = [n for n in self._order if n != name] + [name]
            while len(self._order) > self.KEEP:
                self._parsed.pop(self._order.pop(0), None)
        return sessions

    def session(self, name, index):
        sessions = self.sessions(name)
        if not isinstance(index, int) or not 0 <= index < len(sessions):
            raise LogError("NO_SUCH_SESSION", "%s #%s" % (name, index))
        return sessions[index]


def stamp_name(prefix, session=None, when=None):
    """log file names: flight_20261005_143012_s7.wblog (the flight controller has no clock: the time is when the app got the log)."""
    t = time.strftime("%Y%m%d_%H%M%S", time.localtime(when))
    return "%s_%s%s%s" % (prefix, t, "_s%s" % session if session not in (None, "") else "", EXT)


def _decimals(scale):
    return max(0, min(7, int(math.ceil(-math.log10(scale) - 1e-9)))) if 0 < scale < 1 else 0


def view(session, names, t0=None, t1=None, px=800, tail=None):
    """The part of `session` between t0 and t1 seconds (or its last `tail` seconds: a recording that is still growing),
    reduced for a chart `px` pixels wide.

    -> {"t0", "t1", "span": [first, last second of the log], "series": {name: {...}}}
       a series is {"unit", "dt", "t": [...], "v": [...]}                 when its samples fit (at most two per pixel)
                or {"unit", "dt", "t": [...], "lo": [...], "hi": [...]}   lowest / highest value of each pixel column, t = its first sample
       dt = the usual time between two samples of that field AS RECORDED (the median step, not the header's nominal rate):
            a step much longer than that is a hole in the log.
    """
    px = max(16, min(int(px), 4000))
    ft, st = session.fast.get("t", ()), session.slow.get("t", ())
    et = [e[0] for e in session.events]                    # an ARM just before the first record, the END just after the last one: they belong on the chart
    first = min([x[0] for x in (ft, st, et) if len(x)] or [0.0])
    last = max([x[-1] for x in (ft, st, et) if len(x)] or [0.0])
    lo_t = first if t0 is None else float(t0)
    hi_t = last if t1 is None else float(t1)
    if tail:
        lo_t, hi_t = max(first, last - float(tail)), last
    if not hi_t > lo_t:
        hi_t = lo_t + 1e-3
    out = {"t0": lo_t, "t1": hi_t, "span": [first, last], "series": {}}
    for name in names:
        try:
            col, t, f = session.field(name)
        except KeyError:
            raise LogError("NO_SUCH_FIELD", name)
        a = max(0, bisect.bisect_left(t, lo_t) - 1)           # one sample outside on each side: the line reaches the edge of the chart
        b = min(len(t), bisect.bisect_right(t, hi_t) + 1)
        nd = _decimals(f.scale)
        s = {"unit": f.unit, "dt": round(session.sample_dt(slow=t is not ft), 6)}
        n = b - a
        if n <= 2 * px:
            s["t"] = [round(x, 6) for x in t[a:b]]
            s["v"] = [round(x, nd) for x in col[a:b]]
        else:
            ts, lo, hi = [], [], []
            for k in range(px):
                i, j = a + k * n // px, a + (k + 1) * n // px
                if j <= i:
                    continue
                part = col[i:j]
                ts.append(round(t[i], 6))
                lo.append(round(min(part), nd))
                hi.append(round(max(part), nd))
            s["t"], s["lo"], s["hi"] = ts, lo, hi
        out["series"][name] = s
    return out


class Recording:
    """A live stream being written to a file and read at the same time (the chart follows it while it grows)."""

    def __init__(self, path, name):
        self.path, self.name = path, name
        self.reader = wblog.Reader()
        self.lock = threading.Lock()
        self.bytes = 0
        self.frames = 0
        self.started = time.monotonic()
        self.stopped = None
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._fp = open(path, "wb")

    def frame(self, msg_type, seq, payload, encoded):
        """One blackbox frame from the firmware (called on the client's reader thread)."""
        with self.lock:
            if self._fp is None:
                return
            self._fp.write(encoded)
            self.bytes += len(encoded)
            self.frames += 1
            self.reader.frame(msg_type, seq, payload)

    def session(self):
        """The newest session of the stream so far (None before its header arrived)."""
        with self.lock:
            sessions = self.reader.sessions()
        return sessions[-1] if sessions else None

    def status(self):
        s = self.session()
        return {"on": self.stopped is None, "file": self.name, "bytes": self.bytes, "records": len(s.fast["t"]) if s else 0,
                "duration": round(s.duration, 2) if s else 0.0, "seconds": round((self.stopped or time.monotonic()) - self.started, 1),
                "complete": bool(s and s.complete), "gaps": s.gaps if s else 0, "dropped": s.dropped if s else 0}

    def close(self):
        with self.lock:
            if self._fp is not None:
                self._fp.close()
                self._fp = None
                self.stopped = time.monotonic()
