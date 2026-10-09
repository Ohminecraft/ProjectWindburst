"""Reader for Windburst flight logs (.wblog) - what the blackbox stores in the flight controller's flash, or streams to the app.

    from wbclient import wblog
    for s in wblog.load("flight.wblog"):               # a file can hold several sessions (one per arming)
        print(s.meta["firmware"], s.duration, "s,", len(s.fast["t"]), "records")
        roll_rate = s.fast["gyro[0]"]                  # array('d'), deg/s; s.fast["t"] is the time in seconds from the session start
        print(s.settings(schema)["pid"]["p"])          # the settings the flight was flown with
        for t, name, data in s.events: ...             # ARM, DISARM, FAILSAFE, MODES, DROPPED, BATTERY, END

A log is a sequence of WBP frames (see framing.py): BB_HEADER frames carry a text that lists every field with its type, scale and
unit, BB_DATA frames carry the records. The reader takes the layout from that text only, so logs of other firmware versions
stay readable. Damaged or missing frames cost the records inside them and are counted (session.gaps); nothing after them is lost.
"""
import struct
from array import array

from . import framing as F

BB_HEADER, BB_DATA, BB_BLOB = F.BB_HEADER, F.BB_DATA, F.BB_BLOB
_FMT = {"u8": "B", "i16": "h", "u16": "H", "i32": "i", "u32": "I"}
END_REASONS = {0: "trigger off", 1: "storage full", 2: "stream stopped", 3: "app took over"}


class Field:
    __slots__ = ("name", "type", "scale", "unit")

    def __init__(self, token):
        self.name, self.type, scale, self.unit = token.split(":", 3)
        self.scale = float(scale)
        if self.type not in _FMT:
            raise ValueError("unknown field type %r" % self.type)

    def __repr__(self):
        return "%s [%s]" % (self.name, self.unit) if self.unit else self.name


def _layout(line):
    fields = [Field(tok) for tok in line.split()]
    return fields, struct.Struct("<" + "".join(_FMT[f.type] for f in fields))


class _Clock:
    """u32 microsecond stamps (they wrap) -> seconds from an origin; keeps its place between calls."""
    __slots__ = ("prev", "acc")

    def __init__(self):
        self.prev, self.acc = None, 0

    def unwrap(self, stamps, origin):
        out, prev, acc = array("d"), self.prev, self.acc
        for t in stamps:
            step = ((t - (origin if prev is None else prev) + 0x80000000) & 0xFFFFFFFF) - 0x80000000      # signed distance from the previous stamp
            acc, prev = acc + step, t
            out.append(acc * 1e-6)
        self.prev, self.acc = prev, acc
        return out


class Session:
    """One recording: from the trigger (arming, the BLACKBOX switch, the app's "record") to its end."""

    def __init__(self):
        self.text = ""                    # the header as written by the firmware
        self.meta = {}                    # header lines: firmware, schema, loop_hz, rate_hz, motors, session, to ...
        self.fast_fields, self.slow_fields = [], []
        self.fast, self.slow = {"t": array("d")}, {"t": array("d")}       # field name -> array('d') in engineering units; "t" = seconds from the first record
        self.events = []                  # (t seconds, name, data)
        self.event_names = {}
        self.frames = 0                   # data frames read
        self.gaps = 0                     # places where frames are missing (lost on the serial link, damaged in flash)
        self.dropped = 0                  # records the firmware itself could not queue (DROPPED events)
        self.bad_records = 0
        self.complete = False             # an END event was found: the session was closed properly
        self._settings_hex = {}
        self._fast_struct = self._slow_struct = None
        self._fbuf, self._sbuf = bytearray(), bytearray()
        self._raw_events = []
        self._expect_seq = None
        self._header_done = False
        self._columns_made = False
        self._t_origin = None             # the u32 time stamp that is t = 0
        self._clocks = (_Clock(), _Clock(), _Clock())       # fast records, slow records, events: each kind is in time order by itself

    # ---- building (used by parse / Reader)
    def _finish_header(self):
        self._header_done = True
        for line in self.text.split("\n"):
            key, _, val = line.partition(" ")
            if key == "fast":
                self.fast_fields, self._fast_struct = _layout(val)
            elif key == "slow":
                self.slow_fields, self._slow_struct = _layout(val)
            elif key == "set":
                name, _, hexdata = val.partition(" ")
                self._settings_hex[name] = hexdata
            elif key == "events":
                self.event_names = {int(k): v for k, v in (tok.split(":") for tok in val.split())}
            elif key:
                self.meta[key] = val

    def _data(self, seq, payload):
        if not self._header_done:
            self._finish_header()
        if self._expect_seq is not None and seq != self._expect_seq:
            self.gaps += 1
        self._expect_seq = (seq + 1) & 0xFF
        self.frames += 1
        fs = self._fast_struct.size if self._fast_struct else 0
        ss = self._slow_struct.size if self._slow_struct else 0
        i, n = 0, len(payload)
        while i < n:
            kind = payload[i]
            i += 1
            if kind == 0x46 and fs and i + fs <= n:               # 'F'
                self._fbuf += payload[i:i + fs]
                i += fs
            elif kind == 0x53 and ss and i + ss <= n:             # 'S'
                self._sbuf += payload[i:i + ss]
                i += ss
            elif kind == 0x45 and i + 6 <= n and i + 6 + payload[i + 5] <= n:      # 'E'  [t u32][code][n][data]
                t_us, code, dn = struct.unpack_from("<IBB", payload, i)
                self._raw_events.append((t_us, code, bytes(payload[i + 6:i + 6 + dn])))
                i += 6 + dn
            else:
                self.bad_records += 1
                break

    def _grow(self, fields, st, buf, cols, clock):
        """Move the whole records waiting in `buf` into the columns (engineering units, time unwrapped)."""
        if not st:
            return
        n = len(buf) - len(buf) % st.size
        if not n:
            return
        columns = list(zip(*st.iter_unpack(bytes(buf[:n]))))
        del buf[:n]
        for k, f in enumerate(fields):
            if f.name == "t":
                cols["t"].extend(clock.unwrap(columns[k], self._t_origin))
            elif f.scale == 1.0:
                cols[f.name].extend(columns[k])
            else:
                cols[f.name].extend(map(f.scale.__mul__, columns[k]))

    def _finish(self, final=True):
        """Bring the columns and the event list up to date with what has been read so far. Fast records, slow records and events
        share one time base: seconds from the earliest of their first time stamps (micros() wraps after 71.6 minutes; the stamps
        are unwrapped). Cheap to call again while a recording grows: only what arrived since the last call is converted.
        final=False: more bytes may follow, so a header that is still arriving is left alone."""
        if not self._header_done:
            if not final and not self.text.endswith("end\n"):
                return self
            self._finish_header()
        if not self._columns_made:
            self._columns_made = True
            for cols, fields in ((self.fast, self.fast_fields), (self.slow, self.slow_fields)):
                for f in fields:
                    cols.setdefault(f.name, array("d"))
        if self._t_origin is None:
            firsts = []
            for fields, st, buf in ((self.fast_fields, self._fast_struct, self._fbuf), (self.slow_fields, self._slow_struct, self._sbuf)):
                if st and len(buf) >= st.size:
                    firsts.append(st.unpack_from(buf)[[f.name for f in fields].index("t")])
            if self._raw_events:
                firsts.append(self._raw_events[0][0])
            if not firsts:
                return self
            self._t_origin = self._origin(firsts)
        self._grow(self.fast_fields, self._fast_struct, self._fbuf, self.fast, self._clocks[0])
        self._grow(self.slow_fields, self._slow_struct, self._sbuf, self.slow, self._clocks[1])
        fresh, self._raw_events = self._raw_events, []
        for (t_us, code, data), t in zip(fresh, self._clocks[2].unwrap([e[0] for e in fresh], self._t_origin)):
            name = self.event_names.get(code, "EVENT_%d" % code)
            self.events.append((t, name, data))
            if name == "DROPPED" and len(data) == 4:
                self.dropped += struct.unpack("<I", data)[0]
            if name == "END":
                self.complete = True
        return self

    @staticmethod
    def _origin(firsts):
        """The earliest of a few u32 time stamps that lie close together (they may straddle a wrap of the counter)."""
        if not firsts:
            return 0
        best = firsts[0]
        for t in firsts[1:]:
            if ((best - t) & 0xFFFFFFFF) < 0x80000000:        # t is earlier than best
                best = t
        return best

    # ---- reading
    @property
    def duration(self):
        t = self.fast.get("t")
        if t and len(t) > 1:
            return t[-1] - t[0]
        t = self.slow.get("t")
        return t[-1] - t[0] if t and len(t) > 1 else 0.0

    @property
    def rate_hz(self):
        try:
            return float(self.meta.get("rate_hz", 0))
        except ValueError:
            return 0.0

    def sample_dt(self, slow=False):
        """The usual time between two records as they really are in the log (the median step), in seconds.

        The header's rate_hz is what the firmware aimed for. A control loop that ran slower than configured spaces its records
        wider, and a chart that breaks its line wherever a step is longer than 3 x the NOMINAL spacing then shows single dots
        instead of a line. Holes are judged against the spacing the log really has."""
        t = (self.slow if slow else self.fast).get("t")
        n = len(t) if t else 0
        nominal = 0.1 if slow else (1.0 / self.rate_hz if self.rate_hz > 0 else 0.0)
        if n < 8:
            return nominal
        key = (slow, n if n < 4000 else n // 1000)             # a recording that is still growing: measured again now and then
        if getattr(self, "_dt_key", None) != key:
            step = max(1, (n - 1) // 2000)                      # up to 2000 steps, spread over the whole log
            d = sorted(t[i + 1] - t[i] for i in range(0, n - 1, step))
            self._dt_key, self._dt_val = key, d[len(d) // 2]
        return self._dt_val if self._dt_val > 0 else nominal

    def field(self, name):
        """(column, time column, Field) of a fast or slow field."""
        for fields, cols in ((self.fast_fields, self.fast), (self.slow_fields, self.slow)):
            for f in fields:
                if f.name == name and name != "t":
                    return cols[name], cols["t"], f
        raise KeyError("no field %r in this log" % name)

    def names(self):
        return [f.name for f in self.fast_fields if f.name != "t"], [f.name for f in self.slow_fields if f.name != "t"]

    def settings(self, schema):
        """The settings stored in the header, decoded with `schema` (sections whose size differs from the schema's are left out:
        the log was written by a firmware with another layout - check meta['schema'])."""
        out = {}
        for name, hexdata in self._settings_hex.items():
            if name not in schema.sections:
                continue
            layout = schema.layout("section", name)
            try:
                raw = bytes.fromhex(hexdata)
            except ValueError:
                continue
            if len(raw) == layout.size:
                out[name] = layout.unpack(raw)
        return out

    def schema_matches(self, schema):
        try:
            return int(self.meta.get("schema", "0"), 16) == schema.hash
        except ValueError:
            return False

    def summary(self):
        fast, slow = self.names()
        return {"session": self.meta.get("session", ""), "firmware": self.meta.get("firmware", ""), "to": self.meta.get("to", ""), "duration": round(self.duration, 3),
                "rate_hz": self.rate_hz, "real_rate_hz": round(1.0 / self.sample_dt(), 1) if self.sample_dt() > 0 else 0.0, "records": len(self.fast.get("t", ())), "slow_records": len(self.slow.get("t", ())), "fields": fast, "slow_fields": slow,
                "units": {f.name: f.unit for f in self.fast_fields + self.slow_fields if f.name != "t"}, "schema": self.meta.get("schema", ""),
                "events": [[round(t, 4), name, list(data)] for t, name, data in self.events], "gaps": self.gaps, "dropped": self.dropped,
                "complete": self.complete, "end": END_REASONS.get(self.events[-1][2][0], "") if self.complete and self.events[-1][2] else ""}

    def to_csv(self, fp, slow=False):
        fields = self.slow_fields if slow else self.fast_fields
        cols = self.slow if slow else self.fast
        fp.write(",".join(f.name + (" (%s)" % f.unit if f.unit else "") for f in fields) + "\n")
        data = [cols[f.name] for f in fields]
        for row in zip(*data):
            fp.write(",".join("%.6f" % row[0] if k == 0 else ("%g" % v) for k, v in enumerate(row)) + "\n")


class Reader:
    """Incremental reader: feed() it bytes as they arrive (a live stream) or all at once; sessions() gives what is known so far."""

    def __init__(self):
        self._parser = F.Parser()
        self._sessions = []

    def feed(self, data):
        for msg_type, seq, payload in self._parser.feed(data):
            self.frame(msg_type, seq, payload)

    def frame(self, msg_type, seq, payload):
        if msg_type == BB_HEADER:
            if seq == 0:
                self._sessions.append(Session())
            if self._sessions and not self._sessions[-1]._header_done:
                self._sessions[-1].text += payload.decode("utf-8", "replace")
        elif msg_type == BB_DATA and self._sessions:
            self._sessions[-1]._data(seq, payload)

    def sessions(self, final=False):
        """The sessions read so far. final=True: nothing more will be fed (a header cut short by the end of a file is used as it is)."""
        return [s._finish(final) for s in self._sessions]


def parse(data):
    """All sessions in a byte string (a .wblog file, a download of the log memory, a captured stream)."""
    r = Reader()
    r.feed(data)
    return r.sessions(final=True)


def load(path):
    with open(path, "rb") as fp:
        return parse(fp.read())
