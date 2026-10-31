"""The protocol schema (protocol/wbp_schema.json) at run time: layouts of settings sections, telemetry streams and commands.

The firmware header src/proto/wbp_schema.h is generated from the same file, and both sides carry its hash: the client refuses
to talk to a firmware built from a different schema instead of silently mis-reading its settings.

Values are converted to what a person would type:
    scaled integers  -> floats in engineering units ("scale" in the schema: wire * scale = value)
    enums            -> their names (unknown numbers stay numbers; an optional slot holding 255 is None)
    bit fields       -> list of the names of the set bits
    bool             -> True / False
"""
import json
import os
import struct
import zlib

_CODES = {"u8": "B", "bool": "B", "u16": "H", "u32": "I", "i16": "h", "i32": "i", "f32": "f"}
_RANGE = {"u8": (0, 255), "bool": (0, 1), "u16": (0, 65535), "u32": (0, 0xFFFFFFFF), "i16": (-32768, 32767), "i32": (-2 ** 31, 2 ** 31 - 1)}


def default_path():
    env = os.environ.get("WBP_SCHEMA")
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, "wbp_schema.json"), os.path.join(here, "..", "..", "protocol", "wbp_schema.json")):
        if os.path.exists(cand):
            return cand
    raise FileNotFoundError("wbp_schema.json not found (set WBP_SCHEMA or keep the repository layout)")


def _strip(o):
    if isinstance(o, dict):
        return {k: _strip(v) for k, v in o.items() if k not in ("desc", "hash", "compat")}
    if isinstance(o, list):
        return [_strip(v) for v in o]
    return o


class Layout:
    """A packed little-endian struct described by a list of schema fields."""

    def __init__(self, schema, fields):
        self.schema = schema
        self.fields = fields
        self.fmt = "<" + "".join(_CODES[f["t"]] * (f.get("count") or 1) for f in fields)
        self.size = struct.calcsize(self.fmt)

    def names(self):
        return [f["n"] for f in self.fields]

    def field(self, name):
        for f in self.fields:
            if f["n"] == name:
                return f
        raise KeyError("no field %r (have: %s)" % (name, ", ".join(self.names())))

    # ---- wire -> values
    def _dec(self, f, raw):
        if f["t"] == "bool":
            return bool(raw)
        if "enum" in f:
            names = self.schema.enums[f["enum"]]
            if f.get("optional") and raw == 255:
                return None
            return names[raw] if raw < len(names) else raw
        if "bits" in f:
            return self.schema.bit_names(f["bits"], raw)
        if "scale" in f:
            return round(raw * f["scale"], 7)
        if f["t"] == "f32":
            return float("%.7g" % raw)                   # a float32 has 7 significant digits: 0.05 instead of 0.05000000074505806
        return raw

    def unpack(self, data):
        if len(data) != self.size:
            raise ValueError("expected %d bytes, got %d" % (self.size, len(data)))
        vals = struct.unpack(self.fmt, data)
        out, i = {}, 0
        for f in self.fields:
            n = f.get("count")
            if n:
                out[f["n"]] = [self._dec(f, v) for v in vals[i:i + n]]
                i += n
            else:
                out[f["n"]] = self._dec(f, vals[i])
                i += 1
        return out

    # ---- values -> wire
    def _enc(self, f, v):
        t = f["t"]
        if t == "bool":
            if isinstance(v, str):
                v = v.strip().lower() in ("1", "true", "on", "yes")
            return 1 if v else 0
        if "enum" in f:
            if v is None:
                if not f.get("optional"):
                    raise ValueError("%s cannot be empty" % f["n"])
                return 255
            if isinstance(v, str) and not v.lstrip("-").isdigit():
                names = self.schema.enums[f["enum"]]
                if v.upper() not in names:
                    raise ValueError("%s: %r is not one of %s" % (f["n"], v, ", ".join(names)))
                return names.index(v.upper())
            return int(v)
        if "bits" in f and not isinstance(v, (int, float)):
            return self.schema.bit_value(f["bits"], v)
        if t == "f32":
            return float(v)
        x = float(v) / f["scale"] if "scale" in f else float(v)
        x = int(round(x))
        lo, hi = _RANGE[t]
        return min(max(x, lo), hi)

    def pack(self, values):
        flat = []
        for f in self.fields:
            if f["n"] not in values:
                raise KeyError("missing field %r" % f["n"])
            v = values[f["n"]]
            n = f.get("count")
            if n:
                if not isinstance(v, (list, tuple)) or len(v) != n:
                    raise ValueError("%s needs %d values" % (f["n"], n))
                flat += [self._enc(f, x) for x in v]
            else:
                flat.append(self._enc(f, v))
        return struct.pack(self.fmt, *flat)

    def defaults(self):
        out = {}
        for f in self.fields:
            d = f.get("d", 0)
            n = f.get("count")
            if n:
                vals = d if isinstance(d, list) else [d] * n
                out[f["n"]] = [self._dec(f, v) if "scale" not in f else v * f["scale"] for v in vals]
            else:
                out[f["n"]] = self._dec(f, d) if "scale" not in f else d * f["scale"]
        return out


class Schema:
    def __init__(self, path=None):
        self.path = path or default_path()
        with open(self.path, "r", encoding="utf-8") as fp:
            self.raw = json.load(fp)
        self.version = self.raw["version"]
        self.hash = int(self.raw["hash"], 16)
        self.enums = self.raw["enums"]
        self.bits = self.raw.get("bits", {})
        self.errors = self.raw["errors"]
        self.sections = {s["name"]: s for s in self.raw["sections"]}
        self.sections_by_id = {s["id"]: s for s in self.raw["sections"]}
        self.streams = {s["name"]: s for s in self.raw["streams"]}
        self.streams_by_id = {s["id"]: s for s in self.raw["streams"]}
        self.commands = {c["name"]: c for c in self.raw["commands"]}
        self._layouts = {}

    def computed_hash(self):
        """crc32 of the canonical json without descriptions - what tools/gen_proto.py writes into "hash" and into the firmware."""
        canon = json.dumps(_strip(self.raw), sort_keys=True, separators=(",", ":")).encode()
        return zlib.crc32(canon) & 0xFFFFFFFF

    def sections_of(self, role):
        """Names of the sections a device of this role ("RX" the flight controller, "TX" the transmitter) has, in schema order."""
        want = "tx" if str(role).upper() == "TX" else "rx"
        return [s["name"] for s in self.raw["sections"] if s.get("role", "rx") == want]

    def streams_of(self, role):
        want = "tx" if str(role).upper() == "TX" else "rx"
        return [s["name"] for s in self.raw["streams"] if s.get("role", "rx") == want]

    def section(self, name):
        if name not in self.sections:
            raise KeyError("no settings section %r (have: %s)" % (name, ", ".join(self.sections)))
        return self.sections[name]

    def stream(self, name):
        if name not in self.streams:
            raise KeyError("no stream %r (have: %s)" % (name, ", ".join(self.streams)))
        return self.streams[name]

    def command(self, name):
        name = name.upper()
        if name not in self.commands:
            raise KeyError("no command %r (have: %s)" % (name, ", ".join(self.commands)))
        return self.commands[name]

    def layout(self, kind, name):
        key = (kind, name)
        if key not in self._layouts:
            if kind == "section":
                fields = self.section(name)["fields"]
            elif kind == "stream":
                fields = self.stream(name)["fields"]
            elif kind == "args":
                fields = self.command(name)["args"]
            else:
                fields = self.command(name).get("result", [])
            self._layouts[key] = Layout(self, fields)
        return self._layouts[key]

    def error_name(self, code):
        return self.errors[code] if 0 <= code < len(self.errors) else "ERR_%d" % code

    def _bit_list(self, group):
        return self.bits[group] if group in self.bits else self.enums[group]

    def bit_names(self, group, value):
        names = self._bit_list(group)
        return [names[i] if i < len(names) else "BIT%d" % i for i in range(32) if (value >> i) & 1]

    def bit_value(self, group, names):
        table = self._bit_list(group)
        v = 0
        for n in names:
            v |= 1 << table.index(n.upper())
        return v
