"""WbClient - talk to a Windburst flight controller (or the wb_sim simulator) over the WBP protocol.

    from wbclient import WbClient
    with WbClient("COM5") as fc:                     # or "/dev/ttyACM0", or "socket://127.0.0.1:5760" for wb_sim
        print(fc.info)                               # firmware version, role, features
        pid = fc.get("pid")                          # dict, engineering units
        fc.set("pid", p=[0.05, 0.05, 0.012])         # returns what the firmware actually stored (after clamping)
        fc.save()
        fc.subscribe("attitude", 50, lambda v: print(v["roll"], v["pitch"]))
        fc.command("MOTOR_TEST", motor=0, value=100) # raises WbError("ARMED") etc. when refused

One reader thread decodes everything that arrives: replies are matched to their request by sequence number, telemetry
frames go to latest() / callbacks, firmware log lines to .logs / on_log.
"""
import collections
import struct
import threading
import time

from . import framing as F
from .schema import Schema


class WbError(Exception):
    """The firmware refused a request. .name is the error name from the schema (ARMED, BAD_ARG, NO_SENSOR ...)."""

    def __init__(self, name, what="", result=None):
        super().__init__("%s%s" % (name, (" (%s)" % what) if what else ""))
        self.name = name
        self.result = result or {}


class WbTimeout(WbError):
    def __init__(self, what):
        super().__init__("TIMEOUT", what)


class SchemaMismatch(Exception):
    pass


class DeviceInfo:
    ROLES = {0: "RX", 1: "TX"}
    FEATURES = ["STREAMS", "CALIBRATION", "MOTOR_TEST", "GPS", "BLACKBOX", "BATTERY", "BEEPER", "STM32", "WIFI", "USB_NAME"]   # STM32: pins are PA0 .. PK15 (port * 16 + pin); WIFI: the board has a radio (section wifi); USB_NAME: general.usb_name names the board on USB

    def __init__(self, payload):
        self.protocol, role, maj, mnr, pat, self.schema_hash, feat = struct.unpack_from("<BBBBBII", payload)
        self.role = self.ROLES.get(role, str(role))
        self.version = "%d.%d.%d" % (maj, mnr, pat)
        self.features = [n for i, n in enumerate(self.FEATURES) if (feat >> i) & 1]
        # [name] and, from 0.25 on, [0][mcu][0][32-byte pin mask]: which chip, which pin numbers exist on it
        rest = payload[13:]
        self.mcu, self.pins = None, None
        if b"\0" in rest:
            name, rest = rest.split(b"\0", 1)
            if b"\0" in rest:
                mcu, mask = rest.split(b"\0", 1)
                self.mcu = mcu.decode("utf-8", "replace")
                if len(mask) >= 32:
                    self.pins = [n for n in range(256) if (mask[n >> 3] >> (n & 7)) & 1]
            rest = name
        self.name = rest.decode("utf-8", "replace")

    def __repr__(self):
        return "%s v%s (%s%s, protocol %d, schema 0x%08X, %s)" % (self.name, self.version, self.role, ", " + self.mcu if self.mcu else "", self.protocol, self.schema_hash, " ".join(self.features))


class WbClient:
    def __init__(self, port, baud=230400, schema=None, timeout=1.0, check_schema=True):
        self.port_name = port
        self.baud = baud
        self.schema = schema or Schema()
        self.timeout = timeout
        self.check_schema = check_schema
        self.info = None
        self.logs = collections.deque(maxlen=500)          # (level, text)
        self.on_log = None
        self.on_disconnect = None                           # called (from the reader thread) when the port itself fails: cable pulled, port closed by the OS
        self.on_raw = None                                  # called with the bytes that were NOT protocol frames (boot text, crash dump, wrong baud rate)
        self.raw_tail = b""                                 # the last such bytes (up to 4 kB): what to show when the firmware does not answer
        self.on_blackbox = None                             # called with (type, seq, payload) for every blackbox log frame the firmware streams
        self._blob_sink = None                              # set while a download runs: gets the payload of each BB_BLOB frame
        self.lost = False
        self.keepalive = 0.3                                # s; when nothing else was sent for this long a HELLO goes out, so the firmware knows the app is
                                                            # still there (bench silence of the beeper, no lost-model beacon on the desk). 0 = off
        self._last_tx = 0.0
        self.stats = {"frames": 0, "errors": 0, "streams": 0, "unmatched": 0}
        self._ser = None
        self._parser = F.Parser()
        self._seq = 0
        self._lock = threading.Lock()                       # one request at a time
        self._rmw = threading.Lock()                        # one read-modify-write of a settings section at a time (set())
        self._pending = {}                                  # seq -> [event, reply]
        self._latest = {}                                   # stream name -> (t_ms, values)
        self._callbacks = {}
        self._stream_events = {}
        self._reader = None
        self._stop = False

    # ------------------------------------------------------------------ connection
    def open(self, hello=True):
        import serial                                        # pyserial; "socket://host:port" URLs work too (the simulator)
        ser = serial.serial_for_url(self.port_name, do_not_open=True)
        ser.baudrate = self.baud
        ser.timeout = 0.02
        try:                                                 # keep DTR/RTS low: on ESP32 dev boards opening the port would reset the flight controller
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass
        try:                                                 # one program per port, on Linux / macOS too (Windows always works this way):
            ser.exclusive = True                             # a second configurator or a serial monitor on the same port would eat half of the answers
        except Exception:
            pass
        ser.open()
        self._ser = ser
        self._stop = False
        self.lost = False
        self._reader = threading.Thread(target=self._read_loop, name="wbclient-reader", daemon=True)
        self._reader.start()
        if hello:
            self.hello()
        return self

    def close(self):
        self._stop = True
        if self._reader:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self._ser:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        try:
            if self._ser:
                self.unsubscribe()
        except Exception:
            pass
        self.close()

    # ------------------------------------------------------------------ reader thread
    def _read_loop(self):
        quiet_since = None
        while not self._stop:
            try:
                data = self._ser.read(4096)
            except Exception:
                if not self._stop:                           # not our own close(): the port died under us
                    self.lost = True
                    if self.on_disconnect:
                        try:
                            self.on_disconnect()
                        except Exception:
                            pass
                break
            if data:
                quiet_since = None
                frames = self._parser.feed(data)
            else:
                now = time.monotonic()
                if quiet_since is None:
                    quiet_since = now
                frames = self._parser.idle() if (now - quiet_since > 0.02 and self._parser.mid_frame) else []
            noise = self._parser.take_noise()
            if noise:
                self.raw_tail = (self.raw_tail + noise)[-4096:]
                if self.on_raw:
                    try:
                        self.on_raw(noise)
                    except Exception:
                        pass
            for msg_type, seq, payload in frames:
                try:
                    self._dispatch(msg_type, seq, payload)
                except Exception:                            # a malformed frame must never kill the reader
                    self.stats["errors"] += 1
            self.stats["frames"] = self._parser.frames
            self.stats["errors"] = max(self.stats["errors"], self._parser.errors)
            self._send_keepalive()

    def _send_keepalive(self):
        if not self.keepalive or self.info is None or self.lost:
            return
        now = time.monotonic()
        if now - self._last_tx < self.keepalive or not self._lock.acquire(blocking=False):      # a request is on its way: that is traffic enough
            return
        try:
            self._ser.write(F.encode(F.HELLO, 0, b""))     # sequence 0 is never used by requests: the answer is dropped in _dispatch
            self._last_tx = now
        except Exception:                                # a dead port is noticed by the read above
            pass
        finally:
            self._lock.release()

    def _dispatch(self, msg_type, seq, payload):
        if msg_type == F.STREAM:
            if len(payload) < 5:
                return
            sd = self.schema.streams_by_id.get(payload[0])
            if not sd:
                return
            layout = self.schema.layout("stream", sd["name"])
            if len(payload) != layout.size + 5:
                return
            t_ms = struct.unpack_from("<I", payload, 1)[0]
            values = layout.unpack(payload[5:])
            values["t_ms"] = t_ms
            self._latest[sd["name"]] = values
            self.stats["streams"] += 1
            ev = self._stream_events.get(sd["name"])
            if ev:
                ev.set()
            cb = self._callbacks.get(sd["name"])
            if cb:
                cb(values)
        elif msg_type == F.LOG:
            if payload:
                entry = (payload[0], payload[1:].decode("utf-8", "replace"))
                self.logs.append(entry)
                if self.on_log:
                    self.on_log(*entry)
        elif msg_type in (F.BB_HEADER, F.BB_DATA):               # blackbox frames number themselves: never a reply to a request
            if self.on_blackbox:
                self.on_blackbox(msg_type, seq, payload)
        elif msg_type == F.BB_BLOB:
            if self._blob_sink:
                self._blob_sink(payload)
        elif seq == 0:
            pass                                             # the answer to a keepalive
        else:
            slot = self._pending.get(seq)
            if slot and slot[1] is None:
                slot[1] = (msg_type, payload)
                slot[0].set()
            else:
                self.stats["unmatched"] += 1

    # ------------------------------------------------------------------ request / reply
    def _request(self, msg_type, payload=b"", timeout=None, retries=0, what=""):
        if not self._ser or self.lost:
            raise WbError("NOT_CONNECTED", what)
        timeout = self.timeout if timeout is None else timeout
        with self._lock:
            for _attempt in range(retries + 1):
                self._seq = self._seq % 255 + 1              # 1..255
                seq = self._seq
                slot = [threading.Event(), None]
                self._pending[seq] = slot
                try:
                    try:
                        self._ser.write(F.encode(msg_type, seq, payload))
                        self._last_tx = time.monotonic()
                    except Exception as e:                   # serial.SerialException / OSError: the port is gone
                        self.lost = True
                        raise WbError("NOT_CONNECTED", "%s: %s" % (what, e))
                    if slot[0].wait(timeout):
                        return slot[1]
                finally:
                    self._pending.pop(seq, None)
            raise WbTimeout(what or "request 0x%02X" % msg_type)

    def _expect_ack(self, reply, what):
        msg_type, payload = reply
        if msg_type != F.ACK or len(payload) != 3:
            raise WbError("PROTOCOL", "%s: unexpected reply 0x%02X" % (what, msg_type))
        if payload[2] != 0:
            raise WbError(self.schema.error_name(payload[2]), what)

    # ------------------------------------------------------------------ API
    def hello(self):
        msg_type, payload = self._request(F.HELLO, retries=3, what="hello")
        if msg_type != F.DEVICE_INFO or len(payload) < 13:
            raise WbError("PROTOCOL", "hello: unexpected reply")
        info = DeviceInfo(payload)
        self.info = info
        if self.check_schema:
            if info.protocol != self.schema.version:
                raise SchemaMismatch("firmware speaks protocol %d, this client %d" % (info.protocol, self.schema.version))
            if info.schema_hash != self.schema.hash:
                raise SchemaMismatch("firmware schema 0x%08X != client schema 0x%08X: flash the firmware built from this repository (or use its wbp_schema.json)"
                                     % (info.schema_hash, self.schema.hash))
        return info

    def get(self, section):
        sd = self.schema.section(section)
        msg_type, payload = self._request(F.SECTION_GET, bytes([sd["id"]]), retries=2, what="get " + section)
        return self._section_reply(sd, msg_type, payload, "get " + section)

    def _section_reply(self, sd, msg_type, payload, what):
        if msg_type == F.ACK:
            self._expect_ack((msg_type, payload), what)
        layout = self.schema.layout("section", sd["name"])
        if msg_type != F.SECTION or len(payload) != layout.size + 3 or payload[0] != sd["id"]:
            raise WbError("PROTOCOL", what + ": malformed section reply")
        return layout.unpack(payload[3:])

    def set(self, section, values=None, **changes):
        """Read-modify-write of one settings section. Returns the values the firmware stored (it clamps out-of-range input).
        The change lives in RAM until save(); status stream field settings_dirty tells if a reboot is needed."""
        sd = self.schema.section(section)
        layout = self.schema.layout("section", section)
        upd = dict(values or {})
        upd.update(changes)
        for k in upd:
            layout.field(k)                                  # KeyError with the list of valid names
        with self._rmw:                                      # read + write as ONE step: two threads changing two fields of a section must both win
            cur = self.get(section)
            cur.update(upd)
            msg_type, payload = self._request(F.SECTION_SET, bytes([sd["id"]]) + layout.pack(cur), retries=1, what="set " + section)
            return self._section_reply(sd, msg_type, payload, "set " + section)

    def get_all(self):
        return {name: self.get(name) for name in self.schema.sections}

    def save(self):
        self._expect_ack(self._request(F.SETTINGS_SAVE, timeout=max(self.timeout, 3.0), what="save"), "save")

    def defaults(self, section=None):
        sid = 0xFF if section is None else self.schema.section(section)["id"]
        self._expect_ack(self._request(F.SETTINGS_DEFAULTS, bytes([sid]), what="defaults"), "defaults")

    def reboot(self, wait=True, timeout=10.0):
        """Restart the flight controller. With wait=True returns once it answers again."""
        self._expect_ack(self._request(F.REBOOT, what="reboot"), "reboot")
        if not wait:
            return None
        time.sleep(0.5)
        end = time.monotonic() + timeout
        while True:
            try:
                return self.hello()
            except WbTimeout:
                if time.monotonic() > end:
                    raise

    def subscribe(self, stream, hz=None, callback=None):
        sd = self.schema.stream(stream)
        hz = sd["rate"] if hz is None else int(hz)
        if callback:
            self._callbacks[stream] = callback
        self._stream_events.setdefault(stream, threading.Event())
        self._expect_ack(self._request(F.STREAM_CFG, struct.pack("<BH", sd["id"], hz), retries=2, what="subscribe " + stream), "subscribe " + stream)

    def unsubscribe(self, stream=None):
        if stream is None:
            self._callbacks.clear()
            self._expect_ack(self._request(F.STREAM_CFG, struct.pack("<BH", 0xFF, 0), retries=2, what="unsubscribe"), "unsubscribe")
        else:
            self._callbacks.pop(stream, None)
            self._expect_ack(self._request(F.STREAM_CFG, struct.pack("<BH", self.schema.stream(stream)["id"], 0), retries=2, what="unsubscribe"), "unsubscribe")

    def latest(self, stream):
        """The most recent frame of a subscribed stream (dict with engineering values + t_ms), or None."""
        self.schema.stream(stream)
        return self._latest.get(stream)

    def wait_stream(self, stream, timeout=2.0):
        """Block until the NEXT frame of this stream arrives and return it."""
        ev = self._stream_events.setdefault(stream, threading.Event())
        ev.clear()
        if not ev.wait(timeout):
            raise WbTimeout("no '%s' frame within %.1f s (subscribed?)" % (stream, timeout))
        return self._latest[stream]

    def command(self, name, timeout=None, **args):
        """Run a firmware command. Returns its result fields (usually {}). Raises WbError(<error name>) when refused;
        the exception's .result holds the result fields too (e.g. the reason of a failed compass calibration)."""
        cd = self.schema.command(name)
        payload = bytes([cd["id"]]) + self.schema.layout("args", cd["name"]).pack(args)
        msg_type, reply = self._request(F.COMMAND, payload, timeout=timeout, what=cd["name"])
        if msg_type == F.ACK:
            self._expect_ack((msg_type, reply), cd["name"])
        if msg_type != F.CMD_RESULT or len(reply) < 2 or reply[0] != cd["id"]:
            raise WbError("PROTOCOL", cd["name"] + ": malformed result")
        rl = self.schema.layout("result", cd["name"])
        result = rl.unpack(reply[2:2 + rl.size]) if rl.size and len(reply) >= 2 + rl.size else {}
        if reply[1] != 0:
            raise WbError(self.schema.error_name(reply[1]), cd["name"], result)
        return result

    # ------------------------------------------------------------------ blackbox (flight log)
    def blackbox_status(self, timeout=2.0):
        """State of the flight log: {state, device, capacity_kb, used_kb, capacity, used, sessions, rate_hz, record_bytes, dropped, erase_pct, byte_rate, buffer_pct}.
        capacity / used are bytes and saturate at 4 GB (an SD card): use the _kb fields for sizes; `used` is the end of the download address range."""
        if "blackbox" not in self._stream_events:
            self.subscribe("blackbox", 5)
        return self.wait_stream("blackbox", timeout)

    def blackbox_claim(self):
        """Before a download: a session that is being written on the ground (trigger ALWAYS, the BLACKBOX switch) is closed, so that
        the list gives its final size. (Since 0.8.8 a mere look at the list leaves it running; a READ closes it - in the air it
        stays BUSY and the download is refused.)"""
        try:
            if self.blackbox_status().get("state") != "LOGGING":
                return
        except WbError:
            return
        busy_until = time.monotonic() + 3.0
        while True:
            try:
                self.command("BLACKBOX_READ", offset=0, length=1)
                break
            except WbError as e:
                if e.name == "ARMED":                                # a flight is being logged: it is not cut short
                    raise WbError("BUSY", "a flight is being logged") from e
                if e.name != "BUSY" or time.monotonic() >= busy_until:
                    raise
                time.sleep(0.1)
        try:
            self.command("BLACKBOX_READ", offset=0, length=0)       # (the one byte is not wanted)
        except WbError:
            pass

    def blackbox_sessions(self, final=False):
        """[(start, size)] of the sessions stored in the flight controller's log memory, oldest first. final: close a session that is
        being written first (blackbox_claim), so that its size does not change any more - for a download."""
        if final:
            self.blackbox_claim()
        out = []
        busy_until = None
        while True:
            try:
                r = self.command("BLACKBOX_SESSION", index=len(out))
            except WbError as e:
                if e.name == "BAD_ARG":
                    return out
                # BUSY on the ground: a session was being written (trigger ALWAYS / the switch) and the firmware is closing it
                # for us - a few milliseconds. In the air, or during an erase, it stays BUSY and that is the answer.
                if e.name == "BUSY":
                    busy_until = busy_until or time.monotonic() + 3.0
                    if time.monotonic() < busy_until:
                        time.sleep(0.1)
                        continue
                raise
            out.append((r["start"], r["size"]))

    def blackbox_read(self, offset, length, progress=None, stall_timeout=1.5, retries=6):
        """`length` bytes of the log memory from `offset`. The firmware sends them as a run of frames; a frame lost on the way
        is asked for again from the first missing byte. progress(done, total) is called as data arrives.
        retries: how often IN A ROW the firmware may be asked again without a single new byte arriving - a long download over
        a link that loses a frame now and then is not given up as long as it moves."""
        buf = bytearray(length)
        have = [0]
        ev = threading.Event()

        def sink(payload):
            if len(payload) <= 4:
                return
            off = struct.unpack_from("<I", payload)[0]
            data = payload[4:]
            if off == offset + have[0] and have[0] + len(data) <= length:      # the next piece in order; anything else is a repeat or comes after a hole
                buf[have[0]:have[0] + len(data)] = data
                have[0] += len(data)
                ev.set()

        self._blob_sink = sink
        try:
            stalls = 0
            busy_until = None
            while have[0] < length and stalls <= retries:
                before = have[0]
                try:
                    self.command("BLACKBOX_READ", offset=offset + have[0], length=length - have[0])
                except WbError as e:
                    if e.name != "BUSY":
                        raise
                    busy_until = busy_until or time.monotonic() + 3.0   # (see blackbox_sessions)
                    if time.monotonic() >= busy_until:
                        raise
                    time.sleep(0.1)
                    continue
                while have[0] < length:
                    ev.clear()
                    if not ev.wait(stall_timeout):
                        break                                    # nothing for a while: a frame was lost, ask again from here
                    if progress:
                        progress(have[0], length)
                stalls = 0 if have[0] > before else stalls + 1
            if have[0] < length:
                raise WbTimeout("log download stopped at %d of %d bytes" % (have[0], length))
            return bytes(buf)
        finally:
            self._blob_sink = None
            if have[0] < length:
                try:
                    self.command("BLACKBOX_READ", offset=0, length=0)       # cancel what may still be on its way
                except WbError:
                    pass

    def blackbox_download(self, index=None, progress=None):
        """One stored session (by its position in blackbox_sessions()) as .wblog bytes, or all of them when index is None."""
        sessions = self.blackbox_sessions(final=True)
        if not sessions:
            return b""
        if index is None:
            start, size = 0, sessions[-1][0] + sessions[-1][1]
        else:
            start, size = sessions[index]
        return self.blackbox_read(start, size, progress)

    def blackbox_erase(self, wait=True, timeout=180.0, progress=None):
        """Erase the whole log memory. Takes seconds to minutes; the craft cannot be armed meanwhile."""
        self.command("BLACKBOX_ERASE")
        if not wait:
            return
        self.subscribe("blackbox", 5)
        end = time.monotonic() + timeout
        while True:
            try:
                st = self.wait_stream("blackbox", 5.0)           # the firmware is busy with the flash: frames come in bursts
            except WbTimeout:
                st = None
            if st and st["state"] != "ERASING":
                return st
            if st and progress:
                progress(st["erase_pct"])
            if time.monotonic() > end:
                raise WbTimeout("erase did not finish in %.0f s" % timeout)

    def blackbox_stream(self, enable=True, rate_hz=0):
        """Ask the firmware to send the log to this client (frames go to on_blackbox). rate_hz 0 = the configured rate."""
        self.command("BLACKBOX_STREAM", enable=bool(enable), rate_hz=int(rate_hz))

    def blackbox_record(self, seconds, rate_hz=0):
        """Record a live stream for `seconds`; returns .wblog bytes (the frames exactly as the firmware sent them)."""
        out = bytearray()
        self.on_blackbox = lambda t, q, p: out.extend(F.encode(t, q, p))
        try:
            self.blackbox_stream(True, rate_hz)
            time.sleep(seconds)
            self.blackbox_stream(False)
            time.sleep(0.4)                                      # the tail and the END marker
        finally:
            self.on_blackbox = None
        return bytes(out)

    def report(self, wait=0.3):
        """The boot report (detected sensors / failure reasons) as a list of text lines."""
        mark = len(self.logs)
        self.command("REPORT")
        time.sleep(wait)
        return [text for _lvl, text in list(self.logs)[mark:]]
