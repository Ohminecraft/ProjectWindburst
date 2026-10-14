"""Windburst configurator GUI:  python -m wbclient gui [--port COM5]

A small web server on 127.0.0.1 that sits between the browser and the flight controller:

    browser (wbclient/web/*)  <-- HTTP + server-sent events -->  this file  <-- WbClient / WBP -->  USB serial or wb_sim

The browser never sees the binary protocol: settings and telemetry arrive as JSON in engineering units (the same values the
command line prints). Only the Python standard library and pyserial are used.

Safety of the local server (it can spin motors, so it must not obey any web page that happens to be open):
    - it listens on 127.0.0.1 only;
    - every /api request must carry the random token that is embedded in the page it served (other sites cannot read it);
    - the Host / Origin headers must name this server (stops DNS-rebinding and cross-site requests).

Language: nothing here is written for the user's eyes. Errors and notes travel as a CODE plus values ({"error": "OPEN_FAILED",
"port": ..., "detail": ...}); the page turns them into Vietnamese or English. "message" is a plain English line for scripts.
"""
import base64
import binascii
import collections
import hmac
import io
import json
import mimetypes
import os
import secrets
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from . import framing as F
from . import logview, wblog
from .client import SchemaMismatch, WbClient, WbError, WbTimeout
from .schema import Schema

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
SIM_URL = "socket://127.0.0.1:5760"


# USB vendors of the serial chips found on ESP32 boards: such a port is listed first and marked as the likely flight controller.
LIKELY_VIDS = {0x303A: "Espressif USB", 0x10C4: "CP210x", 0x1A86: "CH34x / CH910x", 0x0403: "FTDI"}
BAUDS = (115200, 230400, 460800, 921600)


class ApiError(Exception):
    def __init__(self, status, code, message="", **extra):
        super().__init__(message or code)
        self.status, self.code, self.message, self.extra = status, code, message or code, extra


def printable(data):
    """Bytes from the port as text a person can read: printable ASCII and newlines kept, everything else a dot."""
    return "".join(chr(b) if 32 <= b < 127 or b == 10 else ("" if b == 13 else ".") for b in data)


class Hub:
    """One flight controller connection shared by every browser tab: requests, wanted stream rates, and the event feed."""

    FAST = "gyro_fast"                      # the only stream whose every sample is forwarded (vibration graph); the others send their latest value

    def __init__(self, schema=None, log_dir=None):
        self.schema = schema or Schema()
        self.library = logview.Library(log_dir)             # the .wblog files on this computer
        self.rec = None                     # logview.Recording: the live blackbox stream being recorded (kept after it stopped, until the next one)
        self.job = None                     # the log download in progress or just finished: {"kind", "state": running | done | failed, "done", "total", "file", "error"}
        self.aux_seq = 1                    # bumped when job / recording state changes
        self.fc = None
        self.port = None
        self.baud = 230400
        self.report = []
        self.note = None                    # why the link is down / what just happened: {"code": "PORT_LOST" | "NOT_RESPONDING" | "REBOOTED" | "STALLED", ...}
        self.responding = True
        self.boots = 0                      # counts firmware restarts seen on this connection: the UI reloads its settings when it changes
        self.rates = {}                     # stream -> Hz wanted by the UI
        self.lock = threading.RLock()       # connect / disconnect / subscriptions / reboot
        self._data = threading.Lock()
        self.seq = 0
        self.latest = {}                    # stream -> (seq, values)
        self.fast = collections.deque(maxlen=4000)      # (seq, [t_ms, raw x3, filtered x3])
        self.logs = collections.deque(maxlen=1000)      # (seq, level, text)
        self.raw = collections.deque(maxlen=400)        # (seq, text): what the port sent outside the protocol (boot text, crash dump)
        self.conn_seq = 1
        self._last_frame = time.monotonic()
        self._busy = False
        self._stop = False
        # the link stream (1 Hz, always on while connected): the firmware's uptime tells a restart from a link that only paused
        self.link = None                    # its latest values
        self._boot_id = None
        self._link_ev = threading.Event()
        self._link_restarted = False
        self.stalls = 0                     # pauses of the streams seen on this connection that were NOT a restart
        threading.Thread(target=self._watchdog, name="wbgui-watchdog", daemon=True).start()

    # ------------------------------------------------------------------ state
    def state(self):
        fc = self.fc
        info = fc.info if fc else None
        return {
            "connected": fc is not None,
            "responding": bool(fc) and self.responding,
            "port": self.port,
            "baud": self.baud,
            "note": self.note,
            "boots": self.boots,
            "stalls": self.stalls,
            "link": self.link,
            "info": None if not info else {"name": info.name, "version": info.version, "role": info.role, "protocol": info.protocol,
                                          "schema_hash": "0x%08X" % info.schema_hash, "features": info.features},
            "report": list(self.report),
            "rates": dict(self.rates),
        }

    def _changed(self):
        with self._data:
            self.conn_seq += 1

    def _need(self):
        fc = self.fc
        if fc is None:
            raise ApiError(409, "NOT_CONNECTED", "not connected to a flight controller")
        return fc

    # ------------------------------------------------------------------ callbacks from the client's reader thread
    def _on_stream(self, name, values):
        with self._data:
            self.seq += 1
            self.latest[name] = (self.seq, values)
            if name == self.FAST:
                self.fast.append((self.seq, [values["t_ms"]] + list(values["raw"]) + list(values["filt"])))
        self._last_frame = time.monotonic()

    def _on_link(self, values):
        self._on_stream("link", values)      # (the page reads it like any other stream: store.streams.link)
        with self._data:
            self.link = dict(values)
            before, now = self._boot_id, values.get("boot_id")
            self._boot_id = now
        if before is not None and now is not None and before != now:     # a new start since the last frame
            self._link_restarted = True
        self._link_ev.set()

    def _log_gui(self, text, level=2):
        with self._data:
            self.seq += 1
            self.logs.append((self.seq, level, "app: " + text))

    def _on_log(self, level, text):
        with self._data:
            self.seq += 1
            self.logs.append((self.seq, level, text))

    def _on_raw(self, data):
        text = printable(data)
        if text:
            with self._data:
                self.seq += 1
                self.raw.append((self.seq, text))

    def _on_lost(self):
        # the serial port itself failed; tidy up from another thread (close() joins the reader thread, which is the caller here)
        threading.Thread(target=self._drop, args=({"code": "PORT_LOST", "port": self.port},), daemon=True).start()

    def _drop(self, note):
        with self.lock:
            self._end_recording()
            fc, self.fc = self.fc, None
            if fc:
                try:
                    fc.close()
                except Exception:
                    pass
            self.note = note
            with self._data:
                self.latest.clear()
                self.fast.clear()
        self._changed()

    # ------------------------------------------------------------------ connection
    def ports(self):
        """Serial ports of this computer, the likely flight controller first, and the simulator when it is running on its default port."""
        out = []
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                desc = p.description if p.description and p.description != "n/a" else ""
                out.append({"port": p.device, "description": desc, "manufacturer": p.manufacturer or "",
                            "vid": p.vid, "pid": p.pid, "chip": LIKELY_VIDS.get(p.vid, ""), "likely": p.vid in LIKELY_VIDS, "sim": False})
        except Exception:
            pass
        out.sort(key=lambda x: (not x["likely"], x["port"]))
        try:
            s = socket.create_connection(("127.0.0.1", 5760), timeout=0.15)
            s.close()
            out.append({"port": SIM_URL, "description": "wb_sim 127.0.0.1:5760", "manufacturer": "", "vid": None, "pid": None, "chip": "", "likely": False, "sim": True})
        except OSError:
            pass
        return out

    def connect(self, port, baud=230400):
        if not port or not isinstance(port, str):
            raise ApiError(400, "BAD_REQUEST", "no port given")
        try:
            baud = int(baud or 230400)
        except (TypeError, ValueError):
            raise ApiError(400, "BAD_REQUEST", "baud must be a number")
        if not 1200 <= baud <= 3000000:
            raise ApiError(400, "BAD_REQUEST", "baud out of range")
        with self.lock:
            self.disconnect()
            with self._data:
                self.raw.clear()
            fc = WbClient(port, baud, schema=self.schema, timeout=1.0)
            fc.on_log = self._on_log
            fc.on_disconnect = self._on_lost
            fc.on_raw = self._on_raw
            try:
                fc.open(hello=False)
            except Exception as e:           # serial.SerialException, OSError ...: no such port, or another program holds it
                raise ApiError(409, "OPEN_FAILED", "cannot open %s: %s" % (port, e), port=port, detail=str(e))
            try:
                fc.hello()
            except SchemaMismatch as e:
                fc.close()
                raise ApiError(409, "SCHEMA_MISMATCH", str(e), port=port, detail=str(e))
            except WbError as e:
                heard = printable(fc.raw_tail)[-1500:]          # what DID come out of the port: boot text, a crash loop, or noise at the wrong baud rate
                fc.close()
                raise ApiError(504, "NO_ANSWER", "%s opened at %d baud, but no Windburst firmware answered (%s)" % (port, baud, e.name),
                               port=port, baud=baud, detail=e.name, raw=heard)
            self.fc, self.port, self.baud, self.note, self.responding = fc, port, baud, None, True
            self.boots = 0
            self._last_frame = time.monotonic()
            self._after_boot()
        self._changed()
        return self.state()

    def disconnect(self, note=None):
        with self.lock:
            if self.fc and self.rec and self.rec.stopped is None:
                try:
                    self.fc.blackbox_stream(False)
                except Exception:
                    pass
            self._end_recording()
            fc, self.fc = self.fc, None
            if fc:
                try:
                    fc.unsubscribe()
                except Exception:
                    pass
                try:
                    fc.close()
                except Exception:
                    pass
            self.note = note
            self.report = []
            with self._data:
                self.latest.clear()
                self.fast.clear()
        self._changed()
        return self.state()

    def _after_boot(self):
        """(Re)establish what a fresh firmware does not remember: the stream subscriptions; and fetch its boot report."""
        fc = self.fc
        if not fc:
            return
        self._end_recording()               # a firmware that restarted has forgotten the stream it was sending
        self._link_restarted = False
        with self._data:
            self.link = None
            self._boot_id = None
        self._subscribe_link(fc)
        for name, hz in list(self.rates.items()):
            if hz > 0:
                try:
                    fc.subscribe(name, hz, callback=(lambda v, n=name: self._on_stream(n, v)))
                except WbError:
                    pass
        try:
            self.report = fc.report()
        except WbError:
            self.report = []

    def _subscribe_link(self, fc):
        try:
            fc.subscribe("link", 1, callback=self._on_link)
            return True
        except (WbError, KeyError):         # (KeyError: a schema without the link stream)
            return False

    def _resubscribe(self, fc):
        """Streams again after a pause that was not a restart: the firmware still has them, asking again costs nothing."""
        for name, hz in list(self.rates.items()):
            if hz > 0:
                try:
                    fc.subscribe(name, hz, callback=(lambda v, n=name: self._on_stream(n, v)))
                except WbError:
                    pass

    # ------------------------------------------------------------------ streams
    def set_rates(self, rates):
        """Make the firmware send exactly these streams ({name: Hz}); every other stream is switched off."""
        for name, hz in rates.items():
            self.schema.stream(name)
            if not isinstance(hz, (int, float)) or hz < 0:
                raise ApiError(400, "BAD_REQUEST", "bad rate for stream %s" % name)
        with self.lock:
            fc = self._need()
            want = {n: int(min(hz, self.schema.stream(n)["max"])) for n, hz in rates.items() if hz > 0}
            for name in sorted(set(self.rates) | set(want)):
                old, new = self.rates.get(name, 0), want.get(name, 0)
                if old == new:
                    continue
                if new > 0:
                    fc.subscribe(name, new, callback=(lambda v, n=name: self._on_stream(n, v)))
                else:
                    fc.unsubscribe(name)
                    with self._data:
                        self.latest.pop(name, None)
            self.rates = want
            self._last_frame = time.monotonic()
        return dict(self.rates)

    def _watchdog(self):
        """A flight controller that restarts (reset button, brown-out, port opened by another tool) forgets its subscriptions and
        simply goes silent. Notice the silence, ask who is there, and subscribe again."""
        while not self._stop:
            time.sleep(0.5)
            fc = self.fc
            if fc is None or self._busy or not any(hz > 0 for hz in self.rates.values()):
                continue
            if time.monotonic() - self._last_frame < 1.5:
                if not self.responding:
                    self.responding = True
                    self._changed()
                continue
            if not self.lock.acquire(timeout=0.1):
                continue
            try:
                if self.fc is not fc or self._busy:
                    continue
                silent = time.monotonic() - self._last_frame
                try:
                    fc.hello()
                except (WbError, SchemaMismatch):
                    if self.responding:
                        self.responding = False
                        self.note = {"code": "NOT_RESPONDING"}
                        self._log_gui("the firmware does not answer (no frame for %.1f s, no answer to HELLO)" % silent)
                        self._changed()
                    continue
                # It answers. Restarted, or did only the streams stop? (Before 0.8.8 every such pause was called a restart, and the
                # boot report read again showed the reset reason of the LAST real start - often "software restart", from a Save.)
                # The link stream carries a number drawn at every start (boot_id): the same one = no restart. (The uptime alone would
                # not do: a firmware that restarted 6 s after it started has been up for longer than it was before.)
                before = self.link or {}
                self._link_ev.clear()
                self._link_restarted = False
                now = None
                if self._subscribe_link(fc) and self._link_ev.wait(2.5):
                    now = self.link or {}
                if now and before.get("boot_id") is not None and now.get("boot_id") == before["boot_id"] and not self._link_restarted:
                    up = now["uptime"]
                    self.stalls += 1
                    lk = self.link or {}
                    self.note = {"code": "STALLED", "seconds": round(silent, 1), "uptime": up}
                    self._log_gui("the streams stopped for %.1f s but the firmware did NOT restart (up %.0f s). link: busy %s, lost %s, dropped %s, "
                                  "kicks %s, longest request %s ms, link pause %s ms, sensor pause %s ms" % (
                                      silent, up, lk.get("busy"), lk.get("tx_lost"), lk.get("dropped"), lk.get("kicks"),
                                      lk.get("cmd_max_ms"), lk.get("link_gap_ms"), lk.get("slow_gap_ms")))
                    self.responding = True
                    self._resubscribe(fc)
                else:
                    self.boots += 1
                    self.note = {"code": "REBOOTED"}
                    self.responding = True
                    self._after_boot()
                self._last_frame = time.monotonic()
                self._changed()
            finally:
                self.lock.release()

    # ------------------------------------------------------------------ settings / commands
    def settings(self):
        return self._need().get_all()

    def set(self, section, changes):
        fc = self._need()
        if not isinstance(changes, dict) or not changes:
            raise ApiError(400, "BAD_REQUEST", "no changes given")
        try:
            return fc.set(section, changes)
        except (KeyError, ValueError, TypeError) as e:
            raise ApiError(400, "BAD_VALUE", str(e).strip("'\""), detail=str(e).strip("'\""))

    def save(self):
        self._need().save()

    def defaults(self, section=None):
        try:
            self._need().defaults(section)
        except KeyError as e:
            raise ApiError(400, "BAD_VALUE", str(e).strip("'\""), detail=str(e).strip("'\""))

    def reboot(self):
        with self.lock:
            fc = self._need()
            if self.job and self.job["state"] == "running":
                raise ApiError(409, "BUSY", "a log download is running")
            self._busy = True
            try:
                fc.reboot(wait=True, timeout=10.0)
                self.boots += 1
                self.note = None
                self.responding = True
                self._after_boot()
                self._last_frame = time.monotonic()
            finally:
                self._busy = False
        self._changed()
        return self.state()

    def command(self, name, args):
        fc = self._need()
        try:
            cd = self.schema.command(name)
        except KeyError:
            raise ApiError(400, "BAD_VALUE", "no command %r" % name, detail="no command %r" % name)
        try:
            return fc.command(cd["name"], **(args or {}))
        except (KeyError, ValueError, TypeError) as e:
            raise ApiError(400, "BAD_VALUE", str(e).strip("'\""), detail=str(e).strip("'\""))

    def refresh_report(self):
        self.report = self._need().report()
        return list(self.report)

    def backup(self):
        fc = self._need()
        return {"windburst_backup": 1, "firmware": str(fc.info), "schema": self.schema.raw["hash"], "settings": fc.get_all()}

    def restore(self, doc):
        """Write the sections of a backup into RAM. Fields this firmware does not know are skipped and reported; nothing is saved to flash."""
        fc = self._need()
        settings = doc.get("settings") if isinstance(doc, dict) and "settings" in doc else doc
        if not isinstance(settings, dict):
            raise ApiError(400, "NOT_A_BACKUP", "the file is not a Windburst settings backup")
        done, skipped, changed = [], [], []
        for section, values in settings.items():
            if section not in self.schema.sections or not isinstance(values, dict):
                skipped.append(section)
                continue
            names = self.schema.layout("section", section).names()
            known = {k: v for k, v in values.items() if k in names}
            skipped += ["%s.%s" % (section, k) for k in values if k not in names]
            if not known:
                continue
            try:
                stored = fc.set(section, known)
            except (KeyError, ValueError, TypeError) as e:
                raise ApiError(400, "BAD_VALUE", "%s: %s" % (section, str(e).strip("'\"")), detail="%s: %s" % (section, str(e).strip("'\"")))
            done.append(section)
            for k, v in known.items():
                if not _same(stored[k], v):
                    changed.append({"field": "%s.%s" % (section, k), "asked": v, "stored": stored[k]})
        return {"sections": done, "skipped": skipped, "changed": changed}

    # ------------------------------------------------------------------ blackbox: the log memory of the craft, live recording
    def _aux_changed(self):
        with self._data:
            self.aux_seq += 1

    def bb_sessions(self):
        return [{"index": i, "start": a, "size": n} for i, (a, n) in enumerate(self._need().blackbox_sessions())]

    def bb_download(self, index=None):
        """Start copying one stored session (or all, index None) into the log folder. Progress and the result travel in the event feed."""
        with self.lock:
            fc = self._need()
            if self.job and self.job["state"] == "running":
                raise ApiError(409, "BUSY", "a log download is running")
            # A live recording is in the way of nothing the user cares about now: it is ended (and kept as a file), then the
            # download starts. Refusing here with "busy" left people guessing why the log could not be fetched.
            if self.rec and self.rec.stopped is None:
                self._stop_recording(fc)
            sessions = fc.blackbox_sessions(final=True)
            if not sessions:
                raise ApiError(409, "NO_LOG", "the log memory holds no session")
            if index is None:
                start, size = 0, sessions[-1][0] + sessions[-1][1]
            elif isinstance(index, int) and not isinstance(index, bool) and 0 <= index < len(sessions):
                start, size = sessions[index]
            else:
                raise ApiError(400, "BAD_VALUE", "no session %r" % (index,), detail="no session %r" % (index,))
            job = {"kind": "download", "state": "running", "index": index, "done": 0, "total": size, "file": None, "error": None}
            self.job = job
        self._aux_changed()
        threading.Thread(target=self._download, args=(fc, job, start, size), name="wbgui-download", daemon=True).start()
        return dict(job)

    def _download(self, fc, job, start, size):
        def progress(done, total):
            job["done"] = done
            self._last_frame = time.monotonic()
            self._aux_changed()
        self._busy = True                   # the link is full of log data: the watchdog must not take the silence of the streams for a reboot
        try:
            data = fc.blackbox_read(start, size, progress)
            found = wblog.parse(data)
            number = found[0].meta.get("session", "") if len(found) == 1 else ""
            job["file"] = self.library.save(logview.stamp_name("flight" if len(found) == 1 else "flights", number), data)
            job["sessions"] = len(found)
            job["done"], job["state"] = size, "done"
        except WbError as e:
            job["state"], job["error"] = "failed", e.name
        except logview.LogError as e:
            job["state"], job["error"] = "failed", e.code
        except Exception as e:              # noqa: BLE001 - the port went away under the download, the disk is full ...
            job["state"], job["error"], job["detail"] = "failed", "INTERNAL", "%s: %s" % (type(e).__name__, e)
        finally:
            self._busy = False
            self._last_frame = time.monotonic()
            self._aux_changed()

    def bb_record(self, on, rate_hz=0):
        """Start / stop recording the live stream of the firmware into a file of the log folder (the chart can follow it: file "@live")."""
        with self.lock:
            fc = self._need()
            rec = self.rec
            if on:
                if self.job and self.job["state"] == "running":
                    raise ApiError(409, "BUSY", "a log download is running")
                if rec and rec.stopped is None:
                    return self.rec_status()
                if not isinstance(rate_hz, (int, float)) or isinstance(rate_hz, bool) or not 0 <= rate_hz <= 2000:
                    raise ApiError(400, "BAD_VALUE", "rate_hz 0..2000", detail="rate_hz")
                name = self.library.free_name(logview.stamp_name("live"))
                rec = logview.Recording(os.path.join(self.library.root, name), name)
                fc.on_blackbox = lambda t, q, p, rec=rec: rec.frame(t, q, p, F.encode(t, q, p))
                try:
                    fc.blackbox_stream(True, int(rate_hz))
                except Exception:
                    fc.on_blackbox = None
                    rec.close()
                    os.remove(rec.path)
                    raise
                self.rec = rec
            elif rec and rec.stopped is None:
                self._stop_recording(fc)
        self._aux_changed()
        return self.rec_status()

    def _stop_recording(self, fc):
        """End the live recording (self.lock is held): the firmware stops the stream, its tail is waited for, the file is kept."""
        rec = self.rec
        try:
            fc.blackbox_stream(False)
        except WbError:
            pass
        end = time.monotonic() + 1.0
        while time.monotonic() < end:               # the tail of the stream and its END marker
            s = rec.session()
            if s and s.complete:
                break
            time.sleep(0.03)
        self._end_recording()

    def _end_recording(self):
        rec, fc = self.rec, self.fc
        if not rec or rec.stopped is not None:
            return
        if fc:
            fc.on_blackbox = None
        rec.close()
        s = rec.session()
        if not s or not len(s.fast["t"]):           # nothing came: no empty file in the list
            try:
                os.remove(rec.path)
            except OSError:
                pass
            rec.name = None
        self._aux_changed()

    def rec_status(self):
        rec = self.rec
        if not rec:
            return {"on": False, "file": None}
        st = rec.status()
        if st["on"] and st["complete"] and self.lock.acquire(timeout=0.05):      # the firmware ended the stream by itself (it found the app silent, or it restarted)
            try:
                self._end_recording()
            finally:
                self.lock.release()
            st = rec.status()
        return st

    # ------------------------------------------------------------------ logs on this computer
    def log_session(self, name, index):
        if name == "@live":
            s = self.rec.session() if self.rec else None
            if s is None:
                raise logview.LogError("NO_SUCH_LOG", "@live")
            return s
        return self.library.session(name, index)

    def log_meta(self, name):
        if name == "@live":
            return {"file": name, "sessions": [self.log_session(name, 0).summary()]}
        return {"file": name, "sessions": [s.summary() for s in self.library.sessions(name)]}

    def log_settings(self, name, index):
        """The settings a flight was flown with, in the form of a settings backup (so "restore" accepts it)."""
        s = self.log_session(name, index)
        return {"windburst_backup": 1, "firmware": s.meta.get("firmware", ""), "schema": s.meta.get("schema", ""), "from_log": name,
                "schema_matches": s.schema_matches(self.schema), "settings": s.settings(self.schema)}

    def log_upload(self, name, data_b64):
        try:
            data = base64.b64decode(data_b64 or "", validate=True)
        except (binascii.Error, ValueError, TypeError):
            raise ApiError(400, "BAD_REQUEST", "data must be base64")
        return self.library.save(name or "log", data)

    # ------------------------------------------------------------------ event feed (one cursor per browser connection)
    def poll(self, cursor):
        """Everything newer than `cursor` ({'seq', 'log', 'conn'}), as one JSON-able dict; None when nothing changed."""
        out = {}
        with self._data:
            seq = cursor["seq"]
            s = {n: v for n, (q, v) in self.latest.items() if q > seq and n != self.FAST}
            if s:
                out["s"] = s
            f = [row for q, row in self.fast if q > seq]
            if f:
                out["f"] = f
            lg = [[lvl, text] for q, lvl, text in self.logs if q > cursor["log"]]
            if lg:
                out["log"] = lg
            rw = "".join(text for q, text in self.raw if q > cursor["log"])
            if rw:
                out["raw"] = rw
            cursor["seq"] = cursor["log"] = self.seq
            conn_changed = cursor["conn"] != self.conn_seq
            cursor["conn"] = self.conn_seq
            aux_changed = cursor.get("aux") != self.aux_seq
            cursor["aux"] = self.aux_seq
        if conn_changed:
            out["conn"] = self.state()
        rec = self.rec
        now = time.monotonic()
        if aux_changed or (rec and rec.stopped is None and now - cursor.get("rec_t", 0) > 0.25):
            cursor["rec_t"] = now
            if self.job:
                out["job"] = dict(self.job)
            if rec or aux_changed:
                out["rec"] = self.rec_status()
        return out or None

    def close(self):
        self._stop = True
        self.disconnect()


def _same(a, b):
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) <= 1e-4 * max(1.0, abs(b))
    if isinstance(a, str) and isinstance(b, str):
        return a.upper() == b.upper()
    return a == b


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "wbgui"

    def log_message(self, fmt, *args):       # quiet: the terminal is for the user, not for an access log
        pass

    # ---- who is asking
    def _host_ok(self):
        if self.headers.get("Host", "") not in self.server.hosts:
            return False
        origin = self.headers.get("Origin")
        return origin is None or origin in self.server.origins

    def _token_ok(self, query):
        got = self.headers.get("X-WB-Token") or (query.get("t") or [""])[0]
        return hmac.compare_digest(got.encode(), self.server.token.encode())

    # ---- replies
    def _send(self, status, body, ctype="application/json; charset=utf-8", extra=None):
        if not isinstance(body, (bytes, bytearray)):
            body = json.dumps(body, ensure_ascii=False, allow_nan=False, default=str).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _fail(self, e):
        if isinstance(e, ApiError):
            return self._send(e.status, dict({"error": e.code, "message": e.message}, **e.extra))
        if isinstance(e, logview.LogError):
            return self._send(404 if e.code in ("NO_SUCH_LOG", "NO_SUCH_SESSION") else 400, {"error": e.code, "message": str(e), "detail": e.detail})
        if isinstance(e, WbTimeout):
            return self._send(504, {"error": "TIMEOUT", "message": "the firmware did not answer (%s)" % e, "detail": str(e)})
        if isinstance(e, WbError):
            return self._send(409, {"error": e.name, "message": str(e), "result": e.result})
        if isinstance(e, KeyError):
            return self._send(400, {"error": "BAD_VALUE", "message": str(e).strip("'\""), "detail": str(e).strip("'\"")})
        return self._send(500, {"error": "INTERNAL", "message": "%s: %s" % (type(e).__name__, e)})

    # ---- GET
    def do_GET(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        if not self._host_ok():
            return self._send(403, {"error": "FORBIDDEN", "message": "bad Host/Origin"})
        if url.path.startswith("/api/"):
            if not self._token_ok(query):
                return self._send(403, {"error": "FORBIDDEN", "message": "missing token: open the address printed by `python -m wbclient gui`"})
            hub = self.server.hub
            try:
                if url.path == "/api/events":
                    return self._events()
                if url.path == "/api/state":
                    return self._send(200, hub.state())
                if url.path == "/api/ports":
                    return self._send(200, {"ports": hub.ports(), "bauds": list(BAUDS), "default_baud": 230400})
                if url.path == "/api/schema":
                    return self._send(200, hub.schema.raw)
                if url.path == "/api/settings":
                    return self._send(200, hub.settings())
                if url.path == "/api/report":
                    return self._send(200, {"report": hub.refresh_report()})
                if url.path == "/api/logs":
                    return self._send(200, {"dir": hub.library.root, "logs": hub.library.list()})
                if url.path == "/api/blackbox/sessions":
                    return self._send(200, {"sessions": hub.bb_sessions()})
                if url.path.startswith("/api/log/"):
                    return self._log_get(url.path[9:], query)
                if url.path == "/api/backup":
                    return self._send(200, json.dumps(hub.backup(), indent=1, ensure_ascii=False).encode("utf-8"),
                                      extra={"Content-Disposition": 'attachment; filename="windburst_settings.json"'})
                return self._send(404, {"error": "NOT_FOUND", "message": url.path})
            except Exception as e:           # noqa: BLE001 - every failure becomes a JSON reply the UI can show
                return self._fail(e)
        return self._static(url.path)

    def _log_get(self, what, query):
        hub = self.server.hub
        one = lambda k, d="": (query.get(k) or [d])[0]              # noqa: E731
        name = one("file")
        try:
            index = int(one("session", "0"))
        except ValueError:
            raise ApiError(400, "BAD_REQUEST", "session must be a number")
        attach = lambda fn: {"Content-Disposition": 'attachment; filename="%s"' % fn}      # noqa: E731 - names are made of [A-Za-z0-9 ._()+-] only
        if what == "meta":
            return self._send(200, hub.log_meta(name))
        if what == "view":
            def num(k):
                try:
                    return float(one(k)) if one(k) != "" else None
                except ValueError:
                    raise ApiError(400, "BAD_REQUEST", "%s must be a number" % k)
            names = [n for n in one("fields").split(",") if n]
            s = hub.log_session(name, index)
            doc = logview.view(s, names, num("t0"), num("t1"), int(num("px") or 800), num("tail"))
            doc.update(records=len(s.fast["t"]), complete=s.complete)
            return self._send(200, doc)
        if what == "file":
            return self._send(200, hub.library.read(name), "application/octet-stream", attach(name))
        if what == "csv":
            out = io.StringIO()
            hub.log_session(name, index).to_csv(out, slow=one("slow") == "1")
            fn = "%s_s%d%s.csv" % (name[:-len(logview.EXT)] if name != "@live" else "live", index, "_slow" if one("slow") == "1" else "")
            return self._send(200, out.getvalue().encode("utf-8"), "text/csv; charset=utf-8", attach(fn))
        if what == "settings":
            fn = "%s_s%d_settings.json" % (name[:-len(logview.EXT)] if name != "@live" else "live", index)
            return self._send(200, json.dumps(hub.log_settings(name, index), indent=1, ensure_ascii=False).encode("utf-8"), extra=attach(fn))
        return self._send(404, {"error": "NOT_FOUND", "message": what})

    def _static(self, path):
        rel = "index.html" if path in ("/", "") else path.lstrip("/")
        full = os.path.normpath(os.path.join(WEB_DIR, rel))
        if not full.startswith(WEB_DIR + os.sep) or not os.path.isfile(full):
            return self._send(404, b"not found", "text/plain")
        with open(full, "rb") as fp:
            body = fp.read()
        if rel == "index.html":
            body = body.replace(b"__WB_TOKEN__", self.server.token.encode())
        ctype = {".js": "text/javascript; charset=utf-8", ".mjs": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8", ".html": "text/html; charset=utf-8",
                 ".json": "application/json; charset=utf-8", ".svg": "image/svg+xml"}.get(os.path.splitext(full)[1].lower()) or mimetypes.guess_type(full)[0] or "application/octet-stream"
        self._send(200, body, ctype)

    def _events(self):
        hub = self.server.hub
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        cursor = {"seq": hub.seq, "log": 0, "conn": 0}       # a new page gets the connection state and the firmware log so far, then only what is new
        quiet = time.monotonic()
        try:
            while not self.server.stopping:
                msg = hub.poll(cursor)
                now = time.monotonic()
                if msg:
                    self.wfile.write(b"data: " + json.dumps(msg, ensure_ascii=False, separators=(",", ":"), default=str).encode("utf-8") + b"\n\n")
                    self.wfile.flush()
                    quiet = now
                elif now - quiet > 2.0:
                    self.wfile.write(b": keep-alive\n\n")
                    self.wfile.flush()
                    quiet = now
                time.sleep(0.033)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass

    # ---- POST
    def do_POST(self):
        url = urlparse(self.path)
        if not self._host_ok() or not self._token_ok(parse_qs(url.query)):
            return self._send(403, {"error": "FORBIDDEN", "message": "bad Host/Origin/token"})
        if (self.headers.get("Content-Type") or "").split(";")[0].strip().lower() != "application/json":
            return self._send(415, {"error": "BAD_REQUEST", "message": "Content-Type must be application/json"})
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if n > (48_000_000 if url.path == "/api/log/upload" else 2_000_000):       # a log of 32 MB as base64
                left = min(n, 64_000_000)                    # take the body off the wire (unread, it would be parsed as the next request), keep none of it
                while left > 0:
                    chunk = self.rfile.read(min(left, 65536))
                    if not chunk:
                        break
                    left -= len(chunk)
                self.close_connection = True
                return self._send(413, {"error": "BAD_REQUEST", "message": "too large"})
            body = json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
            if not isinstance(body, dict):
                raise ValueError("object expected")
        except ValueError as e:
            return self._send(400, {"error": "BAD_REQUEST", "message": "JSON: %s" % e})
        hub = self.server.hub
        try:
            p = url.path
            if p == "/api/connect":
                return self._send(200, hub.connect(body.get("port"), body.get("baud") or 230400))
            if p == "/api/disconnect":
                return self._send(200, hub.disconnect())
            if p == "/api/set":
                return self._send(200, {"values": hub.set(body.get("section"), body.get("changes"))})
            if p == "/api/save":
                hub.save()
                return self._send(200, {"ok": True})
            if p == "/api/defaults":
                hub.defaults(body.get("section"))
                return self._send(200, {"ok": True})
            if p == "/api/reboot":
                return self._send(200, hub.reboot())
            if p == "/api/command":
                return self._send(200, {"result": hub.command(body.get("name") or "", body.get("args"))})
            if p == "/api/streams":
                return self._send(200, {"rates": hub.set_rates(body.get("rates") or {})})
            if p == "/api/restore":
                return self._send(200, hub.restore(body))
            if p == "/api/blackbox/download":
                return self._send(200, {"job": hub.bb_download(body.get("index"))})
            if p == "/api/blackbox/record":
                return self._send(200, {"rec": hub.bb_record(bool(body.get("on")), body.get("rate_hz") or 0)})
            if p == "/api/log/delete":
                hub.library.delete(body.get("file"))
                return self._send(200, {"ok": True})
            if p == "/api/log/upload":
                return self._send(200, {"file": hub.log_upload(body.get("name"), body.get("data"))})
            return self._send(404, {"error": "NOT_FOUND", "message": p})
        except Exception as e:               # noqa: BLE001
            return self._fail(e)


class GuiServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, http_port=8765, hub=None, token=None, log_dir=None):
        last = None
        for port in ([http_port] if http_port == 0 else range(http_port, http_port + 20)):
            try:
                super().__init__(("127.0.0.1", port), Handler)
                break
            except OSError as e:
                last = e
        else:
            raise last
        self.hub = hub or Hub(log_dir=log_dir)
        self.token = token or secrets.token_urlsafe(18)
        self.stopping = False
        port = self.server_address[1]
        self.hosts = {"127.0.0.1:%d" % port, "localhost:%d" % port}
        self.origins = {"http://" + h for h in self.hosts}
        self.url = "http://127.0.0.1:%d/" % port

    def handle_error(self, request, client_address):
        # a browser tab that goes away in the middle of a reply is normal life, not something to print a traceback for
        if isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)

    def start(self):
        threading.Thread(target=self.serve_forever, name="wbgui-http", daemon=True).start()
        return self

    def stop(self):
        self.stopping = True
        self.shutdown()
        self.server_close()
        self.hub.close()


def run(port=None, baud=230400, http_port=8765, open_browser=True, log_dir=None):
    """Entry point of `python -m wbclient gui`."""
    srv = GuiServer(http_port, log_dir=log_dir)
    print("Windburst configurator: %s" % srv.url)
    print("  flight logs: %s" % srv.hub.library.root)
    if port:
        try:
            srv.hub.connect(port, baud)
            print("  connected: %s" % srv.hub.fc.info)
        except ApiError as e:
            print("  not connected to %s: %s" % (port, e.message))
    print("  (Ctrl+C to quit)")
    srv.start()
    if open_browser:
        try:
            import webbrowser
            webbrowser.open(srv.url)
        except Exception:
            pass
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()
    return 0
