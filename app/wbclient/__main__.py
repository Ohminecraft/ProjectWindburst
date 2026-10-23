"""Command line configurator:  python -m wbclient --port <PORT> <command> ...

    PORT        COM5, /dev/ttyACM0 ...  or  socket://127.0.0.1:5760  for the simulator (build/wb_sim)

    info                                   firmware identity + boot report (what was detected / why not)
    get [section]                          settings as JSON (all sections when none is named)
    set <section> key=value [key=value..]  change settings in RAM       e.g.  set pid slider_master=130 tpa_rate=50
    save | reboot | defaults [section]     write to flash / restart / restore defaults (RAM, then 'save')
    backup <file.json> | restore <file.json>
    monitor <stream>[,<stream>..] [--hz N] [--seconds S]     live telemetry (attitude, rc, status, motors, sensors, gps, estimate)
    cmd <NAME> [arg=value ..]              any firmware command          e.g.  cmd SET_HOME
    calibrate gyro | level | acc | mag     guided calibrations
    motor <index|all> <percent> [--seconds S] --props-off     spin a motor (REMOVE THE PROPELLERS)
    blackbox status | list | download [N] [-o DIR] | erase | record FILE [--seconds S] [--hz N]
                                           the flight log in the controller's flash: state, sessions, fetch as .wblog, erase, record a live stream
    blackbox info FILE | csv FILE [--slow] [-o OUT.csv]       read a .wblog file (no flight controller needed)
    schema                                 list sections, streams and commands of the protocol
    gui [--http 8765] [--no-browser] [--logs DIR]   the graphical configurator in your browser (--port is optional: choose it in the page)
"""
import os
import argparse
import json
import sys
import time

from . import WbClient, WbError, Schema


def _parse_value(text):
    if "," in text:
        return [_parse_value(t) for t in text.split(",")]
    low = text.lower()
    if low in ("none", "null", "-"):
        return None
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(text, 0)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


def _kv(pairs):
    out = {}
    for p in pairs:
        if "=" not in p:
            raise SystemExit("expected key=value, got %r" % p)
        k, v = p.split("=", 1)
        out[k] = _parse_value(v)
    return out


def _print_json(obj):
    print(json.dumps(obj, indent=2, default=str))


def cmd_info(fc, a):
    print(fc.info)
    for line in fc.report():
        print("  " + line)


def cmd_get(fc, a):
    _print_json(fc.get(a.args[0]) if a.args else fc.get_all())


def _wire(layout, key, value):
    """The number(s) that go on the wire for one field - used to tell 'stored as asked' from 'limited by the firmware'."""
    f = layout.field(key)
    vals = value if isinstance(value, (list, tuple)) else [value]
    out = []
    for v in vals:
        w = layout._enc(f, v)
        out.append(round(w, 6) if isinstance(w, float) else w)
    return out


def cmd_set(fc, a):
    if len(a.args) < 2:
        raise SystemExit("usage: set <section> key=value ...   (one element of an array: key[index]=value)")
    section = a.args[0]
    layout = fc.schema.layout("section", section)
    want, current = {}, None
    for k, v in _kv(a.args[1:]).items():
        if k.endswith("]") and "[" in k:                    # one element of an array field: read-modify-write of that array
            name, idx = k[:-1].split("[", 1)
            if current is None:
                current = fc.get(section)
            if name not in want:
                want[name] = list(current[layout.field(name)["n"]])
            if not idx.isdigit() or int(idx) >= len(want[name]):
                raise SystemExit("%s: index %s out of range (0..%d)" % (name, idx, len(want[name]) - 1))
            want[name][int(idx)] = v
        else:
            want[k] = v
    stored = fc.set(section, want)
    for k, v in want.items():
        s = stored[k]
        same = _wire(layout, k, v) == _wire(layout, k, s)
        print("%s.%s = %s%s" % (section, k, s, "" if same else "   (you asked for %s - the firmware limited it)" % (v,)))
    live = fc.schema.section(section).get("live")
    print("stored in RAM%s. Use 'save' to keep it." % ("" if live else "; this section takes effect after 'save' + 'reboot'"))


def cmd_save(fc, a):
    fc.save()
    print("saved")


def cmd_reboot(fc, a):
    print("restarted:", fc.reboot())


def cmd_defaults(fc, a):
    fc.defaults(a.args[0] if a.args else None)
    print("defaults restored in RAM (use 'save' to keep them)")


def cmd_backup(fc, a):
    data = {"firmware": str(fc.info), "schema": "0x%08X" % fc.schema.hash, "settings": fc.get_all()}
    with open(a.args[0], "w", encoding="utf-8") as fp:
        json.dump(data, fp, indent=2)
    print("wrote", a.args[0])


def cmd_restore(fc, a):
    with open(a.args[0], "r", encoding="utf-8") as fp:
        data = json.load(fp)
    for name, values in data["settings"].items():
        if name not in fc.schema.sections:
            print("skipping unknown section", name)
            continue
        known = {k: v for k, v in values.items() if k in fc.schema.layout("section", name).names()}
        fc.set(name, known)
        print("restored", name)
    print("in RAM now - use 'save' (and 'reboot') to make it permanent")


def cmd_monitor(fc, a):
    names = a.args[0].split(",") if a.args else ["attitude", "status"]
    for n in names:
        fc.subscribe(n, a.hz)
    end = time.monotonic() + a.seconds if a.seconds else None
    shown = {}
    try:
        while end is None or time.monotonic() < end:
            time.sleep(0.05)
            for n in names:
                v = fc.latest(n)
                if v and shown.get(n) != v.get("t_ms"):          # a sample is printed once; a silent stream prints nothing
                    shown[n] = v.get("t_ms")
                    print(n + ": " + "  ".join("%s=%s" % (k, ("%.2f" % x) if isinstance(x, float) else x) for k, x in v.items()), flush=True)
            while fc.logs:
                print("LOG:", fc.logs.popleft()[1], flush=True)
    except KeyboardInterrupt:
        pass
    fc.unsubscribe()


def cmd_cmd(fc, a):
    if not a.args:
        raise SystemExit("usage: cmd <NAME> [arg=value ...]")
    res = fc.command(a.args[0], **_kv(a.args[1:]))
    print("OK", res if res else "")


def _wait_cal(fc, kind, timeout=20.0):
    end = time.monotonic() + timeout
    last = -1
    while time.monotonic() < end:
        c = fc.wait_stream("calibration", 2.0)
        if c["kind"] != kind:
            continue
        if c["progress"] != last and c["state"] == "RUNNING":
            last = c["progress"]
            print("\r  %3d %%" % last, end="", flush=True)
        if c["state"] in ("DONE", "FAILED"):
            print("\r  %s      " % c["state"])
            return c
    raise SystemExit("calibration did not finish")


def cmd_calibrate(fc, a):
    what = a.args[0] if a.args else ""
    fc.subscribe("calibration", 20)
    if what == "gyro":
        print("Keep the craft still...")
        fc.command("GYRO_CAL")
        _wait_cal(fc, "GYRO")
    elif what == "level":
        input("Put the craft on a level surface, keep it still, then press Enter ")
        fc.command("LEVEL_TRIM")
        c = fc.get("calibration")
        print("level trim stored: roll %.1f  pitch %.1f deg  (use 'save')" % (c["acc_trim_roll"], c["acc_trim_pitch"]))
    elif what == "acc":
        print("Lay the craft still on each of its 6 faces (any order). The firmware recognises the face by itself.")
        faces = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
        mask = 0
        while mask != 0x3F:
            missing = [faces[i] for i in range(6) if not (mask >> i) & 1]
            input("  still missing (chip axis): %s - place the craft on a new face and press Enter " % " ".join(missing))
            fc.command("ACC_CAL_POINT")
            c = _wait_cal(fc, "ACCEL")
            if c["state"] == "FAILED":
                print("  rejected: the craft moved or was not resting on a face. Try again.")
            mask = c["mask"]
        fc.command("ACC_CAL_FINISH")
        c = fc.get("calibration")
        print("accelerometer calibrated: offset %s scale %s  (use 'save')" % ([round(x, 4) for x in c["acc_offset"]], [round(x, 4) for x in c["acc_scale"]]))
    elif what == "mag":
        fc.command("MAG_CAL_START")
        print("Rotate the craft slowly through every orientation (all six faces up, and turn around). Ctrl+C to cancel.")
        try:
            while True:
                c = fc.wait_stream("calibration", 2.0)
                print("\r  coverage %3d %%" % c["progress"], end="", flush=True)
                if c["progress"] >= 95:
                    try:
                        fc.command("MAG_CAL_FINISH", timeout=5.0)
                        print("\ncompass calibrated (use 'save')")
                        break
                    except WbError as e:
                        if e.name != "FAILED":
                            raise
                        print("\n  not good enough yet (%s) - keep rotating" % e.result.get("reason", "?"))
                        time.sleep(1.0)
        except KeyboardInterrupt:
            fc.command("MAG_CAL_CANCEL")
            print("\ncancelled")
    else:
        raise SystemExit("usage: calibrate gyro|level|acc|mag")
    fc.unsubscribe("calibration")


def cmd_motor(fc, a):
    if len(a.args) != 2:
        raise SystemExit("usage: motor <index|all> <percent> [--seconds S] --props-off")
    if not a.props_off:
        raise SystemExit("refusing: add --props-off after you REMOVED THE PROPELLERS")
    motor = 255 if a.args[0] == "all" else int(a.args[0])
    value = int(round(float(a.args[1]) * 10))
    end = time.monotonic() + (a.seconds or 2.0)
    try:
        while time.monotonic() < end:
            fc.command("MOTOR_TEST", motor=motor, value=value)       # must be refreshed: the firmware stops after 0.5 s of silence
            time.sleep(0.15)
    finally:
        fc.command("MOTOR_STOP")
    print("stopped")


def _log_summary(path):
    from . import wblog
    sessions = wblog.load(path)
    if not sessions:
        print("%s: no log session found" % path)
        return sessions
    for i, s in enumerate(sessions):
        fast, slow = s.names()
        print("session %d: %s, %.1f s, %d records at %g Hz, %d slow records%s%s%s" % (
            i, s.meta.get("firmware", "?"), s.duration, len(s.fast["t"]), s.rate_hz, len(s.slow["t"]),
            ", %d frame gaps" % s.gaps if s.gaps else "", ", %d records dropped by the firmware" % s.dropped if s.dropped else "", "" if s.complete else ", NOT closed (power cut?)"))
        print("  fast: " + " ".join(fast))
        print("  slow: " + " ".join(slow))
        for t, name, data in s.events:
            if name in ("ARM", "DISARM", "END", "DROPPED") or (name == "FAILSAFE" and data and data[0]):
                print("  %9.3f s  %s %s" % (t, name, " ".join(str(b) for b in data)))
    return sessions


def cmd_blackbox_file(a):
    """blackbox info / csv: work on a file, no connection."""
    from . import wblog
    action, path = a.args[0], a.args[1]
    if action == "info":
        _log_summary(path)
        return 0
    sessions = wblog.load(path)
    if action == "tune":                                     # PID tuning from the log (tune.py): step response per axis and what to change
        from . import tune
        if not sessions:
            raise SystemExit("%s: no log session found" % path)
        r = tune.analyse(sessions[int(a.args[2]) if len(a.args) > 2 else len(sessions) - 1], Schema())
        for ax, v in r["axes"].items():
            m = v["metrics"]
            print("%-5s " % ax + ("peak %.2f at %.0f ms, half the step after %s ms, steady %.2f, ringing %d" % (m["peak"], m["peak_ms"], m["latency_ms"], m["steady"], m["ringing"]) if m else "no stick moves to learn from")
                  + ("   gyro noise %.1f deg/s" % v["gyro_noise"] if v["gyro_noise"] is not None else "") + ("   D share %.0f %%" % (100 * v["d_share"]) if v["d_share"] is not None else ""))
        for s in r["suggestions"]:
            print("  %s %s: %s -> %s (%s)" % (s["axis"], s["param"], s["now"], s["new"] if s["new"] is not None else "keep", s["why"]))
        for n in r["notes"]:
            print("  note: %s" % n)
        return 0
    if not sessions:
        raise SystemExit("%s: no log session found" % path)
    index = int(a.args[2]) if len(a.args) > 2 else len(sessions) - 1
    out = a.out or (os.path.splitext(path)[0] + (".slow" if a.slow else "") + ".csv")
    with open(out, "w", newline="") as fp:
        sessions[index].to_csv(fp, slow=a.slow)
    print("session %d -> %s" % (index, out))
    return 0


def cmd_blackbox(fc, a):
    action = a.args[0] if a.args else "status"
    if action == "status":
        st = fc.blackbox_status()
        print("device %s, state %s, %d sessions, %.0f of %.0f kB used, fast record %d bytes%s" % (
            st["device"], st["state"], st["sessions"], st["used_kb"], st["capacity_kb"], st["record_bytes"], ", %d records dropped since boot" % st["dropped"] if st["dropped"] else ""))
    elif action == "list":
        for i, (start, size) in enumerate(fc.blackbox_sessions()):
            print("%3d  %8.1f kB at %d" % (i, size / 1024, start))
    elif action == "download":
        sessions = fc.blackbox_sessions()
        which = [int(a.args[1])] if len(a.args) > 1 else list(range(len(sessions)))
        out_dir = a.out or "."
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        for i in which:
            data = fc.blackbox_download(i, progress=lambda done, total: print("\rsession %d: %3d %%" % (i, 100 * done // total), end="", flush=True))
            path = os.path.join(out_dir, "windburst_%s_%02d.wblog" % (stamp, i))
            with open(path, "wb") as fp:
                fp.write(data)
            print("\rsession %d: %d bytes -> %s" % (i, len(data), path))
    elif action == "erase":
        fc.blackbox_erase(progress=lambda pct: print("\rerasing %3d %%" % pct, end="", flush=True))
        print("\rerased        ")
    elif action == "record":
        if len(a.args) < 2:
            raise SystemExit("blackbox record needs a file name")
        data = fc.blackbox_record(a.seconds or 10.0, a.hz or 0)
        with open(a.args[1], "wb") as fp:
            fp.write(data)
        print("%d bytes -> %s" % (len(data), a.args[1]))
        _log_summary(a.args[1])
    else:
        raise SystemExit("unknown blackbox action %r (status, list, download, erase, record, info, csv)" % action)


def cmd_schema(schema):
    print("protocol %d, schema 0x%08X (%s)" % (schema.version, schema.hash, schema.path))
    print("settings sections:")
    for s in schema.raw["sections"]:
        print("  %-12s %3d bytes %s  %s" % (s["name"], schema.layout("section", s["name"]).size, "live  " if s.get("live") else "reboot", ", ".join(f["n"] for f in s["fields"])))
    print("streams:")
    for s in schema.raw["streams"]:
        print("  %-12s default %3d Hz, max %4d Hz  %s" % (s["name"], s["rate"], s["max"], ", ".join(f["n"] for f in s["fields"])))
    print("commands:")
    for c in schema.raw["commands"]:
        print("  %-16s %s  %s" % (c["name"], " ".join("%s=<%s>" % (x["n"], x["t"]) for x in c["args"]), c.get("desc", "")))


COMMANDS = {"info": cmd_info, "get": cmd_get, "set": cmd_set, "save": cmd_save, "reboot": cmd_reboot, "defaults": cmd_defaults, "backup": cmd_backup,
            "restore": cmd_restore, "monitor": cmd_monitor, "cmd": cmd_cmd, "calibrate": cmd_calibrate, "motor": cmd_motor, "blackbox": cmd_blackbox}


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m wbclient", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port or socket://host:port")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--hz", type=int, default=None)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--props-off", action="store_true")
    ap.add_argument("--http", type=int, default=8765, help="gui: local web port")
    ap.add_argument("--no-browser", action="store_true", help="gui: do not open the browser")
    ap.add_argument("--logs", help="gui: folder for flight logs (default ~/Windburst/logs, or the WBCLIENT_LOG_DIR environment variable)")
    ap.add_argument("-o", "--out", help="blackbox: output directory (download) or file (csv)")
    ap.add_argument("--slow", action="store_true", help="blackbox csv: the slow records instead of the fast ones")
    ap.add_argument("command")
    ap.add_argument("args", nargs="*")
    a = ap.parse_args(argv)
    if a.command == "schema":
        cmd_schema(Schema())
        return 0
    if a.command == "gui":
        from .gui import run
        return run(a.port, a.baud, a.http, not a.no_browser, a.logs)
    if a.command == "app":                           # the same page in a window of its own (needs pywebview)
        from .desktop import run as run_app
        return run_app(a.logs)
    if a.command == "blackbox" and a.args and a.args[0] in ("info", "csv", "tune"):
        if len(a.args) < 2:
            ap.error("blackbox %s needs a file name" % a.args[0])
        return cmd_blackbox_file(a)
    if a.command not in COMMANDS:
        ap.error("unknown command %r" % a.command)
    if not a.port:
        ap.error("--port is required")
    try:
        with WbClient(a.port, a.baud) as fc:
            COMMANDS[a.command](fc, a)
    except WbError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1
    except BrokenPipeError:                      # output piped into `head` etc.: the reader left, that is not an error
        try:
            sys.stdout = open(os.devnull, "w")
        except OSError:
            pass
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
