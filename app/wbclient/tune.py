"""PID tuning from a flight log (0.31).

What PIDtoolbox and Betaflight's Blackbox Explorer show, worked out here and turned into suggestions:

  step response   how the craft's rotation answers a step of the setpoint. Flown sticks are not steps, so the response is
                  recovered by Wiener deconvolution (PIDtoolbox, Brian White): for windows of the log where the sticks moved,
                  impulse response = IFFT( G x conj(S) / (|S|^2 + lambda) ) with S, G the spectra of setpoint and gyro; its
                  running sum is the step response. The windows are averaged.
  numbers         peak (1.0 = no overshoot), latency (time to half the step, ms), steady value (200 .. 500 ms: 1.0 = the craft
                  ends where it was told), ringing (how often the response crosses 1.0 again)
  noise           what the gyro carries above 80 Hz after the filters, and the D term's share of the PID output: D can only
                  be raised while these are small
  suggestions     the usual rules of thumb (PIDtoolbox's guide, Betaflight's tuning notes): overshoot -> more D (or less P when
                  D is already noisy); slow and no overshoot -> more P (or feed-forward); steady value short of 1 -> more I;
                  ringing -> less P. Never more than 25 % at a time: fly, log, look again.

Pure Python (numpy is used when it is installed, only for speed). Used by the GUI (/api/log/tune) and `wbclient blackbox tune`.
"""
import cmath
import math

AXES = ("roll", "pitch", "yaw")

try:                                         # optional: 20x faster; everything below also works without it
    import numpy as _np
except ImportError:                          # pragma: no cover - depends on the computer
    _np = None


# ------------------------------------------------------------------ FFT (radix 2), used when numpy is not there
def _fft(x, inverse=False):
    n = len(x)
    a = list(x)
    j = 0
    for i in range(1, n):                    # bit reversal
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j |= bit
        if i < j:
            a[i], a[j] = a[j], a[i]
    size = 2
    sign = 1 if inverse else -1
    while size <= n:
        w = cmath.exp(sign * 2j * math.pi / size)
        half = size // 2
        for start in range(0, n, size):
            wk = 1.0
            for k in range(start, start + half):
                u, v = a[k], a[k + half] * wk
                a[k], a[k + half] = u + v, u - v
                wk *= w
        size <<= 1
    if inverse:
        a = [v / n for v in a]
    return a


def _deconvolve(s, g, lam):
    """Impulse response of g from s (same length, a power of two), Wiener-regularised."""
    if _np is not None:
        S, G = _np.fft.fft(s), _np.fft.fft(g)
        den = (S * _np.conj(S)).real
        return list(_np.fft.ifft(G * _np.conj(S) / (den + lam * den.max())).real)
    S, G = _fft(s), _fft(g)
    den = [(v * v.conjugate()).real for v in S]
    eps = lam * max(den)
    return [v.real for v in _fft([G[k] * S[k].conjugate() / (den[k] + eps) for k in range(len(S))], inverse=True)]


def _hann(n):
    return [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]


def step_response(setpoint, gyro, rate_hz, window_s=2.0, resp_s=0.5, min_input=20.0, lam=1e-5, max_windows=12):
    """-> {"t": [s], "y": [step response], "windows": n} or None when the log has no stick movement to learn from.
    setpoint / gyro: deg/s at rate_hz."""
    if rate_hz <= 0 or len(setpoint) != len(gyro):
        return None
    n = 1
    while n < window_s * rate_hz:
        n <<= 1
    m = int(resp_s * rate_hz)
    if len(setpoint) < n or m < 4:
        return None
    hop = n // 2
    cands = []
    for a in range(0, len(setpoint) - n + 1, hop):
        seg = setpoint[a:a + n]
        peak = max(abs(v) for v in seg)
        if peak < min_input:
            continue                                         # sticks still: nothing to learn (and only noise to divide by)
        energy = sum(v * v for v in seg)
        cands.append((energy, a))
    if not cands:
        return None
    cands.sort(reverse=True)
    win = _hann(n)
    acc = [0.0] * m
    used = 0
    for _, a in cands[:max_windows]:
        s = [setpoint[a + i] * win[i] for i in range(n)]
        g = [gyro[a + i] * win[i] for i in range(n)]
        imp = _deconvolve(s, g, lam)
        y, run = [], 0.0
        for k in range(m):
            run += imp[k]
            y.append(run)
        steady = sum(y[int(0.2 * rate_hz):]) / max(1, m - int(0.2 * rate_hz))
        if not (0.5 < steady < 1.5):
            continue                                         # a window that did not say anything sensible (PIDtoolbox drops these too)
        for k in range(m):
            acc[k] += y[k]
        used += 1
    if not used:
        return None
    return {"t": [k / rate_hz for k in range(m)], "y": [v / used for v in acc], "windows": used}


def metrics(resp, rate_hz):
    """Peak, latency (ms to half the step), steady value (200 .. 500 ms) and ringing (crossings of 1.0 after the peak)."""
    y = resp["y"]
    k_peak = max(range(len(y)), key=lambda k: y[k])
    k_half = next((k for k, v in enumerate(y) if v >= 0.5), None)
    lo = min(len(y) - 1, int(0.2 * rate_hz))
    steady = sum(y[lo:]) / max(1, len(y) - lo)
    cross, above = 0, y[k_peak] > 1.0
    for v in y[k_peak:]:
        if (v > 1.0) != above:
            cross += 1
            above = v > 1.0
    return {"peak": round(y[k_peak], 3), "peak_ms": round(1000.0 * k_peak / rate_hz, 1),
            "latency_ms": round(1000.0 * k_half / rate_hz, 1) if k_half is not None else None,
            "steady": round(steady, 3), "ringing": cross}


def noise(values, rate_hz, cutoff_hz=80.0):
    """RMS of what is left above about cutoff_hz (deg/s for the gyro): the signal minus a centred moving average of 1 / cutoff_hz
    (centred: no lag, so a stick move is not counted as noise)."""
    n = len(values)
    w = int(rate_hz / cutoff_hz) | 1                          # an odd number of samples
    if n < 4 * w or w < 3:
        return None
    pre = [0.0]
    for v in values:
        pre.append(pre[-1] + v)
    h, acc, cnt = w // 2, 0.0, 0
    for i in range(h, n - h):
        acc += (values[i] - (pre[i + h + 1] - pre[i - h]) / w) ** 2
        cnt += 1
    return round(math.sqrt(acc / cnt), 2)


def _ratio(part, whole):
    s_w = sum(v * v for v in whole)
    return math.sqrt(sum(v * v for v in part) / s_w) if s_w > 0 else 0.0


def suggest(axis_metrics, pid, d_noise, gyro_noise):
    """Changes to the PID settings: [{axis, param, now, new, why}] (why: a short code; the page words it).
    axis_metrics: {axis index: metrics}; pid: the log's pid section ({"p": [3], "i": [3], "d": [3], "f": [3], ...})."""
    out = []

    def change(ax, param, factor, why):
        now = pid.get(param, [0, 0, 0])[ax]
        if not now:
            return
        new = int(round(now * factor))
        if new == now:
            new = now + (1 if factor > 1 else -1)
        out.append({"axis": AXES[ax], "param": param, "now": now, "new": max(0, new), "why": why})

    for ax, m in axis_metrics.items():
        if m is None:
            continue
        noisy = (d_noise.get(ax) or 0) > 0.30 or (gyro_noise.get(ax) or 0) > 6.0
        peak, steady, ring = m["peak"], m["steady"], m["ringing"]
        if peak > 1.10 and ax != 2:
            over = min(0.25, (peak - 1.05) * 1.5)                     # 5 % too much -> about 7 % more D, at most 25 %
            if noisy:
                change(ax, "p", 1.0 - over, "OVERSHOOT_NOISY_D")
            else:
                change(ax, "d", 1.0 + over, "OVERSHOOT")
                if pid.get("d_max") and pid["d_max"][ax]:
                    change(ax, "d_max", 1.0 + over, "OVERSHOOT")
        elif peak <= 1.05 and m["latency_ms"] is not None and m["latency_ms"] > 18 and ax != 2:
            change(ax, "p", 1.15 if m["latency_ms"] > 25 else 1.08, "SLOW")
            if pid.get("f") and pid["f"][ax]:
                change(ax, "f", 1.10, "SLOW")
        if ring >= 3 and peak > 1.02:
            change(ax, "p", 0.90, "RINGING")
        if steady < 0.95:
            change(ax, "i", min(1.25, 1.0 + (1.0 - steady) * 2.0), "STEADY_LOW")
        if ax != 2 and noisy and peak <= 1.10:
            out.append({"axis": AXES[ax], "param": "d", "now": pid.get("d", [0, 0, 0])[ax], "new": None, "why": "NOISE_NO_MORE_D"})
    return out


def analyse(session, schema=None):
    """The whole analysis of one log session (wblog.Session) -> a JSON-able dict for the GUI."""
    fast = session.fast
    t = fast.get("t")
    out = {"axes": {}, "suggestions": [], "pid": None, "sliders": False, "notes": []}
    if not t or len(t) < 100:
        out["notes"].append("TOO_SHORT")
        return out
    rate = 1.0 / session.sample_dt() if session.sample_dt() > 0 else session.rate_hz
    out["rate_hz"] = round(rate, 1)
    if rate < 250:
        out["notes"].append("RATE_LOW")                    # PIDtoolbox wants at least ~1 kHz; below 250 Hz the response is too coarse
    axis_metrics, d_noise, g_noise = {}, {}, {}
    for ax in range(3):
        sp, gy = fast.get("setpoint[%d]" % ax), fast.get("gyro[%d]" % ax)
        if sp is None or gy is None:
            out["notes"].append("NO_SETPOINT_GYRO")
            break
        resp = step_response(list(sp), list(gy), rate)
        m = metrics(resp, rate) if resp else None
        g_noise[ax] = noise(list(gy), rate)
        dterm, total = fast.get("pid_d[%d]" % ax), None
        if dterm is not None:
            parts = [fast.get("pid_%s[%d]" % (k, ax)) for k in ("p", "i", "d", "f")]
            if all(p is not None for p in parts):
                total = [a + b + c + d for a, b, c, d in zip(*parts)]
                d_noise[ax] = round(_ratio([v for v in dterm], total), 3) if total else None
        axis_metrics[ax] = m
        step = None
        if resp:
            k = max(1, len(resp["t"]) // 250)              # at most 250 points to the page
            step = {"t": [round(v, 4) for v in resp["t"][::k]], "y": [round(v, 4) for v in resp["y"][::k]], "windows": resp["windows"]}
        out["axes"][AXES[ax]] = {"metrics": m, "step": step, "gyro_noise": g_noise[ax], "d_share": d_noise.get(ax)}
    if schema is not None:
        try:
            settings = session.settings(schema)
        except Exception:                                    # noqa: BLE001 - a log of another firmware: no settings, still the analysis
            settings = {}
        pid = settings.get("pid")
        if pid:
            out["pid"] = {k: list(pid[k]) for k in ("p", "i", "d", "d_max", "f") if k in pid}
            out["sliders"] = bool(pid.get("slider_mode"))
            if axis_metrics:
                out["suggestions"] = suggest(axis_metrics, out["pid"], d_noise, g_noise)
        else:
            out["notes"].append("NO_SETTINGS")
    if not any(a["metrics"] for a in out["axes"].values()) and "NO_SETPOINT_GYRO" not in out["notes"]:
        out["notes"].append("NO_STICK_MOVES")
    return out
