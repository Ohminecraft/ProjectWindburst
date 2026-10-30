"""The configurator as a desktop program: the same page and server as `python -m wbclient gui`, shown in a window of its own
(pywebview: Edge WebView2 on Windows, WebKit on macOS / Linux) instead of a browser tab.

    python -m wbclient app            (or the packaged Windburst.exe)

The local server listens on a free port of 127.0.0.1 only; closing the window stops it and lets go of the serial port.
What the page keeps in its browser storage (theme, language, last baud rate) lives in the user's own folder, so it survives a
restart of the program.
"""
import os
import sys

from .gui import GuiServer

TITLE = "Windburst Configurator"
# The page's own storage (theme, size, language, baud rate) belongs to its origin, port included: a new random port at every
# start would forget it. A fixed port (or the next free of the 20 after it) keeps the same origin from one start to the next.
DESKTOP_PORT = 48765


def _storage_dir():
    base = os.environ.get("LOCALAPPDATA") or os.environ.get("XDG_DATA_HOME") or os.path.join(os.path.expanduser("~"), ".local", "share")
    path = os.path.join(base, "Windburst", "webview")
    os.makedirs(path, exist_ok=True)
    return path


def allow_location(origin, chromium=None, core=None):
    """The Mission tab's "my position" asks the page for the location; WebView2 would first ask with a box of its own, then
    Windows' location service (GPS, Wi-Fi around) asks again. The page is ours: WebView2 lets it ask, Windows keeps the last word
    (Settings > Privacy & security > Location). Applied to the windows made after this; False where it cannot be (no WebView2,
    a pywebview built otherwise) - the page then gets WebView2's box, nothing worse."""
    try:
        if chromium is None:
            from webview.platforms import edgechromium as chromium
        if core is None:
            from Microsoft.Web.WebView2 import Core as core
        kind, state = core.CoreWebView2PermissionKind.Geolocation, core.CoreWebView2PermissionState.Allow
        ready = chromium.EdgeChrome.on_webview_ready
    except Exception:            # noqa: BLE001 - not Windows, no .NET, another pywebview
        return False

    def allow(sender, args):
        if args.PermissionKind == kind and str(args.Uri).startswith(origin):
            args.State = state

    def on_ready(self, sender, args):
        ready(self, sender, args)
        if args.IsSuccess:
            sender.CoreWebView2.PermissionRequested += allow
    chromium.EdgeChrome.on_webview_ready = on_ready
    return True


def run(log_dir=None, debug=False):
    try:
        import webview
    except ImportError:
        sys.stderr.write("The desktop window needs pywebview (pip install pywebview); `python -m wbclient gui` opens the browser instead.\n")
        return 2
    srv = GuiServer(http_port=DESKTOP_PORT, log_dir=log_dir).start()
    allow_location(srv.url)
    webview.settings["ALLOW_DOWNLOADS"] = True            # backups, CSV exports, downloaded logs: saved like in a browser
    win = webview.create_window(TITLE, srv.url, width=1440, height=920, min_size=(900, 600), background_color="#171c21", text_select=True)

    def closing():
        srv.stop()
    win.events.closed += closing
    try:
        webview.start(private_mode=False, storage_path=_storage_dir(), debug=debug)
    finally:
        if not srv.stopping:
            srv.stop()
    return 0


def main():
    """Entry point of the packaged program (no console)."""
    return run(debug="--debug" in sys.argv)
