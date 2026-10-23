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


def _storage_dir():
    base = os.environ.get("LOCALAPPDATA") or os.environ.get("XDG_DATA_HOME") or os.path.join(os.path.expanduser("~"), ".local", "share")
    path = os.path.join(base, "Windburst", "webview")
    os.makedirs(path, exist_ok=True)
    return path


def run(log_dir=None, debug=False):
    try:
        import webview
    except ImportError:
        sys.stderr.write("The desktop window needs pywebview (pip install pywebview); `python -m wbclient gui` opens the browser instead.\n")
        return 2
    srv = GuiServer(http_port=0, log_dir=log_dir).start()
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
