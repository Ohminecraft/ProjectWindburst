"""The desktop window (wbclient/desktop.py) around pywebview: what can be checked without opening a window."""
from types import SimpleNamespace as NS

from wbclient import desktop

ORIGIN = "http://127.0.0.1:48765/"
CORE = NS(CoreWebView2PermissionKind=NS(Geolocation="GEO", Camera="CAM"), CoreWebView2PermissionState=NS(Allow="ALLOW", Default="DEFAULT"))


class Event(list):
    """A .NET event as pythonnet shows it: `+=` adds a handler."""
    def __iadd__(self, fn):
        self.append(fn)
        return self


def _fake_chromium():
    class EdgeChrome:
        def on_webview_ready(self, sender, args):
            self.ready = True
    return NS(EdgeChrome=EdgeChrome)


def test_the_page_may_ask_windows_for_the_location():
    """0.37 Mission tab, "my position": WebView2 would ask with a box of its own before Windows' location service asks. Our own
    page is let through to Windows (which keeps the last word); another page, or another permission, is left to WebView2."""
    chromium = _fake_chromium()
    assert desktop.allow_location(ORIGIN, chromium, CORE)
    sender = NS(CoreWebView2=NS(PermissionRequested=Event()))
    ec = chromium.EdgeChrome()
    ec.on_webview_ready(sender, NS(IsSuccess=True))
    assert ec.ready, "pywebview's own start-up still runs"
    (allow,) = sender.CoreWebView2.PermissionRequested

    def ask(kind, uri):
        a = NS(PermissionKind=kind, Uri=uri, State="DEFAULT")
        allow(None, a)
        return a.State
    assert ask("GEO", ORIGIN + "#mission") == "ALLOW"
    assert ask("CAM", ORIGIN) == "DEFAULT", "only the location"
    assert ask("GEO", "https://evil.example/") == "DEFAULT"
    assert ask("GEO", "http://127.0.0.1:487650/") == "DEFAULT", "another port that starts the same"

    failed = NS(CoreWebView2=NS(PermissionRequested=Event()))
    chromium.EdgeChrome().on_webview_ready(failed, NS(IsSuccess=False))
    assert not failed.CoreWebView2.PermissionRequested, "WebView2 did not start: nothing to hook"


def test_no_webview2_no_harm():
    """Not Windows, or a pywebview made otherwise: nothing is changed, the window then shows WebView2's own question."""
    assert not desktop.allow_location(ORIGIN, NS(), CORE)
    assert not desktop.allow_location(ORIGIN, _fake_chromium(), NS())


def test_the_packaged_program_can_open_socket_urls(monkeypatch):
    """Windburst.exe said "invalid URL, protocol 'socket' not known" for socket://127.0.0.1:5760 (the simulator): pyserial imports
    its URL handlers by name, so PyInstaller must be told to pack them."""
    import importlib.util
    import os
    import sys
    import types
    seen = []
    pyi = types.ModuleType("PyInstaller"); pyi.__path__ = []
    main = types.ModuleType("PyInstaller.__main__"); main.run = seen.append
    pyi.__main__ = main
    monkeypatch.setitem(sys.modules, "PyInstaller", pyi)
    monkeypatch.setitem(sys.modules, "PyInstaller.__main__", main)
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "packaging", "build_exe.py")
    spec = importlib.util.spec_from_file_location("build_exe", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.main()
    args = seen[0]
    packed = [args[i + 1] for i, a in enumerate(args) if a in ("--hidden-import", "--collect-submodules")]
    assert "serial.urlhandler" in packed or "serial.urlhandler.protocol_socket" in packed
    import serial.urlhandler.protocol_socket  # noqa: F401  (the module that is packed exists in this pyserial)
