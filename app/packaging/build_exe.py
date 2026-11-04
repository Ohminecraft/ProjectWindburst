"""Builds dist/Windburst.exe: the configurator as one program, opening in its own window (no browser, no console).

    pip install pywebview pyinstaller pyserial
    python packaging/build_exe.py           (from the app folder)

Inside: Python, wbclient, the page (wbclient/web) and the protocol schema (protocol/wbp_schema.json). Runs on Windows 10 / 11
with the Edge WebView2 runtime (there by default on Windows 11 and on an up-to-date Windows 10).
"""
import os
import sys

import PyInstaller.__main__

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.dirname(HERE)
ROOT = os.path.dirname(APP)
SEP = ";" if os.name == "nt" else ":"


def main():
    schema = os.path.join(ROOT, "protocol", "wbp_schema.json")
    if not os.path.exists(schema):
        sys.exit("protocol/wbp_schema.json not found: build from the project's app folder")
    args = [
        os.path.join(HERE, "windburst_app.py"),
        "--name", "Windburst",
        "--onefile", "--windowed", "--noconfirm", "--clean",
        "--icon", os.path.join(HERE, "windburst.ico"),
        "--paths", APP,
        "--add-data", os.path.join(APP, "wbclient", "web") + SEP + "wbclient/web",
        "--add-data", schema + SEP + "wbclient",                # schema.default_path() looks next to schema.py first
        "--add-data", os.path.join(APP, "wbclient", "schemas") + SEP + "wbclient/schemas",      # the released firmware's schemas: older firmware works too (0.43)
        "--hidden-import", "serial.tools.list_ports",
        # serial_for_url() imports its URL handlers by name (socket:// = serial.urlhandler.protocol_socket): PyInstaller cannot see them,
        # and without them the simulator and the craft's WiFi fail with "protocol 'socket' not known"
        "--collect-submodules", "serial.urlhandler",
        "--collect-submodules", "webview",
        "--distpath", os.path.join(APP, "dist"),
        "--workpath", os.path.join(APP, "build", "pyinstaller"),
        "--specpath", os.path.join(APP, "build"),
    ]
    PyInstaller.__main__.run(args)
    print("built:", os.path.join(APP, "dist", "Windburst.exe"))


if __name__ == "__main__":
    main()
