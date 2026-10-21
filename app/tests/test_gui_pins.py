"""The pins tab names and reads pins exactly as the firmware does (src/port/pin_names.h): build/ui_reference prints the firmware's
names and parses, node runs the page's own js/pinname.js on the same inputs. On an STM32 board (device feature STM32) a pin is
PA0 .. PK15 = port * 16 + pin; a wrong name there would silently put a sensor on another pin."""
import json
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
REF = os.path.join(ROOT, "build", "ui_reference" + (".exe" if os.name == "nt" else ""))
PINJS = os.path.join(ROOT, "app", "wbclient", "web", "js", "pinname.js")

JS = r"""
import { pathToFileURL } from 'node:url';
import { readFileSync } from 'node:fs';
const m = await import(pathToFileURL(process.argv[2]).href);
const ref = JSON.parse(readFileSync(0, 'utf8'));
const out = {
  names: ref.pin_names.map(([c]) => m.pinName(c, true)),
  parse_stm32: ref.pin_parse.map(([t]) => m.pinParse(t, true)),
  parse_esp: ref.pin_parse.map(([t]) => m.pinParse(t, false)),
};
process.stdout.write(JSON.stringify(out));
"""


@pytest.fixture(scope="module")
def both(tmp_path_factory):
    if not os.path.exists(REF):
        pytest.skip("build/ui_reference not built (make apptest builds it)")
    if not shutil.which("node"):
        pytest.skip("node not installed")
    ref = json.loads(subprocess.run([REF], capture_output=True, text=True, check=True).stdout)
    script = tmp_path_factory.mktemp("js") / "pins.mjs"
    script.write_text(JS)
    js = subprocess.run(["node", str(script), PINJS], input=json.dumps(ref), capture_output=True, text=True)
    assert js.returncode == 0, js.stderr
    return ref, json.loads(js.stdout)


def test_names_match_the_firmware(both):
    ref, js = both
    for (code, fw_name, _esp), name in zip(ref["pin_names"], js["names"]):
        want = "" if code == 255 else fw_name          # (the box shows nothing for "no pin"; the boot report says "-")
        assert name == want, "pin %d: firmware %r, page %r" % (code, fw_name, name)
    assert js["names"][19] == "PB3" and js["names"][175] == "PK15"


def test_parsing_matches_the_firmware(both):
    ref, js = both
    for (text, fw_stm32, fw_esp), st, esp in zip(ref["pin_parse"], js["parse_stm32"], js["parse_esp"]):
        if text.strip() == "":
            assert st == 255 and esp == 255, "an empty box means no pin"
            continue
        assert st == (None if fw_stm32 < 0 else fw_stm32), "STM32 %r: firmware %d, page %r" % (text, fw_stm32, st)
        assert esp == (None if fw_esp < 0 else fw_esp), "ESP32 %r: firmware %d, page %r" % (text, fw_esp, esp)


def test_device_info_names_the_chip_and_its_pins():
    """DEVICE_INFO from 0.25 on: [name][0][mcu][0][32-byte pin mask] (src/proto/wbp_server.h set_chip); before: [name] alone."""
    import struct
    from wbclient.client import DeviceInfo
    head = struct.pack("<BBBBBII", 1, 0, 0, 25, 0, 0x12345678, 1 << 7)
    mask = bytearray(32); mask[0] = 0x05; mask[31] = 0x80
    info = DeviceInfo(head + b"Windburst RX\0STM32F405\0" + bytes(mask))
    assert info.name == "Windburst RX" and info.mcu == "STM32F405" and info.pins == [0, 2, 255] and "STM32" in info.features
    old = DeviceInfo(head + b"Windburst RX")
    assert old.name == "Windburst RX" and old.mcu is None and old.pins is None
