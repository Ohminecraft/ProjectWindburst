#!/usr/bin/env python3
"""ESP Web Tools manifests for the web flasher (the release workflow: .github/workflows/release.yml).

    python3 web-flasher/make_manifests.py <site dir> <tag>

The images (site/firmware/windburst-<env>.bin) are whole flash images written at 0x0 (esptool merge_bin)."""
import json, os, sys

BOARDS = [("rx_esp32_s3", "Windburst flight controller (ESP32-S3)", "ESP32-S3"),
          ("rx_esp32", "Windburst flight controller (ESP32)", "ESP32")]

site, tag = sys.argv[1], sys.argv[2]
for env, name, chip in BOARDS:
    m = {"name": name, "version": tag.lstrip("v"), "new_install_prompt_erase": True,
         "builds": [{"chipFamily": chip, "parts": [{"path": "firmware/windburst-%s.bin" % env, "offset": 0}]}]}
    with open(os.path.join(site, "manifest-%s.json" % env), "w", encoding="utf-8", newline="\n") as f:
        json.dump(m, f, indent=1)
    print("manifest-%s.json" % env)
