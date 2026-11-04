# Command line - CLI (0.16.0)

Like Betaflight's CLI, using the app's serial port (USB / UART0). Open a terminal (PuTTY, `screen`, the Arduino IDE Serial Monitor …), choose the link's port and baud (over USB any baud works; UART0 is 230400), type `#` then Enter to enter. Close the app while using the CLI, since the app and the terminal cannot share one port.

```
get [part of a name]        get loop_hz   → general.loop_hz = 4000, allowed: 500 .. 8000
set <section.field> = <value>
    set general.loop_hz = 8000
    set general.board_trim_roll = -2.5     (units as in the app: degrees, not 0.1 degrees)
    set general.board_align = CW90_FLIP    (enum: by name, case-insensitive)
    set general.runaway_takeoff = OFF      (bool: ON / OFF)
    set pins.motor = 4,5,6,7,255,255,255,255   |   set pins.motor[2] = 16
    set beeper.conditions = GYRO_CALIBRATED,RX_LOST   (bit set: by name, or NONE)
    set general.usb_name = "Quad tôi"       (string: in double quotes, UTF-8)
diff [section]   set lines for everything that differs from the defaults (paste them back into the CLI to restore that exact configuration)
dump             every setting, as set lines
defaults [section]   back to defaults (RAM only: type save to keep)
save   reboot   status   version   help   exit
```

- A change goes through exactly the same steps as a change from the app (`wbp::Server::commit_section`): clamp to limits, recompute derived values, apply immediately or mark "reboot needed". After `set`, the CLI prints back the value the firmware actually holds.
  - Example: `pid.p` is overwritten by the slider computation if `pid.slider_mode` is not manual tuning, as in the app.
- While the craft is armed, `set` / `save` / `defaults` are not allowed.
- Pins are shown as numbers: GPIO on the ESP32; on the STM32 port × 16 + pin (PB3 = 19).
- The field name table is generated from the schema (`tools/gen_proto.py` → `src/proto/wbp_fields.h`). A new setting added to the schema is immediately available in the CLI.
- The CLI only opens when `#` arrives **between two frames** (of the WBP or MSP protocol), so the app and BLHeliSuite never open it by accident.
- While in the CLI, the firmware does not send streams out of the port. `exit` returns to the app protocol.

## Tests
- `test_cli`:
  - field table: each field's offset matches the struct exactly;
  - `get` / `set` for each type (scaled numbers, enum, bool, array, array element, bit set, string, secret field);
  - rejected values;
  - `diff` pasted into another board gives the same configuration;
  - `dump`; while armed; live sections; `save` then reload;
  - `defaults`; flash failure;
  - echo, backspace, CR LF.
- Mutants: `python3 tools/mutate.py cli:` (9).
- The part going through the real serial port (`LinkTask`) has only been compiled, **not yet tried with a terminal on the board**.
