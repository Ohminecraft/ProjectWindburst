# WS2812 / SK6812 LED strip (0.16.0)

Section `led`:
- `count`: number of LEDs, 0 – 32 (0 = off);
- `brightness`: 5 – 100 %;
- `layout`: `ALL` (every LED the same colour), `NAV` (navigation lights) or `MANUAL` (assign each LED yourself, from 0.22).

The data pin is `pins.led` (Pins tab, Other group).

The firmware sends LED data over the MOSI pin of the **second SPI bus** (HSPI) at 3.2 MHz. Each data bit is 4 SPI bits (`1000` = 0, `1110` = 1), so the precise timing comes from the SPI clock and does not depend on the CPU. Why not RMT: all 4 RMT transmitters of the ESP32-S3 are already reserved for DShot when there are 4 motors. As a consequence the LEDs **cannot be used at the same time as an SPI-mode SD card or a PMW3901** (both also use that bus); on a conflict, the boot report says so. Not available on STM32 yet.

The sensor thread draws 50 frames per second (`app/led_logic.h`), sent outside the lock. Each LED takes about 30 µs on the bus.

| State | Colour |
|---|---|
| failsafe | red, blinking 5 Hz |
| battery empty (critical) | red, double blink every second |
| motor stop / runaway (disarmed) | alternating red – yellow |
| calibrating | cyan, slowly fading up and down |
| disarmed, something still blocks arming | yellow, blinking 1 Hz |
| disarmed, ready | green |
| flying | colour by mode: Acro white, Angle blue, Stabilize cyan, Altitude Hold purple, Position Hold teal, GPS Rescue blinking orange. Battery low (warning): an extra yellow blink every 2 s |
| `NAV`, flying | first half of the strip red (left), second half green (right), the middle LED shows the mode colour, the last LED flashes white |

## `MANUAL` layout (0.22)
Each LED (1 – `count`) has a function `led.fn[i]` and a colour `led.color[i]` (0xRRGGBB). In the app: the **Battery and buzzer** tab, LED block, choose the layout "Assign each LED"; the table has one row per LED, the "Apply to every LED" button assigns the same function and colour to the whole strip, and a preview strip runs exactly the firmware's formula (choose the preview state: disarmed / flying / failsafe).

| Function | What it shows |
|---|---|
| `OFF` | off |
| `STATUS` | the status pattern as in the `ALL` layout (table above) |
| `COLOR` | the LED's colour, steady |
| `MODE` | the flight mode colour (table above), even when disarmed |
| `ARMED` | the LED's colour when armed, off when disarmed |
| `BLINK` | the LED's colour, on for half a cycle and off for the other half |
| `STROBE` | 60 ms flash, twice per cycle |
| `BREATHE` | fades up then down (never fully off, minimum 8 %) |
| `RAINBOW` | running rainbow; the `RAINBOW` LEDs share the colour wheel in order along the strip |
| `LARSON` | a light spot sweeping back and forth over the `LARSON` LEDs (neighbouring LEDs at 20 %) |
| `BATTERY` | full: green, empty: red, mixed in between (by remaining battery %); no battery: off |
| `THROTTLE` | throttle stick low: blue, high: red |

- `led.speed` 1 – 10: one effect cycle lasts 10 s / speed (5 = 2 s).
- `led.warnings` (on by default): on failsafe, battery empty (critical) or a sudden motor stop, the whole strip shows the warning pattern regardless of LED assignments. When off, only the `STATUS` LEDs show warnings.
- By default every LED is `STATUS` in white, so switching to `MANUAL` without assigning anything looks like `ALL`.
- Old led section records (3 bytes, 0.16 – 0.21) can still be read; new fields take their defaults.

Tests:
- `test_app` led strip group: patterns by state; WS2812 bits are encoded then decoded back for comparison.
- `test_fw` group `led`: colours from the craft's real state (ready, Altitude Hold, failsafe); bus taken; pin conflict.
- `test_app`: each MANUAL function, rainbow / sweep over time, warnings taking over the strip, brightness, speed.
- `test_fw` group `led`: MANUAL on the virtual board (fixed colour, throttle up turns the throttle LED gradually red, failsafe).
- `test_gui_math`: the app's preview strip against the firmware in 288 cases.
- Mutants: `python3 tools/mutate.py led:` (24).
- **Not tried with a real LED strip yet.**
