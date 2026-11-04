# OSD for digital FPV goggles — section `osd` (firmware 0.21)

The OSD shows in the goggles of digital FPV systems: DJI O3 / O4, Walksnail Avatar, HDZero. The firmware sends it over **MSP DisplayPort**, like Betaflight (`io/displayport_msp.c`, `osd/osd_elements.c`). The goggles draw it themselves with the Betaflight font built into them, so the firmware uses exactly Betaflight's character codes (`drivers/osd_symbols.h`). Analog OSD (MAX7456 chip) is not available yet.

## Wiring and settings
- **Wiring:** the flight controller's TX to the video transmitter's RX, RX to its TX, common GND. The video transmitter asks for the arm state over the RX wire to start recording; without the RX wire the OSD still shows but the goggles do not record by themselves.
- **Settings:** in the **OSD** tab ("Connection" block):

  | Field | Default | Meaning |
  |---|---|---|
  | `type` | `NONE` | `MSP_DISPLAYPORT` = on |
  | `canvas` | `AUTO` | character grid. AUTO: HDZero / Walksnail report their grid themselves (`MSP_SET_OSD_CANVAS`); if not reported, 53 x 20 (DJI). Or pick a fixed one: 53 x 20, 50 x 18, 30 x 16 |
  | `baud` | 115200 | UART speed |
  | `tx_pin`, `rx_pin` | empty | UART pins |
  | `rate` | 10 | redraws per second (1 – 30) |
  | `name` | empty | craft name ("Craft name" element, and the `MSP_NAME` reply) |
  | `pos[24]` | see below | position of each element, Betaflight's encoding: column = bits 0-5, row = bits 6-10, visible = bit 11 |
- **UART selection:**
  - AUTO: a UART still free after GPS, receiver, optical flow, ESC telemetry wire. The OSD is chosen **before** MAVLink telemetry: if only one UART is left, the OSD takes it and MAVLink reports no UART left.
  - TABLE: the UART with the `DISPLAYPORT` function (Pins and ports tab).
- **In the goggles:** select the "Custom" / "Betaflight" OSD type (DJI: canvas mode / custom OSD).
- Changing settings requires saving and rebooting.

## Elements (order in `pos[]`, new ones appended at the end)
RSSI / LQ, battery voltage (7-level battery symbol according to the cell range; blinks when the battery is critical), average voltage per cell, current, mAh, flight mode (ACRO / ANGL / STAB / ALTH / POSH / RESC / LOIT, `FLIP` during crash flip, `!FS!` during failsafe), craft name (upper case), flight time (only counts while armed), power-on time, satellite count (blinks while there is no 3D fix), GPS speed (km/h), direction to home (arrow relative to the craft's nose, 16 directions), distance to home, altitude (estimator), climb / descent rate, throttle, crosshair, artificial horizon (pitch limit 20°, roll 40° as in Betaflight's defaults), warnings, heading, pitch angle, roll angle, ESC temperature (highest), DISARMED text.

**Warnings:** only one is shown, in priority order:
1. `FAILSAFE`
2. `LAND NOW` (battery critical)
3. `LOW BATTERY`
4. `RXLOSS`
5. while disarmed: the first reason arming is blocked, e.g. `GYRO NOT CAL`.

Shown by default: LQ, battery, current, mAh, mode, name, flight time, satellites, speed, home, altitude, throttle, crosshair, warnings, DISARMED. The default layout is for the 53 x 20 grid.

**App:** in the OSD tab, drag an element on the preview screen to move it (or click the element and use the arrow keys), and use the switches in the "Elements" table to hide / show. The symbols on the preview screen are only illustrations of the same width.

## On the wire
- **Each screen:** heartbeat (once per second), clear, one "write string" command per element (row, column, attribute, at most 30 characters), then draw. All are MSP v1 frames of the reply kind (`$M>`), command 182. Attribute `0x40` = blink, done by the goggles.
- **Bandwidth:** one screen is about 0.5 kB, i.e. 5 kB/s at 10 screens per second (115200 baud carries 11 kB/s).
- **Never blocks:** the firmware never hands the UART more than its free space, and only builds a new screen once the previous one has been fully sent.
- **The video transmitter asks back** on the same wire:
  - `MSP_STATUS` / `MSP_STATUS_EX`: answered like Betaflight 4.5 (`msp::Bridge`). The ARM box is set when armed, so the goggles start recording on arm.
  - `MSP_SET_OSD_CANVAS`: the transmitter's grid, only used when `canvas = AUTO`, and only a sensible grid is accepted (20 – 63 columns, 10 – 31 rows).
  - `MSP_NAME`: answered with the craft name.
  - `MSP_FC_VARIANT` ("BTFL"), `MSP_API_VERSION` ...: answered like Betaflight 4.5.
  - ESC passthrough is always refused on this port.

Boot report: `osd: MSP DisplayPort on UART0 TX GPIO43 RX GPIO44, 115200 baud, 53 x 20 (or what the video transmitter asks for), 10 screens/s`, or `osd: FAILED - <reason>`. The TX / RX pins are checked for conflicts with other pins.

Source code: `src/osd/osd.h` (portable), `FlightApp::osd_in()` / `osd_send()` in `src/app/flight_app.h`.

## Verification
- `test_osd` (57 checks): each element compared with how Betaflight draws it (battery symbol, arrows, decimals, blinking), the artificial horizon using Betaflight's integer formula, drawing within the grid, bytes on the wire, slow UART (16 bytes at a time, frames are never cut), the video transmitter's requests.
- `test_fw` group `osd` (`WB_ONLY=osd`): virtual board, 2 s gives 20 screens, the transmitter asks for the arm state and changes the grid, on arming the mode, timer and ARM box change, UART sharing with MAVLink, port table, pin conflicts.
- 28 `osd:` mutants; `test_gui_browser.py::test_osd_tab` (written but cannot run on this machine yet).
- **Not yet tested with real goggles.** To test: DJI O3 (canvas mode), Walksnail, HDZero. Do the goggles show the OSD, are the elements in the right place, does it record video on arm.

## 0.33: more elements, statistics screen, new preview screen

**New elements** (appended to the end of `pos[]`, so `pos` now has 32 slots; settings from older versions are kept, the 8 new elements are hidden by default). Drawn following Betaflight's `osd_elements.c`:

| Element | Example | Notes |
|---|---|---|
| POWER | `198W` | current × voltage; needs battery voltage and current measurement |
| FLIGHT_DIST | `⟷1235m` | distance flown since arming, from GPS speed while there is a 3D fix |
| GPS_LAT / GPS_LON | `10.7654321` | symbol, degrees, 7 decimals, with minus sign (`osdFormatCoordinate`) |
| COMPASS_BAR | `W·┴·N·┴·E` | 9 characters, heading in the middle, each step 1/16 of a turn (`osdElementCompassBar`) |
| SIDEBARS | | two vertical bars of 7 rows, 7 columns from the centre, tick marks at column ±6 (`osdElementHorizonSidebars`) |
| G_FORCE | `1.0G` | magnitude of the measured acceleration |
| BATT_USAGE | `[■■■■■■■◧□□]` | 10 cells, each 10 % of remaining battery; needs `battery.capacity_mah`; blinks when the battery is low |

**Post-flight statistics screen** (`osd.stats`, on by default; like Betaflight's `osdShowStats`):
- After disarming, the goggles show the statistics screen instead of the elements, for 20 seconds or until the next arm. A new arm restarts the figures from scratch.
- The screen has the title `--- STATS ---` and the lines:
  - FLY TIME;
  - MAX SPEED, MAX DISTANCE, FLIGHT DISTANCE (with GPS);
  - MAX ALTITUDE;
  - MIN BATTERY;
  - MAX CURRENT, USED MAH (with current measurement).

**App, OSD tab:**
- **Preview as in the goggles:** the background is a camera image (sky and ground) tilting with the craft. There is a character grid. Each character sits in exactly one cell.
- **Symbols:** redrawn as vector shapes with the same meaning as the Betaflight font, white with a black outline as the goggles show them (`js/gfx/osdglyph.js`). Betaflight's bitmap font is not copied.
- **The numbers shown are what the craft is reporting:** battery, current, GPS, altitude, attitude, throttle, mode. When not connected, example numbers are shown.
- **The artificial horizon and sidebars** are drawn with exactly the firmware's formula.
- **Layout:** the "Screen" block is at the top of the left column; the element list is in the right column.

**Tests:**
- `test_osd`: added groups `[elements 0.33]` and `[statistics]`, 84 checks in total.
- `test_fw` group `osd`: fly, land, disarm and the goggles show the statistics screen.
- `test_proto`: the old 73-byte `osd` record can be read.
- 15 `osd33:` mutants.
