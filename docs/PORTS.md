# Serial ports (UART) — section `ports` (firmware 0.8)

The ESP32-S3 has three UARTs; each UART can be routed to any GPIO pin, so each device's **pins** are still set in its own group (GPS, serial receiver, optical flow) in the **Pins and ports** tab. The `ports` section only decides **which UART** serves which device.

- `ports.assign = AUTO` (default, as in previous versions): GPS on UART1, serial receiver (CRSF / SBUS / IBUS / Spektrum) on UART2, the MSP optical flow sensor takes a free port (so does MAVLink telemetry, after optical flow and the ESC telemetry wire) (UART1 if there is no GPS, UART2 if the receiver is PPM / PWM or absent, UART0 when the app connects over the chip's USB). The function table is **not** read: whatever an old settings record has in the table changes nothing.
- `ports.assign = TABLE`: `ports.function[0..2]` = the job of UART0, UART1, UART2: `NONE`, `SERIAL_RX`, `GPS`, `FLOW_MSP`, `ESC_SENSOR`, `TELEMETRY` (from 0.19: MAVLink telemetry, `docs/MAVLINK.md`; CRSF telemetry still goes on the receiver's TX pin), `DISPLAYPORT` (from 0.21: OSD for digital goggles, `docs/OSD.md`). A job chosen for two UARTs → the first UART. A device with no UART in the table → boot report `gps: FAILED - no UART has the function GPS ...` (likewise `rx`, `flow`). UART0 is only usable when the app connects over the chip's USB port; on a board with a USB-UART bridge, UART0 is the app link → `ports: WARNING - UART0 carries the app link ...`, and that device gets no UART.
- `ports.baud` is unused: each device chooses its own baud (GPS auto-detects, the receiver per its protocol, optical flow `flow.baud`). The app no longer shows a baud column.
- Changes need save and reboot. The app checks beforehand: duplicate jobs, no UART for the GPS (when GPS is enabled) or for the serial receiver.

Code: `app::plan_uarts()` in `src/app/sync.h` (portable), used in `FlightApp::begin()`.

Verification (PC): `test_app` `[uart plan]` (AUTO, table, UART0 with the app using it / not using it, duplicate jobs, TELEMETRY); `test_fw` group `ports` (`WB_ONLY=ports`): virtual board with GPS on UART2, receiver on UART1, MSP flow sensor on UART0 (app over USB) — all three run; table missing GPS; GPS placed on UART0 while the app uses UART0; AUTO ignores the table. Real page: `test_serial_ports_table`. **Not tried on a real board yet.**
