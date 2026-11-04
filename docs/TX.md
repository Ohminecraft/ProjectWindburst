# DIY transmitter (ESP32-C5 + nRF24) and nRF24 receiver — 0.38

A home-made transmitter for Windburst: one ESP32-C5 DevKitC-1 board, two joysticks, one nRF24L01(+) module. On the drone side, the flight controller receives with an nRF24 module plugged into the second SPI bus (receiver protocol `NRF24`). Both ends run this project's firmware; the app connects to either (when the transmitter is plugged in, only the transmitter's tabs are shown).

Design: `docs/superpowers/specs/2026-10-30-tx-nrf24-design.md`. Plan and the decisions made along the way: `docs/superpowers/plans/2026-10-30-tx-nrf24.md`.

**Not yet tested on hardware.** Everything below has run on the PC (the nRF24 driver with a register-level simulated chip, both real firmwares in the same simulated "air" with packet loss). Board test steps: end of the file.

## 1. Hardware

### Transmitter (ESP32-C5 DevKitC-1)
Default pins (taken from the old project `Desktop/ESP32/ProjectWindburst`, changeable in the app):

| Part | Pin |
|---|---|
| joy1 X / Y (roll / pitch) | GPIO4 / GPIO5 |
| joy2 X / Y (yaw / throttle) | GPIO3 / GPIO2 |
| nRF24 SCK / MISO / MOSI | GPIO7 / GPIO6 / GPIO8 |
| nRF24 CSN / CE | GPIO24 / GPIO23 |

- Joystick: the potentiometer's wiper goes to the ADC, its two ends to 3.3 V and GND.
- nRF24: supply **3.3 V** (not 5 V), add a **10 µF capacitor** right at the module's two power pins (nRF24 modules are very picky about power; a missing capacitor is the most common cause of signal loss). Modules with PA/LNA need a strong 3.3 V supply.
- The app connects through **either port** on the DevKit: the "UART" port (CH343 / CP2102 USB-UART bridge chip, **230400 baud** = the app's default) or the "USB" port (the C5's own USB Serial/JTAG, any baud). The firmware listens on both and replies on the port the app just spoke on.
- If the app sees no port: that is a Windows issue, not the firmware. Open Device Manager while plugging in the board: a "USB Serial" device with an exclamation mark (not under Ports) = no driver yet. WCH chips (CH340 / CH341 / CH343, VID 1A86) need WCH's driver (CH341SER / CH343SER) from wch-ic.com; after installing, replug and a COM port appears. The C5's "USB" port (VID 303A) uses Windows' built-in driver. A charge-only cable (no data lines) also makes the port not appear.

### Flight controller (ESP32-S3)
The nRF24 module sits on the **second SPI bus** (HSPI), with 5 dedicated pins: `nrf_sck`, `nrf_miso`, `nrf_mosi`, `nrf_cs`, `nrf_ce` (Pins and ports tab, "nRF24 receiver" group). This bus is also the bus for the SPI-mode SD card, the PMW3901 sensor and the LED strip:
- with an SPI-mode SD card or a PMW3901 → the nRF24 receiver does not run, boot report: `rx: FAILED - the second SPI bus is taken (...)`;
- with an LED strip → the receiver runs, the LED strip does not (`led: FAILED - the second SPI bus is taken (the nRF24 receiver)`).

The STM32 builds do not have the nRF24 receiver yet (`rx: FAILED - the nRF24 receiver is not in the STM32 builds yet`).

## 2. Flashing the transmitter firmware
```
pio run -e tx_c5 -t upload          (from PowerShell the first time: pioarduino's RISC-V toolchain installer does not run in Git Bash)
```
Boot report (the transmitter's System tab): `radio: nRF24 found (...)` or `NOT FOUND` (wiring, power, capacitor), `sticks: ADC pins ...`, `settings: NVS`.

## 3. First use
1. **Stick calibration** (Joystick tab → Start calibration): push each stick fully in all four directions, release to centre, press Done. An axis not pushed far enough (under 500 ADC units) keeps its old calibration. The transmitter saves to flash by itself.
2. **Axis mapping** if needed (default: roll = joy1 X, pitch = joy1 Y, throttle = joy2 Y, yaw = joy2 X), **reverse** any axis that runs backwards (watch the channels in the Overview tab: pushing roll right must give close to 2000).
3. **Throttle stick mode** (Joystick tab) — the joysticks self-centre:
   - **ALT / POS**: throttle = stick position, centre = 1500. Use when the drone flies ALT HOLD / POS HOLD (centre = hold altitude).
   - **Manual** (default): pushing the stick up increases throttle gradually (rate: `throttle_rate` % per second), releasing holds the throttle, pulling down decreases it. Within 15 % around centre the stick does not change throttle (cheap sticks often sit a few % off-centre). Use for ANGLE / ACRO. When the drone reports disarmed (or has never been heard), throttle is always 1000: arming with the sticks and letting go does not make the drone take off. If contact with the drone is lost in flight (the drone reported armed, then went silent): throttle **stays** where it was, not 1000, not rising or falling, until the drone is heard again.
   - The Overview tab warns when the throttle mode does not match the flight mode the drone reports.
4. **Flight controller**: Receiver tab → protocol `nRF24 (Windburst DIY transmitter)`, set the 5 nRF24 pins (Pins and ports tab), save and reboot. Arming with sticks: Mode tab → "Stick arming" (throttle and pitch fully down, yaw and roll to opposite sides, hold 1 second). Without switches, the flight mode is selected with an always-on range on AUX1 (channel 5 is always 1000: range 900..1100).
5. **Bind**: put the transmitter right next to the drone.
   - Drone never bound: power the drone, within 60 seconds press "Bind with drone" in the transmitter's Radio tab.
   - Drone already bound to another transmitter: press "Bind nRF24" in the drone's Receiver tab (listens for 30 seconds), then "Bind with drone" on the transmitter.
   - Done: the transmitter's Radio tab shows "bound", the drone's Receiver tab shows "bound"; both save the new address by themselves. Drone boot report: `rx: bound to ...`.

## 4. Radio protocol (`src/radio/wb_radio.h`)
- nRF24 Enhanced ShockBurst: hardware 16-bit CRC, auto-ACK, dynamic payload, **ACK payload** (the drone's telemetry rides on ACK packets).
- Default 250 kbps, maximum power, **100 packets/s**; selectable 50 / 100 / 150 packets/s, 250 kbps / 1 / 2 Mbps. The number of automatic retransmits is computed so that all transmissions of a packet finish within 80 % of the period (100 packets/s at 250 kbps: 3 retransmits).
- 13-byte control packet: type, sequence number, 8 channels × 10 bit (1000..2000 µs), CRC8 DVB-S2. A corrupt packet / bad CRC8 / out-of-range channel drops the whole packet (`rc.bad_frames`); packets with an old sequence number (retransmits) are dropped.
- 7-byte telemetry (ACK): armed, failsafe, flight mode (the running mode, or the mode the pilot selected while still on the ground), battery (cV), link quality on the drone side.
- Bind: a random 5-byte pair address and frequency channel (generated by the transmitter; the address does not start with 0x00 / 0x55 / 0xAA / 0xFF; channels 74..82 = 2474..2482 MHz, between WiFi channel 11 and the top edge of the 2.4 GHz band), exchanged on address "WBND!" channel 83 (2483 MHz), at the lowest power. The drone only accepts an invitation stronger than -64 dBm (RPD: the transmitter must be placed right next to the drone; invitations from other sets far away are ignored). The drone answers by returning the address in an ACK; both switch to the new address / channel; each side only considers itself bound once control packets are flowing there (none within 2 seconds → back to inviting / listening for bind).
- Changing frequency channel / data rate / packet rate (the transmitter's Radio tab): only while the drone is being heard and the drone is not armed (otherwise the fields immediately revert to the running values). Sent on the old settings, both switch after the ACK; no packets on the new settings within 1 second → both return to the old settings. The drone ignores settings-change packets while armed. Both save to flash the settings actually running (confirmed, or the old settings they returned to).
- Not bound: the transmitter sends nothing, the drone accepts no control packets (two unbound sets cannot fly each other's drones via the default address).
- While the drone is armed (or reported armed and then went silent), the transmitter treats itself as armed: the app cannot change settings, reset to defaults, reboot the transmitter or calibrate sticks.
- Every 100 ms both ends read back the nRF24 module's registers; a module that browned out (lost its configuration) is reconfigured immediately.
- Signal loss: as with every receiver, `receiver.rx_timeout_ms` → link lost; on the ground arming is blocked (`RX_LOST`), in the air failsafe engages.

## 5. Limitations
- One fixed frequency channel (no frequency hopping yet). The permitted 2.4 GHz band only goes up to 2483.5 MHz: channels 0..83. Channels 84..125 (2484..2525 MHz) can still be transmitted by the chip but are outside the permitted band in most countries (including Vietnam): do not use them. Version 0.38.0 defaulted to channel 100 (out of band) - fixed in 0.38.1 (default 76, bind picks 74..82).
- Rare case: changing the channel over the air with every ACK lost within 1 second leaves the drone on the new channel (and saved) while the transmitter returns to the old channel → signal lost. Binding again fixes it.
- Old nRF24L01 modules (without the +) and some clones need the ACTIVATE command: the firmware sends it automatically. These do not support 250 kbps: choose 1 Mbps.
- The transmitter has no switches, display, battery or LEDs yet; channels 5..8 are always 1000.
- The nRF24 receiver is not available on STM32 yet.
- Range: not measured yet (a plain module without PA ~ a few tens of metres; PA/LNA modules reach further).

## 6. Verification on PC
- `test_radio` (driver + simulated nRF24 chip + air: registers, packets, wrong address / channel / data rate, packet loss, corrupt packets, retransmit timing; packet encoding, CRC, sequence number, address).
- `test_tx` (sticks, calibration, both throttle modes, radio path: link / link lost / telemetry / bind / bind timeout / channel change / drone not following; app host: commands, streams, boot report, unsaved flag).
- `test_rx` (receiver: packets, corrupt packets, old packets, LQ, bind, 60 s window, settings change, revert).
- `test_fw` groups `nrf24` and `txfly`: a real TxApp flies a real FlightApp: sticks to channels, stick arming, Manual throttle with hands off does not take off, ALT / POS holds altitude, transmitter off → link lost / failsafe, still flies with 30 % packet loss, two sets side by side do not hear each other, first bind, channel change over the air (including after a reboot), telemetry back to the transmitter, SPI bus taken, pins not set.
- App: `test_e2e.py::test_the_simulated_transmitter`, `test_gui_browser.py` (`test_tx_tabs_only`, `test_tx_sticks_live_and_calibrate`, `test_tx_bind_and_throttle_warning`, `test_receiver_nrf24_block`). `wb_sim` has a simulated transmitter on port +2 (`socket://127.0.0.1:5762`), tester commands `stick <1..4> <0..4095>`, `txpower on|off`.

## 7. Board tests (not done yet)
0. (added after review) Unplug the drone's nRF24 module power alone for 1 second and plug it back in while bound: the link must recover by itself; leave the Manual throttle stick slightly off-centre: throttle does not creep; cut the signal for 2 seconds while hovering: throttle does not drop to 1000.
1. Transmitter on the bench: boot report `radio: nRF24 found`; Overview tab: the dot follows the stick, calibration.
2. Drone on the bench (props off): receiver NRF24, first bind within 60 seconds, Receiver tab: LQ ~100 %, 100 packets/s.
3. Arm with the sticks in Manual throttle mode, let go: motors stay at idle only.
4. Walk progressively farther away (props off, someone holding it) until the signal is lost: the Receiver tab reports link lost, coming back restores it.
5. Near a WiFi router: compare LQ on the chosen bind channel (74..82) with channel 40 (middle of WiFi channel 6).
6. Test flight in ALT HOLD with ALT / POS throttle, then ANGLE with Manual throttle.
