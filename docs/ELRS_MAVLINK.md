# MAVLink over ExpressLRS (ELRS)

How to get Windburst's MAVLink telemetry to Mission Planner / QGroundControl using ELRS hardware. Written from the ELRS documentation (`expresslrs.org/software/mavlink`), the ELRS source (receiver: `src/src/rx-serial/SerialMavlink.cpp`; TX: `src/lib/MAVLink/MAVLink.cpp`), Betaflight (`rx/mavlink.c`, `telemetry/mavlink.c`) and ArduPilot (`GCS_MAVLINK::handle_radio_status`), cross-checked against Windburst firmware 0.40.0. **Not yet tried on a real board.**

## Summary: which methods work

| Method | Control commands go over | Telemetry goes to | With Windburst |
|---|---|---|---|
| 1. Normal ELRS (CRSF) | ELRS, CRSF | transmitter screen (EdgeTX "Discover new") | **Works** (since 0.8.0, `docs/RX.md`) |
| 2. A second ELRS pair as a telemetry radio | main receiver (ELRS CRSF or another type) | Mission Planner / QGroundControl | **Works, view only** (not yet tried on a board) |
| 3. ELRS in MAVLink mode: one link for both control and telemetry | ELRS, MAVLink | transmitter + ground station | **Works since 0.39.0** (not yet tried on a board) |

Method 3 is the recommended one if you need a ground station: one receiver, one UART, no second radio.

## Warning: firmware and settings must match the Link Mode

In MAVLink mode the ELRS receiver **no longer sends CRSF**. It sends the control commands as `RC_CHANNELS_OVERRIDE` messages. Therefore:
- Firmware **older than 0.39.0** cannot read this message: switching Link Mode to MAVLink means **losing control** (cannot arm; in flight it goes into failsafe).
- Since 0.39.0: when switching Link Mode to MAVLink you **must** change `receiver.protocol` to `MAVLINK` at the same time, and vice versa. If the protocol does not match, the receiver still sends but the flight controller recognises no frames. Before flying, check the Receiver tab: "Link: yes", about 100 frames/s.

## Method 1: normal ELRS, telemetry to the transmitter

ELRS receiver left on the CRSF protocol (default), two wires to a UART. In the app, Receiver tab: protocol CRSF, TX pin set, `receiver.telemetry` on. On the EdgeTX transmitter: Telemetry → "Discover new" will find battery, current, mAh, attitude, GPS, altitude, vario, flight mode. Details: `docs/RX.md`, section "CRSF telemetry".

This method has no ground station on a computer.

## Method 3: one ELRS link for both control and telemetry (since 0.39.0)

### Requirements
- **ESP-based ELRS TX module and receiver** (the kind with WiFi). Receivers with an STM32 chip cannot run MAVLink mode.
- **ELRS firmware 3.5.0 or newer** on both. If you use the backpack's WiFi, the TX Backpack must be 1.5.0 or newer.
- **Windburst 0.39.0 or newer** (flash the firmware and rebuild the app, since the schema changed).
- Preferably 2.4 GHz or LR1121 hardware; old 900 MHz hardware (SX127x) carries very little telemetry.

### Step 1: set up ELRS
1. Power the transmitter and receiver, wait until the ELRS Lua script shows a **C** in the top right corner (connected).
2. Set the receiver's serial protocol to **MAVLink**.
   - ELRS 3.x: Lua script → Other Devices → select the receiver → serial protocol = MAVLink.
   - ELRS 4.x: skip this step, the receiver switches by itself when the TX changes Link Mode.
3. Power off the receiver (Link Mode cannot be changed while connected), then set **Link Mode = MAVLink** in the Lua script's main menu. The telemetry ratio becomes 1:2 automatically and cannot be changed.
4. Choose the packet rate from the bandwidth table below.

### Step 2: wiring
Exactly like a CRSF receiver: two wires to the receiver UART (pins `rx_rx`, `rx_tx` in the Pins and ports tab, group "Serial receiver").

| Flight controller (Windburst) | ELRS receiver |
|---|---|
| `pins.rx_rx` (UART RX) | TX |
| `pins.rx_tx` (UART TX) | RX |
| 5 V, GND | 5 V, GND |

This time **both wires are needed**: the incoming wire carries the sticks and `RADIO_STATUS`, the outgoing wire carries telemetry.

### Step 3: set up Windburst
- Receiver tab: **Protocol = MAVLink (ELRS in MAVLink mode)**, **Telemetry to the transmitter = on**. Channel order stays as for ELRS CRSF (AETR).
- Pins and ports tab, "Telemetry (MAVLink)" block: only the **System ID** (leave at 1, ELRS's default Target SysID) and each group's **send rate** are used. This block's protocol, baud, TX / RX pins are unused; leaving the protocol at NONE is fine.
- The receiver UART baud is automatically **460800** (the speed ELRS uses in MAVLink mode), no need to set it.

Save and restart. The startup report must have two lines:
```
rx: MAVLINK on UART2, MAVLink telemetry on its TX pin
telemetry: MAVLink 2, system id 1, on the receiver's UART (UART2 TX GPIOnn, 460800 baud)
```
If you see `telemetry: FAILED - the receiver's UART has no TX pin (pins.rx_tx)`: the receiver's TX pin is not set. The sticks still work, there is just no telemetry.

### Step 4: transmitter
Sticks, switches and modes are set up as usual in EdgeTX. The ELRS TX module converts MAVLink into CRSF telemetry for the transmitter by itself: Telemetry → "Discover new" finds battery, GPS, attitude, altitude / vario, flight mode. Mode names follow ArduCopter's codes, with a `*` added while disarmed (for example `STAB*`). ELRS only reads frames with component id 1, which is exactly what Windburst sends.

### Step 5: ground station
As step 4 of method 2 below: plug USB into the TX module (460800 baud), or via the backpack's WiFi (UDP 14550).

### The firmware throttles its send rate by itself
The ELRS receiver has a 1 kB telemetry buffer. When the buffer is full new bytes are lost, and the MAVLink frames containing them are corrupted. Every 10 ms the receiver sends `RADIO_STATUS`, where `txbuf` = % of the buffer still free. Windburst uses it in two ways (`telemetry/mavlink_telemetry.h`):
- **Like Betaflight** (`mavlink_min_txbuff` = 35): when `txbuf` ≤ 35 % nothing is sent, waiting for the receiver to drain.
- **Like ArduPilot** (`stream_slowdown_ms`): `txbuf` < 20 → each group's period gets 60 ms longer, < 50 → 20 ms longer (up to about 2 s); > 95 → 40 ms shorter, > 90 → 20 ms shorter. HEARTBEAT alone stays at 1 Hz and STATUSTEXT is not slowed, as in ArduPilot. Missing HEARTBEAT for too long makes the ground station report a lost connection.

Tried on a computer with a simulated buffer: packet rate 250Hz (about 360 B/s downlink) with the default rates (about 1400 B/s) lost no byte in 10 seconds, the buffer peaked at about 870 / 1024 bytes. HEARTBEAT still kept 1 Hz, the other groups slowed down. For smoother ATTITUDE at low packet rates, set the send rates from the table below so it does not have to wait on throttling.

### Limits
- Since 0.40 the ground station can **upload / download missions, change mode, RTL, start a mission, disarm** over this same link; **arm** when `telemetry.remote_arm` is on (`docs/MAVLINK.md`, section "Receiving from the ground station"). Parameters not yet: Mission Planner will try to load parameters and time out; ignore that message.
- The `target_system` of `RC_CHANNELS_OVERRIDE` is not checked (as in Betaflight): on this wire the only sender is the receiver.
- Signal loss: the receiver stops sending `RC_CHANNELS_OVERRIDE`; after `receiver.rx_timeout_ms` the flight controller goes into failsafe as with any other protocol.

## Method 2: a second ELRS pair as a telemetry radio

Like a 433 / 915 MHz telemetry radio set (SiK), but using ELRS hardware. This pair **carries telemetry only**. Control commands still go through the main receiver as usual. Use it when the main receiver is not ELRS, or you want to keep the control link on CRSF.

### Requirements
- **ESP32-based ELRS TX module** (with WiFi). It plugs into the computer over USB, it does not need to be attached to a transmitter. A module inside a transmitter (for example a TX16S with internal ELRS) cannot be plugged in over USB: you must use the backpack's WiFi.
- **ESP-based ELRS receiver**, **ELRS firmware 3.5.0 or newer**, as for method 3.
- **A free UART** on the flight controller.
- **A separate binding phrase** for this pair, different from the control pair.

### Step 1: set up ELRS
1. Attach the TX module to the transmitter, select **Model ID 0** in EdgeTX. When running without a transmitter, ELRS uses the Model ID 0 settings.
2. Do step 1 of method 3 (serial protocol MAVLink, Link Mode MAVLink, packet rate).

### Step 2: wiring
| Flight controller (Windburst) | Second ELRS receiver |
|---|---|
| `telemetry.tx_pin` (UART TX) | RX |
| `telemetry.rx_pin` (UART RX) | TX |
| 5 V, GND | 5 V, GND |

The RX wire carries commands and missions from the ground station (0.40); without it, view only. On STM32 `rx_pin` must be set, because the firmware needs this pin to start the UART. This receiver's `RC_CHANNELS_OVERRIDE` and `RADIO_STATUS` are ignored: this pair cannot control the craft, and there is **no rate throttling** - you must pick the send rates yourself from the bandwidth table.

### Step 3: set up Windburst (Pins and ports tab, "Telemetry (MAVLink)" block)
- Protocol: **MAVLink**. Baud: **460800**. TX / RX pins: per step 2.
- System ID: **1**. Send rates: per the bandwidth table.
- If the main receiver is also ELRS in MAVLink mode (method 3) with `receiver.telemetry` on, telemetry goes through the main receiver and this block no longer uses the TX pin (the startup report says `telemetry.tx_pin is not used`).

Save and restart. The startup report must have the line `telemetry: MAVLink 2, system id 1, UARTx TX GPIOnn, 460800 baud`. How the UART and pins are chosen: `docs/MAVLINK.md`.

### Step 4: connect the ground station (shared by methods 2 and 3)
- **Over USB (try this first):** plug the TX module into the computer, power the craft.
  - Mission Planner: select the TX module's COM port, baud **460800**, press Connect.
  - QGroundControl: turn off all auto-connect, create a Serial Comm Link, baud 460800. **On Windows you must tick "Force DTR Low"** (Advanced Settings); otherwise the TX module restarts or hangs. This option is new, so ELRS recommends the QGC daily build.
  - A module inside a transmitter cannot be plugged in over USB: use WiFi.
- **Over WiFi (needs a TX Backpack):** in the Lua script, Backpack → **Telemetry = WiFi**. Do **not** turn on WiFi with the "Enable Backpack WiFi" item: that puts the backpack into firmware update mode, not MAVLink.
  - The computer joins the network `ExpressLRS TX Backpack XXXXXX`, password `expresslrs` (the backpack is 10.0.0.1).
  - Mission Planner: select UDP, port **14550**. QGroundControl: turn on UDP auto-connect.
- Mission Planner: turn on `telemetry.mp_home_item` so that Read WPs does not lose the first command (`docs/MAVLINK.md`, section "Using with Mission Planner").

### Step 5: what you see on the ground station
- **Shown:** HUD (attitude, altitude, speed), GPS position on the map, battery, flight mode (by ArduCopter mode code), STATUSTEXT lines (startup report, "Arming blocked: ...", "Armed").
- **Method 3 and method 2 (with the telemetry UART's RX wire connected):** upload / download missions, change mode, RTL, start a mission, disarm, arm if `remote_arm` is on (`docs/MAVLINK.md`).
- **Not yet:** parameters.

## Bandwidth: choosing the send rates

Method 3 throttles itself, but rates set to fit the capacity give more even data (no bursts followed by waits). Method 2 does **not** throttle, so the total sent must be clearly below the capacity. Otherwise the receiver's buffer fills and frames are dropped (the MAVLink page on the backpack's WebUI has a "Buffer overflows" counter).

**How much Windburst sends** (MAVLink 2: 12 bytes of framing + payload):

| Message | Bytes / frame | Rate (field) | Default | B/s at default |
|---|---|---|---|---|
| HEARTBEAT | 21 | always 1 Hz | 1 | 21 |
| ATTITUDE | 40 | `rate_attitude` | 10 | 400 |
| GLOBAL_POSITION_INT + GPS_RAW_INT | 40 + 64 | `rate_position` | 5 | 520 |
| VFR_HUD | 32 | `rate_hud` | 4 | 128 |
| SYS_STATUS + BATTERY_STATUS | 55 + 66 | `rate_status` | 2 | 242 |
| RC_CHANNELS | 54 | `rate_rc` | 2 | 108 |
| **Total** | | | | **≈ 1420** |

Bytes / frame is the maximum (GPS_RAW_INT, SYS_STATUS, BATTERY_STATUS also send the MAVLink 2 extensions). MAVLink 2 trims trailing zero bytes, so in practice less is usually sent. GPS_RAW_INT is only sent when there is a GPS, BATTERY_STATUS only when the battery is measured. In method 3, RC_CHANNELS only repeats sticks the transmitter already knows: `rate_rc` can be 0.

**How much ELRS can carry** in the craft → ground direction, per the ELRS table at a 1:2 ratio. The "suggested" column is a rate set that stays under about 70 % of capacity:

| Packet rate (2.4 GHz) | Capacity | Suggested `attitude / position / hud / status / rc` | Windburst sends |
|---|---|---|---|
| F1000 (SX128x), K1000 (LR1121) | ~2375 B/s | default 10 / 5 / 4 / 2 / 2 | ≤ 1420 B/s |
| 333Hz Full | ~1470 B/s | 5 / 2 / 2 / 1 / 1 | ≤ 670 B/s |
| 500Hz | ~1185 B/s | 5 / 2 / 2 / 1 / 1 | ≤ 670 B/s |
| 250Hz | ~590 B/s | 2 / 1 / 1 / 1 / 0 | ≤ 360 B/s |
| 100Hz Full | ~435 B/s | 1 / 1 / 0 / 1 / 0 | ≤ 290 B/s |
| 150Hz | ~350 B/s | not recommended: the smallest set (1 / 1 / 0 / 1 / 0) already reaches 82 % | – |
| 50Hz | ~110 B/s | not enough | – |

A high packet rate carries more but has shorter range. In method 3 the packet rate is also the stick update rate: choose it for flying first (range, latency), then fit the telemetry rates to it. In method 2, if the telemetry pair loses signal you only lose the picture on the ground station, so 333Hz Full is a balanced choice. 900 MHz hardware: see the table on the ELRS site; at 200Hz (~470 B/s) use the set 1 / 1 / 1 / 1 / 0 (≤ 320 B/s).

## What to check when running for real
- Method 3:
  - The startup report has the two lines `rx: MAVLINK ...` and `telemetry: MAVLink 2 ... on the receiver's UART`.
  - Receiver tab: "Link: yes", about 100 frames/s, channels follow the sticks.
  - "Quality" in the Receiver tab equals the LQ in the Lua script (taken from `RADIO_STATUS`).
  - Turn off the transmitter: signal lost, failsafe exactly as configured.
- Ground station: the HUD follows when tilting the craft, GPS and battery are right, HEARTBEAT does not drop out (no lost-connection message).
- Transmitter (method 3): "Discover new" finds battery, GPS, flight mode.
- If using a backpack: the MAVLink page on the WebUI (http://10.0.0.1) shows "Downlink packets" rising and "Buffer overflows" staying at 0. In method 2, if overflows rise, lower the send rates; in method 3, rising overflows are a throttling bug: send me the numbers and the packet rate.
- Method 2, two ELRS links on the same 2.4 GHz band on one craft: place the two receivers and antennas far apart, and check whether the control link's LQ drops when the telemetry pair is turned on.

## Still missing
- Answering parameter requests (`PARAM_REQUEST_LIST`) so the ground station does not have to time out.
