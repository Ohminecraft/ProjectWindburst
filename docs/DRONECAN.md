# DroneCAN (0.15.0, barometer, battery and ESC added in 0.17.0)

The ESP32 (S3 / plain) uses its built-in TWAI controller plus an external 3.3 V CAN transceiver (SN65HVD230, TJA1051T/3 …). The two pins are wired as follows: `pins.can_tx` to the transceiver's D (TXD) pin, `pins.can_rx` from its R (RXD) pin. The bus needs a 120 Ω termination resistor at both ends; most CAN GPS modules already have one built in.

| Setting (section `can`, needs a reboot) | Meaning |
|---|---|
| `mode` | `OFF` / `DRONECAN` |
| `bitrate` | 125k / 250k / 500k / **1M** (DroneCAN modules use 1 Mbit/s) |
| `node_id` | the board's node id, default 10 (same as ArduPilot) |
| `dna_server` | assigns node ids to modules that request one (dynamic node id allocation), on by default |
| `gps.provider = DRONECAN` | GPS takes its fix from the bus (Fix2 + gnss.Auxiliary) |
| `sensors.mag_model = DRONECAN` | compass taken from the bus (MagneticFieldStrength2). Orientation: `mag_align` CW0 means the arrow on the module points forward |
| `sensors.baro_model = DRONECAN` (0.17) | barometer taken from the bus (air_data.StaticPressure, temperature from StaticTemperature). `AUTO` = BMP280 / BMP388 on I2C as before |
| `battery.source = DRONECAN` / `battery.current_source = DRONECAN` (0.17) | battery voltage / current taken from a power module on the bus (power.BatteryInfo). mAh is computed by integrating current, as with the ADC sensor. `AUTO` never selects DRONECAN by itself |
| `motors.protocol = DRONECAN` (0.17) | ESCs on the bus: esc.RawCommand, value 0 (stop) … 8191, ESC index = motor slot. No motor pins needed |

## Firmware implementation
- `src/hal/can/dronecan.h`: the transport layer (CAN id, tail byte, frame splitting, CRC seeded by the data type signature) and bit encoding, done the same way as libcanard. Hardware-independent.
- `dronecan_msgs.h`: NodeStatus, GetNodeInfo, dynamic_node_id.Allocation, gnss.Fix2, gnss.Auxiliary, ahrs.MagneticFieldStrength2; since 0.17 also air_data.StaticPressure / StaticTemperature, power.BatteryInfo, esc.RawCommand, esc.Status. Signatures and layouts taken from pydronecan's DSDL.
- `dronecan_node.h`: the board acts as a node.
  - Sends NodeStatus at 1 Hz and answers GetNodeInfo (name `org.windburst.fc`).
  - Node id server: prefers the id the module requests if it is free, otherwise takes the highest free id from 125 downward. A module that reboots still gets its old id (within the same firmware run; the allocation table is not saved to flash).
  - Table of at most 8 nodes, with names (GetNodeInfo is asked once per node).
  - GNSS, compass, barometer and battery are taken from the first node that sends them. If that node is silent for 2 s, another node is used.
  - Pressure outside 1 000 … 120 000 Pa is dropped. Temperature is only taken from the node that is supplying the pressure.
  - esc.Status is stored by ESC index (0 … 7), with a mask of "which ESCs spoke within the last 1 s".
  - `send_esc()`: called by the flight task, has its own transfer id and counters, so it does not contend with the sensor thread calling `update()`. The TWAI driver is itself safe when several tasks send.
- `twai_bus.h`: the ESP-IDF TWAI driver, non-blocking, recovers from bus-off by itself.
- The sensor thread reads the bus every 1 ms. The fix is fed into the GPS layer as an external source (like MSP), the field into the compass via `ExternalMag`, pressure into `BaroSensor` via `ExternalBaro` (`hal/baro/external_baro.h`), BatteryInfo into `BatterySensor::setExternal()`.

### ESCs over CAN (`hal/motor/can_esc.h`)
- `CanEscHw` is a `MotorHw` backend like DShot / PWM. `MotorOutputManager` still handles idle, throttle limit, motor order, motor test. The difference is the value scale: disarmed 0, armed from 1 + idle × 8190 to 8191 × limit, never 0 while flying.
- One write per control cycle, but a RawCommand is only sent every `round(pid_hz / pulse_hz)` cycles, about 400 Hz by default (`motors.pulse_hz` left at 0). Four ESCs = 7 bytes = one frame, about 130 µs at 1 Mbit/s. Eight ESCs = three frames.
- esc.Status (rpm, V, A, °C) goes into `feed_can()` by slot, mapped to logical motors via the order table. Telemetry stays valid for 500 ms (esc.Status is usually only 10 … 50 Hz). Therefore: the motor table shows rpm, the "ESC" battery source still works, and **the motor stall detector works** with CAN ESCs. The gyro RPM filter does not (it needs bidirectional DShot).
- No DShot commands: beacon, direction reversal and enabling extended telemetry are all refused. No motor beeper. Rotation direction is set in the ESC's own configuration.
- ESC passthrough (BLHeli) is skipped: `escPin()` returns "no pin".
- Converting Fix2 to `gps::Solution`:
  - horizontal / vertical / velocity accuracy taken from the covariance (6 or 36 values);
  - HDOP taken from Auxiliary; if absent, PDOP is used instead;
  - GPS time (time of week) taken from `gnss_timestamp`.

### Also fixed two old bugs in the "external" GPS source (MSP / VIRTUAL share this path)
- Bug 1: an injected solution was never counted as "new", so Position Hold and the estimator never received it.
- Bug 2: after a signal loss, the engine jumped to "baud detection" and got stuck there forever.

## App
- Pins tab: "CAN bus" group (TX / RX).
- Sensors tab, "CAN bus (DroneCAN)" block: the setting fields, bus status (frames/s, TX/RX errors, bus-off count, number of ids assigned; since 0.17 also RawCommands sent / dropped and which ESCs are talking), and the node list (id, name, health, uptime, which node supplies GNSS / compass / barometer / battery). The Module block gains a "Barometer type" field.
- Motor tab: the "DroneCAN (ESCs on the CAN bus)" protocol hides the pulse / DShot fields, keeps the Motor idle field, and has an explanatory line. The rpm column comes from esc.Status.
- Power tab: voltage / current source "DroneCAN power module".
- The `can` stream (2 Hz) gains `baro_node`, `batt_node`, `esc_mask`, `esc_sent`, `esc_dropped`; health gains the `CAN` bit.

## Limitations
- Not yet available: module parameter configuration (param GetSet), module firmware update, esc.RPMCommand (RawCommand only), ESC index different from the motor slot (ESCs must be configured with index 0 … n-1 by slot).
- Not on STM32 yet (the H5's FDCAN will come later).
- The TWAI driver's interrupt runs on core 1, the same core as the flight loop, costing a few µs per frame. **Not measured on the board yet.**
- 0.17: section `sensors` gains 1 byte (`baro_model`); the 18-byte (0.13 – 0.16) and 16-byte (before 0.13) versions still load. The enums `esc_protocol`, `batt_source`, `curr_source` gain the value DRONECAN at the end (old numbers unchanged).
- Settings: section `pins` gains 2 bytes. Pin maps from 0.14 (48 bytes), 0.11 (47), 0.8 – 0.10 (46), 0.7.1 (37) still load, since from this version each section keeps four old sizes.

## Testing
- `test_can`:
  - frames produced by the firmware's encoder are **bit-identical** to frames produced by pydronecan (`tools/dronecan_vectors.py` → `test/dronecan_vectors.h`); the decoder reads pydronecan's frames back correctly;
  - busy bus: two GPS units sending interleaved, lost frames, flipped bits (caught by the CRC), abandoned transfers, running out of slots;
  - the node on a virtual bus with two modules: id assignment, names, GNSS / compass, switching to another GPS, a rebooted module keeping its id, full transmit queue.
- `test_fw` group `dronecan`: virtual board with a DroneCAN GPS + compass module.
  - the module is assigned id 125; the `can` stream shows it correctly;
  - heading taken from the module's compass; flies Position Hold on DroneCAN GPS, holding within 0.8 m for 10 s;
  - the module goes silent then speaks again;
  - error cases: missing pins, controller fails to start, pin conflict, id server off, CAN off.
- `test_can` second-node group (0.17): a barometer + power module and four ESCs; implausible pressure is dropped, a second barometer is only listed, BatteryInfo older than 2 s is no longer valid, esc.Status with an index outside the table is dropped, RawCommand as one frame (4 ESCs) / three frames (8 ESCs), values above 8191 are clamped, a full queue is counted.
- `test_motor` group `DroneCAN ESCs`: endpoints (idle, limit, never 0 while armed), motor order, no DShot commands, esc.Status reaching the right motor, 500 ms timeout, 400 Hz rate at a 4 kHz loop, ESCs hearing the right values.
- `test_fw` group `dronecan2`: the whole craft on CAN (GPS + compass, barometer + power, four ESCs, no I2C barometer, no motor pins). Boot report; about 400 RawCommands/s, all ESCs hear them; flies Altitude Hold on the CAN barometer; rpm / temperature / current from esc.Status; current and mAh from the power module; broken prop on motor 2 → stall guard disarms, ESCs receive 0; with CAN off, motors / barometer / battery each state the reason; the power module goes silent then speaks again.
- Mutants: `python3 tools/mutate.py dcan` (13), plus 3 mutants in the GPS / compass layer; 0.17: `python3 tools/mutate.py can2` (23).
- **The TWAI part has not been tested on a real chip**: so far only compiled for the ESP32-S3. Real CAN ESCs (Zubax, Vertiq, Holybro Kotleta, AM32 CAN …) not tested yet: need to check the ESC index, the 0 … 8191 scale, and that the ESC stops by itself when commands are lost.
