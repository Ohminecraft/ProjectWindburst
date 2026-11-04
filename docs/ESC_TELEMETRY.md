# ESC telemetry over a dedicated wire (0.11.0)

BLHeli_32, AM32 and KISS ESCs have a **T** (telemetry) pin. Wire the T pins of all ESCs together to the **RX of one UART** on the flight controller. Betaflight calls this feature "ESC sensor". Code: `src/hal/motor/esc_sensor.h`.

## 1. Setup
1. Motor tab: protocol **DShot** (the telemetry request bit lives in the DShot frame; PWM / Oneshot do not have it), enable **"ESC telemetry wire (KISS)"**.
2. Pins tab: **ESC telemetry** group, RX field: the pin wired to the T wire.
3. Serial port:
   - AUTO: the firmware takes the UART left free after GPS (UART1), receiver (UART2) and MSP optical flow. On the ESP32-S3 when the app connects over the chip's USB this is usually **UART0**.
   - Table: choose the function "ESC telemetry (dedicated wire)" for one UART.
4. Save and reboot. Boot report: `esc tlm: KISS telemetry on UART0 RX GPIO44, 115200 baud`.

## 2. How it runs
- 115200 baud, receive only. No requests for the first 5 seconds (ESCs still booting), like Betaflight.
- Every 1 ms tick of the sensor thread: the next DShot frame to **one** motor carries the telemetry bit. That ESC returns 10 bytes: temperature (°C), voltage (0.01 V), current (0.01 A), mAh, eRPM / 100, CRC8 (polynomial 0x07). On a reply it moves to the next motor. After 100 ms without a reply it moves on (counted as "no reply"). A bad CRC is dropped (counted).
- 4 ESCs: one round takes about 4 - 5 ms. Enough for:
  - battery: current source = ESC, sum of the 4 ESCs; voltage takes the highest value;
  - per-ESC temperature and current (the "Per motor" table in the Motor tab);
  - RPM for display.
  **Not** fast enough for the RPM filter and dynamic idle (those need bidirectional DShot, one reply per frame). When bidirectional DShot is on, RPM still comes from bidirectional DShot.
- Motor tab, line "ESC telemetry wire, per second": good replies, CRC failures, ESCs not replying. Normal: a few hundred replies, 0 failures, 0 not replying. One ESC not replying costs 100 ms per round, i.e. about 10 times per second.
- Health `ESC_TELEMETRY`: set when a good reply has been received within the last second.

## 3. Verification on PC
- `test_motor` group "ESC telemetry wire":
  - CRC8 and frame decoding;
  - polling round (5 s wait, timeout, bad CRC, stray bytes while ESCs boot);
  - the telemetry bit goes into the right hardware slot for the logical motor, one frame at a time, also while disarmed; Oneshot has no bit.
- `test_fw` group `escwire`: virtual board with a shared telemetry wire (virtual ESCs reply when their frame carries the bit). Checks:
  - boot report, free UART;
  - no requests during the first 5 s;
  - about 780 replies per second;
  - per-ESC temperature / current in motor order, battery current, RPM during the motor test;
  - one silent ESC: 10 no-replies per second;
  - corrupt frames;
  - error cases: Oneshot, no pin, incomplete port table, pin conflict.
- 5 mutants (request immediately without waiting, no timeout, bit in the wrong slot, reply going nowhere, wrong UART chosen): all caught.

**Not tried on the board yet.**
