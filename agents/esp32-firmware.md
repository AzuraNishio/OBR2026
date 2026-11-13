# ESP32/ — Sensor pack firmware

Arduino firmware for the ESP32 add-on board ("sensor pack") that gives the
SPIKE hub a laser distance sensor and rear bumper buttons. The hub talks to
it over UART through `Spike/ESP32_bridge.py`.

## Files

- [../ESP32/SPIKE_Bridge_v2/SPIKE_Bridge_v2.ino](../ESP32/SPIKE_Bridge_v2/SPIKE_Bridge_v2.ino) —
  **current firmware**. VL53L1X ToF sensor (Long mode, 50 ms budget,
  continuous 50 ms), two rear bumper buttons (GPIO 5 = L, GPIO 18 = R,
  active-LOW), RGB status LED (15/2/4), UART1 to the hub on GPIO 16 (RX) /
  17 (TX) at 115200 8N1. On sensor timeout it resends the last valid distance
  with a TIMEOUT flag and re-inits the sensor.
- [../ESP32/SPIKE_Bridge_v2.ino](../ESP32/SPIKE_Bridge_v2.ino) — older single-purpose version
  (distance only, prints ASCII over UART2, blinks GPIO 2). Kept for reference.
- `ESP32 Sensor Pack V1 diagram.fzz` — Fritzing wiring diagram.

## UART protocol (v2)

Byte-oriented packets terminated by `STOP_FLAG` (0xA5). Because 0xA0/0xA5 can
appear in payload bytes, they are escaped with `ALT_FLAG` (0xA0):

| Wire bytes | Meaning |
|------------|---------|
| `0xA0 0xA1` | literal `0xA0` (ALT_FLAG) in payload |
| `0xA0 0xA6` | literal `0xA5` (STOP_FLAG) in payload |
| `0xA5` | end of packet |

Outgoing packet (5 bytes, sent every loop ≈ 100 ms):

| Byte | Content |
|------|---------|
| 0 | distance low byte |
| 1 | distance high byte |
| 2 | 0x00 valid / 0x01 timeout (last-valid distance resent) |
| 3 | rear button L (0/1) |
| 4 | rear button R (0/1) |

The ESP32 also echoes back the last packet it received from the hub
(8-deep ring buffer, `tick_listener()`), so the hub can send packets too —
the Python side has `send_packet` commented out but the escaping logic
(`alt_map`) exists on both sides.

## Hub side ([../Spike/ESP32_bridge.py](../Spike/ESP32_bridge.py))

- `ESP32_bridge` — `UARTDevice(port, 115200, power_pin=1)`; 8-packet ring
  buffer, `tick_packet_receiver()` drains UART, `get_packet()` returns the
  newest complete packet.
- `ESP32_distance_sensor` — decodes distance (`low | high<<8`, 9999 when
  empty/zero) and button states.
- `Esp32SensorPack` — wrapper used by the robot: tries `UltrasonicSensor`,
  then `ColorSensor`, then falls back to `ESP32_distance_sensor`, so the same
  port works with or without the ESP32 attached. `laser_distance()` and
  `rear_sensor_pressed()` are async and awaited by callers.

## Notes for agents

- Firmware changes must keep the escape/stop-byte scheme in sync with the
  Python decoder; both sides define the same flag constants.
- Distance is in millimeters (VL53L1X Long mode, up to ~3600 mm).
- `room3_const.U = 300` (mm per room cell) and `r3_real_robot.measure()`
  assume this packet's distance semantics.
- Known hub-side decode bug ([../Spike/ESP32_bridge.py](../Spike/ESP32_bridge.py)):
  `distance()` returns 9999 when the packet's first byte is 0, discarding valid
  distances that are multiples of 256 mm (256, 512, 768…). The timeout flag
  (packet byte 2) is ignored, so stale last-valid distances are
  indistinguishable from fresh ones.
- No build tooling is configured here; use Arduino IDE / arduino-cli with the
  VL53L1X (Pololu) library.
