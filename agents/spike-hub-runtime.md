# Spike/ — Hub runtime (Pybricks / MicroPython)

Everything in `Spike/` is compiled onto the SPIKE Prime hub. Constraints from
[AGENTS.md](AGENTS.md) in this docs folder apply: **no third-party libs**,
**≤ 52300 bytes of compiled code**, star imports, cooperative async.

## Boot flow

1. `Spike/__init__.py` imports `main`, `ESP32_bridge`, `V3.NekoMath`, then calls `main.init()`.
2. `main.py`:
   - `FailHandler()` → `Context(fail_handler)` (hub, ports, display, battery).
   - `LineFollowerRobot(...)` with the port map below; retries with
     `ctx.reload_empty_ports()` until every expected device is found.
   - `TreadmilDifferentialDriveRobotModel(ctx, robot)` (odometry).
   - `FollowerAlgorithm(ctx, model, robot)`.
   - If `isTest()` (TestSuite.py, currently hardcoded `False`) runs the test
     suite instead of the follower.
   - `run_task(multitask(follower_routine(), ctx.routine()))` — follower ticks
     forever; `ctx.routine()` refreshes the fail-handler display and does an
     emergency stop when a sensor sees RED.

## Port map (main.py)

| Port | Device |
|------|--------|
| A | Left drive motor (CLOCKWISE) |
| B | Right drive motor (COUNTERCLOCKWISE) |
| C | Left color sensor |
| D | Right color sensor |
| E | Claw motor (`garra_motor`) |
| F | ESP32 sensor pack (`Esp32SensorPack`) |

Wheels: 40 mm diameter, 110 mm track. Sensor is 60 mm ahead / 15 mm right of
the center of rotation.

## Core classes

### Context ([../Spike/Scaffolding.py](../Spike/Scaffolding.py))
- Owns `PrimeHub`, speaker, imu, display, buttons, battery, `StopWatch` clock.
- `time()` in seconds, `time_ms()` in ms; `wait(s)` busy-wait coroutine.
- `setup_ports()` probes every port with `connect_port_device(port)` from
  [../Spike/libs/Utills.py](../Spike/libs/Utills.py) → tries Motor, then AzColorSensor, then
  Esp32SensorPack; stores `(label, device)` per port.
- `get_port_of_type(port, type)` raises `WrongPortError` on mismatch.
- Display helpers: `indicate_port`, `show_string_encoded_img` (5×5 pixel
  strings, chars `0-9.A-V`), `hub_info_dump` (battery/ports at boot).
- `get_waiter(duration)` → truthy waiter object usable as `if w:` in loops.
- `throw(e)` routes to FailHandler.

### LineFollowerRobot ([../Spike/LineFollowerRobot.py](../Spike/LineFollowerRobot.py))
- `__getattr__` forwards unknown attributes to `drive_base`.
- Wraps motors with explicit direction; builds `AwDriveBase`.
- `straight(distance, speed=None)` uses `modified_straight_speed` when set
  (used by room 3 to speed up); `turn(angle)` overshoots by 1° to settle.
- Claw: `garraUp()` / `garraDown()` (motor E, guarded by `garra_up` flag).
- Sensor helpers: `is_seeing_any_black()`, `is_seeing_any_silver()`,
  `black_dir()`, `alignOnLinePerTime()`, `driveUntillConditions()`,
  `move_time()`, `signal_blink()`.

### TreadmilDifferentialDriveRobotModel ([../Spike/Model.py](../Spike/Model.py))
- Integrates wheel encoder deltas + IMU orientation into a world-frame
  `position` vector (mm). Gravity-corrected acceleration available.
- `sensor_turn_rate_to_linear` converts turn rate to lateral sensor speed —
  used by the follower to keep the sensor under its speed limit.

### FollowerAlgorithm ([../Spike/FollowerAlgorithm.py](../Spike/FollowerAlgorithm.py))
Main loop `tick()` → `fuzzy_loop(dT)`:
1. Read both sensors (reflection + green detection); update green counters.
2. Optional handlers, each gated by a flag in [../Spike/constants.py](../Spike/constants.py):
   `do_knock_down_save` (claw tilt recovery), `do_gap_save` (distance-based
   gap search), `do_obstacle` (laser-based obstacle detour).
3. Silver (`darkest > silver_min_threshold`) → `room3check()` in [../Spike/Room3.py](../Spike/Room3.py).
4. Black (`darkest < 11`) → green maneuver or center-aligned maneuver.
5. `line_follow_control`: PD(+D2) on `L-R+bias`, turn rate clamped so the
   sensor's lateral speed stays under `sensor_linear_speed` (ramped up from
   `sensor_reduced_linear_speed` after maneuvers).

### AwDriveBase ([../Spike/libs/AwareRobotics/AwDriveBase.py](../Spike/libs/AwareRobotics/AwDriveBase.py))
- Subclass of Pybricks `DriveBase`; tracks driving state and detects stuck
  motors by comparing commanded vs actual wheel speed, reporting to
  `FailHandler.stuck_while_driving` (display pattern).
- `distance()` falls back to the last good value if the read throws.

### FailHandler ([../Spike/FailHandler.py](../Spike/FailHandler.py))
Prints state + hub dump on exception, shows port letter on PortError,
renders stuck-motor patterns. `Context.routine()` calls `refresh_display()`.

## Tuning

All numbers live in [../Spike/constants.py](../Spike/constants.py):
- `line_follower_constants`: speeds, PD gains (`KP/KD/KD2`), `bias`,
  silver threshold, gap/obstacle distances, feature flags.
- `room3_const`: `U` (cell size 300 mm), `straight_speed`, torque thresholds.
- `surprises`: flags for the surprise challenge (signal blink on obstacle/room3).

## ESP32 link

[../Spike/ESP32_bridge.py](../Spike/ESP32_bridge.py) — `Esp32SensorPack` wraps an
`UltrasonicSensor`-compatible device or falls back to `ESP32_distance_sensor`
(UART). See [esp32-firmware.md](esp32-firmware.md) for the protocol.
Note: `laser_distance()`/`rear_sensor_pressed()` are awaited everywhere.

## Gotchas

- `AzColorSensor` ([../Spike/libs/AzSensors.py](../Spike/libs/AzSensors.py)) adds snap/line
  helpers but its `line()` reads `self.line_r_weight` etc., which are never
  defined — don't use `line()`/`isDeepLine()`/`isShallowLine()` as-is.
- `Scaffolding.waiter.__bool__` inverts logic when `inverted=True`
  (`get_invereted_waiter`); read carefully before using.
- `Room3.py` drives 6×~12 mm to detect the ramp via `imu.tilt()[0] > 5`,
  then hands off to `V3.r3_main.run_room3`.
- `Room3 legacy.py` is reference-only; `V3/` replaced it.
- `simulator/` contains only stale `__pycache__`; no simulator source exists.
