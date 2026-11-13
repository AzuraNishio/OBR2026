# AGENTS.md — RoboEcus 2026 OBR

Agent guide for the RoboEcus 2026 OBR robot code (by Azura and Haig).
All agent documentation is centralized in this folder; the root
[AGENTS.md](../AGENTS.md) is only a pointer here. Read this before changing
anything. Deeper docs:

- [spike-hub-runtime.md](spike-hub-runtime.md) — hub runtime: boot flow, line follower, robot/model/context
- [room3-solver.md](room3-solver.md) — Room 3 (rescue) solver: model, beam search, opening book
- [real-robot-contract.md](real-robot-contract.md) — expected real-robot movement/sensing contract for Room 3
- [esp32-firmware.md](esp32-firmware.md) — ESP32 sensor-pack firmware and UART protocol

## What this is

Competition code for a LEGO SPIKE Prime robot running **Pybricks** firmware
(MicroPython). The robot line-follows a course, handles green squares, gaps,
obstacles and silver, and solves the rescue room ("room 3") with a probabilistic
search. An ESP32 add-on board provides a laser distance sensor (VL53L1X) and
rear bumper buttons over UART.

## Hard constraints (non-negotiable)

1. **No third-party libraries.** The code is compiled to MicroPython on the hub.
   Only these imports exist at runtime: `pybricks.*` (parameters, pupdevices,
   iodevices, robotics, hubs, tools), `umath`, `sys`, `gc`, and the standard
   built-ins. Never add `pip` dependencies to hub code. Desktop-only scripts
   (e.g. the opening-book builder) may use CPython stdlib only.
2. **Code memory limit: 52300 bytes.** The compiled user program must fit in
   52300 bytes on the hub. Keep code compact: short helpers, no dead code, no
   large literals, prefer packed data (see the opening book's `bytes` blob).
   If a change pushes size up, check what can be trimmed. `gc.mem_free()` /
   `TestSuite.room3_algorithm_speed_test()` print RAM usage on-hub.
3. **Star imports are the norm.** Modules do `from pybricks import *`,
   `from libs.Utills import *`, etc. Names like `vector`, `Matrix`, `pi`,
   `wait`, `StopWatch`, `multitask`, `run_task`, `Direction`, `Port`, `Color`
   come from those. Match the existing style; don't "fix" star imports.
4. **Async everywhere on-hub.** Sensor reads and moves are awaited
   (`await robot.R_sensor.reflection()`). Long loops must `await wait(...)`
   or drive via `run_task`/`multitask` so the cooperative scheduler runs.

## Repo map

```
ESP32/                 Arduino firmware for the sensor pack (VL53L1X + bumpers)
Spike/                 Everything that runs on the hub (entry: __init__.py)
  main.py              Entry point: builds Context, robot, model, follower
  constants.py         All tuning constants (line follower, room 3, surprises)
  Scaffolding.py       Context: hub, port probing, display, battery, waiters
  LineFollowerRobot.py Robot hardware abstraction + move helpers (garra = claw)
  Model.py             Odometry model (IMU + wheel encoders)
  FollowerAlgorithm.py Main line-follow loop: PD control, green/gap/obstacle/silver
  Room3.py             room3check(): ramp detection, hands off to V3
  V3/                  Room 3 solver (see room3-solver.md)
  libs/                Utills (port probe, vector math), AzSensors, Exceptions,
                       AwareRobotics/AwDriveBase (DriveBase + stuck detection)
  ESP32_bridge.py      UART protocol client for the ESP32 pack
  FailHandler.py       Error display + stuck reporting
  TestSuite.py         On-hub tests, gated by isTest() (currently False)
  Room3 legacy.py      Old room 3 implementation, kept for reference
  simulator/           Vestigial (only __pycache__ remains)
agents/                Agent documentation (this folder)
.venv/                 Desktop venv: pybricksdev (deploy), numpy, pygame
```

## Workflows

- **Deploy to hub** (needs the hub powered on + Bluetooth): VS Code task
  `main` → `python -m pybricksdev run ble Spike/__init__.py`.
  Task `stop` stops a running program over BLE.
- **Rebuild opening book** (desktop, CPython): VS Code task
  `build_oppening_book` → `python Spike/V3/r3_build_opening_book.py --depth 3 --information-weight 40`.
  Regenerates `Spike/V3/r3_opening_book_data.py`.
- **Syntax-check without a hub**: `python -m py_compile <file>` (imports need
  pybricks, so never import hub modules on desktop; the opening-book builder
  shims `umath` → `math` for this reason).
- There is no linter/test runner configured. Validation = syntax check +
  on-hub run + `TestSuite` (set `isTest()` to `True` temporarily).

## Conventions

- Tuning values go in [../Spike/constants.py](../Spike/constants.py), not inline.
- "Room 3" = rescue room; `r3_` prefix = room 3; `garra` = claw.
- Room geometry uses cell coordinates with the entrance at `y = -1`;
  directions are 0=up, 1=right, 2=down, 3=left (see room3-solver.md).
- Known latent bug (do not silently change behavior): `LineFollowerRobot.py`
  calls `ctx.show_string_enxcoded_img(...)` (typo) while `Scaffolding.py`
  defines `show_string_encoded_img`; it only triggers in the missing-port
  animation path.
