# Spike/V3/ — Room 3 (rescue room) solver

"Room 3" = rescue room (historic name). `r3_` prefix = room 3. The robot
enters an unknown walled room, must find the exit, and scores by delivering
victims to triangles. The solver is a probabilistic beam search over every
legal room layout, refined by sensor measurements.

## Room geometry ([../Spike/V3/r3_model.py](../Spike/V3/r3_model.py))

- Cells: `x` right, `y` into the room. Entrance wall at `y = -1`; robot enters
  at cell `(offset, 0)`. Shapes: `3x3`, `3x4`, `4x3` (width×height).
- Directions: `0=up(+y) 1=right(+x) 2=down(-y) 3=left(-x)`; right turn `+1`,
  left turn `-1` (mod 4). `STEP_TUPLE_FROM_DIR` in [../Spike/V3/NekoMath.py](../Spike/V3/NekoMath.py).
- Wall numbering: counterclockwise from the entrance wall. Entrance gap =
  wall `offset`, exit gap = wall `output`.
- `kind(x, y)` → `"free" | "obstacle" | "entrance" | "exit" | "wall"`.
- `reading(x, y, d)`: 0..3 = free cells before something solid; `4`
  (`SENSOR_FAR`) = beam left through a gap or found nothing.
- Rules: obstacle is ≤1 cell, never in front of entrance/exit; triangles sit
  on corner cells, never on the obstacle, never adjacent to a gap.
- **Descriptors**: every room packs into one int —
  `obstacle_x | obstacle_y<<2 | shape_id<<4 | offset<<6 | output<<8`
  (`0b1111` obstacle code = no obstacle). `all_room_descriptors()` enumerates
  every legal room; `room_from_descriptor()` unpacks (can reuse a room object
  to avoid allocation). `room_kind_from_descriptor()` checks a cell without
  building a room.

## Data flow at runtime ([../Spike/V3/r3_main.py](../Spike/V3/r3_main.py))

```
run_room3(robot, ctx):
  speed up, drive 200 mm in, gyro on
  interface = r3_real_robot(ctx, robot)   # real hardware
  alg = algorithm(interface)
  loop:
    await interface.update_async()   # run queued moves + take laser reading
    await alg.measure()              # filter hypotheses by measurement
    finished = alg.step()            # beam search → queue next action
    await interface.update_async()
  until robot.exited == "exit"
```

The real robot runs async; the algorithm is synchronous. `update_async()`
executes the move queue and stores `real_last_measurement`.

## Key modules

- **[../Spike/V3/r3_robot_interface.py](../Spike/V3/r3_robot_interface.py)** — shared pose API:
  `x, y, _dir`, `exited` (`None|'exit'|'entrance'`), `noclip`,
  `garra_touched_wall`, `touched_triangle`, `last_dir`, `log`;
  `turn_left/turn_right/forward/backyard/measure/on_triangle`.
  (`pose()/_x()/_y()/kind()` were removed in the Phase 1 cleanup — they had no
  callers; `noclip`/`log` attributes stay because `r3_real_robot.copy()` and
  `_pay()` still reference them.)
- **[../Spike/V3/r3_model_robot.py](../Spike/V3/r3_model_robot.py)** — simulated robot on a known
  room; `forward()` returns False when blocked; `backyard()` bounces off
  triangles (robot stays on original tile, `touched_triangle=True`);
  `_pay()` accumulates time from `avg_time_per_action`.
- **[../Spike/V3/r3_real_robot.py](../Spike/V3/r3_real_robot.py)** — hardware implementation.
  Movement/sensing contract: [real-robot-contract.md](real-robot-contract.md) — the model and
  measurement code must match it.
  `real_move(dir)`: backs up 80 mm, drives until ~370 mm traveled, wall by
  motor load > 160, triangle by load > 100 + heading error > 6 (then realigns
  and reports `touched_triangle`); exit detected by seeing black under a
  color sensor. `real_turn(dir)`: claw up, 90° turn, optional wall-align
  nudge when laser < 120 mm. `measure()` = `min(floor((dist+30)/300), 4)`
  (300 mm cells = `room3_const.U`).
- **[../Spike/V3/r3_algorithm.py](../Spike/V3/r3_algorithm.py)** — `algorithm` class + `decision_policy_2`.
  - `measure()`: `info.pop_from_measurement(...)` keeps only rooms whose
    simulated reading matches; falls back to live search if nothing matches.
  - `step()`: opening book first (see below), else `decision_policy_2` —
    beam search, 7 plies, beam shrinking 165→45, actions F/B/L/R, states
    deduplicated by `beam_state_key`, moves into guaranteed-blocked cells
    rejected, score = expected follower score × victim multiplier with
    frozen exit scores (see `r3_score_functions.py`). Returns the winner's
    first action.
  - `cleanup()` + `destroy_algorithm()` free memory (matters on-hub).
- **[../Spike/V3/r3_info.py](../Spike/V3/r3_info.py)** — hypothesis set + probability maps.
  `room_probability(descriptor)` from shape/obstacle priors;
  `update_probability_spread()` builds 6×9 wall/exit/obstacle/victim/triangle
  probability maps; `pop_from_measurement()` filters; `exit_probability_at()`,
  `spread_at()`, `is_guaranteed_blocked()`, `print_information()` (ASCII map).
- **[../Spike/V3/r3_score_functions.py](../Spike/V3/r3_score_functions.py)** — expected score model:
  follower score before/after r3, `1.3^delivered_victims` multiplier, exit
  bonus. Constants come from [../Spike/V3/obrExperimentalData.py](../Spike/V3/obrExperimentalData.py)
  (marked PLACEHOLDER — tune here, not in the algorithm).
- **[../Spike/V3/NekoMath.py](../Spike/V3/NekoMath.py)** — MicroPython-safe math (wraps `umath`),
  grid helpers, entropy, `PrettyPrint` ASCII boxes. Desktop code must shim
  `umath` (the builder does `sys.modules["umath"] = math`).

## Opening book

Precomputed policy for the first N actions (saves RAM/CPU on-hub).

- **[../Spike/V3/r3_build_opening_book.py](../Spike/V3/r3_build_opening_book.py)** — desktop-only
  (argparse, os, sys). Builds a decision tree: at each node runs
  `decision_policy_2` with per-action information-gain bonuses
  (`--information-weight`, points per bit of measurement entropy reduction),
  branches on the 6-bit encoded measurement, dedups identical subtrees.
  Run: `python Spike/V3/r3_build_opening_book.py --depth 3 --information-weight 40`
  (VS Code task `build_oppening_book`). Writes `r3_opening_book_data.py`.
- **[../Spike/V3/r3_opening_book.py](../Spike/V3/r3_opening_book.py)** — binary format `R3OB`:
  header + 16-bit node offset table + per-node `(action, edge_count,
  (measurement_code, child_id)...)`. Measurement encoding: 3 bits distance,
  bit3 touched_triangle, bit4 touched_wall, bits5-6 exit code.
  `OpeningBook.action_for(node)` / `next_node(node, measurement)`.
- **[../Spike/V3/r3_opening_book_data.py](../Spike/V3/r3_opening_book_data.py)** — generated; do not
  hand-edit. `BOOK_CONFIG` signature guards compatibility (format version +
  policy revision only; tuning may drift). Current blob ≈ 34 nodes.
- Runtime: `algorithm` loads it, follows nodes while measurements match;
  any mismatch or unknown edge drops the book and switches to live search.

## Testing

`TestSuite.room3_algorithm_speed_test()` (on-hub, `isTest()` gate) runs the
algorithm against simulated rooms, printing per-run ms and `gc` free RAM
before/after — use it to check the 52300-byte / RAM budget after changes.
`r3_model_robot` + `room_supplier()` let you simulate full runs on desktop
(shim `umath` first).
