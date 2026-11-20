# Real robot contract — Room 3 movement & sensing

Single source of truth for what the physical robot does, so the digital model
(`r3_model_robot`), the measurement/filtering, and the planner never drift
away from hardware. If hardware and this doc disagree, fix one of them —
never let the algorithm silently diverge from both.

## Physical reference

- Drive: ports A/B, 40 mm wheels, 110 mm track. One tile = 300 mm (`room3_const.U`).
- Sensors: C/D color sensors (line/exit), F = ESP32 pack (VL53L1X laser + rear bumper buttons).
- **The robot walks ONE TILE per move action — same granularity as the digital model.**

## Garra (claw) mechanics

- The garra is mounted on the robot's **back**; the laser sensor is on the
  **front**. Port E drives it. **Down (open) while backing up, up otherwise.**
- The garra **grabs victims on contact while moving into them** — i.e. backward
  moves with the claw down. (Victim *delivery* at triangles is a separate,
  active action — see the scoring note in room3-solver.md.)
- **Buttons at the garra tip** detect moving into a wall while backing up.
  Rear contact geometry therefore depends on garra state.
- **Garra open ≈ 1.5 tiles long** (≈ 450 mm overall); garra up (closed) is
  shorter (assumed ≈ 1 tile — verify). Turning always raises it first, because
  a 1.5-tile-long robot sweeping a 90° turn would hit adjacent walls.

### Garra-state implications the model must respect

1. **Garra state persists across moves.** `real_move(dir=1)` never raises the
   garra, so a forward move following a backward move runs garra-open. Only
   `real_turn` (always) and the rear-press handler close it. The digital model
   is tile-granular and ignores garra length (fine per the contract), but the
   **contact and reading calibration constants differ per garra state** and
   must be measured on the physical robot:
   - tip protrusion beyond the body at rest, garra open vs closed
   - distance traveled before bumper contact when backing into a wall
   - laser offset ahead of the center of rotation
2. **Bumper may press before the move starts.** Opening the garra extends the
   tip backward ~150 mm; if the tile behind is solid and close, the bumper can
   press during the opening itself (before any driving). `rear_sensor_pressed()`
   is only polled inside the drive loop, so a pre-move press is missed until
   the robot is already pushing.
3. **Rest pose is not tile-centered.** `real_move()`'s net displacement for a
   completed move is **340 mm, not 300 mm** (back 80, forward to +370, trim
   30). The `+30 mm` fudge in `measure()` absorbs some of this. The true rest
   offset of the laser (and tip) within a tile is unmeasured — both the
   trust-band matching and the contact model depend on it.

## Move actions

### `forward()` — attempt one tile ahead

| Tile ahead is... | Behavior |
|---|---|
| free | Robot ends centered in that tile. |
| wall / obstacle | Robot cannot enter; stays on its origin tile. **No event — forward never detects walls.** The wall is discovered by the next laser reading (0–1 cells). |
| triangle | Contact → robot returns to the tile it occupied before the move. `touched_triangle`, contact cell = the attempted tile. |
| exit gap | Robot drives into the gap; color sensors see black → `exited='exit'`. |

### `backyard()` — attempt one tile behind (claw down)

| Tile behind is... | Behavior |
|---|---|
| free | Robot ends centered in that tile. |
| wall / obstacle | Rear bumper contacts → robot stays on its origin tile. `touched_wall`. **Wall detection happens ONLY when moving backwards.** |
| triangle | Contact → returns to the origin tile. `touched_triangle`, contact cell = the attempted tile. |
| exit gap | Drives into the gap → `exited='exit'`. |

### `turn_left()` / `turn_right()` — 90° in place

Claw up first. Never produces wall/triangle events.

### Wall alignment after a turn (`real_turn`)

Intent: if a wall is nearby ahead, physically square the robot against it to
remove drift. Current implementation: after the 90° turn + a 10 mm nudge, if
`laser < 200 mm` (retuned from 120 by hand during hardware testing) **and**
`moves_since_align > 4`, drive forward 140 mm (ram the wall), then back off
50 mm; reset `moves_since_align = 0`.

**Diagnosis — why it doesn't work:**

1. **Eligibility rarely coincides with the trigger.** `moves_since_align` is
   incremented in `update_async()`, which `r3_main` calls **twice per loop**;
   with the `> 4` threshold alignment is eligible only every ~3rd iteration.
   The trigger (wall < 120 mm ahead right after a turn) happens mostly after
   bounces and in dead ends — and the planner actively avoids facing walls, so
   on eligible turns the laser is usually far. The two conditions rarely hold
   together, and a missed opportunity rarely recurs.
2. **The ram is blind and too fast.** `straight(140)` runs at room-3 speed
   (430 mm/s) into a wall that may be 0–120 mm away. The DriveBase stalls
   against the wall mid-profile: wheel slip and chassis twist during the stall
   make alignment *worse*, and the following `straight(-50)` just freezes
   whatever pose the slip produced. Squaring should be slow and contact-based
   (motor-load stop, like the bounce logic in `real_move`), not a fixed-distance
   ram.
3. **Stale or corrupted trigger reading.** The laser value comes from the
   newest buffered ESP32 packet (ring of 8, ~100 ms/packet) with the timeout
   flag ignored, and distances that are multiples of 256 mm decode as 9999
   (bridge bug) — so the trigger can fire facing open space or miss a real wall.
4. **No verification or retry** after the maneuver, and the end pose (front
   ~50 mm off the wall) is not a pose the digital model represents, so even a
   successful squaring shifts later laser readings in a way the measurement
   model does not expect.

**Recommended redesign (for later):** make alignment *planned, not
opportunistic* — the algorithm already knows from the laser reading when the
robot faces a near wall; schedule a squaring action then. Implement it as:
slow approach (~100 mm/s) until motor load rises, small fixed back-off, verify
with a fresh laser read, retry once. Track garra state (must be up).

## Sensing contract

- **Laser distance**: read after the move completes, from the tile the robot
  actually ended on (origin tile if bounced), in the facing direction.
  Encoding: `cells = min(floor((mm + 30) / 300), 4)`; 4 = beam left through a gap.
  Real readings drift between adjacent values (typically 2/3/4) — the
  algorithm must tolerate this (trust-band matching), never clamp-to-match.
- **Rear bumpers**: buttons at the garra tip; only make contact while backing
  up (garra down).
- **Color sensors**: black line = exit gap. Red = emergency stop (follower
  only, not room 3). Silver = entrance (not implemented).
- **Claw**: down during backward moves, up otherwise.

## What the algorithm must model per room (`simulate_measurement`)

Given the pose before the move + the action:

1. `destination` = pose + step (forward) / pose − step (backward)
2. position after move = destination, **except triangle contact → origin tile**
3. `touched_wall` = moving backward AND destination is solid (wall/obstacle)
4. `touched_triangle` = destination hosts a triangle corner (per room; triangles are NOT part of descriptors)
5. `exited` = destination is the exit/entrance gap
6. distance = laser from the final position + facing

## Triangle data rules

- Triangles are **not** encoded in descriptors — that would multiply the
  hypothesis count by the number of legal triangle pairs (~2–3×, a speed
  problem). Descriptor count stays as-is; triangle info only **prunes**.
- A `touched_triangle` contact keeps only rooms that can host a triangle at
  the contact cell, updates triangle data, and applies the attempt reward.
  The digital robot does not move (bounce).
- Real-robot delivery (special handling) comes later.

## Deviations in current code (to fix in the rework)

- `r3_model_robot.forward()` sets `garra_touched_wall` on forward collision —
  contradicts the contract (forward never produces a wall event).
- Triangle bounce exists only in `backyard()`; the contract requires both directions.
- `is_on_triangle` is stored but never consumed.
- `real_move()` rear-button contact brakes, raises the garra and sets
  `touched_wall`, but does **not** break the drive loop — termination still
  relies on the motor-load threshold. If the wall yields (load stays < 160),
  the robot keeps pushing with the event already latched.
- `real_move()`'s load-based bounce-back only handles **left**-motor contact
  (`left_motor_load > 160`); right-only contact breaks the loop without
  backing off, leaving the robot pressed against the wall off-center.
- `real_move()` never raises the garra for forward moves: a forward move right
  after a backward move runs with the garra open (~1.5 tiles long).
- ESP32 bridge ([../Spike/ESP32_bridge.py](../Spike/ESP32_bridge.py)):
  `distance()` returns 9999 when the packet's first byte is 0 — valid distances
  that are multiples of 256 mm (256, 512, 768…) are discarded and read as
  "4 = no wall", corrupting measurements. The timeout flag (packet byte 2) is
  also ignored, so stale last-valid distances are indistinguishable from fresh
  ones.
- `ESP32_distance_sensor.L_button_pressed()` returns `False` unconditionally
  (the real decode `packet[3] == 1` is commented out). Consequence on hardware:
  `rear_sensor_pressed()` never fires → the rear-button branch in `real_move()`
  is dead → `garra_touched_wall` is **never set on the real robot** → the
  `touched_wall` measurement signal is always False. The only working wall
  sensing is the motor-load terminator, which sets no event. (`R_button_pressed`
  works: `packet[4] == 1`.)
- Two firmware variants exist under `ESP32/`: the top-level
  `SPIKE_Bridge_v2.ino` sends ASCII (`UART2.println(distance)`), while
  `ESP32/SPIKE_Bridge_v2/SPIKE_Bridge_v2.ino` sends the binary packet protocol
  (ALT_FLAG escaping; bytes 0–1 little-endian distance, byte 2 timeout flag,
  byte 3 L button, byte 4 R button) that `ESP32_bridge.py` parses. If the ASCII
  variant is flashed, **all** hub readings are garbage.
- `Esp32SensorPack.__init__` tries `UltrasonicSensor(Port.F)` first, then
  `ColorSensor`, and only falls back to `ESP32_distance_sensor` if both fail.
  If an ultrasonic sensor is physically on Port F, `laser_distance()` reads the
  ultrasonic sensor, not the VL53L1X bridge.
- Dead code in `real_move()`: `sonic_check_time` / `sonic_dist` are computed
  and never used; `do_triangle` is a constant local.

### Clarification (not a deviation)

`real_move()`'s motor-load threshold (> 160) firing in **both** directions is
**not** a contract violation: it is the physical move *terminator* (without it
a blocked move would never end) and it sets **no** wall event — forward
contact just ends the move early with the robot on its origin tile, which
matches "forward never detects walls". The wall *event* correctly comes only
from the rear bumper. The rework must keep a forward terminator (load-based or
laser-pre-read) while producing no event.

## Open questions

- Obstacle behind on a backward move: assumed to trigger the bumper like a wall.
- Entrance (silver) detection unimplemented; the model can set `exited='entrance'`.
- After a backward-into-wall (robot stays), the move is simply consumed; the algorithm picks the next action.
- Exact offsets not measured: laser sensor position relative to the center of
  rotation, garra-tip position, and the robot's rest pose within a tile. The
  measurement model's `+30 mm` tolerance implicitly assumes the sensor sits
  about one tile-leading-edge ahead of the CoR. Also note `real_move()`'s net
  displacement for a free move is **340 mm, not 300 mm** (back 80, forward to
  +370, trim 30) — consistent with a rest pose offset ~40 mm behind tile
  center, but unverified. Measure both on the physical robot before trusting
  fine-grained reading matching.
