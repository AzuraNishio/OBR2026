"""
COORDINATES (room frame)
    x goes right, y goes "up" into the room. The entrance wall is at y = -1.
    Room cells are 0 <= x < width, 0 <= y < height.
    The robot enters at cell (offset, 0).

DIRECTIONS    0 = up (+y)   1 = right (+x)   2 = down (-y)   3 = left (-x)
              turning right = +1, turning left = -1 (mod 4)

WALL NUMBERING: counterclockwise starting at the
    entrance wall. 0..W-1 bottom (left->right), then right wall going up,
    then top wall going right->left, then left wall going top->bottom.
    The entrance gap is wall number `offset`, the exit gap is wall `output`.

MISCELANEOUS
    * obstacle is 1 cell, or None
    * obstacle never sits right in front of the entrance or exit gap
    * a triangle sits in a corner cell, never on the obstacle, never next to a gap
    * sensor reading: 0..3 = number of free cells before something solid,
      4 = beam found nothing solid (it left through a gap)
"""

from dataclasses import dataclass
import V3.NekoMath as nk
from V3.obrExperimentalData import *

SENSOR_FAR = 4
SHAPES = {"3x3": (3, 3), "3x4": (3, 4), "4x3": (4, 3)}   # (width, height)



# --------------------------------------------------------------------------
# The room: pure data, never changes
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class r3_model:
    shape: str                 # "3x3", "3x4" or "4x3"
    offset: int                # entrance gap = wall number `offset`
    output: int                # exit gap = wall number `output`
    obstacle: tuple = None     # (x, y) or None
    triangles: frozenset = frozenset()   # set of corner numbers {0,1,2,3}
    dist_maps: dict = None

    @property
    def width(self):  return SHAPES[self.shape][0]
    @property
    def height(self): return SHAPES[self.shape][1]
    @property
    def perimeter(self): return 2 * (self.width + self.height)

    def wall_cell(self, n):
        """Position (outside the room) of wall segment number n."""
        W, H = self.width, self.height
        if n < W: return (n, -1)
        n -= W
        if n < H: return (W, n)
        n -= H
        if n < W: return (W - 1 - n, H)
        n -= W
        return (-1, H - 1 - n)

    @property
    def entrance_cell(self): return self.wall_cell(self.offset)
    @property
    def exit_cell(self):     return self.wall_cell(self.output)

    @property
    def frequency_score(self):     
        score = 1
        if self.obstacle is None: score *= 1 - probabilitie_of_room3_having_a_obstacle
        else                    : score *= probabilitie_of_room3_having_a_obstacle

        score *= probabilitie_of_room3_per_shape[self.shape]

        return score

    @property
    def free_area(self):
        a = self.height * self.width
        a -= 2
        if not self.obstacle is None: a -= 1
        return a

    def corner_cell(self, c):
        W, H = self.width, self.height
        return [(0, 0), (W - 1, 0), (W - 1, H - 1), (0, H - 1)][c]

    def inside(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def kind(self, x, y):
        """What is at (x, y)? 'free' | 'obstacle' | 'entrance' | 'exit' | 'wall'"""
        if self.inside(x, y):
            return "obstacle" if (x, y) == self.obstacle else "free"
        if (x, y) == self.entrance_cell: return "entrance"
        if (x, y) == self.exit_cell:     return "exit"
        return "wall"

    def distance(self, x, y, d):
        """Walk from (x,y) in direction d. Returns (free_cells, what_stopped_us).
        Gaps are NOT solid: a beam through a gap just keeps going."""
        dx, dy = nk.STEP_TUPLE_FROM_DIR[d]
        n = 0
        while True:
            x += dx; y += dy
            k = self.kind(x, y)
            if k == "free":
                n += 1
            else:
                return n, k

    def reading(self, x, y, d):
        """What the distance sensor reports."""
        n, k = self.distance(x, y, d)
        if k in ("entrance", "exit") or n >= SENSOR_FAR:
            return SENSOR_FAR
        return n

    def render(self, robot=None):
        W, H = self.width, self.height
        g = {}
        for y in range(-1, H + 1):
            for x in range(-1, W + 1):
                k = self.kind(x, y)
                g[(x, y)] = {"free": "  ", "obstacle": "[]", "wall": "██",
                             "entrance": "^^", "exit": "--"}[k]
        for c in self.triangles:
            g[self.corner_cell(c)] = "TT"
        if robot is not None:
            g[(robot._x, robot._y)] = "/>\\<"[robot.dir] + "\\>/<"[robot.dir]
        return "\n".join("".join(g[(x, y)] for x in range(-1, W + 1))
                         for y in range(H, -2, -1))

    def is_triangle(self, x, y):
        return any(self.corner_cell(c) == (x, y)
                   for c in self.triangles)

    def read_dist_map(self, map, robot, offX = 0, offY = 0):
        return self.dist_maps[map][robot.y][robot.x + self.offset + offX][robot.dir]

    def read_entrance_map(self, robot, offX = 0, offY = 0): return self.read_dist_map("entrance", robot, offX, offY)
    def read_exit_map(self, robot, offX = 0, offY = 0): return self.read_dist_map("exit", robot, offX, offY)
    def read_triangle_map(self, robot, offX = 0, offY = 0): return self.read_dist_map("triangle", robot, offX, offY)

    
NULL_ROOM = r3_model("3x4", 0, 1)   # used for simulating hypothetical poses


def compute_distance_map(room: r3_model, target: tuple = None, target2: tuple = None):
    """
    Returns a [height][width][4] distance map.

    Each value is the minimum number of actions required to reach
    the target from (x, y, direction).

    If target2 is provided, computes distance to the closest target.
    Targets can be free cells or entrance/exit gaps.
    """

    W, H = room.width, room.height

    # Initialize every state as unreachable.
    hm = [[ [90, 90, 90, 90] for x in range(W)] for y in range(H)]

    targets = [target]
    if target2 is not None:
        targets.append(target2)

    # Seed the heat map.
    for tx, ty in targets:
        if room.kind(tx, ty) == "free":
            for d in range(4):
                hm[ty][tx][d] = 0
        else:
            # For an entrance/exit gap, seed states that can move into it.
            for x in range(W):
                for y in range(H):
                    if room.kind(x, y) != "free":
                        continue

                    for d in range(4):
                        dx, dy = nk.STEP_TUPLE_FROM_DIR[d]

                        for sign in (1, -1):
                            if (x + sign * dx, y + sign * dy) == (tx, ty):
                                hm[y][x][d] = min(hm[y][x][d], 1)

    # Repeatedly propagate distances.
    while True:
        new_hm = [[row[:] for row in hm[y]] for y in range(H)]

        for x in range(W):
            for y in range(H):
                if room.kind(x, y) != "free":
                    continue

                for d in range(4):
                    dx, dy = nk.STEP_TUPLE_FROM_DIR[d]

                    # Turn left or right.
                    turn_distance = min(
                        hm[y][x][(d - 1) % 4],
                        hm[y][x][(d + 1) % 4]
                    ) + 1

                    # Move forward or backward.
                    move_distance = 90

                    for sign in (1, -1):
                        nx = x + sign * dx
                        ny = y + sign * dy

                        if room.kind(nx, ny) == "free":
                            move_distance = min(
                                move_distance,
                                hm[ny][nx][d] + 1
                            )

                        elif (nx, ny) in targets:
                            move_distance = min(move_distance, 1)

                    new_hm[y][x][d] = min(
                        hm[y][x][d],
                        turn_distance,
                        move_distance
                    )

        if new_hm == hm:
            break

        hm = new_hm

    # Convert unreachable states from 90 to -1.
    for y in range(H):
        for x in range(W):
            for d in range(4):
                if hm[y][x][d] >= 90:
                    hm[y][x][d] = -1

    return hm


def compute_room_distance_maps(room: r3_model):
    """Precompute distances to the entrance, exit, and nearest triangle."""
    triangles = [
        room.corner_cell(c)
        for c in room.triangles
    ]

    return {
        "entrance": compute_distance_map(room, room.entrance_cell),
        "exit": compute_distance_map(room, room.exit_cell),
        "triangle": compute_distance_map(
            room, triangles[0], triangles[1]
        ),
    }


# --------------------------------------------------------------------------
# Every legal room
# --------------------------------------------------------------------------
def all_rooms():
    rooms = []
    for shape, (W, H) in SHAPES.items():
        for off in range(W):                       # entrance is on the bottom wall
            for out in range(2 * (W + H)):
                if out == off:
                    continue
                base = r3_model(shape, off, out)
                gaps = [base.entrance_cell, base.exit_cell]

                def in_front(g):                   # room cell right inside a gap
                    return (min(max(g[0], 0), W - 1), min(max(g[1], 0), H - 1))
                blocked = {in_front(g) for g in gaps}

                obstacles = [None] + [(x, y) for x in range(W) for y in range(H)
                                      if (x, y) not in blocked]
                for obs in obstacles:
                    free_corners = []
                    for c in range(4):
                        cx, cy = base.corner_cell(c)
                        if (cx, cy) == obs:
                            continue
                        if any(abs(gx - cx) + abs(gy - cy) == 1 for gx, gy in gaps):
                            continue
                        free_corners.append(c)
                    for a, b in nk.combinations(free_corners, 2):
                        
                        room = r3_model(shape, off, out, obs, frozenset((a, b)))

                        #object.__setattr__(
                        #    room,
                        #    "dist_maps",
                        #    compute_room_distance_maps(room)
                        #)

                        rooms.append(room)
    return rooms