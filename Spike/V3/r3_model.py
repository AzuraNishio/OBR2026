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

from V3.NekoMath import *
from V3.obrExperimentalData import *

SENSOR_FAR = 4
SHAPES = {"3x3": (3, 3), "3x4": (3, 4), "4x3": (4, 3)}   # (width, height)
ONLY_OBSTACLE_LESS_ROOMS = False
OBSTACLE_MASK = 0x000F
SHAPE_MASK = 0x0030
OFFSET_MASK = 0x00C0
OUTPUT_MASK = 0x0F00
ROOM_SHAPES = ("3x3", "3x4", "4x3")
ROOM_SHAPE_IDS = {"3x3": 0, "3x4": 1, "4x3": 2}
ROOM_TRIANGLE_PAIRS = (
    (0, 1), (0, 2), (2, 3), None,
    (1, 2), (1, 3), (0, 3), None,
)
NO_OBSTACLE = (3, 3)



# --------------------------------------------------------------------------
# The room: pure data, never changes
# --------------------------------------------------------------------------
# @dataclass(frozen=True)
class r3_model:
    shape: str                 # "3x3", "3x4" or "4x3"
    offset: int                # entrance gap = wall number `offset`
    output: int                # exit gap = wall number `output`
    obstacle: tuple = None     # (x, y) or None

    def __init__(self, shape, offset, output, obstacle=None):
        self.shape = shape
        self.offset = offset
        self.output = output
        self.obstacle = obstacle

    def width(self):  return SHAPES[self.shape][0]
    def height(self): return SHAPES[self.shape][1]

    def wall_cell(self, n):
        """Position (outside the room) of wall segment number n."""
        W, H = self.width(), self.height()
        if n < W: return (n, -1)
        n -= W
        if n < H: return (W, n)
        n -= H
        if n < W: return (W - 1 - n, H)
        n -= W
        return (-1, H - 1 - n)

    def entrance_cell(self): return self.wall_cell(self.offset)
    def exit_cell(self):     return self.wall_cell(self.output)
    def free_area(self):
        a = self.height() * self.width()
        a -= 2
        if not self.obstacle is None: a -= 1
        return a

    def corner_cell(self, c):
        W, H = self.width(), self.height()
        return [(0, 0), (W - 1, 0), (W - 1, H - 1), (0, H - 1)][c]

    def inside(self, x, y):
        return 0 <= x < self.width() and 0 <= y < self.height()

    def kind(self, x, y):
        """What is at (x, y)? 'free' | 'obstacle' | 'entrance' | 'exit' | 'wall'"""
        if self.inside(x, y):
            return "obstacle" if (x, y) == self.obstacle else "free"
        if (x, y) == self.entrance_cell(): return "entrance"
        if (x, y) == self.exit_cell():     return "exit"
        return "wall"

    def distance(self, x, y, d):
        """Walk from (x,y) in direction d. Returns (free_cells, what_stopped_us).
        Gaps are NOT solid: a beam through a gap just keeps going."""
        dx, dy = STEP_TUPLE_FROM_DIR[d]
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
        W, H = self.width(), self.height()
        g = {}
        for y in range(-1, H + 1):
            for x in range(-1, W + 1):
                k = self.kind(x, y)
                g[(x, y)] = {"free": "  ", "obstacle": "[]", "wall": "██",
                             "entrance": "^^", "exit": "--"}[k]
        from V3.r3_triangle_data import is_triangle
        for y in range(H):
            for x in range(W):
                if is_triangle(self, x, y):
                    g[(x, y)] = "TT"
        if robot is not None:
            g[(robot.x + self.offset, robot.y)] = "/>\\<"[robot.dir()] + "\\>/<"[robot.dir()]
        return "\n".join("".join(g[(x, y)] for x in range(-1, W + 1))
                         for y in range(H, -2, -1))

NULL_ROOM = r3_model("3x4", 0, 1)   # used for simulating hypothetical poses


def room_descriptor(shape, offset, output, obstacle):
    if obstacle is None:
        obstacle_x, obstacle_y = NO_OBSTACLE
    else:
        obstacle_x, obstacle_y = obstacle

    return (
        obstacle_x
        | (obstacle_y << 2)
        | (ROOM_SHAPE_IDS[shape] << 4)
        | (offset << 6)
        | (output << 8)
    )


def room_from_descriptor(descriptor, reuse=None):
    obstacle_code = descriptor & OBSTACLE_MASK
    shape_id = (descriptor & SHAPE_MASK) >> 4
    offset = (descriptor & OFFSET_MASK) >> 6
    output = (descriptor & OUTPUT_MASK) >> 8
    obstacle = None
    if obstacle_code != 0b1111:
        obstacle = (obstacle_code & 0b11, obstacle_code >> 2)

    if reuse is None:
        return r3_model(ROOM_SHAPES[shape_id], offset, output, obstacle)

    reuse.shape = ROOM_SHAPES[shape_id]
    reuse.offset = offset
    reuse.output = output
    reuse.obstacle = obstacle
    return reuse


def room_kind_from_descriptor(x, y, descriptor):
    obstacle_code = descriptor & OBSTACLE_MASK
    shape_id = (descriptor & SHAPE_MASK) >> 4
    offset = (descriptor & OFFSET_MASK) >> 6
    width, height = SHAPES[ROOM_SHAPES[shape_id]]

    if 0 <= x < width and 0 <= y < height:
        if obstacle_code != 0b1111 and obstacle_code == (x | (y << 2)):
            return "obstacle"
        return "free"

    if x == offset and y == -1:
        return "entrance"

    if room_is_exit_from_descriptor(x, y, descriptor):
        return "exit"
    return "wall"


def room_is_exit_from_descriptor(x, y, descriptor):
    shape_id = (descriptor & SHAPE_MASK) >> 4
    width, height = SHAPES[ROOM_SHAPES[shape_id]]
    number = (descriptor & OUTPUT_MASK) >> 8
    if number < width:
        return x == number and y == -1
    number -= width
    if number < height:
        return x == width and y == number
    number -= height
    if number < width:
        return x == width - 1 - number and y == height
    number -= width
    return x == -1 and y == height - 1 - number


def room_frequency_score(descriptor):
    obstacle_code = descriptor & OBSTACLE_MASK
    shape_id = (descriptor & SHAPE_MASK) >> 4
    if obstacle_code == 0b1111:
        obstacle_probability = 1 - probabilitie_of_room3_having_a_obstacle
    else:
        obstacle_probability = probabilitie_of_room3_having_a_obstacle
    return obstacle_probability * probabilitie_of_room3_per_shape[ROOM_SHAPES[shape_id]]



# --------------------------------------------------------------------------
# Every legal room
# --------------------------------------------------------------------------
def legal_triangle_pairs(shape, offset, output, obstacle):
    base = r3_model(shape, offset, output, obstacle)
    gaps = [base.entrance_cell(), base.exit_cell()]
    free_corners = []
    for corner in range(4):
        cx, cy = base.corner_cell(corner)
        if (cx, cy) == obstacle:
            continue
        if any(abs(gx - cx) + abs(gy - cy) == 1 for gx, gy in gaps):
            continue
        free_corners.append(corner)
    return [(a, b) for a, b in combinations(free_corners, 2)]


def descriptor_triangle_pairs(descriptor):
    obstacle_code = descriptor & OBSTACLE_MASK
    shape_id = (descriptor & SHAPE_MASK) >> 4
    offset = (descriptor & OFFSET_MASK) >> 6
    output = (descriptor & OUTPUT_MASK) >> 8
    obstacle = None if obstacle_code == 0b1111 else (obstacle_code & 0b11, obstacle_code >> 2)
    return legal_triangle_pairs(ROOM_SHAPES[shape_id], offset, output, obstacle)


def descriptor_triangle_probability_at(descriptor, x, y):
    shape_id = (descriptor & SHAPE_MASK) >> 4
    offset = (descriptor & OFFSET_MASK) >> 6
    output = (descriptor & OUTPUT_MASK) >> 8
    obstacle_code = descriptor & OBSTACLE_MASK
    obstacle = None if obstacle_code == 0b1111 else (obstacle_code & 0b11, obstacle_code >> 2)
    room = r3_model(ROOM_SHAPES[shape_id], offset, output, obstacle)
    pairs = legal_triangle_pairs(room.shape, offset, output, obstacle)
    if not pairs:
        return 0
    matches = sum(1 for pair in pairs if any(room.corner_cell(c) == (x, y) for c in pair))
    return matches / len(pairs)


def descriptor_can_have_triangle_at(descriptor, x, y):
    shape_id = (descriptor & SHAPE_MASK) >> 4
    offset = (descriptor & OFFSET_MASK) >> 6
    room = r3_model(ROOM_SHAPES[shape_id], offset, (descriptor & OUTPUT_MASK) >> 8,
                    None if (descriptor & OBSTACLE_MASK) == 0b1111 else
                    (descriptor & 0b11, (descriptor & OBSTACLE_MASK) >> 2))
    return any(room.corner_cell(c) == (x, y) for pair in descriptor_triangle_pairs(descriptor) for c in pair)


def _iter_room_descriptors():
    for shape, (W, H) in SHAPES.items():
        for off in range(W):                       # entrance is on the bottom wall
            for out in range(2 * (W + H)):
                if out == off:
                    continue
                base = r3_model(shape, off, out)
                gaps = [base.entrance_cell(), base.exit_cell()]

                def in_front(g):                   # room cell right inside a gap
                    return (min(max(g[0], 0), W - 1), min(max(g[1], 0), H - 1))
                blocked = {in_front(g) for g in gaps}

                if ONLY_OBSTACLE_LESS_ROOMS:
                    obstacles = [None]
                else:
                    obstacles = [None] + [(x, y) for x in range(W) for y in range(H)
                                          if (x, y) not in blocked]
                for obs in obstacles:
                    yield room_descriptor(shape, off, out, obs)


def all_room_descriptors():
    return list(_iter_room_descriptors())

def room_supplier(descriptors):
    room = r3_model("3x3", 0, 1)
    from V3.r3_triangle_data import set_triangles
    for descriptor in descriptors:
        room_from_descriptor(descriptor, room)
        pairs = descriptor_triangle_pairs(descriptor)
        set_triangles(room, pairs[0] if pairs else ())
        yield room
                        