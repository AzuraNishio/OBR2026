from V3.obrExperimentalData import *
from V3.NekoMath import *
from V3.r3_robot_interface import r3_robot_interface


class r3_model_robot(r3_robot_interface):
    def __init__(self, room):
        super().__init__(room)          # hidden truth
        self.time = 0.0

    def copy(self):
        n = r3_model_robot(self._room)
        n.x, n.y, n._dir =    self.x, self.y, self._dir
        n.exited          =    self.exited
        n.last_dir        =    self.last_dir
        n.noclip          =    self.noclip
        n.time            =    self.time
        n.garra_touched_wall = self.garra_touched_wall
        n.touched_triangle = self.touched_triangle
        n.log             =    self.log
        return n


    def _turn(self, s):
        self._dir = (self._dir + s) % 4
        self.garra_touched_wall = False
        self.touched_triangle = False
        self._pay("turn")

    def forward(self):
        """Move one cell. Returns False if blocked (wall/obstacle)."""
        self._pay("move")
        self.garra_touched_wall = False
        self.touched_triangle = False
        dx, dy = STEP_TUPLE_FROM_DIR[self._dir]
        self.last_dir = (dx, dy)
        nx, ny = self.x + dx, self.y + dy
        k = "free" if self.noclip else self._room.kind(nx + self._room.offset, ny)
        self.garra_touched_wall = k in ("wall", "obstacle")
        if self.garra_touched_wall:
            return False

        self.x, self.y = nx, ny


        if k in ("exit", "entrance"):
            self.exited = k
        return True

    def backyard(self):
        """Move one cell back; triangle contact restores the original tile."""
        self._pay("move")
        self.garra_touched_wall = False
        self.touched_triangle = False
        dx, dy = STEP_TUPLE_FROM_DIR[self._dir]
        self.last_dir = (-dx, -dy)
        nx, ny = self.x - dx, self.y - dy
        k = "free" if self.noclip else self._room.kind(nx + self._room.offset, ny)
        if k in ("wall", "obstacle"):
            self.garra_touched_wall = True
            return False

        self.x, self.y = nx, ny
        if self.on_triangle(room=self._room):
            self.x, self.y = nx + dx, ny + dy
            self.touched_triangle = True
            return False

        if k in ("exit", "entrance"):
            self.exited = k
        return True

    def measure(self, room = None):
        if room is None: room = self._room
        self._pay("measure")
        return room.reading(self.x + room.offset, self.y, self._dir)

    def on_triangle(self, room=None, dx=0, dy=0):
        """True if standing on a triangle corner (assumes that is how it is found)."""
        if room is None: room = self._room
        if room is None: return False

        from V3.r3_triangle_data import is_triangle
        return is_triangle(room, self.x + room.offset + dx, self.y + dy)

    def _pay(self, what):
        self.time += avg_time_per_action[what]
