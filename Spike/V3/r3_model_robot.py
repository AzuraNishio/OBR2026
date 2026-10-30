from V3.obrExperimentalData import *
import V3.NekoMath

class r3_model_robot:
    def __init__(self, room):
        self._room = room              # hidden truth
        self.x, self.y, self._dir = 0, 0, 0
        self.exited = None             # None | 'exit' | 'entrance'
        self.time = 0.0
        self.log = []
        self.noclip = False                 # if True, robot can move through walls/obstacles
        self.pushed_on = 0
        self.last_dir = (0, 0)

    def copy(self):
        n = r3_model_robot(self._room)
        n.x, n.y, n._dir =    self.x, self.y, self._dir
        n.exited          =    self.exited
        n.last_dir        =    self.last_dir
        n.noclip          =    self.noclip
        n.time            =    self.time
        n.pushed_on       =    self.pushed_on
        n.log             =    self.log
        return n


    # what a real robot can know: pose in the ENTRANCE frame (starts 0,0,up)
    @property
    def pose(self):
        return (self.x, self.y, self._dir)

    # used by render()/debugging only
    @property
    def _x(self): return self.x + self._room.offset
    @property
    def _y(self): return self.y
    @property
    def dir(self): return self._dir

    def turn_right(self): self._turn(+1)
    def turn_left(self):  self._turn(-1)
    def _turn(self, s):
        self._dir = (self._dir + s) % 4
        self._pay("turn")

    def forward(self):
        """Move one cell. Returns False if blocked (wall/obstacle)."""
        self._pay("move")
        dx, dy = V3.NekoMath.STEP_TUPLE_FROM_DIR[self._dir]
        self.last_dir = (dx, dy)
        nx, ny = self.x + dx, self.y + dy
        k = self._room.kind(nx + self._room.offset, ny)
        if k in ("wall", "obstacle") and not self.noclip:
            self.pushed_on = 1
            return False
        self.x, self.y = nx, ny
        if k in ("exit", "entrance"):
            self.exited = k
        self.pushed_on = 0
        return True

    def backyard(self):
        """Move one cell back. Returns False if blocked (wall/obstacle)."""
        self._pay("move")
        dx, dy = V3.NekoMath.STEP_TUPLE_FROM_DIR[self._dir]
        self.last_dir = (-dx, -dy)
        nx, ny = self.x - dx, self.y - dy
        k = self._room.kind(nx + self._room.offset, ny)
        if k in ("wall", "obstacle") and not self.noclip:
            self.pushed_on = -1
            return False
        self.x, self.y = nx, ny
        if k in ("exit", "entrance"):
            self.exited = k
        self.pushed_on = 0
        return True

    def measure(self, room = None):
        if room is None: room = self._room
        self._pay("measure")
        return room.reading(self.x + room.offset, self.y, self._dir)

    def kind(self, room=None):
        if room is None: room = self._room
        return room.kind(self.x + room.offset, self.y)

    def on_triangle(self, room = None):
        """True if standing on a triangle corner (assumes that is how it is found)."""
        if room is None: room = self._room

        return any(room.corner_cell(c) == (self.x + room.offset, self.y)
                   for c in room.triangles)

    def _pay(self, what):
        self.time += avg_time_per_action[what]
        self.log.append(what)
