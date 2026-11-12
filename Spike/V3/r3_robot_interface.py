from V3.r3_model import r3_model


class r3_robot_interface:
    """Shared pose and environment interface for simulated and real robots."""

    def __init__(self, room=None):
        self._room = room
        self.x, self.y, self._dir = 0, 0, 0
        self.exited = None             # None | 'exit' | 'entrance'
        self.noclip = False            # if True, simulation ignores obstacles
        self.garra_touched_wall = False
        self.touched_triangle = False
        self.last_dir = (0, 0)
        self.log = []

    # What a robot can know: pose in the entrance frame (starts 0,0,up).
    def pose(self):
        return self.x, self.y, self._dir

    # Used by render()/debugging only.
    def _x(self):
        return self.x + self._room.offset

    def _y(self):
        return self.y

    def dir(self):
        return self._dir

    def turn_right(self):
        self._turn(+1)

    def turn_left(self):
        self._turn(-1)

    def kind(self, room=None):
        if room is None:
            room = self._room
        return room.kind(self.x + room.offset, self.y)


