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

    def dir(self):
        return self._dir

    def turn_right(self):
        self._turn(+1)

    def turn_left(self):
        self._turn(-1)


