"""Compatibility import for the physical R3 robot implementation."""

from Spike.V3.r3_real_robot import r3_real_robot


class r3_robot(r3_real_robot):
    """Backward-compatible name for :class:`r3_real_robot`."""

    pass
