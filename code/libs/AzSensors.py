from pybricks import *
from pybricks.hubs import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.pupdevices import *

# from Utills import *


class AzColorSensor(ColorSensor):

    # ===============[ Constructor ]===============

    def __init__(self, port):
        super().__init__(port)

        # Multipliers
        self.reflection_multiplier = 1.0

        self.rgb_r_multiplier = 1.0
        self.rgb_g_multiplier = 1.0
        self.rgb_b_multiplier = 1.0

        # Line weights
        self.line_r_weight = 1.55
        self.line_g_weight = 0.3
        self.line_b_weight = 1.05

        # Snap values
        self.reflection_snap_value = 0
        self.color_snap_value = None
        self.rgb_snap_value = (0, 0, 0)

    # ===============[ Reflection ]===============

    async def reflection(self):

        return (
            await super().reflection()
        ) * self.reflection_multiplier

    def reflection_snap(self):
        return self.reflection_snap_value

# ===============[ RGB ]===============

    async def rgb(self):

        hsv = await super().hsv()
        
        h = hsv.h
        s = hsv.s / 100
        v = hsv.v / 100

        c = v * s
        x = c * (1 - abs((h / 60) % 2 - 1))
        m = v - c

        if h < 60:
            rp, gp, bp = c, x, 0

        elif h < 120:
            rp, gp, bp = x, c, 0

        elif h < 180:
            rp, gp, bp = 0, c, x

        elif h < 240:
            rp, gp, bp = 0, x, c

        elif h < 300:
            rp, gp, bp = x, 0, c

        else:
            rp, gp, bp = c, 0, x
        r = (rp + m) * 100
        g = (gp + m) * 100
        b = (bp + m) * 100

        return (
            r * self.rgb_r_multiplier,
            g * self.rgb_g_multiplier,
            b * self.rgb_b_multiplier
        )


    def rgb_snap(self):
        return self.rgb_snap_value

    # ===============[ Color ]===============

    async def color(self):
        return await super().color()

    def color_snap(self):
        return self.color_snap_value

    # ===============[ Snap System ]===============

    async def snap(self):
        self.reflection_snap_value = await self.reflection()
        self.color_snap_value = await self.color()
        self.rgb_snap_value = await self.rgb()
        self.line_snap_value = await self.line()

    # ===============[ Color Helpers ]===============

    async def isColor(self, color):
        return (await self.color()) == color

    async def isGreen(self):
        return await self.isColor(Color.GREEN)

    async def isRed(self):
        return await self.isColor(Color.RED)

    async def isBlue(self):
        return await self.isColor(Color.BLUE)

    async def isWhite(self):
        return await self.isColor(Color.WHITE)

    async def isBlack(self):
        return await self.isColor(Color.BLACK)

    # ===============[ Line Detection ]===============

    async def line(self):

        r, g, b = await self.rgb()

        return (
            r * self.line_r_weight +
            g * self.line_g_weight +
            b * self.line_b_weight
        ) * 0.23

    def lineSnap(self):
        return self.line_snap_value

    # ===============[ Deep/Shallow Line ]===============

    async def isDeepLine(self):
        return (await self.line()) < 7

    async def isShallowLine(self):
        return (await self.line()) < 9

    def isDeepLineSnap(self):
        return self.lineSnap() < 7

    def isShallowLineSnap(self):
        return self.lineSnap() < 9

    # ===============[ Setters ]===============

    def setReflectionMultiplier(self, value):
        self.reflection_multiplier = value

    def setRGBMultipliers(self, r=1.0, g=1.0, b=1.0):

        self.rgb_r_multiplier = r
        self.rgb_g_multiplier = g
        self.rgb_b_multiplier = b

    def setLineWeights(self, r=1.0, g=1.0, b=1.0):

        self.line_r_weight = r
        self.line_g_weight = g
        self.line_b_weight = b


class AzDualColorSensor:

    # ===============[ Constructor ]===============

    def __init__(self, left_sensor, right_sensor, sensor_distance=15):

        self.left: AzColorSensor = left_sensor
        self.right: AzColorSensor = right_sensor

        # Distance between sensors in mm
        self.sensor_distance = sensor_distance

    # ===============[ Snap System ]===============

    async def snap(self):
        await self.left.snap()
        await self.right.snap()

    # ===============[ Reflection ]===============

    async def reflection(self):

        return (
            await self.left.reflection(),
            await self.right.reflection()
        )

    def reflection_snap(self):

        return (
            self.left.reflection_snap(),
            self.right.reflection_snap()
        )

    async def left_reflection(self):
        return await self.left.reflection()

    async def right_reflection(self):
        return await self.right.reflection()

    # ===============[ RGB ]===============

    async def rgb(self):

        return (
            await self.left.rgb(),
            await self.right.rgb()
        )

    def rgb_snap(self):

        return (
            self.left.rgb_snap(),
            self.right.rgb_snap()
        )

    async def left_rgb(self):
        return await self.left.rgb()

    async def right_rgb(self):
        return await self.right.rgb()

    # ===============[ RGB Difference ]===============

    async def rgb_difference(self):

        lr, lg, lb = await self.left.rgb()
        rr, rg, rb = await self.right.rgb()

        return (
            lr - rr,
            lg - rg,
            lb - rb
        )

    def rgb_snap_difference(self):

        lr, lg, lb = self.left.rgb_snap()
        rr, rg, rb = self.right.rgb_snap()

        return (
            lr - rr,
            lg - rg,
            lb - rb
        )

    # ===============[ Color ]===============

    async def color(self):

        return (
            await self.left.color(),
            await self.right.color()
        )

    def color_snap(self):

        return (
            self.left.color_snap(),
            self.right.color_snap()
        )

    async def left_color(self):
        return await self.left.color()

    async def right_color(self):
        return await self.right.color()

    # ===============[ Color Helpers ]===============

    async def isColor(self, color):

        return (
            await self.left.isColor(color),
            await self.right.isColor(color)
        )

    async def bothColor(self, color):

        return (
            await self.left.isColor(color) and
            await self.right.isColor(color)
        )

    async def eitherColor(self, color):

        return (
            await self.left.isColor(color) or
            await self.right.isColor(color)
        )

    # ===============[ Line ]===============

    async def line(self):

        return (
            await self.left.line(),
            await self.right.line()
        )

    def line_snap(self):
        return (
            self.left.lineSnap(),
            self.right.lineSnap()
        )

    async def left_line(self):
        return await self.left.line()

    async def right_line(self):
        return await self.right.line()

    def left_line_snap(self):
        return self.left.lineSnap()

    def right_line_snap(self):
        return self.right.lineSnap()

    # ===============[ Line Operations ]===============

    async def line_difference(self):
        return (
            await self.left.line() -
            await self.right.line()
        )

    def line_snap_difference(self):
        return (
            self.left.lineSnap() -
            self.right.lineSnap()
        )

    async def line_difference_sign(self):
        difference = await self.line_difference()

        if difference > 0:
            return 1

        if difference < 0:
            return -1

        return 0

    def line_snap_difference_sign(self):

        difference = self.line_snap_difference()

        if difference > 0:
            return 1

        if difference < 0:
            return -1

        return 0

    async def average_line(self):

        return (
            await self.left.line() +
            await self.right.line()
        ) / 2

    def average_line_snap(self):

        return (
            self.left.lineSnap() +
            self.right.lineSnap()
        ) / 2

    async def minimum_line(self):

        return min(
            await self.left.line(),
            await self.right.line()
        )

    def minimum_line_snap(self):

        return min(
            self.left.lineSnap(),
            self.right.lineSnap()
        )

    # ===============[ Deep/Shallow Line ]===============

    async def isDeepLine(self):

        return (
            await self.left.isDeepLine(),
            await self.right.isDeepLine()
        )

    async def isShallowLine(self):

        return (
            await self.left.isShallowLine(),
            await self.right.isShallowLine()
        )

    def isDeepLineSnap(self):

        return (
            self.left.isDeepLineSnap(),
            self.right.isDeepLineSnap()
        )

    def isShallowLineSnap(self):

        return (
            self.left.isShallowLineSnap(),
            self.right.isShallowLineSnap()
        )

    async def isAnyDeepLine(self):

        return (
            await self.left.isDeepLine() or
            await self.right.isDeepLine()
        )

    async def isAnyShallowLine(self):

        return (
            await self.left.isShallowLine() or
            await self.right.isShallowLine()
        )

    def isAnyDeepLineSnap(self):

        return (
            self.left.isDeepLineSnap() or
            self.right.isDeepLineSnap()
        )

    def isAnyShallowLineSnap(self):

        return (
            self.left.isShallowLineSnap() or
            self.right.isShallowLineSnap()
        )

    # ===============[ Distance ]===============

    def setSensorDistance(self, distance_mm):
        self.sensor_distance = distance_mm

    # ===============[ Helpers ]===============

    def sensors(self):
        return (self.left, self.right)