from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.parameters import *
from libs.Utills import *
from libs.AwareRobotics.AwDriveBase import *
from LineFollowerRobot import LineFollowerRobot
from Scaffolding import *
from math import *

class Room3info:
    def __init__(self, robot: LineFollowerRobot):
        self.robot = robot
        self.input = -1
        self.output = -1
        self.triangle1 = -1
        self.triangle2 = -1
        self.U = 300
        self.pos = (0, 0)
        self.crossed = []
        pass

    async def testOutputIfMissing(self, n):
        self.crossed.append(n)
        if self.output == -1:

            await self.robot.drive_base.curve(0, -5)
            self.robot.drive_base.curve(0, 10)

            smooth = 0
            maxDist = 0

            while not self.robot.drive_base.done():
                dist = await self.robot.distance_sensor.distance()
                smooth = (smooth * 4 + dist) * 0.2
                maxDist = max(maxDist, smooth)
            self.robot.drive_base.hold()

            await self.robot.drive_base.curve(0, -5)

            if maxDist > self.U * 3:
                print(f"Marked output {n}!")
                self.output = n

            return maxDist > self.U * 3

    def isSolid(self, n):
        return ((self.input != -1 or n in self.crossed) and self.input != n) and self.output != n

async def room3check(robot: LineFollowerRobot, ctx: Context):
    print("Checking for room3!!")
    await robot.drive_base.straight(70) # Drive straight to check wether its a ramp
    if not abs(ctx.imu.tilt()[0]) > 10: # If it's not tilted, run room3
        await robot.drive_base.straight(-80)
        await room3(robot, ctx)
    else:
        await robot.drive_base.straight(-30)

async def room3(robot: LineFollowerRobot, ctx: Context):
    print("Running room 3!!")
    data = Room3info(robot)
    robot.drive_base.use_gyro(True)
    ctx.imu.reset_heading
    
    await align_room_3_entrance(robot, ctx, data)
    await test_room_3_entrance_position(robot, ctx, data)


async def align_room_3_entrance(robot: LineFollowerRobot, ctx: Context, data:Room3info):
    await robot.drive_base.straight(40)
    await robot.alignOnLinePerTime(time=4, black_threshold=90, k=4, speedK = 0.025, doPreAlign=False)


async def test_room_3_entrance_position(robot: LineFollowerRobot, ctx: Context, data:Room3info):
    await robot.drive_base.straight(data.U * 0.5 + 80)
    detectedAligned = await data.testOutputIfMissing(-1)
    await robot.drive_base.curve(0, 90)
    check = await data.testOutputIfMissing(4)
    if check:
        await robot.drive_base.straight(-70)
        await robot.drive_base.curve(0, -180)
        inputDistFromLeft = await robot.distance_sensor.distance() + 70
        check = await data.testOutputIfMissing(12)
        await robot.driveUntillConditions(200, 0, ultrasonic_less_than= -70 + ( data.U * 1.5) )
        await robot.driveUntillConditions(-60, 0, ultrasonic_more_than= -70 + ( data.U * 1.5) )
        await robot.drive_base.curve(0, 90)
    else:
        inputDistFromLeft = (data.U * 3) - (await robot.distance_sensor.distance() + 70)
        await robot.driveUntillConditions(200, 0, ultrasonic_less_than= -70 + ( data.U * 1.5) )
        await robot.driveUntillConditions(-60, 0, ultrasonic_more_than= -70 + ( data.U * 1.5) )
        await robot.drive_base.curve(0, -90)

    data.input = floor(inputDistFromLeft / data.U) + 1
    print(f"Found input at {data.input}!!")
    ctx.display.char(str(data.input))

    if detectedAligned:
        data.output = 10 - data.input
    
    if data.isSolid(2):
        await robot.drive_base.straight(-70)
    
    await robot.drive_base.straight(data.U)

    

    while True:
        pass

    


