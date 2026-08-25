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
        self.ctx = robot.ctx
        self.input = -1
        self.output = -1
        self.triangle1 = -1
        self.triangle2 = -1
        self.U = 260
        self.pos = (0, 0)
        self.crossed = []
        pass

    def mark_triangle(self, t):
        if self.triangle1 == -1:
            self.triangle1 = t
        else:
            self.triangle2 = t

    def room3_in_out_img(self):
            s = """
            9IHG9
            J...F
            K...E            
            L...D
            9ABC9
            """.replace("-ABCDEFGHIJKL-"[int(self.input)], "4").replace("-ABCDEFGHIJKL-"[int(self.output)], "0")

            for i in range(12):
                s = s.replace("ABCDEFGHIJKL"[i], "9")

            return s

    def markInOutPosDisplay(self):
        self.ctx.show_string_encoded_img(self.room3_in_out_img())

    def markOutput(self, n):
        if n != -1 and not n == self.input:
            print(f"Marked output {n}!")
        self.markInOutPosDisplay()

    def markInput(self, n):
        if n != -1 and not n == self.output:
            print(f"Marked input {n}!")
            self.input = n
        self.markInOutPosDisplay()
        

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
                self.markOutput(n)

            return maxDist > self.U * 3

    def isSolid(self, n):
        return ((self.input != -1 or n in self.crossed) and self.input != n) and self.output != n

async def room3check(robot: LineFollowerRobot, ctx: Context):
    print("Checking for room3!!")
    await robot.drive_base.straight(70) # Drive straight to check wether its a ramp
    print(ctx.imu.tilt()[0])
    if not abs(ctx.imu.tilt()[0]) > 6: # If it's not tilted, run room3
        await robot.drive_base.straight(-80)
        await room3(robot, ctx)
    else:
        await robot.drive_base.straight(-30)

async def straight_back_untill_black_or_wall_and_mark(n, robot: LineFollowerRobot, data):
    base = robot.drive_base
    start_dist = base.distance()
    start_heading = robot.ctx.imu.heading()
    triangle = False
    while True:
        base.drive(-100, 0)
        if abs(base.distance() - start_dist) > data.U * 3.5:
            break
        if abs(robot.L_motor.load()) > 180 or (abs(robot.ctx.imu.heading() - start_heading) > 4 and abs(robot.L_motor.load()) > 90):
            if abs(robot.L_motor.load()) > 180:
                await base.straight(-50)

            print("detected load!")
            if abs(robot.ctx.imu.heading() - start_heading) > 4:
                triangle = True 
                print("Triangle!")
            break
        if  await robot.R_sensor.reflection() < 30:
            data.markOutput(n)
            await base.straight(data.U * 0.5)
            base.brake()
            break
    base.brake()
    return abs(base.distance() - start_dist) + 20 , triangle

async def room3(robot: LineFollowerRobot, ctx: Context):
    print("Running room 3!!")
    base = robot.drive_base
    data = Room3info(robot)
    data.markInOutPosDisplay()

    robot.drive_base.use_gyro(True)
    # ctx.imu.reset_heading
    
    await align_room_3_entrance(robot, ctx, data)
    robot.drive_base.use_gyro(True)

    await base.straight(data.U * 0.5 + 80)
    await base.turn(90)

    await data.testOutputIfMissing(4)
    if await robot.distance_sensor.distance() < 100:
        base.use_gyro(False)
        await base.straight(200)
        base.use_gyro(True)

    await robot.garraDown()

    # Passing on low sector, testing entrance and testing out 44    
    input_distance_from_left_wall, triangle = await straight_back_untill_black_or_wall_and_mark(4, robot, data)

    if triangle:
        data.mark_triangle("0-1")

    if triangle:
        input_distance_from_left_wall += data.U

    data.markInput(int(1 + floor(0.1 + (input_distance_from_left_wall / data.U)))) # Calculate input pos from input_distance_from_left_wall
    await robot.garraUp()
    await base.straight(-20)

    # Afterwards, turn and move to mid sector. Contour triangle if its there
    if not triangle:
        await base.turn(90)
        await robot.garraDown()
        await data.testOutputIfMissing(1)
        await base.straight(-data.U)
        await robot.garraUp()
        await base.turn(90)
    else:
        await base.straight(20)
        await robot.garraDown()
        await base.straight(-30)
        await base.curve(-270, -90)
        await robot.garraUp()
        await data.testOutputIfMissing(1)
        await base.turn(90)


    # Mid sector
    await robot.garraDown()
    await straight_back_untill_black_or_wall_and_mark(5, robot, data)
    await robot.garraUp()
    await base.straight(data.U * 1.2)

    # Moving to high sector
    await base.turn(-90)
    await data.testOutputIfMissing(2)

    await robot.garraDown()
    await straight_back_untill_black_or_wall_and_mark(8, robot, data)
    # await base.straight(40)
    await robot.garraUp()
    await base.straight(-30)

    await base.turn(90)

    # High sector
    await robot.garraDown()
    a, triangle = await straight_back_untill_black_or_wall_and_mark(6, robot, data)

    if triangle:
        data.mark_triangle("6-7")


    await robot.garraUp()
    if not triangle: # If theres a triangle on sector 67, them skip testting out 7
        await base.turn(90)
        await data.testOutputIfMissing(7)
        await base.turn(90)
    else:
        await base.turn(180)

    await robot.garraDown()
    a, triangle = await straight_back_untill_black_or_wall_and_mark(10, robot, data)
    await base.straight(90)
    await robot.garraUp()

    # Currently in sector 9-10
    
    await base.turn(-90)

    if not triangle: # Only test 9 if has not a triangle at 9-10 sector
        await data.testOutputIfMissing(9)
    await robot.garraDown()

    await base.straight(-data.U)
    await robot.garraUp()
    await base.turn(-90)
    await robot.garraDown()  
    if not triangle: # Only do straight line untill center if there is not a triangle at 9-10 sector
        await base.straight(-data.U)
    await robot.garraUp()

    if   data.triangle1 == "0-1":
        await base.turn(-45)
    elif data.triangle1 == "6-7":
        await base.turn(90 + 45)
    elif data.triangle1 == "9-10":
        await base.turn(45)
    elif data.triangle1 == "3-4":
        await base.turn(-90 - 45)

    for i in range(2):
        await robot.garraDown()
        await base.straight(2 *data.U)
        await base.straight(-data.U)


    if   data.triangle1 == "0-1":
        await base.turn(135)
    elif data.triangle1 == "6-7":
        await base.turn(-135)
    elif data.triangle1 == "9-10":
        await base.turn(45)
    elif data.triangle1 == "3-4":
        await base.turn(-45)

    if data.output in [12, 4]:
        await base.straight(-data.U)
    if data.output in [10, 6]:
        await base.straight(data.U)


    if data.output in [4,  5,  6, 3, 7]:
        await base.turn(90)
    if data.output in [10, 11, 12, 9, 1]:
        await base.turn(-90)

    if data.output in [1, 3, 7, 9]:
        await base.straight(data.U)
        if data.output in [1, 7]:
            await base.turn(-90)
        if data.output in [3, 9]:
            await base.turn(90)

    await base.straight(data.U)

    
async def align_room_3_entrance(robot: LineFollowerRobot, ctx: Context, data:Room3info):
    await robot.drive_base.straight(40)
    await robot.alignOnLinePerTime(time=4, black_threshold=90, k=3, speedK = 5, doPreAlign=False)

# async def move_untill_black_or_stopped_moving():



