# import inspect

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
from constants import room3_const, surprises
from V3.r3_algorithm import *

async def beep_n(robot: LineFollowerRobot, times):
    robot.ctx.speaker.volume(100)
    for i in range(times):
        await robot.ctx.wait(0.1)
        await robot.ctx.speaker.beep()


def stage_log(label: str, text: str):
    print(label)
    print(text)


class Room3info:
    def __init__(self, robot: LineFollowerRobot):
        self.robot = robot
        self.ctx = robot.ctx
        self.input = -1
        self.output = -1
        self.triangle1 = -1
        self.triangle2 = -1
        self.U = room3_const.U
        self.pos = (0, 0)
        self.crossed = []
        pass

    def mark_triangle(self, t):
        if self.triangle1 == -1:
            self.triangle1 = t
        else:
            self.triangle2 = t
        self.markInOutPosDisplay()

    def room3_in_out_img(self):
            s = """
            9IHG9
            Jz.yF
            K...E            
            Lw.xD
            9ABC9
            """.replace("-ABCDEFGHIJKL-"[int(self.input)], "4").replace("-ABCDEFGHIJKL-"[int(self.output)], "0").replace("wxyz-"[["0-1", "3-4", "6-7", "9-10", -1].index(self.triangle1)], "9").replace("wxyz-"[["0-1", "3-4", "6-7", "9-10", -1].index(self.triangle1)], "6")

            for i in range(12):
                s = s.replace("ABCDEFGHIJKL"[i], "9")

            for i in range(4):
                s = s.replace("wxyz"[i], "0")

            return s

    def markInOutPosDisplay(self):
        self.ctx.show_string_encoded_img(self.room3_in_out_img())

    def markOutput(self, n):
        if self.output == -1 and not n == self.input:
            print(f"Marked output {n}!")
            self.output = n
        self.markInOutPosDisplay()

    def markInput(self, n):
        if n != -1 and not n == self.output:
            print(f"Marked input {n}!")
            self.input = n
        self.markInOutPosDisplay()
        

    async def testOutputIfMissing(self, n, mark_dist = room3_const.U * 3):
        self.crossed.append(n)
        if self.output == -1:
            start_angle = self.robot.ctx.imu.heading()
            await self.robot.drive_base.turn(-6)
            self.robot.drive_base.turn(12)

            smooth = 0
            maxDist = 0

            while not self.robot.drive_base.done():
                dist = await self.robot.distance_sensor.distance()
                smooth = (smooth * 4 + dist) * 0.2
                maxDist = max(maxDist, smooth)
            self.robot.drive_base.brake()

            await self.robot.drive_base.turn(start_angle - self.robot.ctx.imu.heading())

            if maxDist > mark_dist:
                self.markOutput(n)

            return maxDist > mark_dist

    def isSolid(self, n):
        return ((self.input != -1 or n in self.crossed) and self.input != n) and self.output != n

async def room3check(robot: LineFollowerRobot, ctx: Context):
    """
               _9_8_7_
           10 |       | 6
           11 |       | 5
           12 |___^___| 4   ^ robot approaching room 3 entrance, about to test for ramp
                1 2 3
    """
    if not abs(ctx.imu.tilt()[0]) > 5: # If it's not tilted, run room3
        print("Checking for room3!!")
        for i in range(6):
            await robot.straight(70 / 6) # Drive straight to check wether its a ramp
            if abs(ctx.imu.tilt()[0]) > 5: # If tilted, move until not tilted OR 30 cm
                for j in range(6):
                    await robot.straight(50) # Drive straight to check wether its not a ramp
                    error = await robot.L_sensor.reflection() - await robot.R_sensor.reflection()
                    if abs(error) > 30:
                        await robot.turn(0.1 * error)
                    if abs(ctx.imu.tilt()[0]) < 6: # If it's not tilted stop moving
                        break
                return
                
        await robot.turn(20)
        if await robot.is_seeing_any_black():
            await robot.turn(-20)
            return
        await robot.turn(-20)
        
        # Surprise challenge (ignore)
        if surprises.signal_on_room3:
            await robot.signal_blink_ultrasonic()

        await room3(robot, ctx)
              

async def room3(robot: LineFollowerRobot, ctx: Context):

    robot.set_modified_straight_speed(room3_const.straight_speed)
    print("Running room 3!!")
    data = Room3info(robot)
    robot.data = data
    data.markInOutPosDisplay()

    robot.drive_base.use_gyro(True)
    alg = algorithm(r3_robot_interface(ctx, robot))

    while True:
        await alg.measure()
        finished = alg.step()
        await alg.robot.update_async()
        if finished: break

    robot.clear_modified_straigth_speed()

    