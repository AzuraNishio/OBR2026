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
from constants import room3_const

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
        if self.output == -1 and not n == self.input:
            print(f"Marked output {n}!")
            self.output = n
        self.markInOutPosDisplay()

    def markInput(self, n):
        if n != -1 and not n == self.output:
            print(f"Marked input {n}!")
            self.input = n
        self.markInOutPosDisplay()
        

    async def testOutputIfMissing(self, n):
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

            if maxDist > self.U * 3:
                self.markOutput(n)

            return maxDist > self.U * 3

    def isSolid(self, n):
        return ((self.input != -1 or n in self.crossed) and self.input != n) and self.output != n

async def room3check(robot: LineFollowerRobot, ctx: Context):
    if not abs(ctx.imu.tilt()[0]) > 6: # If it's not tilted, run room3
        print("Checking for room3!!")

        await robot.straight(70) # Drive straight to check wether its a ramp
        if not abs(ctx.imu.tilt()[0]) > 6: # If it's not tilted, run room3
            await room3(robot, ctx)
        else:
            await robot.straight(-30)

async def align_wall_distance_if_reasonable(robot: LineFollowerRobot, distance, n = -2):
    start_dist = await robot.distance_sensor.distance()
    displaced_dist = distance - 110
    error = start_dist - displaced_dist 
    if error < 700 and error > -300 and not robot.data.output == n:
        await robot.straight(error)
        await robot.straight(displaced_dist - await robot.distance_sensor.distance())   

async def straight_back_untill_black_or_wall_and_mark(n, robot: LineFollowerRobot, data: Room3info, do_triangle = True):
    base = robot.drive_base
    start_dist = base.distance()
    await robot.straight(-40)
    await robot.ctx.wait(0.5)
    start_heading = robot.ctx.imu.heading()
    triangle = False
    # base.use_gyro(False)
    sonic_check_time = robot.ctx.time()
    sonic_dist = await robot.distance_sensor.distance()
    while True:
        base.drive(-room3_const.straight_till_speed, 0)
        if robot.ctx.time() - sonic_check_time > 1:
            dist = await robot.distance_sensor.distance()
            sonic_check_time = robot.ctx.time() 
            if abs(dist - sonic_dist) < 1 and dist < 1000:
                break
            sonic_dist = dist
            
        if abs(base.distance() - start_dist) > data.U * 3.6:
            break
        left_motor_load = abs(robot.L_motor.load())
        right_motor_load = abs(robot.R_motor.load())
        heading_error = abs(robot.ctx.imu.heading() - start_heading)
        wall_detected = (
            left_motor_load > room3_const.straight_to_wall_torque_threshold
            or right_motor_load > room3_const.straight_to_wall_torque_threshold
        )
        triangle_detected = (
            do_triangle and
            heading_error > room3_const.triangle_angle_threshold
            and (
                left_motor_load > room3_const.triangle_single_motor_torque
                or right_motor_load > room3_const.triangle_single_motor_torque
            )
        )

        if wall_detected or triangle_detected:
            if left_motor_load > room3_const.straight_to_wall_torque_threshold:
                w = robot.ctx.get_waiter(0.5)
                cancel = False
                while True:
                    if w:
                        break
                    robot.drive(-room3_const.straight_till_speed, 0)
                    if abs(robot.L_motor.load()) < room3_const.straight_to_wall_torque_threshold * 0.9:
                        cancel = True
                        break
                if cancel:
                    continue
                await robot.ctx.speaker.beep()

            print("detected load!")
            if triangle_detected:
                triangle = True 
                print("Triangle!")
                await robot.straight(20)
                await robot.turn(start_heading - robot.ctx.imu.heading())
                await robot.ctx.speaker.beep()
                await robot.ctx.wait(0.2)
                await robot.ctx.speaker.beep()

            break
        if await robot.is_seeing_any_black():
            data.markOutput(n)
            base.brake()


            await robot.ctx.speaker.beep()
            await robot.ctx.wait(0.2)
            await robot.ctx.speaker.beep()
            await robot.ctx.wait(0.2)
            await robot.ctx.speaker.beep()

            await robot.garraUp()
            await base.straight(data.U * 0.7)
            base.brake()
            break

    base.brake()
    base.use_gyro(True)
    return abs(base.distance() - start_dist) + 20 , triangle

async def align_at_wall_if_close(robot: LineFollowerRobot, wall = None, reposition = True):
    if wall:
        print(f"Aligning at wall {wall}")
    start_dist = await robot.distance_sensor.distance()
    if start_dist < 150:
        robot.use_gyro(False)
        await robot.straight(170)
        robot.use_gyro(True)
        if reposition:
            await robot.straight(-start_dist * 0.9)


async def room3(robot: LineFollowerRobot, ctx: Context):
    robot.set_modified_straight_speed(room3_const.straight_speed)
    print("Running room 3!!")
    data = Room3info(robot)
    robot.data = data
    data.markInOutPosDisplay()

    robot.drive_base.use_gyro(True)

    await robot.straight(data.U * 0.6)
    await robot.turn(95)

    await align_at_wall_if_close(robot, 4)
    await data.testOutputIfMissing(4)

    await robot.straight(50)
    await robot.garraDown()

    # Passing on low sector, testing entrance and testing out 44    
    input_distance_from_left_wall, triangle = await straight_back_untill_black_or_wall_and_mark(12, robot, data)

    if triangle:
        data.mark_triangle("0-1")

    if triangle:
        input_distance_from_left_wall += data.U

    data.markInput(int(1 + floor(0.1 + (input_distance_from_left_wall / data.U)))) # Calculate input pos from input_distance_from_left_wall

    # Afterwards, turn and move to mid sector. Contour triangle if its there
    if not triangle:
        await robot.garraUp()
        await robot.straight(-20)
        await robot.turn(90)
        await robot.garraDown()
        await data.testOutputIfMissing(1)
        await align_at_wall_if_close(robot, 1, reposition=True)
        await robot.straight(-data.U - 150)
        await robot.garraUp()
        await robot.turn(90) # Facing wall 11, garra to wall 5
        await align_at_wall_if_close(robot, 11, reposition=False)
    else:
        await robot.straight(-30)
        await robot.curve(-(data.U - 30), -90)
        await robot.straight(-50)
        await robot.garraUp()
        await robot.straight(-100)

        await data.testOutputIfMissing(1)
        await robot.turn(90)
        await align_at_wall_if_close(robot, 11, reposition=False)

    # Mid sector
    await data.testOutputIfMissing(11)

    await robot.garraDown()
    await straight_back_untill_black_or_wall_and_mark(5, robot, data, do_triangle=False)
    await robot.garraUp()
    await align_wall_distance_if_reasonable(robot, data.U * 2.5, n = 11) 
    await robot.curve(-80, -90)
    await align_wall_distance_if_reasonable(robot, data.U * 1.55) 
    await robot.curve(-80, -100)
    await data.testOutputIfMissing(5)


    await align_at_wall_if_close(robot, 5, reposition=False)
    await robot.garraDown()
    await straight_back_untill_black_or_wall_and_mark(11, robot, data)
    await robot.garraUp()
    await robot.straight(-data.U * -1)
    await align_wall_distance_if_reasonable(robot, data.U * 1.3, n = 5) 
    # At this moment the robot should be at the exaclty middle! <-----------------------------------------

    # Moving to high sector
    await robot.garraUp()
    await robot.turn(90)
    await data.testOutputIfMissing(2)
    await robot.straight(60)

    await robot.garraDown()
    await straight_back_untill_black_or_wall_and_mark(8, robot, data, do_triangle=False)
    # await base.straight(40)
    await robot.garraUp()
    await robot.straight(-60)

    await robot.turn(90)

    # High sector
    await robot.straight(60)

    await robot.garraDown()
    a, triangle = await straight_back_untill_black_or_wall_and_mark(6, robot, data)

    if triangle:
        data.mark_triangle("6-7")


    await robot.garraUp()
    if not triangle: # If theres a triangle on sector 67, them skip testting out 7
        await robot.turn(90)
        await data.testOutputIfMissing(7)
        await robot.turn(90)
    else:
        await robot.turn(180)

    await align_at_wall_if_close(robot, 4)


    await robot.garraDown()
    a, triangle = await straight_back_untill_black_or_wall_and_mark(10, robot, data)
    # Currently in sector 9-10

    await robot.garraUp()
    if data.output == 10: # Do only if found exit at 10
        await robot.straight(90)

    
    await robot.turn(-90)

    if not triangle: # Only test 9 if has not a triangle at 9-10 sector
        await data.testOutputIfMissing(9)
    await robot.garraDown()

    await robot.straight(-data.U)
    await robot.garraUp()

    triangle_turn_offset = 0

    if data.output != 5:
        await robot.turn(90)
        triangle_turn_offset = 180
        await robot.straight(data.U * 2)
        await align_at_wall_if_close(robot, 11, reposition=False)
        await robot.straight(data.U * -1.5)

        await align_wall_distance_if_reasonable(robot, data.U * 1.6) 
    else:
        await robot.turn(-90)
        await robot.straight(data.U * 2)
        await align_at_wall_if_close(robot, 11, reposition=False)
        await robot.straight(data.U * -1.5)

        await align_wall_distance_if_reasonable(robot, data.U * 1.6) 

    if not triangle: # Only do straight line untill center if there is not a triangle at 9-10 sector
        await robot.straight(-data.U)
    await robot.garraUp()

    if   data.triangle1 == "6-7":
        await robot.turn(-45 + triangle_turn_offset)
    elif data.triangle1 == "0-1":
        await robot.turn(90)
        await align_wall_distance_if_reasonable(robot, data.U * 1.6) 
        await robot.turn(45 - triangle_turn_offset)
    elif data.triangle1 == "3-4":
        await robot.turn(45 - triangle_turn_offset)
    elif data.triangle1 == "9-10":
        await robot.turn(-90)
        await align_wall_distance_if_reasonable(robot, data.U * 1.6) 
        await robot.turn(- 45 + triangle_turn_offset)

    triangles_aligned = (data.triangle1 in ["0-1", "6-7"] and data.triangle2 in ["0-1", "6-7"]) or (data.triangle1 in ["3-4", "9-10"] and data.triangle2 in ["3-4", "9-10"])
    
    await robot.garraDown()
    await robot.straight(-2 *data.U)
    await robot.garraUp()
    await robot.turn(180)

    await robot.garraDown()

    if triangles_aligned:
        await robot.straight(-3 *data.U)
        await robot.garraUp()
        await robot.garraDown()
        await robot.straight(2 *data.U)


    await robot.straight(2 *data.U)
    await robot.straight(-data.U * 0.5)
    await robot.garraUp()
    await robot.garraDown()
    await robot.straight(2 *data.U)
    await robot.straight(-data.U * 0.5)


    await robot.straight(-data.U * 0.7)

    await robot.garraUp()

    if   data.triangle1 == "0-1":
        await robot.turn(135)
    elif data.triangle1 == "6-7":
        await robot.turn(-135)
    elif data.triangle1 == "9-10":
        await robot.turn(45)
    elif data.triangle1 == "3-4":
        await robot.turn(-45)

    if data.output in [12, 4]:
        await robot.straight(-data.U * 1.3)
    if data.output in [10, 6]:
        await robot.straight(data.U * 1.3)


    if data.output in [4,  5,  6, 3, 7]:
        await robot.turn(90)
    if data.output in [10, 11, 12, 9, 1]:
        await robot.turn(-90)

    if data.output in [1, 3, 7, 9]:
        await robot.straight(data.U * 1.3)
        if data.output in [1, 7]:
            await robot.turn(-90)
        if data.output in [3, 9]:
            await robot.turn(90)

    await robot.straight(data.U * 0.7)
    start_angle = ctx.imu.heading()
    while True:
        robot.drive(200, 0)
        if await robot.is_seeing_any_black():
            break

        if abs(start_angle - ctx.imu.heading()) > 7:
            await robot.straight(-80)
            await robot.turn((start_angle - ctx.imu.heading()) * 2)
            await ctx.wait(0.5)
            start_angle = ctx.imu.heading()

    robot.brake()
    await robot.straight(-50)
    await robot.alignOnLinePerTime(3)
    await robot.straight(80)
    await robot.turn(-30)
    robot.turn(120)
    while not robot.drive_base.done():
        if await robot.L_sensor.reflection() < 30:
            break
    
    robot.brake()
    robot.turn(-20)

    while not robot.drive_base.done():
        if await robot.L_sensor.reflection() > 30:
            break

    robot.brake()

    robot.clear_modified_straigth_speed()

    
async def align_room_3_entrance(robot: LineFollowerRobot, ctx: Context, data:Room3info):
    await robot.straight(40)
    await robot.alignOnLinePerTime(time=4, black_threshold=90, k=3, speedK = 5, doPreAlign=False)

# async def move_untill_black_or_stopped_moving():



