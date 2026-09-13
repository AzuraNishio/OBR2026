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
                
async def align_wall_distance_if_reasonable(robot: LineFollowerRobot, distance, n = -2):
    # Generic wall-distance alignment helper, called from many positions inside the room -
    # no fixed spot to diagram here (see call sites for position context).
    start_dist = await robot.distance_sensor.distance()
    displaced_dist = distance - 110
    error = start_dist - displaced_dist 
    if error < 700 and error > -300 and not robot.data.output == n:
        await robot.straight(error)
        await robot.straight(displaced_dist - await robot.distance_sensor.distance())   

async def straight_back_untill_black_or_wall_and_mark(n, robot: LineFollowerRobot, data: Room3info, do_triangle = True):
    # Generic backward-drive-until-wall/line/triangle helper, reused from several
    # sectors with output id `n` - see each call site's diagram for the position context.
    print(f"Straight to wall {n} and checking!")
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
            if abs(dist - sonic_dist) < 4 and dist < 1300:
                break
            sonic_dist = dist
            
        if abs(base.distance() - start_dist) > data.U * 4:
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
                await robot.straight(50)
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
            await base.straight(data.U * 0.74)
            base.brake()
            break

    base.brake()
    base.use_gyro(True)
    return abs(base.distance() - start_dist) + 20 , triangle

async def align_at_wall_if_close(robot: LineFollowerRobot, wall = None, reposition = True, max_dist = 150):
    # Generic "nudge to wall if within range" helper - position depends on caller,
    # see the diagram at each call site.
    if wall:
        print(f"Aligning at wall {wall}")
    start_dist = await robot.distance_sensor.distance()
    if start_dist < max_dist and not robot.data.output == wall:
        robot.use_gyro(False)
        await robot.straight(max_dist + 30)
        robot.use_gyro(True)
        if reposition:
            await robot.straight(-start_dist * 0.9)

async def align_at_wall_or_black_if_close(robot: LineFollowerRobot, wall = None, reposition = True):
    # Same as align_at_wall_if_close, but also aligns on the line if the wall in
    # question turned out to be the (already found) output wall.
    if wall:
        print(f"Aligning at wall {wall}")
    start_dist = await robot.distance_sensor.distance()
    if start_dist < 150:
        robot.use_gyro(False)
        await robot.straight(170)
        robot.use_gyro(True)
        if reposition:
            await robot.straight(-start_dist * 0.9)
    elif wall == robot.data.output:
        robot.straight(50)
        while not robot.drive_base.done():
            if await robot.is_seeing_any_black():
                break
        robot.brake()
        await robot.alignOnLinePerTime(3)

async def room3_entrance_and_low_sector(robot: LineFollowerRobot, data: Room3info):
    stage_log("LOW-1", """
    Handle low sector (entrance and sectors 0-1, 4, 12).
               _9_8_7_
           10 |       | 6
           11 |_ _ _ _| 5
           12 |__low__| 4
                1 2 3
    """)
    await robot.turn(10)

    await robot.straight(data.U * 0.6)
    start_angle = robot.ctx.imu.heading()
    start_time  = robot.ctx.time()
    await robot.turn(85)



    stage_log("LOW-2", """
               _9_8_7_
           10 |       | 6
           11 |_ _ _ _| 5
           12 |__low__| 4   o robot after entering + turning 95, about to test wall 4
                1 2 3
    """)

    await align_at_wall_if_close(robot, 4)
    await data.testOutputIfMissing(4)

    await robot.straight(50)
    await robot.garraDown()

    stage_log("LOW-3", """
               _9_8_7_
           10 |       | 6
           11 |_ _ _ _| 5
           12 |_>_low_| 4   > robot moved off wall 4, garra down, heading back to test entrance
                1 2 3
    """)

    # Passing on low sector, testing entrance and testing out 4
    input_distance_from_left_wall, triangle = await straight_back_untill_black_or_wall_and_mark(12, robot, data)

    if triangle:
        data.mark_triangle("0-1")

    if triangle:
        input_distance_from_left_wall += data.U

    data.markInput(int(1 + floor(0.1 + (input_distance_from_left_wall / data.U))))

    stage_log("LOW-4", """
               _9_8_7_
           10 |       | 6
           11 |_ _ _ _| 5
           12 |<__low_| 4   < robot backed up along low sector, input marked, near wall 12/entrance
                1 2 3
    """)

    # Turn and move to mid sector. Contour triangle if it's there
    if not triangle:
        await robot.garraUp()
        await robot.straight(-20)
        await robot.turn(90)
        await robot.garraDown()
        await data.testOutputIfMissing(1)
        await align_at_wall_if_close(robot, 1, reposition=True)
        await robot.straight(-data.U - 100)
        await robot.garraUp()
    else:
        await robot.straight(-30)
        await robot.curve(-(data.U - 30), -90)
        await robot.straight(-50)
        await robot.garraUp()
        await robot.straight(-50)
        await data.testOutputIfMissing(1)

    await robot.turn(90)
    await data.testOutputIfMissing(11, mark_dist=data.U)

    await align_at_wall_if_close(robot, 11, reposition=False, max_dist = 600)

    stage_log("LOW-5", """
               _9_8_7_
           10 |       | 6
           11 |<=     | 5
           12 |_______| 4   ^ robot at end of low sector, facing wall 11, ready for mid sector
                1 2 3
    """)

async def room3_mid_sector(robot: LineFollowerRobot, data: Room3info):
    stage_log("MID-1", """
    Handle mid sector (sectors 5 and 11).
               _9_8_7_
           10 |       | 6
           11 |<=     | 5
           12 |_______| 4   <= robot with garra facing right
                1 2 3
    """)

    await robot.garraDown()
    await robot.straight(-data.U)
    await robot.turn(3)
    #await straight_back_untill_black_or_wall_and_mark(5, robot, data) #,do_triangle=False) #############################################################
    
    await robot.ctx.wait(0.2)
    start_angle = robot.ctx.imu.heading()
    
    start_dist = robot.drive_base.distance()###################################################################
    while True:
        error = start_angle - robot.ctx.imu.heading()
        robot.drive(-400, error * 16)
        if abs(robot.drive_base.distance() - start_dist) > data.U * 4:
            break
        if robot.is_forcing_motor(room3_const.straight_to_wall_torque_threshold):
            await robot.move_time(0.5)
            break
        if await robot.is_seeing_any_black():
            data.markOutput(5)
            robot.brake()
            await robot.garraUp()

            await robot.straight(data.U)
            await beep_n(robot, 3)
            break
        if abs(error) > 6:
            robot.brake()
            robot.brake()
            await beep_n(robot, 2)
            await robot.garraUp()
            await robot.straight(300)
            await robot.turn(error * 2)
            robot.straight(300)

    robot.brake()

    await robot.garraUp()

    stage_log("MID-2", """
               _9_8_7_
           10 |       | 6
           11 |=>     | 5   => robot crossed sector 11, garra up, heading right toward 5
           12 |_______| 4
                1 2 3
    """)

    await robot.curve(-80, -90)
    await align_wall_distance_if_reasonable(robot, data.U * 1.53)
    await robot.curve(-80, -100)
    await data.testOutputIfMissing(5)
    await robot.move_time(0.2)

    stage_log("MID-3", """
               _9_8_7_
           10 |       | 6
           11 |     ^ | 5   ^ robot curved up to wall 5, testing output
           12 |_______| 4
                1 2 3
    """)

    await align_at_wall_if_close(robot, 5, reposition=False)
    await robot.garraDown()
    await straight_back_untill_black_or_wall_and_mark(11, robot, data)
    await robot.garraUp()

    stage_log("MID-4", """
               _9_8_7_
           10 |       | 6
           11 |     <=| 5   <= robot crossing back across sector 5 toward 11
           12 |_______| 4
                1 2 3
    """)

    await robot.straight(-data.U * -1)
    await align_wall_distance_if_reasonable(robot, data.U * 1.3, n=5)
    # At this moment the robot should be at the exactly middle!
    stage_log("MID-5", """
               _9_8_7_
           10 |       | 6
           11 |  =>   | 5
           12 |_______| 4   <= robot with garra facing left
                1 2 3
    """)


async def room3_high_sector(robot: LineFollowerRobot, data: Room3info):
    stage_log("HIG-1", """
    Handle high sector (sectors 2, 6-7, 8, 9-10).
               _9_8_7_
           10 |       | 6
           11 |   ^   | 5   ^ robot at middle, about to turn up into high sector
           12 |_______| 4
                1 2 3
    """)
    # Moving to high sector
    await robot.garraUp()

    if data.output == 5: # If output is 5, align with distance on 11
        await robot.turn(180)
        await align_wall_distance_if_reasonable(robot, data.U * 1.3, n=11)
        await robot.turn(-90)
    else:
        await robot.turn(90)

    await data.testOutputIfMissing(2)
    await robot.straight(60)
    await robot.garraDown()
    await straight_back_untill_black_or_wall_and_mark(8, robot, data, do_triangle=False)
    await robot.garraUp()
    await robot.straight(-60)
    await robot.turn(90)

    stage_log("HIG-2", """
               _9_8_7_
           10 |  <=   | 6
           11 |       | 5
           12 |_______| 4   <= robot with garra facing right
                1 2 3
    """)

    # High sector
    await robot.straight(60)

    await robot.garraDown()
    a, triangle = await straight_back_untill_black_or_wall_and_mark(6, robot, data)

    if triangle:
        data.mark_triangle("6-7")

    await robot.garraUp()
    if not triangle:  # If there's a triangle on sector 6-7, skip testing out 7
        await robot.turn(90)
        await data.testOutputIfMissing(7)
        await robot.turn(90)
    else:
        await robot.turn(180)

    await align_at_wall_if_close(robot, 4)

    stage_log("HIG-3", """
               _9_8_7_
           10 |     =>| 6
           11 |       | 5
           12 |_______| 4   => robot with garra facing left
                1 2 3
    """)

    await robot.garraDown()
    a, triangle = await straight_back_untill_black_or_wall_and_mark(10, robot, data)
    # Currently in sector 9-10

    await robot.garraUp()
    if data.output == 10:  # Do only if found exit at 10
        await robot.straight(90)

    await robot.turn(-90)

    stage_log("HIG-4", """
               _9_8_7_
           10 |^      | 6
           11 |       | 5
           12 |_______| 4   ^ robot with garra facing down
                1 2 3
    """)

    if not triangle:  # Only test 9 if there's not a triangle at 9-10 sector
        await data.testOutputIfMissing(9)
    
    await robot.garraDown()
    await robot.straight(-data.U)
    await align_wall_distance_if_reasonable(robot, data.U * 1.3, n=9)
    await robot.garraUp()

    stage_log("HIG-5", """
               _9_8_7_
           10 |       | 6   ^ robot finished high sector sweep, garra down, facing up
           11 |^      | 5
           12 |_______| 4
                1 2 3
    """)


async def test(triangle, robot: LineFollowerRobot, output, ctx):
    data = Room3info(robot)
    data.markOutput(output)
    data.mark_triangle(triangle)
    robot.data = data
    robot.use_gyro(True)
    await aligned_on_triangle_deliver(robot, data)
    await room3_exit_maneuver(robot, ctx, data)

async def room3_alignment_and_delivery(robot: LineFollowerRobot, data: Room3info):
    """Handle triangle alignment and delivery sequence."""
    triangle_turn_offset = 0
    
    if data.output != 5:
        await robot.turn(90)
        triangle_turn_offset = 180
        await robot.straight(data.U * 2)
        await align_at_wall_if_close(robot, 5, reposition=False)
        await robot.straight(data.U * -1.3)
        await align_wall_distance_if_reasonable(robot, data.U * 1.6, n = 5)
    else:
        await robot.turn(-90)
        await robot.straight(data.U * 1)
        await align_at_wall_if_close(robot, 11, reposition=False)
        await robot.straight(data.U * -1.3)
        await align_wall_distance_if_reasonable(robot, data.U * 1.6, n = 11)


    stage_log("DELIVERY-1", """
               _9_8_7_
           10 |       | 6   if output at 5 facing <-----
           11 |   O   | 5   else  facing   -----> and triangle_turn_offset = 180
           12 |_______| 4   ^ robot with garra facing down
                1 2 3
    """)

    # Align based on triangle position
    print(f"Delivering at {data.triangle1}!")
    if data.triangle1 == "6-7":
        await robot.turn(-45 + triangle_turn_offset)
    elif data.triangle1 == "0-1":
        await robot.turn(90 + 45 - triangle_turn_offset)
    elif data.triangle1 == "3-4":
        await robot.turn(45 - triangle_turn_offset)
    else: # If none do 9-10
        await robot.turn(-90 - 45 + triangle_turn_offset)



    stage_log("DELIVERY-2", """
               _9_8_7_
           10 |       | 6
           11 |   *   | 5   * robot rotated to face the marked triangle corner
           12 |_______| 4
                1 2 3
    """)
async def aligned_on_triangle_deliver(robot: LineFollowerRobot, data: Room3info):

    triangles_aligned = (
        (data.triangle1 in ["0-1", "6-7"] and data.triangle2 in ["0-1", "6-7"])
        or (data.triangle1 in ["3-4", "9-10"] and data.triangle2 in ["3-4", "9-10"])
    )

    await robot.garraDown()
    await robot.move_time(2, -500)
    await robot.garraUp()
    await robot.turn(180)

    await robot.garraDown()

    if triangles_aligned:
        await robot.move_time(3, -500)
        await robot.garraUp()
        await robot.garraDown()
        await robot.straight(2 * data.U)

    await robot.move_time(2)
    await robot.straight(-data.U * 0.3, speed = 600)
    await robot.garraUp()
    await robot.garraDown()
    robot.drive_base.use_gyro(False)
    await robot.move_time(0.6)

    robot.drive_base.use_gyro(True)
    await robot.straight(-data.U * 0.5)

    await robot.straight(-data.U * 0.7)
    await robot.garraUp()


async def room3_exit_maneuver(robot: LineFollowerRobot, ctx: Context, data: Room3info):
    """Handle exit maneuver to find and follow the line out."""

    # Orient based on triangle position
    robot.use_gyro(True)
    if data.triangle1 == "0-1":
        await robot.turn(135)
    elif data.triangle1 == "3-4":
        await robot.turn(-135)
    elif data.triangle1 == "6-7":
        await robot.turn(-45)
    else:  # If none do 9-10
        await robot.turn(45)

    stage_log("EXIT-1", """
               _9_8_7_
           10 |       | 6
           11 |   V   | 5   V robot after delivery sequence, garra up, back at center
           12 |_______| 4
                1 2 3
    """)
    # Move based on output location
    if data.output in [12, 4]:
        await robot.straight(-data.U * 1.3)
    if data.output in [10, 6]:
        await robot.straight(data.U * 1.3)

    stage_log("EXIT-2", """
               _9_8_7_
           10 |   A   | 6   at A if output 10, 6
           11 |   B   | 5   at B if other
           12 |___C___| 4   at C if output 12, 4
                1 2 3
    """)
    # Turn based on output
    if data.output in [4, 5, 6, 3, 7]:
        await robot.turn(90)                 # Facing ---->
    if data.output in [10, 11, 12, 9, 1]:    # Facing <----
        await robot.turn(-90)
    if data.output in [2]:    # Facing V
        await robot.turn(180)

    stage_log("EXIT-3", """
               _9_X_7_
           10X|   -   |X6   
           11X|   -   |X5     
           12X|___-___|X4   
                1 X 3
    """)

    # Final positioning ONLY FOR MISSING OUTPUTS

    if data.output in [1, 3, 7, 9]:
        await robot.straight(data.U * 1.1)
        stage_log("EXIT-3.1", """
               _9_X_7_
           10X|<A   B>|X6  at A if out 9
           11X|       |X5  at B if out 7 
           12X|<D___C>|X4  at C if out 3
                1 X 3      at D if out 1
        """)
        if data.output in [1, 7]:
            await robot.turn(-90)
        if data.output in [3, 9]:
            await robot.turn(90)

    stage_log("EXIT-4", """
               _9_8_7_
           10 |       | 6
           11 |   *   | 5   * robot facing output, ready to search for the line
           12 |_______| 4
                1 2 3
    """)

    # Search for line
    await robot.straight(data.U * 0.7)
    start_angle = ctx.imu.heading()
    while True:
        robot.drive(200, 0)
        if await robot.is_seeing_any_black():
            break

        if abs(start_angle - ctx.imu.heading()) > 7:
            await robot.straight(-80)
            await robot.turn((start_angle - ctx.imu.heading()) * 8)
            await ctx.wait(0.2)
            start_angle = ctx.imu.heading()

    robot.brake()
    robot.use_gyro(True)
    await robot.straight(-40)
    await robot.alignOnLinePerTime(2)
    await robot.straight(-100)
    await robot.turn(90)
    await align_wall_distance_if_reasonable(robot, data.U * 1.55, n = 11) #<---------------------------------------
    await robot.turn(-90)
    await robot.straight(180)

    robot.brake()
    await robot.turn(-20)
    await robot.align_on_line_thingie(robot.R_sensor, robot.L_sensor, 1)


    robot.brake()

    stage_log("EXIT-6", """
               _9_8_7_
           10 |       | 6
           11 |       | 5
           12 |_______| 4
                1 2 3   robot has exited room 3 and re-acquired the line outside
    """)


async def room3(robot: LineFollowerRobot, ctx: Context):

    robot.set_modified_straight_speed(room3_const.straight_speed)
    print("Running room 3!!")
    data = Room3info(robot)
    robot.data = data
    data.markInOutPosDisplay()

    robot.drive_base.use_gyro(True)

    # Execute each sector in sequence
    await room3_entrance_and_low_sector(robot, data)
    await room3_mid_sector(robot, data)
    await room3_high_sector(robot, data)
    await room3_alignment_and_delivery(robot, data)
    await aligned_on_triangle_deliver(robot, data)
    await room3_exit_maneuver(robot, ctx, data)

    robot.clear_modified_straigth_speed()

    
async def align_room_3_entrance(robot: LineFollowerRobot, ctx: Context, data:Room3info):
    """
               _9_8_7_
           10 |       | 6
           11 |       | 5
           12 |___^___| 4   ^ robot lining up on the entrance line before starting room 3
                1 2 3
    """
    await robot.straight(40)
    await robot.alignOnLinePerTime(time=4, black_threshold=90, k=3, speedK = 5, doPreAlign=False)

# async def move_untill_black_or_stopped_moving():
