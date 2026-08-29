from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.parameters import *
from libs.Utills import *
from Scaffolding import *
from LineFollowerRobot import *
from Model import *
from umath import *
from libs.AwareRobotics.AwDriveBase import *
from Room3 import room3check
from constants import *

class FollowerAlgorithm:
    def __init__(self, ctx: Context, model: TreadmilDifferentialDriveRobotModel, robot: LineFollowerRobot):
        self.ctx = ctx
        self.robot = robot
        self.model = model

    # region Setup

    def setup(self):
        ctx = self.ctx
        robot = self.robot
        model = self.model

        ctx.imu.reset_heading(0)

        self.R_green_counter = 0
        self.L_green_counter = 0
        self.tps = 500
        self.tps_counter = 0
        self.last_tps_check = ctx.time()
        # initialize sensor linear speed to configured max
        self.sensor_linear_speed = line_follower_constants.max_sensor_linear_speed

        # runtime state
        # `sensor_linear_speed` initialized to configured max

        self.last_time = self.ctx.time()
        self.obstacleTriggerDistance = 80
        self.lastCheck = 0
        self.last_error = 0
        self.last_derivative = 0
        self.garra_was_down = False

        #robot.drive_base.use_gyro(True)

    # endregion

    # region Main Loop

    async def tick(self):
        self.tps_counter += 1
        dT = self.ctx.time() - self.last_time
        self.last_time = self.ctx.time()
        if (0.5 < self.ctx.time() - self.last_tps_check):
            self.tps = ((self.tps_counter * 2) + (self.tps * 4)) * 0.2
            self.tps_counter = 0
            self.last_tps_check = self.ctx.time()

        await self.fuzzy_loop(max(0.1, dT))

    def freeze_tps(self):
        self.interval = self.ctx.time() - self.last_tps_check

    def unfreeze_tps(self):
        self.last_tps_check = self.ctx.time() - self.interval

    async def fuzzy_loop(self, dT):
        robot = self.robot

        await wait(1)

        R_reflection, L_reflection, darkest, R_green, L_green = await self.read_sensors()

        self.update_green_counters(dT, R_green, L_green)

        if line_follower_constants.do_knock_down_save:await self.handle_claw_tilt()

        # Distance-based gap detection (replaces the old cruise-mode gap detection)
        if line_follower_constants.do_gap_save: await self.handle_gap_detection()

        if R_reflection < 50 or L_reflection < 50:
            self.lastCheck = robot.drive_base.distance()

        if line_follower_constants.do_obstacle: self.handle_obstacle_avoidance()

        if darkest > line_follower_constants.silver_min_threshold:
            await room3check(robot, self.ctx)

        if darkest < 11: # Check wether a sensor detected black. If so run green check and crossing check
            self.freeze_tps()  # Freeze tick per second counter

            # If either sensor has seen green less than 0.1 seconds ago, do green
            if self.L_green_counter < 0.1 or self.R_green_counter < 0.1: 
                await self.green()
            else: # Else do center aligned manuever
                await self.center_aligned_manuever(sign(L_reflection - R_reflection))

            self.unfreeze_tps()

        await self.line_follow_control(dT, R_reflection, L_reflection)

    # endregion

    # region Sensor Reading

    async def read_sensors(self):
        robot = self.robot

        R_reflection = await robot.R_sensor.reflection()
        L_reflection = await robot.L_sensor.reflection()

        darkest = min(R_reflection, L_reflection)

        R_green = await robot.R_sensor.color() == Color.GREEN
        L_green = await robot.L_sensor.color() == Color.GREEN

        return R_reflection, L_reflection, darkest, R_green, L_green

    def update_green_counters(self, dT, R_green, L_green):
        self.R_green_counter = 0 if R_green else min(20, self.R_green_counter + dT)
        self.L_green_counter = 0 if L_green else min(20, self.L_green_counter + dT)

    # endregion

    # region Claw / Tilt Handling

    async def handle_claw_tilt(self):
        ctx = self.ctx
        robot = self.robot

        if abs(ctx.imu.tilt()[0]) > 27:
            if not self.garra_was_down:
                self.garra_down_time = ctx.time()
            self.garra_was_down = True

            if not abs(ctx.time() - self.garra_down_time) > 2:
                robot.garra_motor.run_time(5000, 1000)
                await robot.drive_base.straight(70)
            else:
                await robot.garra_motor.run_time(-200, 1000)
                await robot.drive_base.straight(70)
        else:
            self.garra_was_down = False
            robot.garra_motor.run_time(-200, 1000)

    # endregion

    # region Gap Detection (distance based)

    async def handle_gap_detection(self):
        robot = self.robot

        if robot.drive_base.distance() - self.lastCheck <= line_follower_constants.gap_check_distance:
            return

        robot.drive_base.use_gyro(True)
        robot.drive_base.curve(0, 20)
        isLine = False
        while not robot.drive_base.done():
            isLine = isLine or await robot.R_sensor.reflection() < 30 or await robot.L_sensor.reflection() < 30
        robot.drive_base.brake()

        if not isLine:
            await robot.drive_base.curve(40, -10)
            for i in range(4):
                start_dist = robot.drive_base.distance()
                while True:
                    robot.drive(-500, 0)
                    if abs(robot.drive_base.distance() - start_dist) > (10 + line_follower_constants.gap_check_distance) / 4:
                        break
                    if await robot.is_seeing_any_black():
                        break

                await robot.turn(-20)
                bb = await robot.is_seeing_any_black()
                await robot.turn(20)
                if bb: break
            await robot.turn(await robot.black_dir() * 6)
            await robot.straight(-60)

            
        else:
            await robot.drive_base.curve(0, -20)

        robot.drive_base.use_gyro(False)

        self.lastCheck = robot.drive_base.distance()

    # endregion

    # region Obstacle Avoidance

    async def handle_obstacle_avoidance(self):
        robot = self.robot
        l = line_follower_constants.obstacle_side

        distance = await robot.distance_sensor.distance()

        if distance >= self.obstacleTriggerDistance:
            return

        await robot.drive_base.straight(distance - 20)
        await robot.drive_base.straight(-40)

        robot.drive_base.use_gyro(True)
        await robot.drive_base.straight(-10)

        await robot.drive_base.turn(l * 90)

        await robot.drive_base.straight(-100)
        await robot.alignOnLinePerTime(3)
        await robot.drive_base.straight(150)

        while True:
            await robot.drive_base.turn(l * -90)
            if await robot.distance_sensor.distance() > 80:
                await robot.drive_base.turn(l * 90)
                await robot.drive_base.straight(110)
                await robot.drive_base.turn(l * -90)
                break
            else:
                await robot.drive_base.turn(l * 90)
                await robot.drive_base.straight(60)

        await robot.drive_base.straight(400)
        await robot.drive_base.turn(l * -90)

        while await robot.R_sensor.reflection() > 30:
            robot.drive_base.drive(100, 0)

        await robot.drive_base.straight(70)
        await robot.drive_base.turn(l * 90)

        robot.drive_base.use_gyro(False)
        self.lastCheck = robot.drive_base.distance()

    # endregion

    # region Green Crossing Handling

    async def green(self):
        robot = self.robot

        # Check side; 1 if right, 0 if 180, -1 if left
        side = (1 if self.R_green_counter < 0.1 else 0) - (1 if self.L_green_counter < 0.1 else 0)
        if side > 0:
            pro_line_sensor = robot.R_sensor
            retro_line_sensor = robot.L_sensor
        else:
            pro_line_sensor = robot.L_sensor
            retro_line_sensor = robot.R_sensor

        print(f"green! {side}")

        # First of all brake
        robot.drive_base.brake()

        robot.drive_base.use_gyro(True)
        # After braking, move forward 70 mm
        await robot.drive_base.straight(70)

        if side == 0:  # If 180 them turn 170
            await robot.drive_base.curve(0, -90)
            await robot.straight(15)
            await robot.drive_base.curve(0, -80)
            side = -1  # Sets side to 1 so turn till line works
        else:  # Else turn 80 in the direction
            await robot.drive_base.curve(0, 60 * side)

        # Afterwards keep turning untill it finds the line
        await self._green_turn_until_line(pro_line_sensor, retro_line_sensor, side)

        robot.drive_base.brake()

        robot.drive_base.use_gyro(False)

    async def _green_turn_until_line(self, pro_line_sensor, retro_line_sensor, side):
        robot = self.robot

        skip = False
        flip = False
        start_angle = self.ctx.imu.heading()
        while True and not skip:  # Loop until skip
            robot.drive_base.drive(0, side * 140)  # Just turn in the direction of the line
            if await pro_line_sensor.reflection() < 21:
                while True:
                    robot.drive_base.drive(0, side * 40)

                    if await pro_line_sensor.reflection() > 21:
                        if await retro_line_sensor.reflection() > 21:
                            skip = True
                            break
                        else:
                            robot.drive_base.curve(0, side * -4)
            elif await retro_line_sensor.reflection() < 21:
                while True:
                    robot.drive_base.drive(0, side * -40)

                    if await retro_line_sensor.reflection() > 21:
                        if await pro_line_sensor.reflection() > 21:
                            skip = True
                            break
                        else:
                            robot.drive_base.curve(0, side * 4)

            if abs(self.ctx.imu.heading() - start_angle) > 50:
                flip = True
                await robot.drive_base.turn(side * -40)
                break

        if flip:
            await self._green_turn_until_line(retro_line_sensor, pro_line_sensor, -side)

    # endregion

    # region Line Follow Control

    async def line_follow_control(self, dT, R_reflection, L_reflection):
        ctx = self.ctx
        robot = self.robot
        model = self.model

        # Robot turning speed
        turning_speed = ctx.imu.angular_velocity(vector(0, 0, 1))

        # Calculate turning rate from error
        error = L_reflection - R_reflection + line_follower_constants.bias
        derivative = (error - self.last_error) / dT
        second_derivative = (derivative - self.last_derivative) / dT
        self.last_error = error
        self.last_derivative = derivative

        # PD controller using gains from constants
        turn_rate = (error * line_follower_constants.KP) + (derivative * line_follower_constants.KD) + (second_derivative * line_follower_constants.KD2)

        if abs(turn_rate) < 10:
            turn_rate = 0

        sensor_linear_speed = self._ramp_linear_speed(dT)

        # Calculate maximum turn rate
        max_turn_rate = sensor_linear_speed / model.sensor_turn_rate_to_linear

        # If too fast, clamp turn rate
        turn_rate = clamp(max_turn_rate, -max_turn_rate, turn_rate)

        # Calculate sideways sensor speed and planned sideways sensor speed
        sideways_sensor_speed = model.sensor_turn_rate_to_linear * turning_speed
        planed_sideways_sensor_speed = model.sensor_turn_rate_to_linear * turn_rate

        # Afterwards calculate linear speed accordingly to sideways planned and real sensor speed to keep sensor under speed limit
        linear = sensor_linear_speed - max(abs(sideways_sensor_speed), abs(planed_sideways_sensor_speed))

        robot.drive_base.drive(linear, turn_rate)

    def _ramp_linear_speed(self, dT):
        sensor_linear_speed = self.sensor_linear_speed
        if self.sensor_linear_speed < line_follower_constants.max_sensor_linear_speed:
            self.sensor_linear_speed += dT * line_follower_constants.max_sensor_linear_speed / line_follower_constants.sensor_linear_speed_ramp_duration
        else:
            self.sensor_linear_speed = line_follower_constants.max_sensor_linear_speed
        return sensor_linear_speed

    # endregion

    # region Center Aligned Manuever

    async def center_aligned_manuever(self, dir):
        self.sensor_linear_speed = line_follower_constants.sensor_reduced_linear_speed
        print("Center aligned manuever")
        robot = self.robot

        await robot.drive_base.straight(5)
        if self.robot.R_sensor.color() == Color.BLACK and self.robot.L_sensor.color() == Color.BLACK:
            robot.drive_base.straight(20)
            return

        if dir < 0:
            pro_line_sensor = robot.L_sensor
            ret_line_sensor = robot.R_sensor
        else:
            pro_line_sensor = robot.R_sensor
            ret_line_sensor = robot.L_sensor

        skip, pro_line_width, ret_line_width = await self._center_align_rotation(dir, pro_line_sensor, ret_line_sensor)

        print(f"Pro line: {pro_line_width}mm, ret line: {ret_line_width}mm")

        if not skip:
            await self._center_turn_until_line(dir, pro_line_sensor, ret_line_sensor)

    async def _center_align_rotation(self, dir, pro_line_sensor, ret_line_sensor):
        robot = self.robot

        # Step to align CoR
        robot.drive_base.curve(110, -25 * dir, wait=False)

        last_dist = robot.drive_base.distance()

        pro_line_width = 0
        ret_line_width = 0

        post_line = 0

        skip = False

        while not robot.drive_base.done():
            dP = robot.drive_base.distance() - last_dist
            last_dist = robot.drive_base.distance()
            if await pro_line_sensor.reflection() < 8:
                pro_line_width += dP
            if await ret_line_sensor.reflection() < 8:
                ret_line_width += dP

            if pro_line_width > 7 and ret_line_width > 7:
                robot.drive_base.stop()
                skip = True
                break

            if pro_line_width > 7:
                post_line += dP

            if post_line > 20:
                break

        return skip, pro_line_width, ret_line_width

    async def _center_turn_until_line(self, dir, pro_line_sensor, ret_line_sensor):
        robot = self.robot

        init_angle = robot.drive_base.angle()
        skip = False

        while True and not skip:  # Loop until skip
            robot.drive_base.drive(0, dir * 140)  # Just turn in the direction of the line

            if abs(robot.drive_base.angle() - init_angle) > 160:  # If has turned more than 160 degrees
                dir *= -1  # Them flip direction
                await robot.drive_base.turn(dir * 70)  # Turn untill its kinda of aligned to the line
                await robot.drive_base.straight(-9)  # Move back 9 mm
                while abs(robot.drive_base.angle() - init_angle) > 160:  # And rotate untill it has undone 20 degrees of its rotation
                    robot.drive_base.drive(0, dir * 140)

            if await pro_line_sensor.reflection() < 21:  # Found line with pro line sensor: spin untill pro line has passed, then set skip
                while True:
                    robot.drive_base.drive(0, dir * 70)

                    if await pro_line_sensor.reflection() > 21:
                        skip = True
                        break

            if await ret_line_sensor.reflection() < 21:  # Found line with retro line: spin opposite untill exited line, then set skip
                while True:
                    robot.drive_base.drive(0, dir * -70)

                    if await ret_line_sensor.reflection() > 21:
                        skip = True
                        break

    # endregion