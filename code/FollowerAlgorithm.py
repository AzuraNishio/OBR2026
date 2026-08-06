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


class FollowerAlgorithm:
    def __init__(self, ctx: Context, model: TreadmilDifferentialDriveRobotModel, robot: LineFollowerRobot):
        self.ctx = ctx
        self.robot = robot
        self.model = model
    


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
        self.gap_timer = ctx.time()
        self.gap_mode = False
        self.max_sensor_linear_speed = 160
        self.sensor_linear_speed = 160
        self.sensor_reduced_linear_speed = 80
        self.sensor_linear_speed_ramp_duration = 0.5
        self.last_time = self.ctx.time()
        self.bias = -4
        self.obstacleTriggerDistance = 80
        self.lastCheck = 0
        self.gapCheckVariable = 180
        

        #robot.drive_base.use_gyro(True)
    
    async def tick(self):
        self.tps_counter += 1
        dT = self.ctx.time() - self.last_time
        self.last_time = self.ctx.time()
        if (0.5 < self.ctx.time() - self.last_tps_check):
            self.tps = ((self.tps_counter * 2) + (self.tps * 4)) * 0.2
            self.tps_counter = 0
            self.last_tps_check = self.ctx.time()
        
        await self.fuzzy_loop(dT) 


    
    def freeze_tps(self):
        self.interval = self.ctx.time() - self.last_tps_check
    
    def unfreeze_tps(self):
        self.last_tps_check = self.ctx.time() - self.interval

    async def fuzzy_loop(self, dT):
        ctx = self.ctx
        robot = self.robot
        model = self.model

        await wait(1)
        
        R_reflection = await robot.R_sensor.reflection()
        L_reflection = await robot.L_sensor.reflection()

        darkest = min(R_reflection, L_reflection)
        sum = R_reflection + L_reflection

        R_green = await robot.R_sensor.color() == Color.GREEN
        L_green = await robot.L_sensor.color() == Color.GREEN

        self.R_green_counter = 0 if R_green else min(20, self.R_green_counter + dT)
        self.L_green_counter = 0 if L_green else min(20, self.L_green_counter + dT)

        if abs(ctx.imu.tilt()[0]) > 27:
            robot.garra_motor.run_time(5000, 1000)
            await robot.drive_base.straight(70)
        else:
            robot.garra_motor.run_time(-200, 1000)


        if robot.drive_base.distance() - self.lastCheck > self.gapCheckVariable:
            robot.drive_base.use_gyro(True)
            robot.drive_base.curve(0, 20)
            isLine = False
            while not robot.drive_base.done():
                isLine = isLine or await robot.R_sensor.reflection() < 30 or await robot.L_sensor.reflection() < 30
            robot.drive_base.brake()

            if not isLine:
                await robot.drive_base.curve(40, -10)
                await robot.drive_base.straight(-10 - self.gapCheckVariable)
                await robot.drive_base.curve(40, -60)
                await robot.driveUntillConditions(-70, 0, L_reflection_less_than=40)
            else:
                await robot.drive_base.curve(0, -20)

                
            robot.drive_base.use_gyro(False)


            self.lastCheck = robot.drive_base.distance()







        if R_reflection < 50 or L_reflection < 50:
            self.lastCheck = robot.drive_base.distance()
            



        distance = await robot.distance_sensor.distance()

        if distance < self.obstacleTriggerDistance:
            await robot.drive_base.straight(distance)
    
            robot.drive_base.use_gyro(True)
            await robot.drive_base.straight(-10)

            await robot.drive_base.turn(90)

            await robot.drive_base.straight(-100)
            await robot.alignOnLinePerTime(2)
            await robot.drive_base.straight(150)

            while True:
                await robot.drive_base.turn(-90)
                if await robot.distance_sensor.distance() > 80:
                    await robot.drive_base.turn(90)
                    await robot.drive_base.straight(110)
                    await robot.drive_base.turn(-90)
                    break
                else:
                    await robot.drive_base.turn(90)
                    await robot.drive_base.straight(60)

            await robot.drive_base.straight(300)
            await robot.drive_base.turn(-90)

            while await robot.R_sensor.reflection() > 30:
                robot.drive_base.drive(50, 0)

            await robot.drive_base.straight(70)
            await robot.drive_base.turn(90)

            robot.drive_base.use_gyro(False)
            self.lastCheck = robot.drive_base.distance()




        
        if self.gap_mode:
            # Cruise gap mode (Fast, ready to position in new line beggining)
            if darkest < 20:
                robot.drive_base.brake()

                dir = sign((L_reflection) - (R_reflection))
                await self.gap_returnal_manuever(dir)

                self.gap_timer = ctx.time()
                self.gap_mode = False
                await ctx.speaker.beep(600, 20)
                await ctx.speaker.beep(500, 20)

            else:
                robot.drive_base.drive(200, 0)
            
        else:
            # Check for black
            if darkest < 11:
                self.freeze_tps() # Freeze tick per second counter
                if self.L_green_counter < 0.1 or self.R_green_counter < 0.1: # If either sensor has seen green less than 0.1 seconds ago, them do crossing

                    # Check side; 1 if right, 0 if 180, -1 if left
                    side = (1 if self.R_green_counter < 0.1 else 0) - (1 if self.L_green_counter < 0.1 else 0)
                    if side > 0:
                        pro_line_sensor   = robot.R_sensor
                        retro_line_sensor = robot.L_sensor
                    else:
                        pro_line_sensor   = robot.L_sensor
                        retro_line_sensor = robot.R_sensor



                    print(f"green! {side}")


                    # First of all brake
                    robot.drive_base.brake()

                    robot.drive_base.use_gyro(True)
                    # After braking, move forward 70 mm
                    await robot.drive_base.straight(70)

                    if side == 0: # If 180 them turn 170
                        await robot.drive_base.curve(0, 170)
                        side = 1 # Sets side to 1 so turn till line works                        
                    else:         # Else turn 80 in the direction
                        await robot.drive_base.curve(0, 60 * side)

                    # Afterwards keep turning untill it finds the line
                    
                    skip = False

                    while True and not skip: # Loop until skip
                        robot.drive_base.drive(0, side * 140) # Just turn in the direction of the line
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


                    robot.drive_base.brake()
                

                    robot.drive_base.use_gyro(False)
                     
                    


                else: #If it has not seen gree, them do center aligned manuever
                    await self.center_aligned_manuever(sign((L_reflection) - (R_reflection)))
                
                self.unfreeze_tps()

            # Robot turning speed
            turning_speed = ctx.imu.angular_velocity(vector(0,0,1))

            # Calculate turning rate from error
            error = L_reflection - R_reflection  + self.bias
            turn_rate = error * 4

            if abs(turn_rate) < 10:
                turn_rate = 0


            sensor_linear_speed = self.sensor_linear_speed
            if (self.sensor_linear_speed < self.max_sensor_linear_speed):
                self.sensor_linear_speed += dT * self.max_sensor_linear_speed / self.sensor_linear_speed_ramp_duration
            else:
                self.sensor_linear_speed = self.max_sensor_linear_speed
            
            
            # Calculate maximum turn rate
            max_turn_rate = sensor_linear_speed / model.sensor_turn_rate_to_linear

            # If too fast, turn rate
            turn_rate = clamp(max_turn_rate, -max_turn_rate, turn_rate)
            
            # Calculate sideways sensor speed and planned sideways sensor speed
            sideways_sensor_speed = model.sensor_turn_rate_to_linear * turning_speed
            planed_sideways_sensor_speed = model.sensor_turn_rate_to_linear * turn_rate
            
            # Afterwards calculate linear speed accordingly to sideways planned and real sensor speed to keep sensor under speed limit
            linear = sensor_linear_speed - max(abs(sideways_sensor_speed), abs(planed_sideways_sensor_speed))

            robot.drive_base.drive(linear, turn_rate)

    async def gap_returnal_manuever(self, dir):
        print("Gap returnal manuever")
        robot = self.robot
        if (dir < 0):
            pro_line_sensor = robot.L_sensor
            ret_line_sensor = robot.R_sensor
        else:
            pro_line_sensor = robot.R_sensor
            ret_line_sensor = robot.L_sensor

        await robot.drive_base.straight(30)
        while await ret_line_sensor.reflection() > 8:
            robot.drive_base.drive(0, dir * 80)
        while await ret_line_sensor.reflection() < 14:
            robot.drive_base.drive(0, dir * -80)
        
        robot.drive_base.straight(-10)




    async def center_aligned_manuever(self, dir):
        self.sensor_linear_speed = self.sensor_reduced_linear_speed
        print("Center aligned manuever")
        ctx = self.ctx
        robot = self.robot
        model = self.model

        await robot.drive_base.straight(5)
        if self.robot.R_sensor.color() == Color.BLACK and self.robot.L_sensor.color() == Color.BLACK:
            robot.drive_base.straight(20)
            return 


        if (dir < 0):
            pro_line_sensor = robot.L_sensor
            ret_line_sensor = robot.R_sensor
        else:
            pro_line_sensor = robot.R_sensor
            ret_line_sensor = robot.L_sensor


        #Step to align CoR
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


        print(f"Pro line: {pro_line_width}mm, ret line: {ret_line_width}mm")

        init_angle = robot.drive_base.angle()
        while True and not skip: # Loop until skip
            robot.drive_base.drive(0, dir * 140) # Just turn in the direction of the line
        
            if abs(robot.drive_base.angle() - init_angle) > 160: # If has turned more than 160 degrees
                dir *= -1                              # Them flip direction
                await robot.drive_base.turn(dir * 70)  # Turn untill its kinda of aligned to the line
                await robot.drive_base.straight(-9)    # Move back 9 mm
                while abs(robot.drive_base.angle() - init_angle) > 160: # And rotate untill it has undone 20 degrees of its rotation
                    robot.drive_base.drive(0, dir * 140)

            if await pro_line_sensor.reflection() < 21: # If found line with pro line sensor, them spin untill pro line has passed the line, them set skip to true
                while True:
                    robot.drive_base.drive(0, dir * 70)

                    if await pro_line_sensor.reflection() > 21:
                        skip = True
                        break

            if await ret_line_sensor.reflection() < 21: # If found line with retro line, them spin in the opposite direction untill exited line, them set skip to true
                while True:
                    robot.drive_base.drive(0, dir * -70)

                    if await ret_line_sensor.reflection() > 21:
                        skip = True
                        break