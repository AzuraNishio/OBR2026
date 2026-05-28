from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.parameters import *
from Utills import *
from Scaffolding import *
from LineFollowerRobot import *
from Model import *
from umath import *
from pybricks.pupdevices import *


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
        

        #robot.drive_base.use_gyro(True)
    
    async def tick(self):
        self.tps_counter += 1
        if (0.5 < self.ctx.time() - self.last_tps_check):
            self.tps = ((self.tps_counter * 2) + (self.tps * 4)) * 0.2
            self.tps_counter = 0
            self.last_tps_check = self.ctx.time()
        
        await self.fuzzy_loop() 
    
    def freeze_tps(self):
        self.interval = self.ctx.time() - self.last_tps_check
    
    def unfreeze_tps(self):
        self.last_tps_check = self.ctx.time() - self.interval

    async def fuzzy_loop(self):
        ctx = self.ctx
        robot = self.robot
        model = self.model

        await wait(1)
        
        await robot.D_sensor.snap()
        L_line, R_line = robot.D_sensor.line_snap()
        line_DIF = robot.D_sensor.line_snap_difference()

        sum = R_line + L_line
        darkest = min(R_line, L_line)

        L_green, R_green = await robot.D_sensor.isColor(Color.GREEN)

        self.R_green_counter = 6 if R_green else max(0, self.R_green_counter - 1)
        self.L_green_counter = 6 if L_green else max(0, self.L_green_counter - 1)
        
        if self.gap_mode:
            # Cruise gap mode (Fast, ready to position in new line beggining)
            if robot.D_sensor.minimum_line_snap() < 20:
                robot.drive_base.brake()

                dir = robot.D_sensor.line_difference_sign()
                await self.gap_returnal_manuever(dir)

                self.gap_timer = ctx.time()
                self.gap_mode = False
                await ctx.speaker.beep(600, 20)
                await ctx.speaker.beep(500, 20)

            else:
                robot.drive_base.drive(200, 0)  
        else:
            # Line mode (predictive PD follower with sensor speed control) 

            #TODO: Gap mode may work with closer sensors, change construction first
            #if sum > 69: # Value that is read in a gap
            #    if ctx.time() - self.gap_timer > 0.6:
            #        self.gap_mode = True
            #        robot.drive_base.brake()
            #        ctx.speaker.volume(100)
            #        await ctx.speaker.beep(600, 20)
            #        print("=======[Gap mode!]=======")
            #    else:
            #        self.gap_timer = ctx.time()
            #        self.gap_mode = False


            # Check for black
            if darkest < 9:
                self.freeze_tps()
                await self.center_aligned_manuever(robot.D_sensor.line_snap_difference_sign())
                self.unfreeze_tps()

            # Robot turning speed
            turning_speed = ctx.imu.angular_velocity(vector(0,0,1))

            # Calculate turning rate from error
            error = line_DIF
            turn_rate = error * 6

            if abs(turn_rate) < 10:
                turn_rate = 0

            max_sensor_linear_speed = 150

            
            # Calculate maximum turn rate
            max_turn_rate = max_sensor_linear_speed / model.sensor_turn_rate_to_linear

            # If too fast, turn rate
            turn_rate = clamp(max_turn_rate, -max_turn_rate, turn_rate)
            
            # Calculate sideways sensor speed and planned sideways sensor speed
            sideways_sensor_speed = model.sensor_turn_rate_to_linear * turning_speed
            planed_sideways_sensor_speed = model.sensor_turn_rate_to_linear * turn_rate

            
            # Afterwards calculate linear speed accordingly to sideways planned and real sensor speed to keep sensor under speed limit
            linear = max_sensor_linear_speed - max(abs(sideways_sensor_speed), abs(planed_sideways_sensor_speed))
            


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
        print("Center aligned manuever")
        ctx = self.ctx
        robot = self.robot
        model = self.model

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
        swapped = 0
        while True and not skip:
            while True and not skip:
                robot.drive_base.drive(0, dir * 120)
                if abs(robot.drive_base.angle() - init_angle) > 160 or (swapped == 2 and abs(robot.drive_base.angle() - init_angle) < 1):
                    dir *= -1
                    if swapped == 2:
                        swapped = 0
                        await robot.drive_base.straight(-60)

                    
                    swapped += 1
                    
                    while abs(robot.drive_base.angle() - init_angle) > 160:
                        robot.drive_base.drive(0, dir * 120)
                    break

                if await pro_line_sensor.line() < 21:
                    while True:
                        robot.drive_base.drive(0, dir * 60)

                        if await pro_line_sensor.line() > 21:
                            skip = True
                            if abs(robot.drive_base.angle() - init_angle) > 60:
                                await robot.drive_base.straight(-20)
                            else:
                                await robot.drive_base.straight(-5)
                            break
                    break