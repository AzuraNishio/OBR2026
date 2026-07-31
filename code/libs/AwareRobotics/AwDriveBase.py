from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.parameters import *
from libs.Utills import *
from Scaffolding import *



class AwDriveBase (DriveBase):
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track, ctx: Context):
        super().__init__(left_motor, right_motor, wheel_diameter, axle_track)
        self.ctx = ctx
        self.drive_target_speed = 0
        self.drive_target_turn_rate = 0
        self.cron = StopWatch()
        self.driving = False
        self.driving_start_ms = 0
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.wheel_diameter = wheel_diameter;
        self.axle_track = axle_track
        self.cm_per_heading_degree = axle_track * pi / 360 / 2
        self.cm_per_rotation = self.wheel_diameter * pi
        self.cm_per_rotation_degree = self.wheel_diameter * pi / 360
        self.stuck = None
        self.stuck_start_time = 0

    def driving_time(self):
        if self.driving:
            return self.cron.time() - self.driving_start_ms
        else:
            return -1

    def drive(self, speed, turn_rate):
        if not self.driving:
            self.driving_start_ms = self.cron.time()
        self.driving = True
        self.drive_target_speed = speed
        self.drive_target_turn_rate = turn_rate

        super().drive(speed, turn_rate)
    
        if self.driving_time() > 200:
            targetSpeed = speed / self.cm_per_rotation_degree
            targetSpeedDiff = turn_rate / self.cm_per_heading_degree
            
            stuck = None
            if abs(self.left_motor.speed(100)) < (targetSpeed + targetSpeedDiff) * 0.05:
                stuck = "left "
            if abs(self.right_motor.speed(100)) < (targetSpeed - targetSpeedDiff) * 0.05:
                if stuck == None:
                    stuck == "right"
                else:
                    stuck = "right left"

            if stuck != None:
                if self.stuck == None:
                    self.stuck_start_time = self.cron.time()

                self.ctx.errorHandler.stuck_while_driving(self, stuck, self.cron.time() - self.stuck_start_time)

            self.stuck = stuck
                
                
            


    def straight(self, distance, then = Stop.HOLD, wait = True):
        self.driving = False
        return super().straight(distance, then, wait)
    
    def curve(self, radius, angle, then = Stop.HOLD, wait = True):
        self.driving = False
        return super().curve(radius, angle, then, wait)
    
    def brake(self):
        self.driving = False
        self.drive_target_speed = 0
        self.drive_target_turn_rate = 0
        return super().brake()
    
    def distance(self):
        try:
            self.last_measured = super().distance()
            return self.last_measured
        except:
            print("Distance read failed! Reusing!")
            return self.last_measured

    
    def angle(self):
        return super().angle()
    

    
