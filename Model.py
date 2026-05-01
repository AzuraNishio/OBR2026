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
from pybricks.tools import *
from umath import *

class TreadmilDifferentialDriveRobotModel:
    def __init__(self, ctx: Context, robot: LineFollowerRobot):
        self.ctx = ctx
        self.robot = robot
        self.position = vector(0, 0, 0)
        self.last_time = ctx.time()
        self.gravity_vector_world_space = vector(0, 0, 9810) #gravity in mm/s²
        self.last_R_wheel_theta = robot.R_motor.angle()
        self.last_L_wheel_theta = robot.L_motor.angle()
        self.degree_to_wheel_linear_constant = robot.wheel_diameter * pi / 360


    def acceleration_gravity_corrected_world_frame(self):
        acceleration = self.ctx.imu.orientation() * self.ctx.imu.acceleration();
        acceleration -=  self.gravity_vector_world_space
        return acceleration

    def wheel_theta_to_linear(self, theta):
        return self.degree_to_wheel_linear_constant * theta

    async def routine(self):
        while True:
            try:
                self.tick()
                await wait(1)
            except Exception as e:
                print(e)
    
    def pos_2d(self):
        return vector(self.position[0], self.position[1])
    
    def orientation_2d(self):
        orientation: Matrix = self.ctx.imu.orientation()
        v = orientation * vector(1, 0, 0)
        v = vector(v[0], v[1])
        v /= length(v)
        return v

    def tick(self):
        ctx:Context = self.ctx
        dt = max(ctx.time() - self.last_time, 0.001)
        self.last_time = ctx.time()

        self.accel:Matrix = self.acceleration_gravity_corrected_world_frame()

        R_wheel_theta = self.robot.R_motor.angle()
        L_wheel_theta = self.robot.L_motor.angle()

        R_wheel_delta_theta = R_wheel_theta - self.last_R_wheel_theta
        L_wheel_delta_theta = L_wheel_theta - self.last_L_wheel_theta
        
        self.last_R_wheel_theta = R_wheel_theta
        self.last_L_wheel_theta = L_wheel_theta

        self.L_wheel_linear_speed = self.wheel_theta_to_linear(L_wheel_delta_theta) / dt
        self.R_wheel_linear_speed = self.wheel_theta_to_linear(R_wheel_delta_theta) / dt

        self.position += self.ctx.imu.orientation() * vector((self.L_wheel_linear_speed + self.R_wheel_linear_speed ) * 0.5 * dt, 0, 0)


        print(pretty_vec_mm_to_cm_pos(self.position))





