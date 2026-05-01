from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.parameters import *
from Utills import *
from Scaffolding import *


class LineFollowerRobot:
    def __init__(self, ctx, L_sensor_port, R_sensor_port, L_motor_port, R_motor_port, wheel_diameter, wheel_distance, R_motor_forward_direction = Direction.CLOCKWISE, L_motor_forward_direction = Direction.CLOCKWISE):

        self.R_motor:Motor = ctx.get_port(R_motor_port)
        self.L_motor:Motor = ctx.get_port(L_motor_port)

        self.R_motor.close()
        self.R_motor = Motor(get_port(R_motor_port), R_motor_forward_direction)

        self.L_motor.close()
        self.L_motor = Motor(get_port(L_motor_port), L_motor_forward_direction)

        self.R_sensor:ColorSensor = ctx.get_port(R_sensor_port)
        self.L_sensor:ColorSensor = ctx.get_port(L_sensor_port)

        self.drive_base = DriveBase(self.L_motor, self.R_motor, wheel_diameter, wheel_distance)
        self.wheel_diameter = wheel_diameter
        self.wheel_distance = wheel_distance


