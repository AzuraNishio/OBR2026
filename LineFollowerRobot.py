from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.parameters import *
from Utills import *
from Scaffolding import *
from AzSensors import *



class LineFollowerRobot:
    def __init__(self, ctx, L_sensor_port, R_sensor_port, L_motor_port, R_motor_port, wheel_diameter, wheel_distance, X_distance_CoR_sensor, Y_distance_CoR_sensor, R_motor_forward_direction = Direction.CLOCKWISE, L_motor_forward_direction = Direction.CLOCKWISE):

        self.R_motor:Motor = ctx.get_port(R_motor_port)
        self.L_motor:Motor = ctx.get_port(L_motor_port)

        try:
            self.R_motor.close()
            self.R_motor = Motor(get_port(R_motor_port), R_motor_forward_direction)
        except Exception as e:
            raise PortError(R_motor_port, e)

        
        try:
            self.L_motor.close()
            self.L_motor = Motor(get_port(L_motor_port), L_motor_forward_direction)
        except Exception as e:
            raise PortError(L_motor_port, e)

            
        
        self.R_sensor:AzColorSensor = ctx.get_port(R_sensor_port)
        self.L_sensor:AzColorSensor = ctx.get_port(L_sensor_port)
        self.D_sensor:AzDualColorSensor = AzDualColorSensor(self.L_sensor, self.R_sensor, 15)

        self.drive_base = DriveBase(self.L_motor, self.R_motor, wheel_diameter, wheel_distance)
        self.wheel_diameter = wheel_diameter
        self.wheel_distance = wheel_distance
        self.X_distance_CoR_sensor = X_distance_CoR_sensor
        self.Y_distance_CoR_sensor = Y_distance_CoR_sensor

        self.drive_base.reset(0, 0)
    
        self.drive_base.distance_control.limits(acceleration= (1000, 3000))
        self.drive_base.heading_control.limits(acceleration=1000)

        linear_limits = self.drive_base.distance_control.limits()
        heading_limits = self.drive_base.heading_control.limits()

        print("Linear limits")
        print(f"Maximum speed: {linear_limits[0]} mm/s, maximum acceleration: {linear_limits[1]} mm/s², maximum torque: {linear_limits[2]} N*mm")
        print("Turning limits")
        print(f"Maximum speed: {heading_limits[0]} degrees/s, maximum acceleration: {heading_limits[1]} degrees/s², maximum torque: {heading_limits[2]} N*mm")


