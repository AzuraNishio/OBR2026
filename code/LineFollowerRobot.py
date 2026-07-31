from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.parameters import *
from libs.Utills import *
from libs.AwareRobotics.AwDriveBase import *
from Scaffolding import *


class LineFollowerRobot:
    def __init__(self, ctx: Context, L_sensor_port, R_sensor_port, L_motor_port, R_motor_port, wheel_diameter, wheel_distance, X_distance_CoR_sensor, Y_distance_CoR_sensor, R_motor_forward_direction = Direction.CLOCKWISE, L_motor_forward_direction = Direction.CLOCKWISE):
        
        # Keep trying to connect
        self.animTime = ctx.time()

        self.R_motor_port = R_motor_port
        self.L_motor_port = L_motor_port

        self.ctx = ctx

        while True:
            try:
                self.R_motor:Motor = ctx.get_port_of_type(R_motor_port, Motor)
                self.L_motor:Motor = ctx.get_port_of_type(L_motor_port, Motor)
                self.R_sensor:ColorSensor = ctx.get_port_of_type(R_sensor_port, ColorSensor)
                self.L_sensor:ColorSensor = ctx.get_port_of_type(L_sensor_port, ColorSensor)
                break
            except WrongPortError as e:
                ctx.reload_empty_ports()

                print(e)
                
                if ctx.time() - self.animTime < 1:
                    ctx.indicate_port(e.port)
                elif ctx.time() - self.animTime < 2:
                    if e.found == "Nothing / Unknown":
                        ctx.show_string_encoded_img(
                            """ 9...9
                                .9.9.
                                ..9..
                                .9.9.
                                9...9"""
                        )
                    else:
                        ctx.show_string_encoded_img(
                            """ .99..
                                9..9.
                                ..9..
                                .....
                                ..9.."""
                        )
                else:
                    self.animTime = ctx.time()

            



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

        self.drive_base = AwDriveBase(self.L_motor, self.R_motor, wheel_diameter, wheel_distance, ctx)
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

        self.drivingFor = 0;
        self.shouldClear = False;


    def integrityCheckTick(self):
        self.drive_base()
    

