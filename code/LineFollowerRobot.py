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
from libs.Exceptions import *


class LineFollowerRobot:
    def __getattr__(self, name):
        """Forward attributes not defined here to the drive base."""
        return getattr(self.drive_base, name)

    async def straight(self, distance, speed = None):
        if not speed is None:
            self.drive_base.settings(straight_speed = speed)
        else:
            self.drive_base.settings(straight_speed = self.modified_straight_speed or self.default_straight_speed, straight_acceleration= 700)

        return await self.drive_base.straight(distance)

    async def turn(self, angle):
        return await self.drive_base.turn(angle + copysign(1, angle))

    def __init__(self, ctx: Context, L_sensor_port, R_sensor_port, L_motor_port, R_motor_port, wheel_diameter, wheel_distance, X_distance_CoR_sensor, Y_distance_CoR_sensor, R_motor_forward_direction = Direction.CLOCKWISE, L_motor_forward_direction = Direction.CLOCKWISE, ultrasonic_sensor = "E", garra_motor = "F"):
        self.garra_up = True
        # Keep trying to connect
        self.animTime = ctx.time()

        self.R_motor_port = R_motor_port
        self.L_motor_port = L_motor_port

        self.ctx = ctx

        while True:
            try:
                self.R_motor:Motor = ctx.get_port_of_type(R_motor_port, Motor)
                self.L_motor:Motor = ctx.get_port_of_type(L_motor_port, Motor)
                self.garra_motor:Motor = ctx.get_port_of_type(garra_motor, Motor)
                self.R_sensor:ColorSensor = ctx.get_port_of_type(R_sensor_port, ColorSensor)
                self.L_sensor:ColorSensor = ctx.get_port_of_type(L_sensor_port, ColorSensor)
                self.distance_sensor:UltrasonicSensor = ctx.get_port_of_type(ultrasonic_sensor, UltrasonicSensor)
                self.distance_sensor.lights.off()
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
        self.default_straight_speed = self.drive_base.settings()[0]
        self.modified_straight_speed = None
    
    def set_modified_straight_speed(self, speed):
        self.modified_straight_speed = speed

    async def is_seeing_any_black(self):
        return await self.R_sensor.reflection() < 30 or await self.R_sensor.reflection() < 30

    def clear_modified_straigth_speed(self):
        self.modified_straight_speed = None

    def integrityCheckTick(self):
        self.drive_base()

    async def _garraDownRoutine(self):
        await self.garra_motor.run_time(1000, 1000)

    async def garraDown(self):
        if not self.garra_up: return
        await self.drive_base.straight(10)
        await self.garra_motor.run_time(1000, 1000)
        self.garra_up = False


        #multitask(self._garraDownRoutine())

    async def _garraUpRoutine(self):
        await self.garra_motor.run_time(-1000, 1000)


    async def garraUp(self):
        if self.garra_up: return
        await self.drive_base.straight(10)
        await self.garra_motor.run_time(-1000, 1300)
        self.garra_up = True

        #multitask(self._garraUpRoutine())

    
    async def driveUntillConditions(self, speed, turn, R_color=None, L_color=None,
                                    L_reflection_less_than=0, R_reflection_less_than=0,
                                    driven_distance=9999999, ultrasonic_less_than=0,
                                    ultrasonic_more_than=9999999):
        start_pos = self.drive_base.distance()

        while True:
            self.drive_base.drive(speed, turn)

            if L_color is not None and await self.L_sensor.color() == L_color:
                break

            if R_color is not None and await self.R_sensor.color() == R_color:
                break

            if abs(start_pos - self.drive_base.distance()) >= driven_distance:
                break

            if await self.R_sensor.reflection() <= R_reflection_less_than:
                break

            if await self.L_sensor.reflection() <= L_reflection_less_than:
                break

            distance = await self.distance_sensor.distance()
            if distance <= ultrasonic_less_than:
                break

            if distance >= ultrasonic_more_than:
                break
        self.drive_base.brake()
        

    async def alignOnLinePerTime(self, time = 3, black_threshold = 30, k = 6, speedK = 6, doPreAlign = True):
        ctx = self.ctx
        self.drive_base.use_gyro(False)


        start_align = ctx.time()
        last_error = 0
        last_time = ctx.time()
        while ctx.time() - start_align < time:
            p = min(1, 1.5 - ((ctx.time() - start_align) / time))
            dt = max(0.02, ctx.time() - last_time)
            last_time = ctx.time()

            R = await self.R_sensor.reflection()
            L = await self.L_sensor.reflection()
            error = (R - L)
            derivative = (error - last_error)  / dt
            last_error = error
            b = 1 if abs(error) > 6 else 0
            self.drive_base.drive((R + L - (black_threshold * 2)) * speedK / (abs(error) + 1) * p, -k * error - (derivative) * p * b) 
        self.drive_base.hold()
        self.drive_base.use_gyro(True)
