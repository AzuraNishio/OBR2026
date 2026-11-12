from V3.obrExperimentalData import *
from V3.NekoMath import *
from LineFollowerRobot import LineFollowerRobot
from Scaffolding import Context
from V3.r3_model_robot import r3_model_robot
from V3.r3_robot_interface import r3_robot_interface


class r3_real_robot(r3_robot_interface):
    def __init__(self, ctx: Context, robot: LineFollowerRobot):
        super().__init__()
        self.start_time = ctx.time()
        self.ctx = ctx
        self.robot = robot
        self.real_last_measurement = None
        self.move_queue = []
        self.is_on_triangle = False
        self.moves_since_align = 100

    def time(self):
        return self.ctx.time()


    async def update_async(self):
        self.moves_since_align += 1
        for m in self.move_queue:
            await m

        self.move_queue.clear()

        self.real_last_measurement = await self.robot.esp32_sensor_pack.laser_distance()
        self.robot.ctx.display.number(self.real_last_measurement)

        pass

    async def real_turn(self, dir):
        self.robot.drive_base.use_gyro(True)
        await self.robot.garraUp()
        await self.robot.turn(90 * dir)
        await self.robot.straight(10)

        if (await self.robot.esp32_sensor_pack.laser_distance()) < 120 and self.moves_since_align > 4:
            self.moves_since_align = 0
            await self.robot.straight(140)
            await self.robot.straight(-50)

    async def real_move(self, dir):      
        if dir == -1:
            await self.robot.garraDown()  
        do_triangle = True
        robot = self.robot

        base = robot.drive_base
        start_dist = base.distance()
        await robot.straight(-80)
        await robot.ctx.wait(0.5)
        start_heading = robot.ctx.imu.heading()
        triangle = False
        # base.use_gyro(False)
        sonic_check_time = robot.ctx.time()
        sonic_dist = await robot.esp32_sensor_pack.laser_distance()
        self.exited = None
        self.garra_touched_wall = False
        self.touched_triangle = False
        while True:
            base.drive(200 * dir, 0)
 
            if abs(base.distance() - start_dist) > 370:
                break

            if await self.robot.is_seeing_any_black(): self.exited = 'exit'
            # if self.robot.is_seeing_any_silver(): self.exited = 'entrance'

            if await self.robot.esp32_sensor_pack.rear_sensor_pressed():
                self.garra_touched_wall = True
                self.robot.drive_base.brake()
                await self.robot.garraUp()


            left_motor_load = abs(robot.L_motor.load())
            right_motor_load = abs(robot.R_motor.load())
            heading_error = abs(robot.ctx.imu.heading() - start_heading)
            wall_detected = left_motor_load > 160 or right_motor_load > 160
            triangle_detected = (
                do_triangle and
                heading_error >6
                and (
                    left_motor_load > 100
                    or right_motor_load > 100
                )
            )

            if wall_detected or triangle_detected:
                if left_motor_load > 160:
                    w = robot.ctx.get_waiter(0.5)
                    cancel = False
                    while True:
                        if w:
                            break
                        robot.drive(-60 * dir, 0)
                        if abs(robot.L_motor.load()) < 160 * 0.9:
                            cancel = True
                            break
                    if cancel:
                        continue
                    await robot.ctx.speaker.beep()
                    self.straight_data(-dir)
                    while True:
                        base.drive(-200 * dir, 0)

                        if abs(base.distance() - start_dist) < 10:
                            break

                print("detected load!")
                if triangle_detected and abs(base.distance() - start_dist) < 300:
                    triangle = True
                    self.straight_data(-dir)
                    self.touched_triangle = True
                    print("Triangle!")
                    await robot.straight(50)
                    await robot.turn(start_heading - robot.ctx.imu.heading())
                    await robot.ctx.speaker.beep()
                    await robot.ctx.wait(0.2)
                    await robot.ctx.speaker.beep()

                break

        base.brake()
        await base.straight(-dir * 30)
        base.use_gyro(True)
        self.is_on_triangle = triangle


    def copy(self):
        n = r3_model_robot(None)
        n.x, n.y, n._dir =    self.x, self.y, self._dir
        n.exited          =    self.exited
        n.last_dir        =    self.last_dir
        n.noclip          =    self.noclip
        n.time            =    self.time()
        n.garra_touched_wall = self.garra_touched_wall
        n.touched_triangle = self.touched_triangle
        n.log             =    self.log
        return n


    def _turn(self, s):
        self._dir = (self._dir + s) % 4
        self.garra_touched_wall = False
        self.touched_triangle = False
        self.move_queue.append(self.real_turn(s))
    

    def straight_data(self, dir):
        """Move one cell. Returns False if blocked (wall/obstacle)."""
        self._pay("move")
        dx, dy = STEP_TUPLE_FROM_DIR[self._dir]
        self.last_dir = (dx, dy)
        nx, ny = self.x + (dx * dir), self.y + (dy * dir)
        self.x, self.y = nx, ny
        

    def forward(self):
        """Move one cell. Returns False if blocked (wall/obstacle)."""
        self.straight_data(1)
        self.move_queue.append(self.real_move(1))
        
        return True

    def backyard(self):
        """Move one cell back. Returns False if blocked (wall/obstacle)."""
        self.straight_data(-1)
        self.move_queue.append(self.real_move(-1))
        
        return True

    def measure(self, room=None):
        dist = self.real_last_measurement
        return min(floor((dist + 30) / 300), 4)

    def on_triangle(self, room=None, dx=0, dy=0):
        return self.is_on_triangle

    def _pay(self, what):
        self.log.append(what)
