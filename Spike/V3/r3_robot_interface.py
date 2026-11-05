from V3.obrExperimentalData import *
from V3.NekoMath import *
from LineFollowerRobot import LineFollowerRobot
from Scaffolding import Context
from V3.r3_model_robot import *


class r3_robot_interface:
    def __init__(self, ctx: Context, robot: LineFollowerRobot):
        self.x, self.y, self._dir = 0, 0, 0
        self.exited = None             # None | 'exit' | 'entrance'
        self.start_time = ctx.time()
        self.log = []
        self.noclip = False                 # if True, robot can move through walls/obstacles
        self.pushed_on = 0
        self.ctx = ctx
        self.robot = robot
        self.last_dir = (0, 0)
        self.real_last_measurement = None
        self.move_queue = []
        self.is_on_triangle = False

    def time(self):
        return self.ctx.time()


    async def update_async(self):
        for m in self.move_queue:
            await m

        self.move_queue.clear()

        self.real_last_measurement = await self.robot.esp32_sensor_pack.laser_distance()
        pass

    async def real_turn(self, dir):
        await self.robot.garraUp()
        await self.robot.turn(90 * dir)

    async def real_move(self, dir):      
        if dir == -1:
            await self.robot.garraDown()  
        do_triangle = True
        robot = self.robot

        base = robot.drive_base
        start_dist = base.distance()
        await robot.straight(-40)
        await robot.ctx.wait(0.5)
        start_heading = robot.ctx.imu.heading()
        triangle = False
        # base.use_gyro(False)
        sonic_check_time = robot.ctx.time()
        sonic_dist = await robot.esp32_sensor_pack.laser_distance()
        self.exited = None
        self.pushed_on = 0
        while True:
            base.drive(200 * dir, 0)
 
            if abs(base.distance() - start_dist) > 370:
                break

            if await self.robot.is_seeing_any_black(): self.exited = 'exit'
            # if self.robot.is_seeing_any_silver(): self.exited = 'entrance'

            if await self.robot.esp32_sensor_pack.rear_sensor_pressed():
                self.pushed_on = 1
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
                if triangle_detected:
                    triangle = True 
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
        n.pushed_on       =    self.pushed_on
        n.log             =    self.log
        return n


    # what a real robot can know: pose in the ENTRANCE frame (starts 0,0,up)
    def pose(self):
        return (self.x, self.y, self._dir)

    # used by render()/debugging only
    def _x(self): return self.x + self._room.offset
    def _y(self): return self.y
    def dir(self): return self._dir

    def turn_right(self): self._turn(+1)
    def turn_left(self):  self._turn(-1)
    def _turn(self, s):
        self._dir = (self._dir + s) % 4
        self.move_queue.append(self.real_turn(s))
    

    def straight_data(self, dir):
        """Move one cell. Returns False if blocked (wall/obstacle)."""
        self._pay("move")
        dx, dy = STEP_TUPLE_FROM_DIR[self._dir]
        self.last_dir = (dx, dy)
        nx, ny = self.x + (dx * dir), self.y + (dy * dir)
        self.x, self.y = nx, ny
        
        self.pushed_on = 0

    def forward(self):
        """Move one cell. Returns False if blocked (wall/obstacle)."""
        self.straight_data(1)
        self.move_queue.append(self.real_move(1))
        
        self.pushed_on = 0
        return True

    def backyard(self):
        """Move one cell back. Returns False if blocked (wall/obstacle)."""
        self.straight_data(-1)
        self.move_queue.append(self.real_move(-1))
        
        self.pushed_on = 0
        return True

    def measure(self, room = None):
        dist = self.real_last_measurement
        return min(floor((dist - 80 - 150) / 300), 4)

    def kind(self, room=None):
        if room is None: room = self._room
        return room.kind(self.x + room.offset, self.y)

    def on_triangle(self, room = None):
        return self.is_on_triangle

    def _pay(self, what):
        self.log.append(what)
