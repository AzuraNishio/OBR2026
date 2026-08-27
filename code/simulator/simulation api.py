from simulation_runner import *
from pathlib import Path
from sim_bridge import Bridge


test_path = Path(__file__).parent.parent / "Room3.py"


class AwaitableResult:
    def __init__(self, value=None):
        self.value = value

    def __await__(self):
        async def _wait():
            return self.value

        return _wait().__await__()

class ignore_missing_attributes:
    def __getattr__(self, name):
        def dynamic_method(*args, **kwargs):
            print(f"{name}{args}")

            result = None
            return AwaitableResult(result)

        return dynamic_method

class fake_drive_base(ignore_missing_attributes):
    async def straight(self, distance):
        print(f"straight({distance})")
        await bridge.send_command_and_wait("straight", distance)

    async def turn(self, angle):
        print(f"turn({angle})")
        await bridge.send_command_and_wait("turn", angle)

    def distance(self):
        return 100

class fake_distance_sensor(ignore_missing_attributes):
    async def distance(self):
        return float(await bridge.send_command_and_wait("sonic_distance"))

class fake_color_sensor(ignore_missing_attributes):
    def __init__(self):
        self.reflection_state = 0
    async def reflection(self):
        return self.reflection_state

class simulation(ignore_missing_attributes):
    def __init__(self):
        self.robot = self
        self.ctx = self
        self.drive_base = fake_drive_base()
        self.imu = self
        self.distance_sensor = fake_distance_sensor()
        self.L_motor = self
        self.heading_state = 0
        self.motor_load = 0
        self.R_sensor = fake_color_sensor()
        self.L_sensor = fake_color_sensor()

    async def garraUp(self):
        await bridge.send_command_and_wait("garra_up")

    async def garraDown(self):
        await bridge.send_command_and_wait("garra_down")
        

    def load(self):
        return self.motor_load

    def show_string_encoded_img(self, img):
        print(img)

    def heading(self):
        return self.heading_state

bridge = Bridge()
sim = simulation()

runSimulationSync(test_path, entry_func="room3", robot=sim.robot, ctx=sim.ctx)