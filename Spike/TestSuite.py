from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.parameters import *
from libs.Utills import *
from Scaffolding import *
from LineFollowerRobot import *
from Model import *
from umath import *
from FollowerAlgorithm import *
from TestSuite import *
from Scaffolding import Context
from usys import stdin
from uselect import poll
import Room3

def isTest():
    return True

def room3_in_out_img(IN, out):
        s = """
        9IHG9
        J...F
        K...E            
        L...D
        9ABC9
        """.replace("-ABCDEFGHIJKL"[int(IN)], "6").replace("-ABCDEFGHIJKL"[int(out)], "0")

        for i in range(12):
             s = s.replace("ABCDEFGHIJKL"[i], "9")

        return s


async def test_suite(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    #!/usr/bin/env pybricks-micropython
    from pybricks.hubs import EV3Brick
    from pybricks.iodevices import UARTDevice
    from pybricks.parameters import Port
    from pybricks.media.ev3dev import SoundFile

    # Initialize the EV3
    hub = ctx.hub

    # Initialize sensor port 2 as a uart device
    ser = UARTDevice(Port.B, baudrate=115200)

    # Write some data
    ser.write(b"\r\nHello, world!\r\n")

    # Play a sound while we wait for some data
    for i in range(3):
        print("Bytes waiting to be read:", ser.waiting())

    # Read all data received while the sound was playing
    data = ser.read_all()
    print(data)
    while True:
         pass


        

        





 
    



def reaction_testing(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    while True:
        print(f"Left: {robot.L_sensor.reflection():<4}, Right: {robot.R_sensor.reflection():<4}, Sum: {(robot.R_sensor.reflection() + robot.L_sensor.reflection()):<4}")


    
    



