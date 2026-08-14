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

def isTest():
    return True


async def test_suite(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    #await robot.alignOnLinePerTime(3)
    # Import required MicroPython libraries.


    # Register the standard input so we can read keyboard presses.
    keyboard = poll()
    keyboard.register(stdin)

    while True:
        # Check if a key has been pressed.
        if keyboard.poll(0):

            # Read the key and print it.
            key = stdin.read(1)
            print("You pressed:", key)


 
    



def reaction_testing(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    while True:
        print(f"Left: {robot.L_sensor.reflection():<4}, Right: {robot.R_sensor.reflection():<4}, Sum: {(robot.R_sensor.reflection() + robot.L_sensor.reflection()):<4}")


    
    



