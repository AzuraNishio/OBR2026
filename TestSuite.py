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
from umath import *
from FollowerAlgorithm import *
from TestSuite import *
from AzSensors import *

TestSuite_test = False
TestSuite_test_async = False

def test_suite(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    
    print("Meoww! Testing time!")


async def test_suite_async(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    print("Meoww! Testing time!")

    while True:
        reflection1, reflection2 = await robot.D_sensor.reflection()
        line1, line2 = await robot.D_sensor.line()

        print(f"r: {reflection1:<6}, {reflection2:<6}, line: {line1:<6}, {line2:<6}")

def reaction_testing(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    while True:
        print(f"Left: {robot.L_sensor.reflection():<4}, Right: {robot.R_sensor.reflection():<4}, Sum: {(robot.R_sensor.reflection() + robot.L_sensor.reflection()):<4}")


    
    



