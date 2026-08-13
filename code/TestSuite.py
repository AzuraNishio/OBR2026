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

def isTest():
    return False


async def test_suite(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    
    print("Meoww! Testing time!")
    while True:
  
        re = await robot.R_sensor.reflection()
        print(f"{re}")
 
    



def reaction_testing(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    while True:
        print(f"Left: {robot.L_sensor.reflection():<4}, Right: {robot.R_sensor.reflection():<4}, Sum: {(robot.R_sensor.reflection() + robot.L_sensor.reflection()):<4}")


    
    



