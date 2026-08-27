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
    return False

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

    await robot.curve(-60, -180)

    





 
    



def reaction_testing(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    while True:
        print(f"Left: {robot.L_sensor.reflection():<4}, Right: {robot.R_sensor.reflection():<4}, Sum: {(robot.R_sensor.reflection() + robot.L_sensor.reflection()):<4}")


    
    



