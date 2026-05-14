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


def test_suite(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    
    print("Meoww! Testing time!")
  #  reaction_testing(ctx, robot, model)
    ctx.speaker.beep()
    # robot.R_motor.run_time(20, 10000)
    
    while True:
        print(f"Direita: {robot.R_motor.angle()}, esquerda: {robot.L_motor.angle()}")
        robot.drive_base.drive(2000, 0)
    
    #robot.drive_base.straight(100)
    ctx.speaker.beep()
    



def reaction_testing(ctx: Context, robot: LineFollowerRobot, model: TreadmilDifferentialDriveRobotModel):
    while True:
        print(f"Left: {robot.L_sensor.reflection():<4}, Right: {robot.R_sensor.reflection():<4}, Sum: {(robot.R_sensor.reflection() + robot.L_sensor.reflection()):<4}")


    
    



