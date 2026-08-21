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
from FailHandler import *

ctx: Context
fail_handler:FailHandler = FailHandler()
ctx:Context = Context(fail_handler)


def main():
    global ctx
    ctx.state = "Initializing robot"
    robot = LineFollowerRobot( ctx,
        L_sensor_port=      "C",
        R_sensor_port=      "D",
        L_motor_port=       "A",
        R_motor_port=       "B",
        ultrasonic_sensor = "F",
        garra_motor       = "E",
        wheel_diameter= 40,
        wheel_distance= 110,
        X_distance_CoR_sensor=60,
        Y_distance_CoR_sensor=15,
        L_motor_forward_direction=Direction.CLOCKWISE,
        R_motor_forward_direction=Direction.COUNTERCLOCKWISE
    )

    ctx.state = "Initializing model"
    model = TreadmilDifferentialDriveRobotModel(ctx, robot)

    ctx.robot = robot

    ctx.state = "Initializing follower algorithm"
    follower = FollowerAlgorithm(ctx ,model, robot)
    

    test_suite_mode = isTest()
    if(test_suite_mode):
        ctx.state = "test suite"
        run_task(test_suite(ctx, robot, model))
        wait(1000000)


    async def follower_routine():
        ctx.state = "stating follower"
        follower.setup()
        while True:
            await follower.tick()
       

    async def main_task():
        await multitask(follower_routine(), ctx.routine())
    
    run_task(main_task())

try:
    main()
except Exception as e:
    ctx.speaker.beep(100, 1500)
    ctx.throw(e)