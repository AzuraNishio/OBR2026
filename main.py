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

ctx: Context
def main():
    global ctx
    ctx = Context()

    robot = LineFollowerRobot( ctx,
        L_sensor_port=  "C",
        R_sensor_port=  "D",
        L_motor_port=   "A",
        R_motor_port=   "B",
        wheel_diameter= 40,
        wheel_distance= 110,
        X_distance_CoR_sensor=60,
        Y_distance_CoR_sensor=15,
        L_motor_forward_direction=Direction.CLOCKWISE,
        R_motor_forward_direction=Direction.COUNTERCLOCKWISE
    )

    model = TreadmilDifferentialDriveRobotModel(ctx, robot)
    follower = FollowerAlgorithm(ctx ,model, robot)
    

    test_suite_mode = False
    if(test_suite_mode):
        test_suite(ctx, robot, model)
        wait(1000000)


    async def follower_routine():
        follower.setup()
        while True:
            await follower.tick()
       

    async def main_task():
        await multitask(follower_routine(), model.routine())

    run_task(main_task())

main()