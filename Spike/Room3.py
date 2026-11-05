# import inspect

from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.parameters import *
from libs.Utills import *
from libs.AwareRobotics.AwDriveBase import *
from LineFollowerRobot import LineFollowerRobot
from Scaffolding import *
from math import *
from constants import room3_const, surprises
from V3.r3_algorithm import *

async def beep_n(robot: LineFollowerRobot, times):
    robot.ctx.speaker.volume(100)
    for i in range(times):
        await robot.ctx.wait(0.1)
        await robot.ctx.speaker.beep()


async def room3check(robot: LineFollowerRobot, ctx: Context):
    """
               _9_8_7_
           10 |       | 6
           11 |       | 5
           12 |___^___| 4   ^ robot approaching room 3 entrance, about to test for ramp
                1 2 3
    """
    if not abs(ctx.imu.tilt()[0]) > 5: # If it's not tilted, run room3
        print("Checking for room3!!")
        for i in range(6):
            await robot.straight(70 / 6) # Drive straight to check wether its a ramp
            if abs(ctx.imu.tilt()[0]) > 5: # If tilted, move until not tilted OR 30 cm
                for j in range(6):
                    await robot.straight(50) # Drive straight to check wether its not a ramp
                    error = await robot.L_sensor.reflection() - await robot.R_sensor.reflection()
                    if abs(error) > 30:
                        await robot.turn(0.1 * error)
                    if abs(ctx.imu.tilt()[0]) < 6: # If it's not tilted stop moving
                        break
                return
                
        await robot.turn(20)
        if await robot.is_seeing_any_black():
            await robot.turn(-20)
            return
        await robot.turn(-20)
        
        # Surprise challenge (ignore)
        if surprises.signal_on_room3:
            await robot.signal_blink()

        await room3(robot, ctx)
              

async def room3(robot: LineFollowerRobot, ctx: Context):

    robot.set_modified_straight_speed(room3_const.straight_speed)
    await robot.straight(150)
    print("Running room 3!!")

    robot.drive_base.use_gyro(True)
    alg = algorithm(r3_robot_interface(ctx, robot))

    while True:

        await alg.robot.update_async()
        await alg.measure()
        finished = alg.step()
        await alg.robot.update_async()
        if finished: break

    robot.clear_modified_straigth_speed()

    