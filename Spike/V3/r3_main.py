from V3.r3_algorithm import algorithm
from constants import room3_const
from V3.r3_real_robot import r3_real_robot

async def run_room3(robot, ctx):

    robot.set_modified_straight_speed(room3_const.straight_speed)
    print("Running room 3!!")

    robot.drive_base.use_gyro(True)

    interface = r3_real_robot(ctx, robot)

    alg = algorithm(interface)

    while True:
        await interface.update_async()
        await alg.measure_real()
        finished = alg.step()
        await interface.update_async()
        if finished: break

    robot.clear_modified_straigth_speed()
