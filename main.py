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
        L_motor_forward_direction=Direction.CLOCKWISE,
        R_motor_forward_direction=Direction.COUNTERCLOCKWISE
    )

    model = TreadmilDifferentialDriveRobotModel(ctx, robot)
    
    def move_to_tick(target_pos, target_heading):
        forward = model.orientation_2d()
        pos = model.pos_2d()

        target_heading_error = angle_diff_2d(target_heading, forward)
        position_error = target_pos - pos
        #TODO: fix target heading movement prediction
        #position_error += target_heading * dot(target_heading, position_error)
        
        distance = length(position_error)
        approach_dampening =  max(0, ((min(0.2, 0.02 * distance) + (0.8 * min(1.0, 0.025 * distance))) * 1.1) - 0.1)

        theta_correction = 180 * angle_diff_2d(position_error, forward) / pi
        theta_heading_correction = 0.6 * target_heading_error
        
        forward_correction = dot(forward, normalize(position_error))
        forward_correction = sign(forward_correction) * max(abs(forward_correction) - 0.2, 0)
        
        
        
        fK = 200.0
        tK = 30.0
        
        # Apply multipliers
        forward_correction *= fK * approach_dampening
        
        theta_correction *= approach_dampening
        #theta_correction += theta_heading_correction
        theta_correction *= tK


        #For theta limit rotation to 80 degrees/s
        theta_correction = sign(theta_correction) * min(abs(theta_correction), 80)
        robot.drive_base.drive(forward_correction, theta_correction)

    async def test():
        target_pos = vector(200,200)
        robot.drive_base.use_gyro(True)
        
        while True:
            await wait(1)
            move_to_tick(target_pos, vector(1, 0))

            if(Button.BLUETOOTH in ctx.buttons.pressed()):
                await wait(1000)
                target_pos = vector(0, 0)





    async def main_task():
        await multitask(model.routine(), test())

    run_task(main_task())

main()