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
#from V3.r3_main import run_room3
from constants import *

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


        robot.r3_mode = True

        await robot.garraUp()
        await robot.straight(200)

        await test2(robot, ctx)
        robot.r3_mode = False






async def test2(robot: LineFollowerRobot, ctx: Context):
    tile_size = 300
    triangle_heading_threshold = 4
    exit_n = 0

    def log(event, value=""):
        print("[room3]", ctx.time(), event, value)

    async def drive_segment(distance):
        nonlocal exit_n
        init = robot.drive_base.distance()
        log("segment_start", distance)
        ctx.imu.reset_heading(0)
        special_surface = None
        direction = 1 if distance >= 0 else -1

        while abs(init - robot.drive_base.distance()) < abs(distance):
            heading = ctx.imu.heading()
            robot.drive(200 * direction, heading * direction * 16)

            if special_surface is None:
                if await robot.is_seeing_any_black():
                    special_surface = "black"
                    exit_n += 1
                    log("surface", special_surface)
                    await robot.garraUp()
                    await robot.straight(init - robot.drive_base.distance())

                    break
                elif await robot.L_sensor.reflection() > line_follower_constants.silver_min_threshold:
                    await robot.straight(4)
                    if await robot.L_sensor.reflection() > line_follower_constants.silver_min_threshold:
                        await robot.straight(4)
                        if await robot.L_sensor.reflection() > line_follower_constants.silver_min_threshold and await robot.R_sensor.reflection() > line_follower_constants.silver_min_threshold:
                            special_surface = "silver"
                            log("surface", special_surface)
                            await robot.garraUp()
                            await robot.straight(init - robot.drive_base.distance())

                            break

            if abs(heading) > triangle_heading_threshold:

                if await robot.esp32_sensor_pack.garra_sensor_pressed():
                    await robot.move_time(1, -1000)
                    return False, special_surface

                log("triangle_detected", heading)
                robot.drive_base.brake()
                await robot.straight(-30 * direction)
                await robot.garraUp()
                await robot.turn(-ctx.imu.heading())
                return True, special_surface

        robot.drive_base.brake()
        log("segment_end", special_surface)
        return False, special_surface

    async def room_return_manuever_from_inout(col):
        log("surface_recovery_start")
        await robot.garraUp()

        await robot.turn(90)
        await robot.straight(50)
        await robot.garraDown()
        await robot.straight(-350)
        log("surface_recovery_end")

    async def align_and_deliver_triangle():
        log("triangle_delivery_start")
        await robot.garraUp()
        can_align = await robot.esp32_sensor_pack.laser_distance_safe() < 100
        await robot.turn(90)

        if can_align:
            await robot.move_time(1, 1000)
        dist = await robot.esp32_sensor_pack.laser_distance_safe()
        if dist < 100: #fake triangle
            await robot.turn(-90)
            await robot.move_time(2, -300)
            await robot.straight(50)
            return


        await robot.garraDown()
        await robot.straight(-350)
        await robot.garraUp()

        await robot.turn(-90)
        await robot.straight(500 - dist)
        await robot.turn(-45)

        await robot.garraDown()
        await robot.move_time(2, -2000)
        await robot.garraUp()
        await robot.turn(170)

        for _ in range(3):
            await robot.garraDown()
            await robot.move_time(2, 2000)
            await robot.straight(-240)
            await robot.garraUp()

        await robot.straight(-150)
        await robot.garraUp()
        await robot.turn(-135)
        await robot.garraDown()
        await robot.straight(-350)
        await robot.garraUp()
        await robot.straight(-100)
        distance = await robot.esp32_sensor_pack.laser_distance_safe()
        await robot.turn(90)
        await robot.straight(distance - 550)
        await robot.garraDown()
        log("triangle_delivery_end")

    await robot.turn(180)
    if await robot.esp32_sensor_pack.laser_distance_safe() > 300:
        await robot.turn(90)
        await robot.garraDown()
        await robot.straight(-300)

    log("room_start")
    await robot.garraDown()
    since_turn = 0

    state = "seeking"
    while state == "seeking":
        distance = await robot.esp32_sensor_pack.laser_distance_safe()
        button_pressed = await robot.esp32_sensor_pack.garra_sensor_pressed()
        log("reading", "distance=" + str(distance) + " button=" + str(button_pressed))

        if button_pressed:
            since_turn = 0
            log("wall_contact")
            state = "wall_contact"
            await robot.garraUp()
            await robot.straight(-100)
            align =  await robot.esp32_sensor_pack.laser_distance_safe() < 100
            await robot.turn(90)
            if align:
                robot.move_time(1,100)
            await robot.garraDown()
            state = "seeking"
            continue

        elif distance > 350:
            since_turn = 0
            log("opening", distance)
            await robot.garraUp()
            await robot.turn(-90)
            await robot.garraDown()
            triangle_detected, in_out_tape = await drive_segment(-tile_size)

        else:
            since_turn += 1
            await robot.garraDown()
            triangle_detected, in_out_tape = await drive_segment(-tile_size)


            if not triangle_detected and not in_out_tape:
                newDist = await robot.esp32_sensor_pack.laser_distance_safe()

                if abs(newDist - distance) < 70 and False:
                    expected_angle = degrees(asin((newDist - distance) / tile_size))
                    print(f"Robot is probably at angle {expected_angle}!")
                    print("prev:", distance, "new:", newDist)
                    if abs(expected_angle) > 1:
                        await robot.turn(-expected_angle)

        if since_turn < 1:
            triangle_detected = False


        if in_out_tape == "black":
            if exit_n > 1:
                await robot.garraUp()
                await robot.turn(180)
                break

        if in_out_tape:
            print("First seen:", in_out_tape)
            state = "surface_recovery"
            await room_return_manuever_from_inout(in_out_tape)
        elif triangle_detected:
            since_turn = 0
        
            state = "triangle_delivery"
            await align_and_deliver_triangle()
   
        state = "seeking"


async def test1(robot, ctx):
    if await robot.esp32_sensor_pack.laser_distance() < 260:
        await robot.turn(90)
        await robot.straight(150)
        await robot.garraDown()

        await robot.straight(-400)
    else:    
        await robot.turn(180)

        await robot.straight(100)
        await robot.garraDown()
        await robot.straight(-400)

    ctx.imu.reset_heading(0)

    n = 0
    
    while True:

        robot.drive(-100, 0)
        dist = await robot.esp32_sensor_pack.laser_distance_safe()
        

        if await robot.is_seeing_any_black():
            if n == 2:
                await robot.garraUp()
                await robot.turn(90)
                await robot.straight(-60)
                await robot.turn(90)
                break
            else:
                n += 1
                await robot.garraUp()
                await robot.straight(300)
                await robot.turn(90)


        if abs(ctx.imu.heading()) > 5:
            await robot.straight(10)
            await robot.drive_base.curve(-270, -90 - ctx.imu.heading())
            await robot.straight(-40)
            await robot.garraUp()
            await robot.turn(90)
            await robot.straight(-250)
            await robot.turn(-45)
            await robot.garraDown()
            await robot.straight(450)
            await robot.garraUp()
            await robot.straight(-100)
            await robot.garraDown()
            await robot.straight(300)
            await robot.straight(-300)
            await robot.garraUp()
            await robot.turn(45)
            await robot.straight(300)
            await robot.turn(-90)
            ctx.imu.reset_heading(0)
            await robot.garraDown()
            continue
        

        if await robot.esp32_sensor_pack.garra_sensor_pressed():
            print("front")
            await robot.straight(20)
            await robot.garraUp()
            await robot.straight(-120)
            await robot.turn(90)
            ctx.imu.reset_heading(0)
            dist = await robot.esp32_sensor_pack.laser_distance()

            await robot.garraDown()
            await robot.straight(-160)

        elif dist > 900:
            if n == 2:
                print("long")
                await robot.garraUp()
                await robot.turn(90)
                ctx.imu.reset_heading(0)
                await robot.straight(-160)
                break
            else:
                n += 1
                await robot.straight(-300)
                
        elif dist > 300:
            print("yelp")

            await robot.straight(-180)
            await robot.garraUp()
            await robot.straight(-120)
            await robot.turn(90)
            ctx.imu.reset_heading(0)
            await robot.garraDown()
            await robot.straight(-160)
        
    robot.r3_mode = False

    #await run_room3(robot, ctx)
              


    