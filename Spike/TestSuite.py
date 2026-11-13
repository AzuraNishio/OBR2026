
import gc
from Scaffolding import Context
from LineFollowerRobot import LineFollowerRobot
from V3.NekoMath import *

def isTest():
    return True  
#
#
#async def hardware_calibration_tests(ctx: Context, robot: LineFollowerRobot):
#    """Hardware calibration for the Room 3 contract.
#
#    Waits for a hub button press before every test so the robot can be
#    repositioned. Paste the full console output back to the agent.
#    For each motion test also note by hand where the robot physically
#    ended (tile center? offset? which side?).
#    """
#
#    async def wait_button():
#        while len(ctx.buttons.pressed()) == 0:
#            await ctx.wait(0.05)
#        while len(ctx.buttons.pressed()) != 0:
#            await ctx.wait(0.05)
#        await ctx.wait(0.3)
#
#    async def laser_batch(n, gap):
#        vals = []
#        for _ in range(n):
#            d = await robot.esp32_sensor_pack.laser_distance()
#            ctx.display.number(d)
#            vals.append(d)
#            await ctx.wait(gap)
#        print("    readings: %s" % vals)
#        print("    min=%d max=%d spread=%d" % (min(vals), max(vals), max(vals) - min(vals)))
#
#    def result(label, odo0, head0, ms):
#        odo1 = robot.drive_base.distance()
#        h1 = ctx.imu.heading()
#        print("  [RESULT] %s" % label)
#        print("    odo %d -> %d (delta %d)" % (odo0, odo1, odo1 - odo0))
#        print("    heading %.1f -> %.1f (err %.1f)" % (head0, h1, h1 - head0))
#        print("    garra_wall=%s triangle=%s exited=%s" % (
#            rr.garra_touched_wall, rr.touched_triangle, rr.exited))
#        print("    laser_raw=%s encoded=%s ms=%d" % (
#            rr.real_last_measurement, rr.measure(), ms))
#
#    async def motion(label, setup, fn):
#        print("[%s] %s" % (label, setup))
#        print("    press button to run")
#        await wait_button()
#        await robot.garraUp()
#        odo0 = robot.drive_base.distance()
#        h0 = ctx.imu.heading()
#        t0 = ctx.time()
#        try:
#            fn()
#            await rr.update_async()
#        except Exception as e:
#            robot.drive_base.brake()
#            print("  [ERROR] %s: %s" % (label, repr(e)))
#            return
#        result(label, odo0, h0, (ctx.time() - t0) * 1000)
#
#    print("=== HARDWARE CALIBRATION TESTS ===")
#    print("WARNING: stay ready to grab the robot; blocked-move bounce loops have no timeout")
#    from V3.r3_real_robot import r3_real_robot
#    rr = r3_real_robot(ctx, robot)
#
#    # ---- T0: sensor identity + laser sanity + rear bumper ----
#    try:
#        sensor = getattr(robot.esp32_sensor_pack, "sensor", None)
#        print("[T0] sensor object: %s" % (
#            type(sensor).__name__ if sensor else "NONE (ColorSensor fallback bug?)"))
#        print("[T0] laser sanity, robot as-is:")
#        await laser_batch(10, 0.25)
#        print("[T0b] rear bumper poll WITHOUT touching (10 reads):")
#        polls = []
#        for _ in range(10):
#            polls.append(await robot.esp32_sensor_pack.rear_sensor_pressed())
#            await ctx.wait(0.2)
#        print("    %s" % polls)
#        print("[T0b] press+hold the garra-tip (rear) buttons, then press hub button")
#        await wait_button()
#        polls = []
#        for _ in range(10):
#            polls.append(await robot.esp32_sensor_pack.rear_sensor_pressed())
#            await ctx.wait(0.2)
#        print("    %s" % polls)
#    except Exception as e:
#        print("[T0 ERROR] %s" % repr(e))
#
#    # ---- T1: laser ladder ----
#    try:
#        print("[T1] laser ladder: 4 poses x 10 readings")
#        for label, hint in (
#            ("wall 1 tile", "front sensor ~300 mm from wall"),
#            ("wall 2 tiles", "front sensor ~600 mm from wall"),
#            ("wall 3 tiles", "front sensor ~900 mm from wall"),
#            ("open/gap", "facing open space or through the exit gap"),
#        ):
#            print("[T1 %s] %s, then press button" % (label, hint))
#            await wait_button()
#            print("  [T1 %s]" % label)
#            await laser_batch(10, 0.25)
#    except Exception as e:
#        print("[T1 ERROR] %s" % repr(e))
#
#    # ---- T2: post-turn staleness ----
#    try:
#        print("[T2] post-turn staleness: turn 90, read immediately vs 1.5 s later")
#        for label, ang in (("left", -90), ("right", 90)):
#            print("[T2 %s turn] any pose; press button" % label)
#            await wait_button()
#            await robot.garraUp()
#            h0 = ctx.imu.heading()
#            await robot.turn(ang)
#            await robot.straight(10)
#            print("  heading %.1f -> %.1f" % (h0, ctx.imu.heading()))
#            print("  immediate reads:")
#            await laser_batch(5, 0.15)
#            await ctx.wait(1.5)
#            print("  settled reads:")
#            await laser_batch(5, 0.15)
#    except Exception as e:
#        print("[T2 ERROR] %s" % repr(e))
#
#    # ---- T3-T8: motion tests through the real contract path ----
#    await motion("T3a free forward x1", "clear path >= 2 tiles ahead", rr.forward)
#    await motion("T3b free backward x1", "clear path >= 2 tiles behind", rr.backyard)
#
#    try:
#        print("[T3c] two free forwards back-to-back (offset accumulation); press button")
#        await wait_button()
#        await robot.garraUp()
#        odo0 = robot.drive_base.distance()
#        t0 = ctx.time()
#        for i in range(2):
#            rr.forward()
#            await rr.update_async()
#            print("    after move %d: odo delta %d laser %s" % (
#                i + 1, robot.drive_base.distance() - odo0, rr.real_last_measurement))
#        print("  [RESULT] T3c total odo delta %d ms %d" % (
#            robot.drive_base.distance() - odo0, (ctx.time() - t0) * 1000))
#    except Exception as e:
#        print("[T3c ERROR] %s" % repr(e))
#
#    await motion("T4 forward into wall", "wall ~1 tile ahead", rr.forward)
#    await motion("T5 backward into wall", "wall ~1 tile behind", rr.backyard)
#    await motion("T6 backward into triangle", "triangle ~1 tile behind", rr.backyard)
#    await motion("T7 forward into triangle", "triangle ~1 tile ahead", rr.forward)
#    await motion("T8 forward into exit gap", "exit gap ~1 tile ahead", rr.forward)
#
#    print("=== DONE - paste everything above back ===")
#
#
async def test_suite(ctx: Context, robot: LineFollowerRobot, model):
    # await robot.straight(270)
    # distance to center - 80mm

    while True:
        robot.drive_base.drive(100, 0)
        if await robot.esp32_sensor_pack.rear_sensor_pressed():
            print(await robot.esp32_sensor_pack.rear_sensor_pressed())
            break

    
    raise 
    # await room3_algorithm_speed_test(max_steps=128, iterations=128)


async def room3_algorithm_speed_test(iterations=1, max_steps=16):
    """Time the shared Room 3 algorithm using simulated robots only."""
    from pybricks.tools import StopWatch
    from V3.r3_algorithm import algorithm, destroy_algorithm
    from V3.r3_model import all_room_descriptors, room_supplier
    from V3.r3_model_robot import r3_model_robot
    from V3.NekoMath import PrettyPrint

    if iterations < 1 or max_steps < 1:
        raise ValueError("iterations and max_steps must be positive")

    descriptors = all_room_descriptors()
    timer = StopWatch()
    total_ms = 0
    total_steps = 0
    finished_runs = 0

    pp = PrettyPrint()

    pp + "Room 3 speed test: simulated robot only; hardware will not move"
    pp.line_break()
    pp + f"Hypotheses per run: {len(descriptors)}" 
    pp + f"Max steps per run: {max_steps}" 
    pp + f"iterations: {iterations}" 
    pp.print()

    for run_index in range(iterations):
        print("Currently running iteration", run_index)
        descriptor_index = (run_index * (len(descriptors) - 1)) // max(iterations - 1, 1)
        simulated_room = next(room_supplier((descriptors[descriptor_index],)))
        simulated_robot = r3_model_robot(simulated_room)

        
        free_memory = getattr(gc, "mem_free", lambda: 0)
        allocated_memory = getattr(gc, "mem_alloc", lambda: 0)
        print("Before alg: Free RAM:", free_memory(), "bytes,", "Used RAM:", allocated_memory(), "bytes")
        alg = algorithm(simulated_robot, verbose=True, render=True)
        print("After alg: Free RAM:", free_memory(), "bytes,", "Used RAM:", allocated_memory(), "bytes")

        start_ms = timer.time()
        steps = 0
        finished = False
        while steps < max_steps and not finished:
            await alg.measure_real()
            finished = alg.step()
            steps += 1

        elapsed_ms = timer.time() - start_ms
        total_ms += elapsed_ms
        total_steps += steps
        if finished:
            finished_runs += 1
        print("Run %d: %d steps, %s, %.1f ms" % (
            run_index + 1,
            steps,
            "finished" if finished else "step limit",
            elapsed_ms,
        ))

        print("Before alg destruction: Free RAM:", free_memory(), "bytes,", "Used RAM:", allocated_memory(), "bytes")
        destroy_algorithm(alg)
        print("After alg destruction: Free RAM:", free_memory(), "bytes,", "Used RAM:", allocated_memory(), "bytes")


    pp = PrettyPrint()
    pp += f"Done running tests {iterations} tests!"
    
    pp.line_break()

    pp + f"Average run time: {(total_ms / iterations):.1f} ms"
    pp + f"Average time per step: {(total_ms / max(total_steps, 1)):.1f} ms"
    pp + f"Runs reaching an exit: {finished_runs} of {iterations}"
    pp.print()




