
import gc


def isTest():
    return True


async def test_suite(ctx, robot, model):
    await room3_algorithm_speed_test(max_steps=128, iterations=128)


async def room3_algorithm_speed_test(iterations=1, max_steps=16):
    """Time the shared Room 3 algorithm using simulated robots only."""
    from pybricks.tools import StopWatch
    from V3.r3_algorithm import algorithm, destroy_algorithm
    from V3.r3_model import all_room_descriptors, room_from_descriptor
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
        simulated_robot = r3_model_robot(room_from_descriptor(descriptors[descriptor_index]))

        
        print("Before alg: Free RAM:", gc.mem_free(), "bytes,", "Used RAM:", gc.mem_alloc(), "bytes")
        alg = algorithm(simulated_robot, verbose=False, render=False)
        print("After alg: Free RAM:", gc.mem_free(), "bytes,", "Used RAM:", gc.mem_alloc(), "bytes")

        start_ms = timer.time()
        steps = 0
        finished = False
        while steps < max_steps and not finished:
            await alg.measure()
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

        print("Before alg destruction: Free RAM:", gc.mem_free(), "bytes,", "Used RAM:", gc.mem_alloc(), "bytes")
        destroy_algorithm(alg)
        print("After alg destruction: Free RAM:", gc.mem_free(), "bytes,", "Used RAM:", gc.mem_alloc(), "bytes")


    pp = PrettyPrint()
    pp += f"Done running tests {iterations} tests!"
    
    pp.line_break()

    pp + f"Average run time: {(total_ms / iterations):.1f} ms"
    pp + f"Average time per step: {(total_ms / max(total_steps, 1)):.1f} ms"
    pp + f"Runs reaching an exit: {finished_runs} of {iterations}"
    pp.print()

    raise RuntimeError()



