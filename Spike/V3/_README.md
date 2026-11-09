# So..... What exaclty is happening here?
Firstly, instead of saying "rescue room" we say "room 3". Why? Because once upon a time that was what it was called. Thus the "r3" prefix stands for "room 3".
 
## So what happens in each file?
Firstly 
the function "run_room3()" from *r3_main.py* gets called

``` python
async def run_room3(robot, ctx):

    robot.set_modified_straight_speed(room3_const.straight_speed)
    await robot.straight(200)
    print("Running room 3!!")

    robot.drive_base.use_gyro(True)

    interface = r3_robot_interface(ctx, robot)

    alg = algorithm(interface)

    while True:
        await interface.update_async()
        await alg.measure()
        finished = alg.step()
        await interface.update_async()
        if finished: break

    robot.clear_modified_straigth_speed()
```
At this point, it will loop untill the algorithm returns that it finished, repeating the loop of

### 1- measure and 2- update hypothesis
The real robot runs measuring async, while the algoritm runs fully sincronized. For that reason we first run
``` python 
await interface.update_async()
```
in order to update everthing that need's to be run async, including measuring, to them after that run
``` python 
await alg.measure()
```
that actually collects the measurement from the interface and uses it to update it's hypothesis.

### 3- planning the next step
After steps 1 and 2, the robot runs a beam search algorithm to determine the next step to take, by running the call
``` python 
finished = alg.step()
```
this will queue the best chosen movement onto the interface robot.

### 4- running the next step
Lastly, we run the queued move, by calling again 
``` python 
await interface.update_async()
```

## During the execution of r3_main.py we already used r3_robot_interface.py and r3_algorithm. Those you'll have to figure out by yourself! Good luck code diving!