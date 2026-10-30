import V3.NekoMath
import json
from V3.obrExperimentalData import *
from V3.r3_model import *
from V3.r3_model_robot import *
from V3.r3_robot_interface import *
import time

FORWARD = 0
RIGHT = 1
BACKWARD = 2
LEFT = 3


"""
Ideal score function:
(S1 + (E * S2)) * 1.2^D
S1 = expected score before room 3
S2 = expected score after room 3
E = probabilitie of exiting room 3
D = sum of probabilities of delivering first, second and third victim

where

S2 = A * max(0, L - B - T) + (step(L - B, T) * 60)
A = follower points per second
L = room 3 start time
B = follower after room 3 expected time
T = time from start of room 3
60 = score for finishing follower

"""

class r3_info:
    def __init__(self, rooms = None):
        if rooms is None: self.rooms = all_rooms()
        else: self.rooms = rooms
        self.update_probability_spread()
        self._frequency_score_sum = sum(self.for_all_rooms(lambda room: room.frequency_score))


    def update_probability_spread(self):
        self.wall_map = [[0 for _ in range(9)] for __ in range(6)]
        self.out_map = [[0 for _ in range(9)] for __ in range(6)]
        self.obstacle_map = [[0 for _ in range(9)] for __ in range(6)]
        self.victim_map = [[0 for _ in range(9)] for __ in range(6)]
        self.triangle_map = [[0 for _ in range(9)] for __ in range(6)]
        increment_base = 1 / (sum([room.frequency_score for room in self.rooms]) + 0.000000000001)

        for room in self.rooms:
            increment = increment_base * room.frequency_score()
            victim_increment = increment * 3 / room.free_area()
            for xg in range(6):
                for y in range(6):
                    room: r3_model
                    kind = room.kind(xg - 1, y - 1)
                    x = xg - room.offset

                    if kind == "wall":
                        self.wall_map[y][x + 3] += increment
                    elif kind == "exit":
                        self.out_map[y][x + 3] += increment
                    elif kind == "obstacle":
                        self.obstacle_map[y][x + 3] += increment
                    elif kind == "free": # Consider not adding victims on triangle?
                        self.victim_map[y][x + 3] += victim_increment

                    if room.is_triangle(xg - 1, y - 1):
                        self.triangle_map[y][x + 3] += increment

              

    def is_guaranteed_condition(self, robot: r3_model_robot, condition):
        for r in self.rooms:
            if not condition(robot, r): return False
        return True

    def is_guaranteed_blocked(self, robot):
        return self.is_guaranteed_condition(robot, lambda robot, room: robot.kind(room) in ["wall", "obstacle"])

    def is_guaranteed_wall(self, robot):
        return self.is_guaranteed_condition(robot, lambda robot, room: robot.kind(room) == "wall")


    def print_information(self, robot: r3_model_robot = None):
        self.update_probability_spread()

        def draw_walls(wall_prob):
            return V3.NekoMath.grayscale_char(wall_prob * wall_prob) * 2

        # Victim map test
        canvas = V3.NekoMath.for_all_grid(self.victim_map, draw_walls)

        def draw_out(out_prob, canvas):
            if out_prob > 0.25:
                return "◊◊"
            return canvas

        canvas = V3.NekoMath.for_all_grid(self.out_map, draw_out, canvas)

        def draw_triangle(out_prob, canvas):
            if out_prob > 0.6:
                return "TT"
            if out_prob > 0.2:
                return "T" + canvas[0]
            return canvas

        canvas = V3.NekoMath.for_all_grid(self.triangle_map, draw_triangle, canvas)

        def draw_obstacle(obstacle_prob, canvas):
            if obstacle_prob > 0.25:
                return "[]"
            return canvas

        canvas = V3.NekoMath.for_all_grid(self.obstacle_map, draw_obstacle, canvas)

        canvas[0][4] = "^^"

        if not robot is None:
            
            canvas[robot.y + 1][robot.x + 4] = "/>\\<"[robot.dir] + "\\>/<"[robot.dir]




        V3.NekoMath.print_grid(canvas)

    def pop_from_measurement(self, result, measurement):
        self.rooms = [room for room in self.rooms if measurement(room)  == result]
        self._frequency_score_sum = sum(self.for_all_rooms(lambda room: room.frequency_score))

    def measurement_entropy(self, measurement):
        groups = V3.NekoMath.Counter()
        for room in self.rooms:
            groups[measurement(room)] += 1
        return V3.NekoMath.entropy(groups.values())

    def spread_at(self, x, y):
        s = {}
        s["wall"] = self.wall_map[y + 1][x + 4]
        s["obstacle"] = self.obstacle_map[y + 1][x + 4]
        s["exit"] = self.out_map[y + 1][x + 4]
        s["victim"] = self.victim_map[y + 1][x + 4]
        s["triangle"] = self.triangle_map[y + 1][x + 4]
        s["x"] = x
        s["y"] = y
        return s

    def spread(self):
        map = {}
        for x in range(-4, 5):
            for y in range(-1, 5):
                map[(x, y)] = self.spread_at(x, y)
        return map

    def room_probabilitie(self, room):        
        return room.frequency_score / self._frequency_score_sum

    def for_all_rooms(self, f):
        return [f(room) for room in self.rooms]

def action_from_key(key, robot: r3_model_robot, print = False, print_call = print):
    if key == FORWARD:
        robot.forward()
    elif key == BACKWARD:
        robot.backyard()
    elif key == LEFT:
        robot.turn_left()
    elif key == RIGHT:
        robot.turn_right()
    
    elif "+" in key or "-" in key:
        if "+" in key: s = 1
        else: s = -1

        d = V3.NekoMath.straight_move_vec_from_4_state_dir(robot.dir)
        s += d[0]
        s += d[1]

        if s > 0:
            robot.forward()
            if print: print_call("forward")
        else:
            robot.backyard()
            if print: print_call("backward")

    elif "forward" in key:
        robot.forward()
        if print: print_call("forward")
    
    elif "left" in key:
        robot.turn_left()
        if print: print_call("left")
        
    elif "right" in key:
        robot.turn_right()
        if print: print_call("right")
    
    elif "backward" in key:
        robot.backyard()
        if print: print_call("backward")

    pass

def measure(rr):
    def mm(room: r3_model):
        pp = 0
        d = rr.last_dir
        if room.kind(d[0] + rr.x + room.offset, d[1] + rr.y) in ["obstacle", "wall"]:
            pp = 1 if V3.NekoMath.straight_move_vec_from_4_state_dir(rr.dir) == rr.last_dir else -1
            
        return (rr.measure(room=room), rr.on_triangle(room=room), rr.exited, pp)
    return mm

def update_r3_run_info(robot: r3_model_robot, info: r3_info, data: dict, action):
    data["visited_positions"] = list(data["visited_positions"])
    spread = info.spread_at(robot.x, robot.y)

    # Onboard and delivered victim info
    if not (robot.x, robot.y) in data["visited_positions"] and action == FORWARD:
        # Only collect victims if not passed there before
        data["onboard_victims"] += spread["victim"]
    if action == BACKWARD:
        data["delivered_victims"] += data["onboard_victims"] * spread["triangle"]
        data["onboard_victims"] *= 1 - spread["triangle"]

    data["visited_positions"].append((robot.x, robot.y))


# Decides to maximize score; what score to use will be hardcoded in a checklist.
# Will beam search
def decision_policy_2(real_robot: r3_model_robot, info: r3_info, persistent_data: dict):
    def info_robot_score_function(candidate_data: dict, info: r3_info, robot: r3_model_robot):
        score = 0

        return score

    def room_robot_score_function(room: r3_model, robot: r3_model_robot, candidate_data: dict):
        search_score = 0
        
        # True score (Competition based round score)
        delivered_victims = candidate_data["delivered_victims"]
        onboard_victims = candidate_data["onboard_victims"]
        is_exit = robot.kind(room) == "exit"

        time_from_round_beggining = avg_time_before_r3 + robot.time
        time_to_round_end = round_duration - time_from_round_beggining

        potencial_score_after_r3 = avg_score_after_r3 * V3.NekoMath.saturate(time_to_round_end, avg_time_after_r3 * 0.2)
        # potencial_score_after_r3 += 60 * nk.sigmoid(time_to_round_end - avg_time_after_r3, 3)
        expected_score = avg_score_before_r3 + potencial_score_after_r3
        expected_score_multiplier = V3.NekoMath.pow(score_multiplier_per_victim, delivered_victims)

        # Note that the -0.05 is here to incentivize delivery!
        potencial_score_multiplier = V3.NekoMath.pow(score_multiplier_per_victim - 0.05, onboard_victims) if not is_exit else 1

        true_score = expected_score * expected_score_multiplier * potencial_score_multiplier
        search_score += true_score 



        # Abstract score (added to reward certain behaviors)
        # Exit if nothing is left to do
        search_score += (10 * V3.NekoMath.sigmoid(delivered_victims - 3)) if is_exit else 0

        return search_score

    # ===================== [Beam search] =====================

    pa = [FORWARD, BACKWARD, LEFT, RIGHT]
    BEAM = [(None, real_robot.copy(), 0, persistent_data.copy(), [None for x in info.rooms])]
    beam_size = 165

    for i in range(8):
        beam_size -= 20
        # Run beam search
        NEW_BEAM = []
        for candidate in BEAM:
            for action in pa:
                # Unpack candidate data
                candidate_first_movement, candidate_robot, candidate_score, candidate_data, candidate_exited_scores = candidate
                if candidate_first_movement is None: candidate_first_movement = action
                candidate_data = candidate_data.copy()

                # Create new moved robot
                new_robot = candidate_robot.copy(); new_robot.noclip = True
                action_from_key(action, new_robot)

                # Disallow movements into a known collider
                if info.is_guaranteed_blocked(new_robot): continue

                update_r3_run_info(new_robot, info, candidate_data, action)

                # Compute weighted sum of score function for all rooms
                move_score = 0
                move_score += info_robot_score_function(candidate_data, info, new_robot)
                for n, r in enumerate(info.rooms):
                    r: r3_model
                    if candidate_exited_scores[n] is None:
                        room_score = room_robot_score_function(r, new_robot, candidate_data)    
                        if new_robot.kind(r) == "exit":
                            candidate_exited_scores[n] = room_score
                    else:
                        room_score = candidate_exited_scores[n]
                    move_score += room_score * info.room_probabilitie(r)

                
                NEW_BEAM.append((candidate_first_movement, new_robot, candidate_score + move_score, candidate_data, list(candidate_exited_scores)))


        BEAM = NEW_BEAM
        BEAM.sort(key = lambda candidate: candidate[2], reverse=True)
        BEAM = BEAM[0:min(beam_size, len(BEAM))]
    
    winner = BEAM[0]
    return winner[0] # First movement of winner beam
    


class algorithm:
    def __init__(self, robot: r3_robot_interface):
        self.robot = robot
        V3.NekoMath.clear_terminal()

        info = r3_info()
        info.print_information(robot=robot)
        self.persistent_data = {"onboard_victims": 0, "delivered_victims": 0, "visited_positions": []}


    async def measure(self):
        info = self.info
        persistent_data = self.persistent_data
        robot = self.robot
        robot.update_async()
        info.pop_from_measurement(measure(robot)(None), measure(robot))
        info.update_probability_spread()

    def step(self):
        info = self.info
        persistent_data = self.persistent_data
        robot = self.robot

        action = decision_policy_2(robot, info, persistent_data)
        action_from_key(action, robot)

        update_r3_run_info(robot, info, persistent_data, action)
        
        print("=" * 20)
        info.print_information(robot)
        
        pp = V3.NekoMath.PrettyPrint(box=True)
        pp + f"Time: {robot.time:.1f}"
        pp.line_break()
        pp + "Ilustrative room:"
        pp + info.rooms[0].render(robot)
        pp.line_break()
        pp + f"Expected delivered victims: {persistent_data["delivered_victims"]:.1f}; Expected onboard victims: {persistent_data["onboard_victims"]:.1f}"
        pp + f"Possible rooms: {len(info.rooms)}"

        pp.print()

        if robot.exited == "exit":
            print("Exited!")
            print(info.rooms[0].render(robot))
            time.sleep(1)
            return True
        return False

