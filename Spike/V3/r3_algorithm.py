from V3.NekoMath import *
import json
from V3.obrExperimentalData import *
from V3.r3_model import *
from V3.r3_model_robot import *
from V3.r3_robot_interface import *

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
    def __init__(self, room_descriptors = None):
        if room_descriptors is None:
            print(" - [r3 info] Building room descriptors...")
            self.room_descriptors = all_room_descriptors()
            print(" - [r3 info] Done!")
        else:
            self.room_descriptors = list(room_descriptors)
        
        self.update_probability_spread()




    def update_probability_spread(self):
        print(" - [r3 info] Updating probability spread...")
        self.wall_map = [[0 for _ in range(9)] for __ in range(6)]
        self.out_map = [[0 for _ in range(9)] for __ in range(6)]
        self.obstacle_map = [[0 for _ in range(9)] for __ in range(6)]
        self.victim_map = [[0 for _ in range(9)] for __ in range(6)]
        self.triangle_map = [[0 for _ in range(9)] for __ in range(6)]
        self._update_room_probabilities()

        for descriptor in self.room_descriptors:
            room = room_from_descriptor(descriptor)
            increment = self.room_probability(descriptor)
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

        print(" - [r3 info] Done!")

    def _update_room_probabilities(self):
        group_frequency_sums = [0 for _ in range(6)]
        group_counts = [0 for _ in range(6)]
        for descriptor in self.room_descriptors:
            group = ((descriptor & SHAPE_MASK) >> 4) * 2
            if (descriptor & 0b1111) != 0b1111:
                group += 1
            group_frequency_sums[group] += room_frequency_score(descriptor)
            group_counts[group] += 1

        self._frequency_score_sum = sum(group_frequency_sums)
        self._room_probability_by_group = [0 for _ in range(6)]
        for group in range(6):
            if group_counts[group]:
                self._room_probability_by_group[group] = (
                    group_frequency_sums[group]
                    / self._frequency_score_sum
                    / group_counts[group]
                )

    
    def is_guaranteed_condition(self, robot: r3_model_robot, condition):
        for r in self.iter_rooms():
            if not condition(robot, r): return False
        return True

    def is_guaranteed_blocked(self, robot):
        for descriptor in self.room_descriptors:
            kind = room_kind_from_descriptor(
                robot.x + ((descriptor & OFFSET_MASK) >> 6),
                robot.y,
                descriptor,
            )
            if kind not in ("wall", "obstacle"):
                return False
        return True

    def is_guaranteed_wall(self, robot):
        for descriptor in self.room_descriptors:
            kind = room_kind_from_descriptor(
                robot.x + ((descriptor & OFFSET_MASK) >> 6),
                robot.y,
                descriptor,
            )
            if kind != "wall":
                return False
        return True


    def print_information(self, robot: r3_model_robot = None):
        # self.update_probability_spread()

        def draw_walls(wall_prob):
            return grayscale_char(wall_prob * wall_prob) * 2

        # Victim map test
        canvas = for_all_grid(self.victim_map, draw_walls)

        def draw_out(out_prob, canvas):
            if out_prob > 0.25:
                return "◊◊"
            return canvas

        canvas = for_all_grid(self.out_map, draw_out, canvas)

        def draw_triangle(out_prob, canvas):
            if out_prob > 0.6:
                return "TT"
            if out_prob > 0.2:
                return "T" + canvas[0]
            return canvas

        canvas = for_all_grid(self.triangle_map, draw_triangle, canvas)

        def draw_obstacle(obstacle_prob, canvas):
            if obstacle_prob > 0.25:
                return "[]"
            return canvas

        canvas = for_all_grid(self.obstacle_map, draw_obstacle, canvas)

        canvas[0][4] = "^^"

        if not robot is None:
            
            canvas[robot.y + 1][robot.x + 4] = "/>\\<"[robot.dir()] + "\\>/<"[robot.dir()]




        print_grid(canvas)

    def pop_from_measurement(self, result, measurement):
        matching_descriptors = [
            descriptor
            for descriptor in self.room_descriptors
            if measurement(room_from_descriptor(descriptor)) == result
        ]
        if not matching_descriptors:
            print(
                "[r3 info] WARNING: measurement",
                result,
                "matched no rooms out of",
                len(self.room_descriptors),
                "; keeping current hypotheses",
            )
            return False

        self.room_descriptors = matching_descriptors
        self._update_room_probabilities()
        return True

    def measurement_entropy(self, measurement):
        groups = Counter()
        for room in self.iter_rooms():
            groups[measurement(room)] += 1
        return entropy(groups.values())

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

    def room_probability(self, descriptor):
        group = ((descriptor & SHAPE_MASK) >> 4) * 2
        if (descriptor & 0b1111) != 0b1111:
            group += 1
        return self._room_probability_by_group[group]

    def for_all_rooms(self, f):
        return (f(room) for room in self.iter_rooms())

    def iter_rooms(self):
        return room_supplier(self.room_descriptors)

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

        d = straight_move_vec_from_4_state_dir(robot.dir())
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
        rr._room = room
        pp = 0
        d = rr.last_dir
        if not room is None and room.kind(d[0] + rr.x + room.offset, d[1] + rr.y) in ["obstacle", "wall"]:
            pp = 1 if straight_move_vec_from_4_state_dir(rr.dir()) == rr.last_dir else -1
        
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


def insert_beam_candidate(beam, candidate, limit):
    score = candidate[2]
    low = 0
    high = len(beam)
    while low < high:
        middle = (low + high) // 2
        if beam[middle][2] >= score:
            low = middle + 1
        else:
            high = middle

    if low >= limit:
        return
    beam.insert(low, candidate)
    if len(beam) > limit:
        beam.pop()


# Decides to maximize score; what score to use will be hardcoded in a checklist.
# Will beam search
def decision_policy_2(real_robot: r3_model_robot, info: r3_info, persistent_data: dict):
    if not info.room_descriptors:
        print("[r3_algorithm] WARNING: no room hypotheses; turning left")
        return LEFT

    def info_robot_score_function(candidate_data: dict, info: r3_info, robot: r3_model_robot):
        score = 0

        return score

    def room_robot_score_functions(robot: r3_model_robot, candidate_data: dict):
        delivered_victims = candidate_data["delivered_victims"]
        onboard_victims = candidate_data["onboard_victims"]

        time_from_round_beggining = avg_time_before_r3 + robot.time
        time_to_round_end = round_duration - time_from_round_beggining

        potencial_score_after_r3 = avg_score_after_r3 * saturate(time_to_round_end, avg_time_after_r3 * 0.2)
        # potencial_score_after_r3 += 60 * nk.sigmoid(time_to_round_end - avg_time_after_r3, 3)
        expected_score = avg_score_before_r3 + potencial_score_after_r3
        expected_score_multiplier = pow(score_multiplier_per_victim, delivered_victims)

        score_before_onboard = expected_score * expected_score_multiplier
        non_exit_score = score_before_onboard * pow(
            score_multiplier_per_victim - 0.05,
            onboard_victims,
        )
        exit_score = score_before_onboard + 10 * sigmoid(delivered_victims - 3)
        return non_exit_score, exit_score

    # ===================== [Beam search] =====================

    pa = [FORWARD, BACKWARD, LEFT, RIGHT]
    BEAM = [(None, real_robot.copy(), 0, persistent_data.copy(), {})]
    beam_size = 165
    print("[r3_algorithm] Running beam search...")
    for i in range(8):
        print(" - [Beam search] at depth", i, "with", len(BEAM), "candidates")
        beam_size -= 20
        # Run beam search
        NEW_BEAM = []
        for candidate in BEAM:
            for action in pa:
                # Unpack candidate data
                candidate_first_movement, candidate_robot, candidate_score, candidate_data, candidate_exited_scores = candidate
                if candidate_first_movement is None: candidate_first_movement = action

                # Create new moved robot
                new_robot = candidate_robot.copy(); new_robot.noclip = True
                action_from_key(action, new_robot)

                # Disallow movements into a known collider
                if info.is_guaranteed_blocked(new_robot): continue

                candidate_data = candidate_data.copy()
                candidate_exited_scores = candidate_exited_scores.copy()
                update_r3_run_info(new_robot, info, candidate_data, action)

                # Compute weighted sum of score function for all rooms
                move_score = 0
                move_score += info_robot_score_function(candidate_data, info, new_robot)
                non_exit_score, exit_score = room_robot_score_functions(
                    new_robot,
                    candidate_data,
                )
                for descriptor in info.room_descriptors:
                    if descriptor not in candidate_exited_scores:
                        is_exit = room_is_exit_from_descriptor(
                            new_robot.x + ((descriptor & OFFSET_MASK) >> 6),
                            new_robot.y,
                            descriptor,
                        )
                        room_score = exit_score if is_exit else non_exit_score
                        if is_exit:
                            candidate_exited_scores[descriptor] = room_score
                    else:
                        room_score = candidate_exited_scores[descriptor]
                    move_score += room_score * info.room_probability(descriptor)

                
                insert_beam_candidate(
                    NEW_BEAM,
                    (
                        candidate_first_movement,
                        new_robot,
                        candidate_score + move_score,
                        candidate_data,
                        candidate_exited_scores,
                    ),
                    beam_size,
                )


        BEAM = NEW_BEAM
        if not BEAM:
            print("[r3_algorithm] WARNING: no viable beam candidates; turning left")
            return LEFT
    
    winner = BEAM[0]
    return winner[0] # First movement of winner beam
    


class algorithm:
    def __init__(self, robot: r3_robot_interface):
        self.robot = robot
        clear_terminal()
        print("Setupping r3_info...")
        self.info = r3_info()
        self.info.print_information(robot=robot)
        self.persistent_data = {"onboard_victims": 0, "delivered_victims": 0, "visited_positions": []}
        print("Done!")



    async def measure(self):
        print("[r3_algorithm] Measuring...")

        info = self.info
        persistent_data = self.persistent_data
        robot = self.robot
        robot.update_async()
        print("[r3_algorithm] Popping...")
        measurement = measure(robot)(None)
        print("[r3_algorithm] dist:", measurement[0], "triangle:", measurement[1], "exited:", measurement[2], "pushed:", measurement[3])
        info.pop_from_measurement(measurement, measure(robot.copy()))
        info.update_probability_spread()
        print("[r3_algorithm] Done! Remaining rooms:", len(info.room_descriptors))


    def step(self):
        print("[r3_algorithm] Starting step decision policy...")
        info = self.info
        persistent_data = self.persistent_data
        robot = self.robot

        action = decision_policy_2(robot, info, persistent_data)
        print("[r3_algorithm] Decision policy done! Action:", action)

        action_from_key(action, robot)

        print("[r3_algorithm] Updating run info...")
        update_r3_run_info(robot, info, persistent_data, action)
        print("Done!")
        
        print("=" * 20)
        info.print_information(robot)
        
        pp = PrettyPrint(box=True)
        pp + f"Time: {robot.time():.1f}"
        pp.line_break()
        pp + "Ilustrative room:"
        pp + next(info.iter_rooms()).render(robot)
        pp.line_break()
        pp + f"Expected delivered victims: {persistent_data["delivered_victims"]:.1f}; Expected onboard victims: {persistent_data["onboard_victims"]:.1f}"
        pp + f"Possible rooms: {len(info.room_descriptors)}"

        pp.print()

        if robot.exited == "exit":
            print("[r3_algorithm] Exited!")
            print("[r3_algorithm] " + next(info.iter_rooms()).render(robot))
            return True
        return False
