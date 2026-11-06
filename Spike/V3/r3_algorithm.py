from V3.NekoMath import *
from V3.obrExperimentalData import *
from V3.r3_model import *
from V3.r3_model_robot import *
from V3.r3_opening_book import OpeningBook, opening_book_compatible

try:
    from V3.r3_robot_interface import r3_robot_interface
except ImportError:
    r3_robot_interface = None

try:
    from V3.r3_opening_book_data import BOOK_CONFIG, BOOK_DATA
except ImportError:
    BOOK_CONFIG = None
    BOOK_DATA = None

FORWARD = 0
RIGHT = 1
BACKWARD = 2
LEFT = 3

clearance_after_r3 = 2

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
    def __init__(self, room_descriptors = None, verbose=True):
        self.verbose = verbose
        if room_descriptors is None:
            if self.verbose:
                print(" - [r3 info] Building room descriptors...")
            self.room_descriptors = all_room_descriptors()
            if self.verbose:
                print(" - [r3 info] Done!")
        else:
            self.room_descriptors = list(room_descriptors)
        self._exit_probability_map = None
        self._spread_valid = False
        self._update_room_probabilities()




    def update_probability_spread(self):
        if self.verbose:
            print(" - [r3 info] Updating probability spread...")
        self.wall_map = [[0 for _ in range(9)] for __ in range(6)]
        self.out_map = [[0 for _ in range(9)] for __ in range(6)]
        self.obstacle_map = [[0 for _ in range(9)] for __ in range(6)]
        self.victim_map = [[0 for _ in range(9)] for __ in range(6)]
        self.triangle_map = [[0 for _ in range(9)] for __ in range(6)]
        self._update_room_probabilities()

        room = r3_model("3x3", 0, 1)
        for descriptor in self.room_descriptors:
            room_from_descriptor(descriptor, room)
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

        if self.verbose:
            print(" - [r3 info] Done!")
        self._spread_valid = True

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

    def print_information(self, robot: r3_model_robot = None):
        # self.update_probability_spread()
        if not self._spread_valid: return
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
        matching_descriptors = []
        room = r3_model("3x3", 0, 1)
        for descriptor in self.room_descriptors:
            room_from_descriptor(descriptor, room)
            if measurement(room, ref=result) == result:
                matching_descriptors.append(descriptor)
        if not matching_descriptors:
            if self.verbose:
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
        self._spread_valid = False
        self._exit_probability_map = None
        return True

    def exit_probability_at(self, x, y):
        if self._exit_probability_map is None:
            probabilities = [0 for _ in range(54)]
            room = r3_model("3x3", 0, 1)
            for descriptor in self.room_descriptors:
                room_from_descriptor(descriptor, room)
                exit_x, exit_y = room.exit_cell()
                robot_x = exit_x - room.offset
                if -4 <= robot_x <= 4 and -1 <= exit_y <= 4:
                    cell = (exit_y + 1) * 9 + robot_x + 4
                    probabilities[cell] += self.room_probability(descriptor)
            self._exit_probability_map = probabilities

        if x < -4 or x > 4 or y < -1 or y > 4:
            return 0
        return self._exit_probability_map[(y + 1) * 9 + x + 4]

    def spread_at(self, x, y):
        if not self._spread_valid:
            spread = {
                "wall": 0,
                "obstacle": 0,
                "exit": 0,
                "victim": 0,
                "triangle": 0,
                "x": x,
                "y": y,
            }
            room = r3_model("3x3", 0, 1)
            for descriptor in self.room_descriptors:
                room_from_descriptor(descriptor, room)
                if not 0 <= x + room.offset + 1 < 6:
                    continue
                increment = self.room_probability(descriptor)
                kind = room.kind(x + room.offset, y)
                if kind == "wall":
                    spread["wall"] += increment
                elif kind == "obstacle":
                    spread["obstacle"] += increment
                elif kind == "exit":
                    spread["exit"] += increment
                elif kind == "free":
                    spread["victim"] += increment * 3 / room.free_area()
                if room.is_triangle(x + room.offset, y):
                    spread["triangle"] += increment
            return spread

        s = {}
        s["wall"] = self.wall_map[y + 1][x + 4]
        s["obstacle"] = self.obstacle_map[y + 1][x + 4]
        s["exit"] = self.out_map[y + 1][x + 4]
        s["victim"] = self.victim_map[y + 1][x + 4]
        s["triangle"] = self.triangle_map[y + 1][x + 4]
        s["x"] = x
        s["y"] = y
        return s

    def room_probability(self, descriptor):
        group = ((descriptor & SHAPE_MASK) >> 4) * 2
        if (descriptor & 0b1111) != 0b1111:
            group += 1
        return self._room_probability_by_group[group]

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
    def mm(room: r3_model, ref = None):
        rr._room = room
        pp = 0
        d = rr.last_dir
        forward = straight_move_vec_from_4_state_dir(rr.dir()) == rr.last_dir
        if not room is None and (
                (    forward and room.kind(d[0] + rr.x + room.offset, d[1] + rr.y) in ["obstacle", "wall"])
            or  (not forward and room.kind(d[0] + rr.x + rr.x + room.offset, d[1] + rr.y + rr.y) in ["obstacle", "wall"])
            ):
            pp = 1
        
        if ref is None: return (rr.measure(room=room), rr.on_triangle(room=room), rr.exited, pp)
        dist = rr.measure(room=room)
        if dist > clearance_after_r3 and  dist < 4: dist = min(dist, ref[0])
        if forward: return (dist, rr.on_triangle(room=room), rr.exited, pp)
        return (dist, rr.on_triangle(room=room, dx=d[0], dy=d[1]), rr.exited, pp)
    return mm

def update_r3_run_info(robot: r3_model_robot, info: r3_info, data: dict, action):
    data["visited_positions"] = list(data["visited_positions"])
    spread = info.spread_at(robot.x, robot.y)

    # Onboard and delivered victim info
    if not (robot.x, robot.y) in data["visited_positions"] and action == BACKWARD:
        # Only collect victims if not passed there before
        data["onboard_victims"] += spread["victim"]
    if action == FORWARD:
        data["delivered_victims"] += data["onboard_victims"] * spread["triangle"]
        data["onboard_victims"] *= 1 - spread["triangle"]

    data["visited_positions"].append((robot.x, robot.y))


def beam_state_key(candidate):
    robot = candidate[1]
    data = candidate[3]
    # Visit order and repeats do not matter: later logic only asks if a cell was visited.
    visited = tuple(sorted(set(data["visited_positions"])))
    # Exclude accumulated path score and first action; for an identical future state,
    # the beam should retain only the path with the higher score so far.
    return (
        robot.x,
        robot.y,
        robot.dir(),
        robot.time,
        robot.exited,
        data["onboard_victims"],
        data["delivered_victims"],
        visited,
        candidate[4],
        candidate[5],
    )


def insert_beam_candidate(beam, candidate, limit, seen_states):
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
    key = beam_state_key(candidate)
    previous = seen_states.get(key)
    if previous is not None:
        # A lower-scoring path to the same future state cannot become the winner.
        if previous[2] >= score:
            return
        beam.remove(previous)
    beam.insert(low, candidate)
    seen_states[key] = candidate
    if len(beam) > limit:
        removed = beam.pop()
        removed_key = beam_state_key(removed)
        if seen_states.get(removed_key) is removed:
            del seen_states[removed_key]


# Decides to maximize score; what score to use will be hardcoded in a checklist.
# Will beam search
def decision_policy_2(
    real_robot: r3_model_robot,
    info: r3_info,
    persistent_data: dict,
    verbose=True,
    first_action_bonuses=None,
):
    if not info.room_descriptors:
        if verbose:
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
    total_room_probability = sum(
        info.room_probability(descriptor)
        for descriptor in info.room_descriptors
    )
    BEAM = [(None, real_robot.copy(), 0, persistent_data.copy(), 0, total_room_probability)]
    beam_size = 165
    if verbose:
        print("[r3_algorithm] Running beam search...")
    for i in range(7):
        if verbose:
            print(" - [Beam search] at depth", i, "with", len(BEAM), "candidates")
        beam_size -= 20
        # Run beam search
        NEW_BEAM = []
        seen_states = {}
        for candidate in BEAM:
            for action in pa:
                # Unpack candidate data
                (candidate_first_movement, candidate_robot, candidate_score,
                 candidate_data, frozen_exit_score, active_probability) = candidate
                if candidate_first_movement is None: candidate_first_movement = action

                # Create new moved robot
                new_robot = candidate_robot.copy(); new_robot.noclip = True
                action_from_key(action, new_robot)

                # Disallow movements into a known collider
                if info.is_guaranteed_blocked(new_robot): continue

                candidate_data = candidate_data.copy()
                first_visit = (new_robot.x, new_robot.y) not in candidate_data["visited_positions"]
                update_r3_run_info(new_robot, info, candidate_data, action)

                # Compute weighted sum of score function for all rooms
                move_score = 0
                move_score += info_robot_score_function(candidate_data, info, new_robot)
                non_exit_score, exit_score = room_robot_score_functions(
                    new_robot,
                    candidate_data,
                )
                # Rooms sharing an exit cell all freeze their score on its first
                # visit. Track their combined probability instead of one bit per room.
                if first_visit:
                    probability = info.exit_probability_at(new_robot.x, new_robot.y)
                    frozen_exit_score += exit_score * probability
                    active_probability -= probability

                move_score += frozen_exit_score + non_exit_score * active_probability
                if i == 0 and first_action_bonuses is not None:
                    move_score += first_action_bonuses.get(action, 0)

                
                insert_beam_candidate(
                    NEW_BEAM,
                    (
                        candidate_first_movement,
                        new_robot,
                        candidate_score + move_score,
                        candidate_data,
                        frozen_exit_score,
                        active_probability,
                    ),
                    beam_size,
                    seen_states,
                )


        BEAM = NEW_BEAM
        if not BEAM:
            if verbose:
                print("[r3_algorithm] WARNING: no viable beam candidates; turning left")
            return LEFT
    
    winner = BEAM[0]
    return winner[0] # First movement of winner beam
    
def destroy_algorithm(instance):
    instance.cleanup()
    del instance
    import gc
    gc.collect()

class algorithm:
    def __init__(self, robot, verbose=True, render=True):
        self.robot = robot
        self.verbose = verbose
        self.render = render
        if self.verbose:
            clear_terminal()
            print("Setupping r3_info...")
        self.info = r3_info(verbose=self.verbose)
        if self.render:
            self.info.print_information(robot=robot)
        self.persistent_data = {"onboard_victims": 0, "delivered_victims": 0, "visited_positions": []}
        self.opening_book = None
        self.opening_book_node = None
        if BOOK_DATA is not None and opening_book_compatible(BOOK_CONFIG):
            try:
                self.opening_book = OpeningBook(BOOK_DATA)
                self.opening_book_node = 0
                if self.verbose:
                    print("[r3_algorithm] Opening book loaded:", self.opening_book.node_count, "nodes")
            except ValueError as error:
                if self.verbose:
                    print("[r3_algorithm] Opening book ignored:", error)
        elif BOOK_DATA is not None:
            if self.verbose:
                print("[r3_algorithm] Opening book version mismatch; using live search")
        if self.verbose:
            print("Done!")


    def cleanup(self):
        self.info.room_descriptors.clear()

        self.info._exit_probability_map = None

        self.info.wall_map = None
        self.info.out_map = None
        self.info.obstacle_map = None
        self.info.victim_map = None
        self.info.triangle_map = None

        self.info._room_probability_by_group = None
        self.info._frequency_score_sum = None

        self.persistent_data.clear()

        self.opening_book = None
        self.opening_book_node = None

        self.robot = None
        self.info = None

        import gc
        gc.collect()

    async def measure(self):
        if self.verbose:
            print("[r3_algorithm] Measuring...")

        info = self.info
        persistent_data = self.persistent_data
        robot = self.robot
        if isinstance(robot, r3_model_robot):
            measurement = measure(robot)(robot._room)
        else:
            robot.update_async()
            measurement = measure(robot)(None)
        if self.verbose:
            print("[r3_algorithm] Popping...")
            print("[r3_algorithm] dist:", measurement[0], "triangle:", measurement[1], "exited:", measurement[2], "pushed:", measurement[3])
        matched = info.pop_from_measurement(measurement, measure(robot.copy()))
        if self.opening_book is not None:
            if not matched:
                self.opening_book = None
            else:
                next_node = self.opening_book.next_node(self.opening_book_node, measurement)
                if next_node is None:
                    self.opening_book = None
                else:
                    self.opening_book_node = next_node
        if self.opening_book is None:
            info.update_probability_spread()
        if self.verbose:
            print("[r3_algorithm] Done! Remaining rooms:", len(info.room_descriptors))


    def step(self):
        if self.verbose:
            print("[r3_algorithm] Starting step decision policy...")
        info = self.info
        persistent_data = self.persistent_data
        robot = self.robot

        action = None
        if self.opening_book is not None:
            action = self.opening_book.action_for(self.opening_book_node)
            if action is None:
                self.opening_book = None
        if action is None:
            if not info._spread_valid:
                info.update_probability_spread()
            action = decision_policy_2(robot, info, persistent_data, verbose=self.verbose)
        if self.verbose:
            print("[r3_algorithm] Decision policy done! Action:", action)

        action_from_key(action, robot)

        if self.verbose:
            print("[r3_algorithm] Updating run info...")
        update_r3_run_info(robot, info, persistent_data, action)
        
        if self.verbose:
            print("Done!")
        if self.render:
            print("=" * 20)
            if info._spread_valid:
                info.print_information(robot)

            robot_time = robot.time() if callable(robot.time) else robot.time
            pp = PrettyPrint(box=True)
            pp + f"Time: {robot_time:.1f}"
            pp.line_break()
            pp + "Ilustrative room:"
            pp + next(info.iter_rooms()).render(robot)
            pp.line_break()
            pp + f"Expected delivered victims: {persistent_data["delivered_victims"]:.1f}; Expected onboard victims: {persistent_data["onboard_victims"]:.1f}"
            pp + f"Possible rooms: {len(info.room_descriptors)}"

            pp.print()

        if robot.exited == "exit":
            if self.verbose:
                print("[r3_algorithm] Exited!")
                print("[r3_algorithm] " + next(info.iter_rooms()).render(robot))
            return True
        return False
