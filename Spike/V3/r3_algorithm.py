from V3.NekoMath import *
from V3.obrExperimentalData import *
from V3.r3_model import *
from V3.r3_model_robot import *
from V3.r3_opening_book import OpeningBook, opening_book_compatible
from V3.r3_score_functions import *
from V3.r3_info import r3_info

try:
    from Spike.V3.r3_real_robot import r3_real_robot
except ImportError:
    r3_real_robot = None

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
        d = rr.last_dir
        touched_triangle = getattr(rr, "touched_triangle", False)
        touched_wall = getattr(rr, "garra_touched_wall", False)
        if ref is None:
            return rr.measure(room=room), touched_triangle, touched_wall, rr.exited
        dist = rr.measure(room=room)
        if dist > clearance_after_r3 and dist < 4:
            dist = min(dist, ref[0])
        return dist, touched_triangle, touched_wall, rr.exited
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
            await robot.update_async()
            measurement = measure(robot)(None)
        if self.verbose:
            print("[r3_algorithm] Popping...")
            print("[r3_algorithm] dist:", measurement[0], "touched triangle:", measurement[1], "touched wall:", measurement[2], "exited:", measurement[3])
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
