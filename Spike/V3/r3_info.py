from V3.NekoMath import *
from V3.obrExperimentalData import *
from V3.r3_model import *
from V3.r3_model_robot import *

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

                    from V3.r3_model import descriptor_triangle_probability_at
                    triangle_probability = descriptor_triangle_probability_at(descriptor, xg - 1, y - 1)
                    self.triangle_map[y][x + 3] += increment * triangle_probability

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
        touched_triangle = bool(result[1])
        robot = getattr(measurement, "robot", None)
        from V3.r3_model import descriptor_can_have_triangle_at
        for descriptor in self.room_descriptors:
            room_from_descriptor(descriptor, room)
            if touched_triangle and robot is not None:
                triangle_x = robot.x + room.offset + robot.last_dir[0]
                triangle_y = robot.y + robot.last_dir[1]
                if not descriptor_can_have_triangle_at(descriptor, triangle_x, triangle_y):
                    continue
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
                # Triangle layouts are not part of candidate room states.
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