"""Build a compact opening policy for the Room 3 search.

Run from the repository root with:
    python Spike/V3/build_r3_opening_book.py
    python Spike/V3/build_r3_opening_book.py --depth 5
"""

import argparse
import math as umath
import os
import sys
from contextlib import redirect_stdout

SPIKE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SPIKE_DIR not in sys.path:
    sys.path.insert(0, SPIKE_DIR)

# NekoMath keeps the hub's native umath import; the desktop builder supplies math.
sys.modules["umath"] = umath

from V3.NekoMath import straight_move_vec_from_4_state_dir
from V3.r3_algorithm import (
    BACKWARD,
    FORWARD,
    LEFT,
    RIGHT,
    action_from_key,
    decision_policy_2,
    r3_info,
    update_r3_run_info,
)
from V3.r3_model import (
    NULL_ROOM,
    r3_model,
    room_from_descriptor,
)
from V3.r3_model_robot import r3_model_robot
from V3.r3_opening_book import (
    FALLBACK_ACTION,
    OPENING_INFORMATION_WEIGHT,
    ROOT_ACTION,
    TERMINAL_ACTION,
    current_config_signature,
    encode_measurement,
    encode_nodes,
)

INITIAL_DATA = {
    "onboard_victims": 0,
    "delivered_victims": 0,
    "visited_positions": [],
}
MOVE_ACTIONS = (FORWARD, BACKWARD)
ACTIONS = (FORWARD, BACKWARD, LEFT, RIGHT)
SENSOR_KEY_INDEXES = (0, 1, 2, 3)


class _QuietOutput:
    def write(self, text):
        return len(text)

    def flush(self):
        pass


def _measurement_for_room(robot, room, action):
    exited = robot.exited
    if action in MOVE_ACTIONS:
        kind = room.kind(robot.x + room.offset, robot.y)
        exited = kind if kind in ("exit", "entrance") else None

    distance = room.reading(
        robot.x + room.offset,
        robot.y,
        robot.dir(),
    )
    # Triangle layouts are intentionally not part of the opening-book room state.
    touched_triangle = False
    touched_wall = False

    dx, dy = robot.last_dir
    if room.kind(
        robot.x + dx + room.offset,
        robot.y + dy,
    ) in ("obstacle", "wall"):
        touched_wall = True

    return distance, touched_triangle, touched_wall, exited


def _sensor_key(measurement):
    return tuple(measurement[index] for index in SENSOR_KEY_INDEXES)


class OpeningBookBuilder:
    def __init__(self, max_depth, max_nodes, information_weight):
        self.max_depth = max_depth
        self.max_nodes = max_nodes
        self.information_weight = information_weight
        self.nodes = [
            (ROOT_ACTION, []),
            (FALLBACK_ACTION, []),
            (TERMINAL_ACTION, []),
        ]
        self._canonical_nodes = {}
        self.decision_count = 0

    def _add_node(self, action, edges):
        edges = tuple(sorted(edges))
        signature = action, edges
        existing = self._canonical_nodes.get(signature)
        if existing is not None:
            return existing

        if len(self.nodes) >= self.max_nodes:
            raise RuntimeError(
                "opening book exceeded --max-nodes; lower --depth or raise the limit"
            )

        node_id = len(self.nodes)
        self.nodes.append((action, edges))
        self._canonical_nodes[signature] = node_id
        return node_id

    def _observation_groups(self, robot, descriptors, action=None):
        outcomes = {}
        sensor_groups = {}
        room = r3_model("3x3", 0, 1)

        for descriptor in descriptors:
            room_from_descriptor(descriptor, room)
            measurement = _measurement_for_room(robot, room, action)
            outcomes.setdefault(measurement, []).append(descriptor)
            sensor_groups.setdefault(_sensor_key(measurement), []).append(descriptor)

        return outcomes, sensor_groups

    def _information_bonuses(self, robot, info):
        bonuses = {}
        for action in ACTIONS:
            moved = robot.copy()
            moved.noclip = True
            action_from_key(action, moved)
            outcomes, _ = self._observation_groups(
                moved,
                info.room_descriptors,
                action,
            )

            # A room deterministically produces one measurement. Its outcome
            # entropy is therefore the expected information gained about the
            # hidden room by taking this action and measuring afterward.
            outcome_probabilities = []
            for descriptors in outcomes.values():
                probability = sum(
                    info.room_probability(descriptor)
                    for descriptor in descriptors
                )
                if probability > 0:
                    outcome_probabilities.append(probability)
            information_gain = -sum(
                probability * umath.log2(probability)
                for probability in outcome_probabilities
            )
            bonuses[action] = information_gain * self.information_weight
        return bonuses

    def _build_action_node(self, robot, info, persistent_data, action_depth):
        if action_depth >= self.max_depth or not info.room_descriptors:
            return 1

        information_bonuses = self._information_bonuses(robot, info)
        action = decision_policy_2(
            robot,
            info,
            persistent_data,
            verbose=False,
            first_action_bonuses=information_bonuses,
        )
        if action not in (FORWARD, BACKWARD, LEFT, RIGHT):
            return 1

        next_robot = robot.copy()
        next_robot.noclip = True
        action_from_key(action, next_robot)

        next_data = persistent_data.copy()
        update_r3_run_info(next_robot, info, next_data, action)

        outcomes, sensor_groups = self._observation_groups(
            next_robot,
            info.room_descriptors,
            action,
        )
        edges = []

        for measurement, _ in outcomes.items():
            if measurement[3] == "exit":
                child_id = 2
            else:
                descriptors = sensor_groups[_sensor_key(measurement)]
                child_info = r3_info(descriptors)
                child_robot = next_robot.copy()
                child_robot.exited = measurement[3]
                child_id = self._build_action_node(
                    child_robot,
                    child_info,
                    next_data,
                    action_depth + 1,
                )
            edges.append((encode_measurement(measurement), child_id))

        self.decision_count += 1
        return self._add_node(action, edges)

    def build(self):
        initial_info = r3_info()
        initial_robot = r3_model_robot(NULL_ROOM)
        initial_robot.noclip = True
        initial_robot.time = 0.0

        outcomes, sensor_groups = self._observation_groups(
            initial_robot,
            initial_info.room_descriptors,
        )
        root_edges = []

        for measurement in outcomes:
            descriptors = sensor_groups[_sensor_key(measurement)]
            child_info = r3_info(descriptors)
            child_robot = initial_robot.copy()
            child_robot.exited = measurement[3]
            child_id = self._build_action_node(
                child_robot,
                child_info,
                INITIAL_DATA.copy(),
                0,
            )
            root_edges.append((encode_measurement(measurement), child_id))

        self.nodes[0] = (ROOT_ACTION, tuple(sorted(root_edges)))
        return self.nodes


def write_module(path, nodes, information_weight):
    data = encode_nodes(nodes)
    config = current_config_signature(information_weight)
    with open(path, "w", encoding="ascii") as output:
        output.write("BOOK_CONFIG = ")
        output.write(repr(config))
        output.write("\nBOOK_DATA = ")
        output.write(repr(data))
        output.write("\n")
    return len(data), os.path.getsize(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--depth", type=int, default=5, help="number of actions to precompute")
    parser.add_argument("--max-nodes", type=int, default=4096, help="maximum serialized nodes")
    parser.add_argument(
        "--information-weight",
        type=float,
        default=OPENING_INFORMATION_WEIGHT,
        help="score bonus per bit of expected measurement information",
    )
    parser.add_argument(
        "--output",
        default=os.path.join(os.path.dirname(__file__), "r3_opening_book_data.py"),
        help="generated Python module path",
    )
    args = parser.parse_args(argv)
    if args.depth < 1:
        parser.error("--depth must be at least 1")
    if args.max_nodes < 3 or args.max_nodes > 0xFFFF:
        parser.error("--max-nodes must be from 3 to 65535")

    builder = OpeningBookBuilder(args.depth, args.max_nodes, args.information_weight)
    try:
        with redirect_stdout(_QuietOutput()):
            nodes = builder.build()
        binary_size, module_size = write_module(
            args.output,
            nodes,
            args.information_weight,
        )
    except RuntimeError as error:
        parser.exit(2, "Opening-book build stopped: {}\n".format(error))

    print("Opening book generated:", args.output)
    print("Actions searched:", builder.decision_count)
    print("Nodes after exact subtree deduplication:", len(nodes))
    print("Information bonus weight:", args.information_weight, "points/bit")
    print("Packed table bytes:", binary_size)
    print("Generated module bytes:", module_size)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())