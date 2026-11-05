from V3.obrExperimentalData import *
from V3.r3_model import ONLY_OBSTACLE_LESS_ROOMS, ROOM_SHAPES, SHAPES

BOOK_FORMAT_VERSION = 1
BOOK_POLICY_REVISION = 1
OPENING_INFORMATION_WEIGHT = 50.0
ROOT_ACTION = 253
TERMINAL_ACTION = 254
FALLBACK_ACTION = 255


def current_config_signature(information_weight=OPENING_INFORMATION_WEIGHT):
    return (
        BOOK_FORMAT_VERSION,
        BOOK_POLICY_REVISION,
        round_duration,
        avg_time_before_r3,
        avg_score_before_r3,
        avg_time_after_r3,
        avg_score_after_r3,
        score_multiplier_per_victim,
        probabilitie_of_room3_having_a_obstacle,
        tuple(
            (shape, probabilitie_of_room3_per_shape[shape])
            for shape in ROOM_SHAPES
        ),
        tuple((shape, SHAPES[shape]) for shape in ROOM_SHAPES),
        tuple(avg_time_per_action[key] for key in ("move", "turn", "measure")),
        ONLY_OBSTACLE_LESS_ROOMS,
        information_weight,
    )


def opening_book_compatible(config):
    # Tuning and prior probabilities may intentionally differ from runtime.
    # Only reject files whose binary layout or policy semantics are incompatible.
    return (
        isinstance(config, (tuple, list))
        and len(config) >= 2
        and config[0] == BOOK_FORMAT_VERSION
        and config[1] == BOOK_POLICY_REVISION
    )


def encode_measurement(measurement):
    distance, triangle, exited, pushed = measurement
    if not 0 <= distance <= 4:
        raise ValueError("distance must be from 0 to 4")

    exit_code = {None: 0, "exit": 1, "entrance": 2}.get(exited)
    if exit_code is None:
        raise ValueError("invalid exit measurement")
    if pushed not in (-1, 0, 1):
        raise ValueError("pushed must be -1, 0, or 1")

    return (
        distance
        | (int(bool(triangle)) << 3)
        | (exit_code << 4)
        | ((pushed + 1) << 6)
    )


def decode_measurement(code):
    distance = code & 0x07
    triangle = bool((code >> 3) & 1)
    exit_code = (code >> 4) & 0x03
    pushed = ((code >> 6) & 0x03) - 1

    if distance > 4 or exit_code > 2 or pushed < -1 or pushed > 1:
        raise ValueError("invalid encoded measurement")
    return distance, triangle, (None, "exit", "entrance")[exit_code], pushed


def encode_nodes(nodes):
    count = len(nodes)
    if count > 0xFFFF:
        raise ValueError("too many opening-book nodes")

    records = []
    offsets = []
    cursor = 7 + count * 2
    for action, edges in nodes:
        if not 0 <= action <= 0xFF or len(edges) > 0xFF:
            raise ValueError("node action or edge count is out of range")
        record = bytearray((action, len(edges)))
        for measurement_code, child in edges:
            if not 0 <= measurement_code <= 0xFF or not 0 <= child < count:
                raise ValueError("invalid opening-book edge")
            record.append(measurement_code)
            record.append(child & 0xFF)
            record.append((child >> 8) & 0xFF)
        offsets.append(cursor)
        records.append(record)
        cursor += len(record)

    if cursor > 0xFFFF:
        raise ValueError("opening book exceeds the 16-bit format limit")

    data = bytearray(b"R3OB")
    data.append(BOOK_FORMAT_VERSION)
    data.append(count & 0xFF)
    data.append((count >> 8) & 0xFF)
    for offset in offsets:
        data.append(offset & 0xFF)
        data.append((offset >> 8) & 0xFF)
    for record in records:
        data.extend(record)
    return bytes(data)


class OpeningBook:
    def __init__(self, data):
        if len(data) < 7 or data[0] != 82 or data[1] != 51 or data[2] != 79 or data[3] != 66:
            raise ValueError("invalid opening-book header")
        if data[4] != BOOK_FORMAT_VERSION:
            raise ValueError("unsupported opening-book version")

        self.data = data
        self.node_count = data[5] | (data[6] << 8)
        self.offset_table_end = 7 + self.node_count * 2
        if self.offset_table_end > len(data):
            raise ValueError("truncated opening-book offsets")

    def _node_offset(self, node_id):
        if node_id < 0 or node_id >= self.node_count:
            return None
        index = 7 + node_id * 2
        offset = self.data[index] | (self.data[index + 1] << 8)
        if offset < self.offset_table_end or offset + 2 > len(self.data):
            return None
        return offset

    def action_for(self, node_id):
        offset = self._node_offset(node_id)
        if offset is None:
            return None
        action = self.data[offset]
        return action if action < 4 else None

    def next_node(self, node_id, measurement):
        offset = self._node_offset(node_id)
        if offset is None:
            return None

        code = encode_measurement(measurement)
        edge_count = self.data[offset + 1]
        edge = offset + 2
        for _ in range(edge_count):
            if edge + 2 >= len(self.data):
                return None
            child = self.data[edge + 1] | (self.data[edge + 2] << 8)
            if self.data[edge] == code:
                return child if child < self.node_count else None
            edge += 3
        return None