# pgzrun

from math import *
from operator import pos
import pgzrun
from pygame import *
from good_math import *
from pgzero.screen import Screen
from pgzero.keyboard import keyboard

adress = 6000

# Connection system #
from multiprocessing.connection import Listener


listener = Listener(("localhost", adress), authkey=b"robot")

print("Waiting for runner...")

connection = listener.accept()

print("Runner connected!")



screen: Screen
def polygons_collide(a, b):
    smallest_overlap = float("inf")
    collision_normal = None

    for polygon in (a, b):
        for i in range(len(polygon)):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % len(polygon)]

            edge = p2 - p1
            axis = Vector2(-edge.y, edge.x + 0.0001).normalize()

            a_projection = [p.dot(axis) for p in a]
            b_projection = [p.dot(axis) for p in b]

            overlap = min(
                max(a_projection),
                max(b_projection)
            ) - max(
                min(a_projection),
                min(b_projection)
            )

            if overlap <= 0:
                return False, None, None

            if overlap < smallest_overlap:
                smallest_overlap = overlap
                collision_normal = axis

    center_a = sum(a, Vector2()) / len(a)
    center_b = sum(b, Vector2()) / len(b)

    # Make normal point from B toward A.
    if (center_a - center_b).dot(collision_normal) < 0:
        collision_normal = -collision_normal

    # Furthest point of A in the direction of B.
    collision_point = min(
        a,
        key=lambda p: p.dot(collision_normal)
    )

    return True, Vector2(collision_point), collision_normal





# ============================================================
# WORLD
# ============================================================

WORLD_W = 400 * 3
WORLD_H = 400 * 3

# World units -> screen pixels
SCALE = 0.3

SCREEN_W = int(WORLD_W * SCALE)
SCREEN_H = int(WORLD_H * SCALE)


# ============================================================
# WORLD <-> SCREEN CONVERSION
# ============================================================

def world_to_screen_single(n):
    """
    Convert a world-space Vector2 to screen-space Vector2.
    """
    return n * SCALE

def world_to_screen(pos):
    """
    Convert a world-space Vector2 to screen-space Vector2.
    """
    return Vector2(
        world_to_screen_single(pos.x) + 100,
        world_to_screen_single(pos.y) + 400
    )

def screen_to_world(pos):
    """
    Convert a screen-space Vector2 to world-space Vector2.
    """
    return Vector2(
        pos.x / SCALE,
        pos.y / SCALE
    )

def world_rect_to_screen(rect):
    """
    Convert a world-space Rect into a screen-space Rect.
    """
    pos = world_to_screen(Vector2(rect.x, rect.y))
    return Rect(
        int(pos.x),
        int(pos.y),
        int(world_to_screen_single(rect.width)),
        int(world_to_screen_single(rect.height))
    )

# ============================================================
# ROBOT
# ============================================================

class command:
    def __init__(self, name, *args):
        self.name = name
        self.args = args


def raycast(origin, direction, max_distance=2000):
    direction = Vector2(direction)

    if direction.length_squared() == 0:
        return None

    direction.normalize_ip()

    closest_distance = max_distance
    closest_point = None
    closest_wall = None

    for wall in walls:
        corners = [
            Vector2(wall.left, wall.top),
            Vector2(wall.right, wall.top),
            Vector2(wall.right, wall.bottom),
            Vector2(wall.left, wall.bottom),
        ]

        for i in range(4):
            a = corners[i]
            b = corners[(i + 1) % 4]

            segment = b - a
            denominator = direction.cross(segment)

            if abs(denominator) < 1e-9:
                continue

            offset = a - origin

            t = offset.cross(segment) / denominator
            u = offset.cross(direction) / denominator

            if t < 0:
                continue

            if not 0 <= u <= 1:
                continue

            if t < closest_distance:
                closest_distance = t
                closest_point = origin + direction * t
                closest_wall = wall

    return closest_distance, closest_point, closest_wall

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 90
        self.width = 120
        self.height = 144
        self.speed = 3
        self.turn_speed = 2
        self.garra = 0
        self.command = None
        self.distance = 0
        self.wasIdle = True

    def idle(self):
        return self.command is None

    def update(self):
        if self.command is None:
            self.wasIdle = True
            return

        handler = getattr(self, f"update_{self.command.name}", None)

        if handler is None:
            self.wasIdle = True
            self.command = None
            return

        result = handler()

        if self.command is None:
            self.wasIdle = True
            return result
        
        self.wasIdle = False

        return result

    def update_garra_down(self):
        self.garra += 0.05

        if self.garra >= 1:
            self.garra = 1
            self.command = None

    def update_sonic_distance(self):
        origin = Vector2(self.x, self.y)
        direction = Vector2(1, 0).rotate(-self.angle)

        result = raycast(origin, direction, 1000)

        if result is None:
            return 1000

        distance, point, wall = result
        self.command = None
        return int(distance)


    def update_garra_up(self):
        self.garra -= 0.05

        if self.garra <= 0:
            self.garra = 0
            self.command = None


    def update_straight(self):
        distance = abs(self.command.args[0])

        if self.wasIdle:
            self.startDist = self.distance

        travelled = abs(self.startDist - self.distance)

        if travelled >= distance:
            self.command = None
            return

        remaining = distance - travelled
        instant_speed = min(self.speed, remaining)

        direction = 1 if self.command.args[0] > 0 else -1

        self.move(instant_speed * direction)
        self.distance += instant_speed * direction


    def update_turn(self):
        angle = abs(self.command.args[0])

        if self.wasIdle:
            self.startAngle = self.angle

        turned = abs(self.startAngle - self.angle)

        if turned >= angle:
            self.command = None
            return

        remaining = angle - turned
        instant_speed = min(self.turn_speed, remaining)

        direction = 1 if self.command.args[0] < 0 else -1

        self.rotate(instant_speed * direction)

    @property
    def corners(self):
        center = Vector2(self.x, self.y)
        corners = [
            Vector2(-self.width / 2, -self.height / 2),
            Vector2(self.width / 2, -self.height / 2),
            Vector2(self.width / 2, self.height / 2),
            Vector2(-self.width / 2, self.height / 2),
        ]
        return [center + corner.rotate(-self.angle) for corner in corners]

    @property
    def corners_garra(self):
        center = Vector2(self.x, self.y)
        forward_garra = 60 * self.garra
        corners = [
            Vector2(self.width / 2, -self.height / 2),
            Vector2(-self.width / 2 - forward_garra, -self.height / 2),
            Vector2(-self.width / 2 - forward_garra, self.height / 2),
            Vector2(self.width / 2, self.height / 2),
        ]
        return [center + corner.rotate(-self.angle) for corner in corners]

    def move(self, distance):
        direction = Vector2(1, 0).rotate(-self.angle)
        self.try_move(direction.x * distance, direction.y * distance)

    def try_move(self, dx, dy):
        self.x += dx
        self.y += dy

        for _ in range(20):
            colliding, point, normal = self.collides()

            if not colliding:
                break

            center = self.center()

            # Direction from collision point toward robot center
            push = (center - point)

            normal_component = push.dot(normal) * normal
            # push -= normal_component

            if push.length_squared() > 0:
                push.normalize_ip()

                # Push robot away from wall
                self.x += push.x
                self.y += push.y

            # Rotational response
            lever = point - center

            torque = lever.y * normal_component.x - lever.x * normal_component.y

            self.angle += torque * 0.0005

    def translate(self, vec):
        self.x += vec.x
        self.y += vec.y

    def rotate(self, d_angle):
        for i in range(20):
            self.angle += d_angle / 20
            colliding, point, normal = self.collides()
            if colliding:
                drag = (point - self.center()) - (point - self.center()).rotate(-d_angle / 20)
                unclip = drag.dot(normal) * normal
                unclip += normal * 0.1
                while True:
                    self.translate(unclip * 3)
                    colliding, point, normal = self.collides()
                    if not colliding:
                        break
                
        

    def collides(self):
        corners = self.corners
        corners_garra = self.corners_garra

        for wall in walls:
            wall_corners = [
                Vector2(wall.left, wall.top),
                Vector2(wall.right, wall.top),
                Vector2(wall.right, wall.bottom),
                Vector2(wall.left, wall.bottom),
            ]

            result = polygons_collide(corners, wall_corners)

            if result[0]:
                return result

            result = polygons_collide(corners_garra, wall_corners)

            if result[0]:
                return result

        return False, None, None

    def center(self):
        return Vector2(self.x, self.y)


    def draw_sonic(self):
        origin = Vector2(self.x, self.y)
        direction = Vector2(1, 0).rotate(-self.angle)

        distance, end, wall = raycast(origin, direction)

        if end is None:
            end = origin + direction * 1000


        screen.draw.line(
            world_to_screen(origin),
            world_to_screen(end),
            "yellow"
        )

    def draw(self):
        self.draw_sonic()
        points = [world_to_screen(p) for p in self.corners_garra]
        for i in range(len(points)):
            screen.draw.line(points[i], points[(i + 1) % len(points)], "green")

        points = [world_to_screen(p) for p in self.corners]
        for i in range(len(points)):
            screen.draw.line(points[i], points[(i + 1) % len(points)], "white")
        
        center_screen_space = world_to_screen(self.center())
        screen.draw.line(center_screen_space, center_screen_space + Vector2(1, 0).rotate(-self.angle) * 15, "red")

# ============================================================
# WALLS
# ============================================================

WALL_SIZE = 300

def generate_walls(input, output):
    walls = []

    WALL_THICKNESS = 20

    offset = Vector2(0, 0) # Vector2(WALL_SIZE / 2, WALL_SIZE / 2)
    offset = Vector2(3 * WALL_SIZE / 2, -3 * WALL_SIZE / 2)

    for wall in range(4):
        angle = wall * pi / 2

        normal   = Vector2(0, -1).rotate_rad(angle)
        parallel = Vector2(1,  0).rotate_rad(angle)

        for segment in range(3):
            n = (2 - segment) + ((2 - wall) % 4) * 3 + 1

            if n == input or n == output:
                continue

            center = (
                offset 
                + normal * 450
                + parallel * ((segment - 1) * 300)
            )
            p1 = (center - parallel * 150 - normal * 10)
            p2 = (center + parallel * 150 - normal * 10)
            p3 = (center + parallel * 150 + normal * 10)
            p4 = (center - parallel * 150 + normal * 10)

            left   = min(p1.x, p2.x, p3.x, p4.x)
            right  = max(p1.x, p2.x, p3.x, p4.x)
            top    = min(p1.y, p2.y, p3.y, p4.y)
            bottom = max(p1.y, p2.y, p3.y, p4.y)

            walls.append(
                Rect(
                    int(left),
                    int(top),
                    int(right - left),
                    int(bottom - top)
                )
            )
    return walls



input = 1
output = 6

walls = generate_walls(input, output)

robot = Robot(
    (WALL_SIZE * -0.5) + (WALL_SIZE * input),
    0
)

def reset():
    global walls, robot

    walls = generate_walls(input, output)

    robot = Robot(
        (WALL_SIZE * -0.5) + (WALL_SIZE * input),
        0
    )

# ============================================================
# INPUT / UPDATE
# ============================================================

commands = []

def update():
    if connection.poll():
        cmd: command = connection.recv()
        if cmd.name == "reset":
            reset()
            return
        robot.command = cmd

        print("Received:", cmd.name)

    result = robot.update()

    if result != None:
        connection.send(result)

    if robot.idle():
        connection.send("idle")
        while not connection.poll():
            pass



# ============================================================
# RENDERING
# ============================================================

def draw():

    screen.fill("black")


    # --------------------------------------------------------
    # Walls
    # --------------------------------------------------------

    for wall in walls:

        screen.draw.filled_rect(
            world_rect_to_screen(wall),
            "white"
        )


    # --------------------------------------------------------
    # Robot
    # --------------------------------------------------------

    robot.draw()

# ============================================================
# START
# ============================================================

pgzrun.go()