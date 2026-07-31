from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import *
from umath import *

from libs.AzSensors import AzColorSensor

tau = pi + pi

def connect_port_device(port: Port):
    
    try:
        return ("Motor", Motor(port))
    except:
        pass

    try:
        return ("Color Sensor", AzColorSensor(port))
    except:
        pass

    try:
        return ("Ultrasonic Sensor", UltrasonicSensor(port))
    except:
        pass

    return ("Nothing / Unknown", None)

ports_strings = {
    "a": Port.A,
    "A": Port.A,
    "b": Port.B,
    "B": Port.B,
    "c": Port.C,
    "C": Port.C,
    "d": Port.D,
    "D": Port.D,
    "e": Port.E,
    "E": Port.E,
    "f": Port.F,
    "F": Port.F
}

def get_port(port:str):
    return ports_strings[port]

def pretty_vec_mm_to_meter(vec:Matrix):
   return f"[x: {(vec[0]/1000):<+7.3f}, y: {(vec[1]/1000):<+7.3f}, z: {(vec[2]/1000):<+7.3f}]"

def pretty_vec_mm_to_cm_pos(vec:Matrix):
   return f"[x: {(vec[0]/10):<+7.3f}cm, y: {(vec[1]/10):<+7.3f}cm, z: {(vec[2]/10):<+7.3f}cm]"

def pretty_vec_cm_pos(vec:Matrix):
   return f"[x: {(vec[0]):<+7.3f}cm, y: {(vec[1]):<+7.3f}cm, z: {(vec[2]):<+7.3f}cm]"


def length_squared(vec):
    return sum(v * v for v in vec)

def length(vec):
    return length_squared(vec) ** 0.5

def dot(a, b):
    return sum(x * y for x, y in zip(a, b))

def normalize(vec):
    l = length(vec)
    if l == 0:
        return vec
    return tuple(v / l for v in vec)

def angle_between(a, b):
    la = length(a)
    lb = length(b)

    if la == 0 or lb == 0:
        return 0  # or handle differently

    cos_theta = dot(a, b) / (la * lb)

    # clamp for numerical safety
    cos_theta = max(-1, min(1, cos_theta))

    return acos(cos_theta)


def angle_diff_2d(a, b):
    # a and b are (x, y)
    return atan2(a[0]*b[1] - a[1]*b[0], dot(a, b))

def sign(v):
    if v >= 0:
        return 1
    else:
        return -1

def clamp(top, down, value):
    return max(down, min(top, value))

class PortError(RuntimeError):
    def __init__(self, port:str, *args):
        super().__init__(*args)
        self.port = port
    
