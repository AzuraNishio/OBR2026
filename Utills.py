from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port

def connect_port_device(port):
    try:
        return ("Motor", Motor(port))
    except:
        pass

    try:
        return ("Color Sensor", ColorSensor(port))
    except:
        pass

    try:
        return ("Ultrasonic Sensor", UltrasonicSensor(port))
    except:
        pass

    return ("Nothing / Unknown", None)