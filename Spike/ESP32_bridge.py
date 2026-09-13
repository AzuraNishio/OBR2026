from pybricks.hubs import PrimeHub
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from umath import *
# Initialize the EV3
hub = PrimeHub()
# Initialize sensor port 2 as a uart device
ser = UARTDevice(Port.B, baudrate=115200, power_pin=1)

# Read all data received while the sound was playing
data = ser.read_all()
print(data)
msg = ""
dist = 2000
canvas = [[0 for x in range(5)] for y in range(5)]
while True:
    if ser.waiting():
        readed = str(ser.read_all())
        msg += readed[2:].replace("'", "").replace("\\r", "")
    if "\\n" in msg:
        msg = int(msg[:-2])
        dist = msg
        print(msg)
        msg = ""
    
    for x in range(5):
        for y in range(5):
            v = floor((dist / 100) - (0.2 * (0 + x))) * 4
            if v != canvas[x][y]:
                canvas[x][y] = v
                hub.display.pixel(x, y, v)