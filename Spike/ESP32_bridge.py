from pybricks.hubs import PrimeHub
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from umath import *
# Initialize the EV3
hub = PrimeHub()
# Initialize sensor port 2 as a uart device
ser = UARTDevice(Port.B, baudrate=115200, power_pin=1)

HEADER_BYTE = 0xA0
SCAPE_BYTE = 0xA1
"""
Addapted SLIP protocol
(Adapted to make my life easier)

Flags:
HEADER - 0xA0
SCAPE  - 0xA1

Scaped flag bytes
F -> F xor 0XA0

Packet structure
1: HEADER (0xA0)
2: Identifier (8bit integer)
CONTENT
"""


class DataPacket:
    def __init__(self, id):
        self.id = id
        self.content = bytearray([])

    def append_byte(self, byte):
        self.content.append(byte)

    def _get_scapped_content(self):
        scaped = bytearray([])
        for byte in self.content:
            if byte in [HEADER_BYTE, SCAPE_BYTE]:
                scaped.append(SCAPE_BYTE)
                scaped.append(byte ^ SCAPE_BYTE)
            else:
                scaped.append(byte)
        return scaped

    def to_bytes(self):
        out = bytearray([HEADER_BYTE, self.id])
        out.extend(self._get_scapped_content())
        return bytes(out)


class PacketReceiver:
    """
    Feed raw bytes in one at a time with feed_byte().
    Returns a completed DataPacket when a new packet starts
    (i.e. an unescaped HEADER_BYTE is seen after the first one),
    otherwise returns None.
    """

    WAIT_HEADER = 0
    WAIT_ID = 1
    IN_CONTENT = 2
    ESCAPED = 3

    def __init__(self):
        self._state = self.WAIT_HEADER
        self._packet = None

    def feed_byte(self, byte):
        completed = None

        if self._state == self.WAIT_HEADER:
            if byte == HEADER_BYTE:
                self._state = self.WAIT_ID

        elif self._state == self.WAIT_ID:
            self._packet = DataPacket(byte)
            self._state = self.IN_CONTENT

        elif self._state == self.IN_CONTENT:
            if byte == HEADER_BYTE:
                # new packet starts, current one is done
                completed = self._packet
                self._packet = None
                self._state = self.WAIT_ID
            elif byte == SCAPE_BYTE:
                self._state = self.ESCAPED
            else:
                self._packet.append_byte(byte)

        elif self._state == self.ESCAPED:
            self._packet.append_byte(byte ^ SCAPE_BYTE)
            self._state = self.IN_CONTENT

        return completed

    def feed_bytes(self, data):
        """Feed multiple bytes, return list of any completed packets."""
        packets = []
        for b in data:
            pkt = self.feed_byte(b)
            if pkt is not None:
                packets.append(pkt)
        return packets

class ESP32_bridge:
    def __init__(self, port):
        self.port = port
        self.UART = UARTDevice(port, baudrate=115200, power_pin=1)



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