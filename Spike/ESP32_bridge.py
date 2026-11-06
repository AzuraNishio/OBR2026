from pybricks.hubs import PrimeHub
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from umath import *
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor



ALT_FLAG      = 0xA0;
STOP_FLAG     = 0xA5;
ALT_ALT_FLAG  = 0xA1;
ALT_STOP_FLAG = 0xA6;


class ESP32_bridge:
    def __init__(self, port):
        self.port = port
        self.device = UARTDevice(port, baudrate=115200, power_pin=1)
        self.packet_buffer = [[] for _ in range(8)]
        self.active_packet = 0
        self.is_alted = False
        self.unalt_map = {ALT_ALT_FLAG: ALT_FLAG, ALT_STOP_FLAG: STOP_FLAG}
        self.alt_map = {ALT_FLAG: [ALT_FLAG, ALT_ALT_FLAG], STOP_FLAG: [ALT_FLAG, ALT_STOP_FLAG]}
        self._avaiable = False

    def avaiable(self): return self._avaiable

    def get_packet(self):
        self._avaiable = False
        return self.packet_buffer[self.active_packet]

    def _write_to_packet_buffer(self, byte):
        self.packet_buffer[(self.active_packet + 1) % 8].append(byte)

    def _end_current_packet(self):
        self.is_alted = False
        self.active_packet = (self.active_packet + 1) % 8
        self.packet_buffer[(self.active_packet + 1) % 8] = []
        self._avaiable = True

    async def tick_packet_receiver(self):
        while self.device.waiting():
            byte = (await self.device.read(1))[0]
            if self.is_alted:
                self._write_to_packet_buffer(self.unalt_map.get(byte, 0x00))
                self.is_alted = False
            else:
                if byte == ALT_FLAG:
                    self.is_alted = True
                elif byte == STOP_FLAG:
                    self._end_current_packet()
                else:
                    self._write_to_packet_buffer(byte)

#    def send_packet(self, packet):
#        encoded = []
#
#        for b in packet:
#            encoded.extend(self.alt_map.get(b, [b]))
#
#        encoded.append(STOP_FLAG)
#
#        self.device.write(bytes(encoded))


class ESP32_distance_sensor(ESP32_bridge):
    def __init__(self, port):
        super().__init__(port)

    async def distance(self):
        await self.tick_packet_receiver()
        if len(self.get_packet()) < 1: return 9999
        if len(self.get_packet()) > 0 and self.get_packet()[0] == 0: return 9999
        return (self.get_packet()[0] | (self.get_packet()[1] << 8))

    
    async def L_button_pressed(self):
        await self.tick_packet_receiver()
        if len(self.get_packet()) < 1: return 9999
        if len(self.get_packet()) > 0 and self.get_packet()[0] == 0: return 9999
        return self.get_packet()[3] == 1

    async def R_button_pressed(self):
        await self.tick_packet_receiver()
        if len(self.get_packet()) < 1: return 9999
        if len(self.get_packet()) > 0 and self.get_packet()[0] == 0: return 9999
        return self.get_packet()[4] == 1


# TODO: prevent eternal pulling and buffer buildup. Make a bg process
class Esp32SensorPack:
    def __init__(self, port):
        try:
            self.sensor = UltrasonicSensor(port)
        except:
            try:
                ColorSensor(port)
            except:                    
                self.sensor = ESP32_distance_sensor(port)

    async def laser_distance(self):
        return await self.sensor.distance() 

    async def rear_sensor_pressed(self):
        return await self.sensor.L_button_pressed()

    def distance_sinc(self):
        return self.sensor.sinc_distance()


def init():
    dist_sensor = Esp32SensorPack(Port.F)
    while True:
        for i in range(1):
            print(f"{(dist_sensor.distance_sinc() or 0) / 10:.0f}")

