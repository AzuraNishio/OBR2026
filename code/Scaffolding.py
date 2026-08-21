from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.parameters import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.pupdevices import *
import sys
from libs.Utills import *
from LineFollowerRobot import LineFollowerRobot

class Context:
    def __init__(self, errorHandler: FailHandler):
        self.state = "initializing context"

        self.errorHandler = errorHandler
        self.hub = PrimeHub()
        self.speaker = self.hub.speaker
        self.imu = self.hub.imu
        self.display = self.hub.display
        self.buttons = self.hub.buttons
        self.battery = self.hub.battery
        self.cron = StopWatch()
        self.display.off()
        self.robot: LineFollowerRobot = None
        errorHandler.bindContext(self)
        print("\033[2J\033[H", end="")
        print("\n")
        self.setup_ports()
        self.hub_info_dump()
    
    def time(self):
        return self.cron.time() * 0.001
    
    def throw(self, e: Exception):
        self.errorHandler.exception(e)
    
    def time_ms(self):
        return self.cron.time()

    def setup_one_port(self, port: Port):
        pp = connect_port_device(port)
        self.ports[str(port)] = pp
        return pp
    
    def reload_one_port(self, port: Port):
        pp = connect_port_device(port)
        if pp[1] != None:
            self.ports[str(port)] = pp
            return pp
        else:
            return self.ports[str(port)]
        
    def setup_ports(self):
        self.ports = {}

        self.pA = self.setup_one_port(Port.A)
        self.pB = self.setup_one_port(Port.B)
        self.pC = self.setup_one_port(Port.C)
        self.pD = self.setup_one_port(Port.D)
        self.pE = self.setup_one_port(Port.E)
        self.pF = self.setup_one_port(Port.F)
    
    def reload_empty_ports(self):
        self.pA = self.reload_one_port(Port.A)
        self.pB = self.reload_one_port(Port.B)
        self.pC = self.reload_one_port(Port.C)
        self.pD = self.reload_one_port(Port.D)
        self.pE = self.reload_one_port(Port.E)
        self.pF = self.reload_one_port(Port.F)
    
    def indicate_port(self, p):
        ind = "ABCDEF".find(p)
        self.display.off()
        self.display.pixel(int("002244"[ind]), int("040404"[ind]), 100)
        self.display.pixel(int("002244"[ind]), int("131313"[ind]), 100)

    def show_string_encoded_img(self, s):
        i = 0
        for c in s:
            if c in "0123456789.ABCDEFGHIJKLMNOPQRSTUV":
                if c in "ABCDEFGHIJKLMNOPQRSTUV":
                    self.display.pixel((i - (i % 5)) / 5, i % 5, 3)
                elif c != ".":
                    self.display.pixel((i - (i % 5)) / 5, i % 5, int(c) * 10)
                else:
                    self.display.pixel((i - (i % 5)) / 5, i % 5, 0)
                i += 1
            

    async def routine(self):
        while True:
            await wait(20)
            self.errorHandler.refresh_display()
            if await self.robot.L_sensor.color() == Color.RED:
                self.robot.drive_base.stop()

                sys.exit()




    def get_port(self, port:str):
        return self.ports[f"Port.{port}"][1]
    
    def get_port_of_type(self, port:str, expected):
        if not isinstance(self.ports[f"Port.{port}"][1], expected):
            raise WrongPortError(port, str(expected)[8:len(str(expected))-2], self.ports[f"Port.{port}"][0])

        return self.ports[f"Port.{port}"][1]
    
    def battery_percent_approximation(self):
        v = self.battery.voltage() / 1000

        if v >= 8.4: return 100
        if v <= 6.6: return 0

        return int((v - 6.6) / (8.4 - 6.6) * 100)

    def hub_info_dump(self):
        state_log = ("\n" * 2) + "System state dump\n" + ("="*100)
        state_log += f"\nBatery current: {self.battery.current()}mA,  Batery voltage: {self.battery.voltage()}mV"
        battery_percent = self.battery_percent_approximation()
        state_log += f"\nBatery level: n{battery_percent}% [{"=" * int(battery_percent * 0.2)}{" " * int((100 - battery_percent) * 0.2)}]"
        
        state_log += "\n" + "-" * 100

        system_info = self.hub.system.info()
        state_log += f"\nBLE connected: {system_info.get("host_connected_ble")}, Hub name: {system_info.get("name")}"

        state_log += "\n" + "-" * 100

        state_log += "\nPORT VIEW"
        
        state_log += f"\nA: {self.ports["Port.A"][0]:<20}" + f"B: {self.ports["Port.B"][0]:<20}"
        state_log += f"\nC: {self.ports["Port.C"][0]:<20}" + f"D: {self.ports["Port.D"][0]:<20}"
        state_log += f"\nE: {self.ports["Port.E"][0]:<20}" + f"F: {self.ports["Port.F"][0]:<20}"

        state_log += "\n" + "-" * 100
        state_log += f"\nCharger current: {self.hub.charger.current()} mA"
        state_log += "\n" + "-" * 100

        print(state_log)

        if (battery_percent < 70):
            self.low_batterie_warn()

    def low_batterie_warn(self):
        print(f"Low battery! {self.battery_percent_approximation()}%")
        self.sound_message(1)
        




    def sound_message(self, n):
        self.boop(300, 200)
        for i in range(n):
            self.boop(600, 300)



    def boop(self, f, t = 400):
        for i in range (10):
            v = (i)/10
            v = v ** 0.7
            v = 1 - v
            v = v
            v *= 100
            self.speaker.volume(v)
            self.speaker.beep(f, t / 10)
