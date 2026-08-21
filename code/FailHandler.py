from pybricks import *
from pybricks.hubs import *
from pybricks import *
from pybricks.iodevices import *
from pybricks.robotics import *
from pybricks.hubs import *
from pybricks.pupdevices import *
from libs.Utills import *
from libs.Exceptions import * 
from libs.AwareRobotics.AwDriveBase import *
import Scaffolding

class FailHandler:
    def __init__(self):
        self.ctx = None
        self.display_state = ""

    def bindContext(self, ctx: Scaffolding.Context):
        self.ctx = ctx
    
    def exception(self, e: Exception):
        print(f"[ Error while {self.ctx.state}! ]")
        self.ctx.hub_info_dump()

        if isinstance(e, PortError):
            self.ctx.display.char(e.port)
            print(f"Port error in {e.port}! {e}")  
        else:
            print(e)

        # raise e
    
    def stuck_while_driving(self, base: AwDriveBase, state: str, time: int):
        if time > 500:
            print(state)
            if state == "left":
                self.display_state = """9....9....9....9....9...."""
            elif state == "right":
                self.display_state = """....9....9....9....9....9"""
            else:
                self.display_state = """9...99...99...99...99...9"""
        
    
    def refresh_display(self):
        if self.display_state != "":
            self.ctx.show_string_encoded_img(self.display_state)
        self.display_state = ""
        

