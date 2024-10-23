import serial.tools.list_ports
import time
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from sphero_sdk import SpheroRvrObserver
import asyncio
from sphero_sdk import SpheroRvrAsync, SerialAsyncDal
from enum import Enum

def initialize():
    ports = serial.tools.list_ports.comports()

    loop = asyncio.get_event_loop()
    rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))

    serialInst = serial.Serial('/dev/ttyACM0', 19200, timeout = 5)

    return ports, loop, rvr, serialInst

# TODO: Implement obstacle detection to differentiate between obstacle and target
def is_obstacle(reading):
    return True

# TODO: Implement obstacle distance sensor reading
def get_distance_senor_reading(reading):
    return 0

# TODO: Select new random heading for RVR
def change_rvr_heading(rvr):
    return 0

# TODO: Implement get uwb distance when we have multithreading
def get_uwb_distance(ports, serialInst):
    return 0

# TODO: Implement standby_mode
def standby_mode():
    pass

class Direction(Enum): #TODO: replace these with heading values
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    BACKWARD = 4

