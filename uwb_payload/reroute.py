# psuedo code

# initialize position
# initialize target_distance
# initialize direction as forward

# while target_distance > threshold:
#     read ultrasonic sensor for obstacle_distance
    
#     if obstacle_distance < safe_distance:
#         # Obstacle detected, change direction
#         turn random direction (left or right)
#         move forward a small distance
#     else:
#         # No obstacle, move towards the target
#         move forward a small distance
    
#     # Update position and target distance
#     update current_position
#     update target_distance based on UWB signal
    
#     if target_distance not decreasing:
#         # Stuck in a local minimum, try a different direction
#         turn random direction
#         move forward a small distance
    
# stop when target_distance <= threshold
import serial.tools.list_ports
import time
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from sphero_sdk import SpheroRvrObserver
import asyncio
from sphero_sdk import SpheroRvrAsync, SerialAsyncDal
from enum import Enum

from util import *


ports, loop, rvr, serialInst = initialize()

position = (0,0)
target_distance = None # TODO: Get initial distance reading
direction = Direction.FORWARD
current_heading = 0
threshold = 500

while True: #TODO: Adjust to resemble main control loop
    if target_distance <= threshold:
        standby_mode()
        continue

    reading = get_distance_senor_reading()
    if is_obstacle(reading):
        current_heading = change_rvr_heading(rvr, current_heading)

        # Drive some short distance in new heading
        # May be able to simplify this later
        rvr.drive_control.drive_forward_seconds(
            speed=25,
            heading=current_heading,  # Valid heading values are 0-359
            time_to_drive=.2
        )
    else:
        rvr.drive_control.drive_forward_seconds(
            speed=25,
            heading=current_heading,  # Valid heading values are 0-359
            time_to_drive=.2
        )
    
    new_target_distance = get_uwb_distance(ports, serialInst)

    if target_distance <= new_target_distance:
        current_heading = change_rvr_heading(rvr)

        # Drive some short distance in new heading
        # May be able to simplify this later
        rvr.drive_control.drive_forward_seconds(
            speed=25,
            heading=current_heading,  # Valid heading values are 0-359
            time_to_drive=.2
        )
        target_distance = new_target_distance

