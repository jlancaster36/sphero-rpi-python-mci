import serial.tools.list_ports
import time
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from sphero_sdk import SpheroRvrObserver

import asyncio

from sphero_sdk import SpheroRvrAsync, SerialAsyncDal
ports = serial.tools.list_ports.comports()

loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))

serialInst = serial.Serial('/dev/ttyACM0', 19200, timeout = 5)

async def main():
    try:
        await rvr.wake()

        # Give RVR time to wake up
        time.sleep(2)
    except KeyboardInterrupt:
            print('\nProgram terminated with keyboard interrupt.')

    await rvr.drive_control.reset_heading()
    dist_threshold = 1000
    delimeter = ','

    while True:
            while serialInst.in_waiting > 50:
                 _ = serialInst.read_all()
            packet = serialInst.readline()
            decoded_packet = packet.decode('utf')
            data = decoded_packet.split(delimeter)
            dist = (int) (data[4])
            print(dist)
            if dist > dist_threshold:
                print("Move Rover Forward")
                try:
                    await rvr.drive_with_heading(
                        speed=25,
                        heading=0
                    )
                    # move forward
                    # rvr.drive_control.drive_forward_seconds(
                    #     speed=25,
                    #     heading=0,  # Valid heading values are 0-359
                    #     time_to_drive=1
                    # )
                except:
                    print("Unkown exception occurred while driving forward")

asyncio.gather(main())