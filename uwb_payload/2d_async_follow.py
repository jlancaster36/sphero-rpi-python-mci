import serial.tools.list_ports
from sphero_sdk import RawMotorModesEnum
from sphero_sdk import SerialAsyncDal
from sphero_sdk import SpheroRvrObserver
from sphero_sdk import SpheroRvrAsync
import asyncio
import os
import sys
import time
sys.path.append(os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../')))


ports = serial.tools.list_ports.comports()
for p, desc, hwid in ports:
    print(p, desc, hwid)
serialRight = serial.Serial('/dev/ttyACM0', 19200, timeout=5)
serialLeft = serial.Serial('/dev/ttyACM1', 19200, timeout=5)
loop = asyncio.get_event_loop()

rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

# rvr_sync = SpheroRvrObserver()

delimeter = ' '
distance_left = 0
distance_right = 0
turn_threshold = 100

prev_ema_ser9 = 0
curr_ema_ser9 = 0
prev_ema_ser3 = 0
curr_ema_ser3 = 0
alpha = 0.3
turn_threshold = 120


async def read_data():
    global distance_left
    global distance_right

    if serialLeft.in_waiting > 10:
        packetLeft = serialLeft.read(serialLeft.in_waiting)
        packetRight = serialRight.read(serialRight.in_waiting)
        dataLeft = packetLeft.decode('utf-8').split(delimeter)[-1]
        dataRight = packetRight.decode('utf-8').split(delimeter)[-1]
        if dataLeft != "occured\r\n":
            try:
                distance_left = (int)(dataLeft.split(":")[-1])

                if abs(distance_left) < 10000:
                    if prev_ema_ser3 == 0:
                        curr_ema_ser3 = distance_left
                        prev_ema_ser3 = curr_ema_ser3
                    else:
                        curr_ema_ser3 = alpha * distance_left + \
                            (1 - alpha) * prev_ema_ser3
                        prev_ema_ser3 = curr_ema_ser3

            finally:
                print('excepition while getting distance value')
                return
        if dataRight != "occured\r\n":
            try:
                distance_right = (int)(dataRight.split(":")[-1])

                if abs(distance_right) < 10000:
                    if prev_ema_ser9 == 0:
                        curr_ema_ser9 = distance_right
                        prev_ema_ser9 = curr_ema_ser9
                    else:
                        curr_ema_ser9 = alpha * distance_right + \
                            (1 - alpha) * prev_ema_ser9
                        prev_ema_ser9 = curr_ema_ser9
            finally:
                print('excepition while getting distance value')
                return


async def drive_rover():

    print("Distance Left: ", curr_ema_ser3)
    print("Distance Right: ", curr_ema_ser9)

    diff = (curr_ema_ser3-curr_ema_ser9)+70
    # print("Diff: ", diff)

    if abs(diff) < turn_threshold:
        print("Drive Forward")
    elif diff > turn_threshold:
        print("Drive Left")
    elif diff < -turn_threshold:
        print("Drive Right")
    if abs(diff) < turn_threshold:
        await rvr.drive_control.drive_forward_seconds(speed=60, heading=0, time_to_drive=0.01)
    elif diff > turn_threshold:
        await rvr.drive_control.drive_forward_seconds(speed=60, heading=330, time_to_drive=0.01)
    elif diff < -turn_threshold:
        await rvr.drive_control.drive_forward_seconds(speed=60, heading=30, time_to_drive=0.01)


async def main():
    """ This program has RVR run its motors using the raw_motors command.
        This command sets a duty cycle for each motor, normalized to the
        range 0-255.  Setting a non-zero duty cycle does not guarantee
        a motor has enough torque to start spinning - in many conditions
        a low duty cycle value will not overcome friction.  In almost all
        situations, using one of the "drive_..." commands will be preferable
        as they employ various types of feedback control systems.
    """

    await rvr.wake()

    # Give RVR time to wake up
    await asyncio.sleep(2)

    while True:
        await read_data()
        await drive_rover()

    await rvr.close()


if __name__ == '__main__':
    try:
        loop.run_until_complete(
            main()
        )

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        loop.run_until_complete(
            rvr.close()
        )

    finally:
        if loop.is_running():
            loop.close()
