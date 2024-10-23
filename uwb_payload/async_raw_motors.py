from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RawMotorModesEnum
import serial.tools.list_ports
import asyncio
import os
import sys

sys.path.append(os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../')))

serialInst = serial.Serial('/dev/ttyACM0', 19200, timeout=5)

loop = asyncio.get_event_loop()

rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

distance = 1000
delimeter = ' '


async def read_data():
    global distance
    packet = serialInst.readline()
    decoded_packet = packet.decode('utf')
    data = decoded_packet.split(delimeter)
    print(data)
    dist = (int)(data[2])
    distance = dist


async def drive_rover():
    if distance >= 1000:
        await rvr.raw_motors(
            left_mode=RawMotorModesEnum.forward.value,
            left_duty_cycle=128,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.forward.value,
            right_duty_cycle=0  # Valid duty cycle range is 0-255
        )


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
