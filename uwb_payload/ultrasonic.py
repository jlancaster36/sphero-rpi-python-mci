import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
import RPi.GPIO as GPIO
import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
import time

loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(

    dal=SerialAsyncDal(

        loop

    )

)
GPIO.setmode(GPIO.BCM)
right_trigger = 6
right_echo = 12
left_trigger = 19
left_echo = 16
front_trigger = 26
front_echo = 20
GPIO.setup(left_trigger, GPIO.OUT)
GPIO.setup(left_echo, GPIO.IN)
GPIO.setup(right_trigger, GPIO.OUT)
GPIO.setup(right_echo, GPIO.IN)

async def main():
    await rvr.wake()
    await rvr.reset_yaw()
    await asyncio.sleep(.5)
    while True:
        dist_r =  distance_right()
        dist_l =  distance_left()
        dist_front = distance_front()
        print(dist_l)
        await asyncio.sleep(.05)
        print('Measurements are {0} cm right and {1} cm left'.format(dist_r, dist_l))
        if dist_r < 35:
            while dist_r < 35:
                await rvr.raw_motors(2,255,1,255)
                dist_r =  distance_right()
                await asyncio.sleep(.05)
                print('turning right')
            await rvr.reset_yaw()
        elif dist_l < 35:
            while dist_l < 35:
                await rvr.raw_motors(1,255,2,255)
                dist_l =   distance_left()
                await asyncio.sleep(.05)
                print('turning left')
            await rvr.reset_yaw()
        elif dist_front < 35:
            await rvr.drive_with_heading(90,0,0)

try:
    loop.run_until_complete(
        asyncio.gather(
            main()
        )
    )

except KeyboardInterrupt:

    print('Program ended by KeyboardInterrupt')

    GPIO.cleanup()

def distance_left():
    GPIO.output(left_trigger, True)
    time.sleep(0.00001)
    GPIO.output(left_trigger, False)
    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(left_echo) == 0:
        start_time = time.time()

    while GPIO.input(left_echo) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2
    return distance


def distance_right():
    GPIO.output(right_trigger, True)
    time.sleep(0.00001)
    GPIO.output(right_trigger, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(right_echo) == 0:
        start_time = time.time()

    while GPIO.input(right_echo) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time

    distance = (time_elapsed * 34300) / 2
    return distance

def distance_front():
    GPIO.output(front_trigger, True)
    time.sleep(0.00001)
    GPIO.output(front_trigger, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(front_echo) == 0:
        start_time = time.time()

    while GPIO.input(front_echo) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time

    distance = (time_elapsed * 34300) / 2
    return distance