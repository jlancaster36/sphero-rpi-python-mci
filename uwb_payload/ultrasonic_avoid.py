import sys
import os
sys.path.append(os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../')))
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
GPIO.setup(front_trigger, GPIO.OUT)
GPIO.setup(front_echo, GPIO.IN)

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

async def main():
    await rvr.wake()
    await rvr.reset_yaw()
    await asyncio.sleep(.5)
    state = 0
    while True:
        dist_r =  distance_right()
        dist_l =  distance_left()
        dist_front = distance_front()
        heading = 0
        await asyncio.sleep(.05)
        print(f"Left: {dist_l} | Right: {dist_r} | Front: {dist_front}")
        match state:
            case 0:
                #Drive forward until front facing obstacle is found
                if (dist_front < 35):
                    print("obstrution ahead")
                    if (dist_r < dist_l):
                        print("turning left to avoid obstacle")
                        #turn(-90), make function to turn 90 degrees right/left
                        state = 1
                    else:
                        print("turning right to avoid obstacle")
                        #turn(90), make function to turn 90 degrees right/left
                        state = 2
                else:
                    # No obstruction in front, drive normally
                    await rvr.drive_with_heading(20,heading,0)       
            case 1:
                #have turned left to avoid obstacle, drive forward until right sensor shows past obstacle
                print("avoiding obstacle left")
                if (dist_r > 60):
                    print("past obstacle, turning back towards goal")
                    #turn(90)
                    state = 0
                else:
                    #await rvr.raw_motors(1,20,1,20)
                    l_heading = (heading + 270) % 360
                    await rvr.drive_with_heading(20,l_heading,0)  
            case 2:
                #have turned right to avoid obstacle, drive forward until left sensor shows past obstacle
                print("avoiding obstacle right")
                if (dist_l > 60):
                    print("past obstacle, turning back towards goal")
                    #turn(-90)
                    state = 0
                else:
                    #await rvr.raw_motors(1,20,1,20)
                    r_heading = (heading + 90) % 360
                    await rvr.drive_with_heading(20,r_heading,0)
            case _:
                #shouldn't reach
                print("error reached invalid state")
try:
    loop.run_until_complete(
        asyncio.gather(
            main()
        )
    )

except KeyboardInterrupt:

    print('Program ended by KeyboardInterrupt')

    GPIO.cleanup()

