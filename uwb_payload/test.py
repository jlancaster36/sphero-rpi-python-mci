import serial
import threading
import multiprocessing
import time
import asyncio
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from sphero_sdk import SpheroRvrObserver, SpheroRvrAsync, SerialAsyncDal
# loop = asyncio.get_event_loop()
# rvr = SpheroRvrAsync(dal=SerialAsyncDal)

# Configure your serial port and baud rate
SERIAL_PORT = '/dev/ttyACM0'  # Adjust this according to your setup
BAUD_RATE = 9600

# Global variable to store received serial data
serial_data = 0

def read_serial():
    global serial_data
    delim = " "
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE) as ser:
            while True:
                # ser.reset_input_buffer()
                # while ser.in_waiting > 50:
                #     _ = ser.readline()
                line = ser.readline().decode('utf-8').split(delim)
                # print(f"Received: {line}")
                print(line)
                serial_data = (int) (line[2])
                # time.sleep(0.5)
    except serial.SerialException as e:
        print(f"Serial exception: {e}")

def process_data():
    global serial_data
    rvr = SpheroRvrObserver()
    try:
        rvr.wake()
        # Give RVR time to wake up
        time.sleep(2)
    except KeyboardInterrupt:
            print('\nProgram terminated with keyboard interrupt.')

    rvr.drive_control.reset_heading()
    dist_threshold = 1000
    while True:
        print(serial_data)
        if serial_data > dist_threshold:
            try:
                print("Move Rover Forward")
                rvr.drive_control.drive_forward_seconds(
                    speed=25,
                    heading=0,  # Valid heading values are 0-359
                    time_to_drive=1
                )
            except:
                print("Unkown exception occurred while driving forward")

def main():
    first = multiprocessing.Process(target=read_serial, args=())
    second = multiprocessing.Process(target=process_data, args=())
    first.start()
    second.start()
    first.join()
    second.join()
    # serial_thread = threading.Thread(target=read_serial)
    # processing_thread = threading.Thread(target=process_data)

    # serial_thread.start()
    # processing_thread.start()

    # serial_thread.join()
    # processing_thread.join()
    # global serial_data
    # delim = " "
    # try:
    #     with serial.Serial(SERIAL_PORT, BAUD_RATE) as ser:
    #         while True:
    #             ser.reset_input_buffer()
    #             # while ser.in_waiting > 50:
    #             #     _ = ser.readline()
    #             line = ser.readline().decode('utf-8').split(delim)
    #             # print(f"Received: {line}")
    #             print(line)
    #             serial_data = (int) (line[2])
    #             # time.sleep(0.5)
    # except serial.SerialException as e:
    #     print(f"Serial exception: {e}")

if __name__ == "__main__":
    main()