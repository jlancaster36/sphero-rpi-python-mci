import serial
import threading
import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from sphero_sdk import SpheroRvrObserver

# Configure your serial port and baud rate
SERIAL_PORT = '/dev/ttyACM0'  # Adjust this according to your setup
BAUD_RATE = 9600

# Global variable to store received serial data
serial_data = 0
data_lock = threading.Lock()

def read_serial():
    global serial_data
    delim = " "
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            while True:
                line = ser.readline().decode('utf-8').split(delim)
                print(f"Received: {line}")
                with data_lock:
                    serial_data = int(line[2])
                time.sleep(0.1)  # Adjust as needed for your application
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
        with data_lock:
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
        time.sleep(0.1)

def main():
    serial_thread = threading.Thread(target=read_serial)
    processing_thread = threading.Thread(target=process_data)

    serial_thread.start()
    processing_thread.start()

    serial_thread.join()
    processing_thread.join()

if __name__ == "__main__":
    main()
