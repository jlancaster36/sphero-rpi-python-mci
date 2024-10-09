import serial.tools.list_ports
import time
from sphero_sdk import SpheroRvrObserver

ports = serial.tools.list_ports.comports()

rvr = SpheroRvrObserver()

try:
    rvr.wake()

    # Give RVR time to wake up
    time.sleep(2)
except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

rvr.drive_control.reset_heading()

print(ports)
serialInst = serial.Serial()

portList = []
for port in ports:
    portList.append(str(port))
    print(str(port))

val = input("Select a port COM:")

for x in range(0, len(portList)):
    if portList[x].startswith("COM" + str(val)):
        portVar = "COM"+str(val)
        print(portList[x])

serialInst.baudrate = 9600
serialInst.port = portVar
serialInst.open()

dist_threshold = 1000
delimeter = ','

while True:
    if serialInst.in_waiting:
        packet = serialInst.readline()
        decoded_packet = packet.decode('utf')
        data = decoded_packet.split(delimeter)
        dist = (int) (data[4])
        print(dist)
        if dist > dist_threshold:
            print("Move Rover Forward")
            try:
                # move forward
                rvr.drive_control.drive_forward_seconds(
                    speed=25,
                    heading=0,  # Valid heading values are 0-359
                    time_to_drive=1
                )
            except:
                print("Unkown exception occurred while driving forward")
