import threading
import time
import serial.tools.list_ports
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from sphero_sdk import SpheroRvrObserver

class MyPlayer(threading.Thread):

    def __init__(self):

        # initialize the inherited Thread object
        threading.Thread.__init__(self)
        self.daemon = True

        # create a data lock
        self.my_lock = threading.Lock()

        # a variable exclusively used by thread1
        self.t1 = 0

        # a variable exclusively used by thread2
        self.t2 = 0

        # a variable shared by both threads
        self.packet = 0

        # start thread 1
        self.thread1()

    def thread1(self):
        # start the 2nd thread
        # you must start the 2nd thread using the name "start"
        self.start()
        serialInst = serial.Serial('/dev/ttyACM0', 19200, timeout = 5)
        while True:
            self.packet = serialInst.readline()
            # with self.my_lock:
            #     self.t1 += 1
            #     self.g = self.t1 + self.t2
            # print('thread 1  t1:{} t2:{} g1:{}'.format(self.t1, self.t2, self.g))
            # time.sleep(1)

    def run(self):
        """
        This the second thread's executable code. It must be called run.
        """
        rvr = SpheroRvrObserver()
        try:
            rvr.wake()

            # Give RVR time to wake up
            time.sleep(2)
        except KeyboardInterrupt:
                print('\nProgram terminated with keyboard interrupt.')

        rvr.drive_control.reset_heading()
        dist_threshold = 800
        delimeter = ' '
        while True:
            decoded_packet = self.packet.decode('utf')
            data = decoded_packet.split(delimeter)
            print(data)
            dist = (int) (data[2])
            if dist > dist_threshold:
                print("Move Rover Forward")
                try:
                    # rvr.drive_with_heading(
                    #     speed=25, 
                    #     heading=0
                    # )
                    # move forward
                    rvr.drive_control.drive_forward_seconds(
                        speed=25,
                        heading=0,  # Valid heading values are 0-359
                        time_to_drive=1
                    )
                except:
                    print("Unkown exception occurred while driving forward")
        # while True:
        #     with self.my_lock:
        #         self.t2 += 1
        #         self.g = self.t1 + self.t2

        #     print('thread 2  t1:{} t2:{} g:{}'.format(self.t1, self.t2, self.g))
        #     time.sleep(2)


MyPlayer()