import serial
import matplotlib.pyplot as plt
import time
import numpy as np


def exponential_moving_average(data, alpha=0.1):
    """
    Apply an exponential moving average filter to the data.
    Args:
        data (numpy array): Input distance data.
        alpha (float): Smoothing factor (0 < alpha <= 1).
    Returns:
        numpy array: Smoothed data.
    """
    ema = np.zeros_like(data)
    ema[0] = data[0]  # Initialize with the first value
    for i in range(1, len(data)):
        ema[i] = alpha * data[i] + (1 - alpha) * ema[i - 1]
    return ema


ser9 = serial.Serial('COM9', 9600)
ser3 = serial.Serial('COM3', 9600)

window = 50

data_x = []
data_ser9 = []
data_ser3 = []
prev_ema_ser9 = 0
curr_ema_ser9 = 0
prev_ema_ser3 = 0
curr_ema_ser3 = 0
alpha = 0.3

turn_threshold = 120

while True:

    if ser3.in_waiting > 5:
        packet3 = ser3.read(ser3.in_waiting)
        data3 = packet3.decode('utf-8').split(' ')[-1]

        if data3 != 'occured' and data3 != 'occured\r\n':
            value3 = (float)(data3.split(":")[-1])

            if abs(value3) < 10000:

                if prev_ema_ser3 == 0:
                    curr_ema_ser3 = value3
                    prev_ema_ser3 = curr_ema_ser3
                else:
                    curr_ema_ser3 = alpha * value3 + \
                        (1 - alpha) * prev_ema_ser3
                    prev_ema_ser3 = curr_ema_ser3

            # print(curr_ema_ser3)

    if ser9.in_waiting > 5:
        packet9 = ser9.read(ser9.in_waiting)
        data9 = packet9.decode('utf-8').split(' ')[-1]

        if data9 != 'occured' and data9 != 'occured\r\n':
            value9 = (float)(data9.split(":")[-1])

            if abs(value9) < 10000:

                if prev_ema_ser9 == 0:
                    curr_ema_ser9 = value9
                    prev_ema_ser9 = curr_ema_ser9
                else:
                    curr_ema_ser9 = alpha * value9 + \
                        (1 - alpha) * prev_ema_ser9
                    prev_ema_ser9 = curr_ema_ser9

        # print(curr_ema_ser9)

    diff = (curr_ema_ser3-curr_ema_ser9)+70
    # print("Diff: ", diff)

    if abs(diff) < turn_threshold:
        print("Drive Forward")
    elif diff > turn_threshold:
        print("Drive Left")
    elif diff < -turn_threshold:
        print("Drive Right")

    # data_x.append(time.time())
    # plt.plot(data_x, ema_ser9)
    # plt.pause(0.2)
    # plt.plot(data_x, ema_ser3)
    # plt.pause(0.2)
