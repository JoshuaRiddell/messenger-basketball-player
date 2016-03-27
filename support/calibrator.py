import cv2 as cv
import numpy as np
import sys
from math import acos, asin, cos, pi
from time import sleep
import serial

CROP = [[543, 257], [1413, 808]]

COM_PORT = "COM3"
BAUD = 115200
TIMEOUT = 0.1

HOME = [0, 65]

last = (0, 0)
def callback(event, x, y, flags, usr):
    global last
    if event == 3:
    	pen(2)
    if flags == 1:
    	a = send_move([x,y])
    	# print("pix_coord " + str((x, y)))
    	# print("coord " + str(a))
    if event == 2:
        print(str((x, y)) + " = " + str(send_move(last)))

def write_servo(ser_obj, pin, val):
    ser_obj.write(chr(pin))
    ser_obj.write(chr(val))

def send_move(pixel_coord, absolute=False):
    # bottom_left = [-26.9, 66.8] -- (813, 86.8) ==== [698,96] [823,86]
    # top_right =   [22.1, 69.8] --   (821, 49) ==== [496,-29] [56,19]
    # top_right = [27, 100] -- [720, 0] ==== [0,27] [551,-20]

    global last

    if not absolute:
        last = pixel_coord[:]

    if not absolute:
        coord = [0, 0]
        # coord[0] = 29.33 - 7 * pixel_coord[1] / 62
        # coord[1] = 172.7 - 30 * pixel_coord[0] / 293
        # coord[0] = 27 - 3 * pixel_coord[1] / 28
        # coord[1] = 172 - pixel_coord[0] / 11
        # coord[0] = 25.1 - 6 * pixel_coord[1] / 55
        # coord[1] = 151.8 - 2 * pixel_coord[0] / 25

        coord[0] = 23.91 - 0.001115 * pixel_coord[0] - 0.1089 * pixel_coord[1]
        coord[1] = 165 - 0.1177 * pixel_coord[0] - 0.006929 * pixel_coord[1]
    else:
        coord = pixel_coord

    theta = [0, 0]

    theta[1] = asin(coord[0] / float(42))
    a = coord[1] - 42 * cos(theta[1])
    theta[0] = asin((50 - a) / 38)
    theta[1] *= -1

    theta = [x * 180 / pi for x in theta]

    for i, angle in enumerate(theta):
        calib = SERVO_CALIB[i]
        write_servo(ser, i, int(calib[0] + calib[1] * angle))
        sleep(0.001)

    return coord

current = True
def pen(activate):
    global current
    if activate == 2:
        current = activate = not current
    if activate:
        send_move(last)
        sleep(0.1)
        write_servo(ser, 2, 50)
        sleep(0.2)
        write_servo(ser, 2, 22)
        pass
    else:
        write_servo(ser, 2, 120)
        send_move(HOME, absolute=True)
        pass
    sleep(0.001)

cam = cv.VideoCapture(0)

cam.set(3, 1920)
cam.set(4, 1080)

SERVO_CALIB = [
    [93, 0.8556],
    [80, 0.8556],
    [80, 20],
]

ser = serial.Serial(COM_PORT, BAUD, timeout=TIMEOUT)
sleep(2)

cv.imshow('frame', cam.read()[1][CROP[0][1]:CROP[1][1], CROP[0][0]:CROP[1][0]])
cv.setMouseCallback('frame', callback)

send_move(HOME, absolute=True)
pen(0)

while(True):
    # quit if q is pressed
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

    cv.imshow('frame', cam.read()[1][CROP[0][1]:CROP[1][1], CROP[0][0]:CROP[1][0]])
