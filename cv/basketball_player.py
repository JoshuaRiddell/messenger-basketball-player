import cv2 as cv
import numpy as np
import sys
from math import acos, asin, cos, pi
from time import sleep
from os.path import join
import serial
import time


# define thresholds
BALL = {
    'thresh': [np.array([0, 45, 15]), np.array([21, 255, 255])],
    'area': [20000, 25000],
    'loc_x': [600, 900],
    }
BASKET = {
    'thresh': [np.array([0, 30, 83]), np.array([216, 255, 255])],
    'area': [2015, 3914],
    'loc_x': [150, 450],
    }

BLOB_DATA = [BALL, BASKET]

# crop bounds
CROP = [[543, 257], [1413, 808]]
DIMENSIONS = (CROP[1][0] - CROP[0][0], CROP[1][1] - CROP[0][1])

# blur kernel
GAUS_KERNEL = np.ones((3, 3), np.float32)/15

# robot home position
HOME = [0, 55]

# aim time for the shoot
SHOOT_DELAY = 0.2
INFER_DELAY = 1.50

# tick frequency (used for timing between frames to get velocities in px/sec)
FREQUENCY = cv.getTickFrequency()

# kernel for morphological operations performed on the binary images
MORPH_KERNEL = np.ones((4, 4),np.uint8)

COM_PORT = "COM3"
BAUD = 115200
TIMEOUT = 0.1

SERVO_CALIB = [
    [93, 0.8556],
    [80, 0.8556],
    [80, 20],
]

LOG_VIDEO = True
LOG_DIRECTORY = "D:\\basketball_logs"

VIDEO_ENCODER = cv.VideoWriter_fourcc(*'XVID')
VID_EXTENSION = ".avi"
VID_FPS = 25.0

def end():
    """ End the program """
    cam.release()
    video_out.release()
    cv.destroyAllWindows()
    ser.close()
    sys.exit()

def write_servo(ser_obj, pin, val):
    ser_obj.write(chr(pin))
    ser_obj.write(chr(val))

def send_move(pixel_coord, absolute=False):
    if not absolute:
        coord = [0, 0]
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

def pen(activate):
    if activate:
        write_servo(ser, 2, 22)
        pass
    else:
        write_servo(ser, 2, 80)
        pass
    sleep(0.001)

cam = cv.VideoCapture(0)

cam.set(3, 1920)
cam.set(4, 1080)

prev_pos = [0, 0]
prev_tick = cv.getTickCount()

# vector of previous velocities for running average
vel_ave = [np.zeros(40, np.float32), np.zeros(40, np.float32)]
# bounds of the basket movement
bounds_ave = [
    [np.zeros(3, np.float32), np.zeros(3, np.float32)],
    [np.zeros(3, np.float32), np.zeros(3, np.float32)]
    ]
bounds = [[0, 0], [0, 0]]

# open serial port to the Arduino
ser = serial.Serial(COM_PORT, BAUD, timeout=TIMEOUT)
# wait for port to settle
sleep(2)

# move to home position and deactivate pen
send_move(HOME, absolute=True)
pen(0)

# number of frames since the last shot. Allows for some settling since last shot
shot = 0

# current (and previous frame) sign of the velocity vector
sign = [0, 0]
prev_sign = [0, 0]

# predicted location of basket after shooting
pred_loc = [0, 0]

# true when frames are recorded
record = False
video_out = cv.VideoWriter('.avi', VIDEO_ENCODER, VID_FPS, DIMENSIONS)

while(True):
    # quit if q is pressed
    if cv.waitKey(1) & 0xFF == ord('q'):
        end()

    # get frame and crop
    ret, frame = cam.read()
    frame = frame[CROP[0][1]:CROP[1][1], CROP[0][0]:CROP[1][0]]

    # make copy for imshowing at the end
    original = frame.copy()

    # apply blur
    frame = cv.filter2D(frame, -1, GAUS_KERNEL)

    # convert to hsv
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # make binary image of ball and basket
    ball_bin = cv.inRange(hsv, BALL['thresh'][0], BALL['thresh'][1])
    basket_bin = cv.inRange(hsv, BASKET['thresh'][0], BASKET['thresh'][1])

    # copy images to keep originals for imshowing at the end
    # ball_bin_original = ball_bin.copy()
    # basket_bin_original = basket_bin.copy()

    # process binary images to retrieve contours of points of interest
    binaries = [ball_bin, basket_bin]
    poi_coords = []
    # contours = []
    for i in range(len(binaries)):
        # filter noise
        binaries[i] = cv.erode(binaries[i], MORPH_KERNEL, iterations=1)
        binaries[i] = cv.dilate(binaries[i], MORPH_KERNEL, iterations=3)

        # find contours
        _, new_contours, heirarchy = cv.findContours(binaries[i].copy(),
            cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        coords = []
        # filter contours
        # filtered_contours = []
        for contour in new_contours:
            area = cv.contourArea(contour)
            if BLOB_DATA[i]['area'][0] < area < BLOB_DATA[i]['area'][1]:
                # filtered_contours.append(contour)
                moments = cv.moments(contour)
                centroid_x = int(moments['m10']/moments['m00'])
                centroid_y = int(moments['m01']/moments['m00'])
                if BLOB_DATA[i]['loc_x'][0] < centroid_x < BLOB_DATA[i]['loc_x'][1]:
                    coords = [centroid_x, centroid_y]
        # contours.append(filtered_contours)
        if coords == []:
            continue
        poi_coords.append(coords)

    # make contours frame to display contours
    # contours_frame = original.copy()
    # _ = cv.drawContours(contours_frame, contours[0], -1, (255,0,0), 3)
    # _ = cv.drawContours(contours_frame, contours[1], -1, (0,255,0), 3)

    if len(poi_coords) != 2:
        print("lost")
        cv.imshow('points', frame)
        if record:
            print("recording")
            video_out.write(original)
        # cv.imshow('contour', contours_frame)
        continue

    # extract ball and basket coords
    ball_coord = poi_coords[0]
    basket_coord = poi_coords[1]

    # get delta time
    delta_time = (cv.getTickCount() - prev_tick) / FREQUENCY
    # calc instantaneous basket velocity vector
    vel = [(x - y) / float(delta_time) for x, y in zip(prev_pos, basket_coord)]

    # add the calculated velocities to the average list
    for i in range(len(vel_ave)):
        vel_ave[i] = np.concatenate(([abs(vel[i])], vel_ave[i][:-1]))

    # predict location of basket after throwing
    for i in range(len(pred_loc)):
        pred_loc[i] = basket_coord[i] - int(INFER_DELAY * sign[i] * np.average(vel_ave[i]))

    # detect and store movement bounds
    for i in range(len(vel)):
        if vel[i] != 0:
            prev_sign[i] = sign[i]
            sign[i] = cmp(vel[i], 0)

            if prev_sign[i] != sign[i] and shot > 20:
                j = 0
                if prev_sign[i] > 0:
                    j = 1
                bounds_ave[i][j] = np.concatenate(([basket_coord[i]], bounds_ave[i][j][:-1]))
                bounds[i][j] = np.average(bounds_ave[i][j])
                # print(bounds_ave)

    # detect if bounds have been reached
    for i in range(len(bounds)):
        if pred_loc[i] > bounds[i][0] and np.average(vel_ave[i]) > 5:
            pred_loc[i] = int(2 * bounds[i][0] - pred_loc[i])
        elif pred_loc[i] < bounds[i][1] and np.average(vel_ave[i]) > 5:
            pred_loc[i] = int(2 * bounds[i][1] - pred_loc[i])

    prev_tick = cv.getTickCount()
    prev_pos = basket_coord[:]

    points_frame = original.copy()
    points_frame = cv.circle(points_frame, tuple(poi_coords[0]), 20, (0,0,255), -1)
    points_frame = cv.circle(points_frame, tuple(poi_coords[1]), 20, (0,255,0), -1)
    points_frame = cv.circle(points_frame, tuple(pred_loc), 20, (255,0,0), -1)

    points_frame = cv.circle(points_frame, tuple(ball_coord), 20, (0,255,255), -1)

    shot += 1

    # print(str(np.std(vel_ave)) + " " + str(ball_coord[1]) + " " + str(pred_loc[1]))

    if pred_loc[0] < 310 and (np.average(vel_ave[1]) < 5 or (np.std(vel_ave[0]) < 30 and np.std(vel_ave[1]) < 45 and ball_coord[1]-5 < pred_loc[1] < ball_coord[1]+5 and np.std(bounds_ave[0][0]) < 20 and np.std(bounds_ave[0][1]) < 20 and np.std(bounds_ave[1][0]) < 20 and np.std(bounds_ave[1][1]) < 20)) and shot > 20:
        print("shoot {0}, {1}".format(ball_coord, pred_loc))

        m = (pred_loc[1] - ball_coord[1]) / float(pred_loc[0] - ball_coord[0])
        c = ball_coord[1] - m * ball_coord[0]

        MAX_PIX = 340

        increments = 20
        time_step = SHOOT_DELAY / float(increments)
        x_step = (MAX_PIX - ball_coord[0]) / float(increments)
        x = ball_coord[0]

        send_move(ball_coord)
        pen(1)
        sleep(0.6)
        for i in range(increments):
            if i == int(increments * 0.60):
                pen(0)
            send_move([x, m * x + c])
            x += x_step
            sleep(time_step)
        pen(0)
        send_move(HOME, absolute=True)

        # record logs
        log_name = str(int(time.time()))
        video_out.release()
        video_out = cv.VideoWriter(join(LOG_DIRECTORY, log_name + VID_EXTENSION), VIDEO_ENCODER, VID_FPS, DIMENSIONS)
        record = True

        shot = 0

    # show images
    # cv.imshow('ball', ball_bin)
    # cv.imshow('basket', basket_bin)
    # cv.imshow('frame', frame)
    cv.imshow('points', points_frame)
    # cv.imshow('contour', contours_frame)