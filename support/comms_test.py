from math import acos, asin, cos, pi
from time import sleep
import serial

LENGTH1 = 85
LENGTH2 = 85

COM_PORT = "COM3"
BAUD = 115200
TIMEOUT = 0.1

SERVO_CALIB = [
	[93, 0.8556],
	[80, 0.8556],
	[80, 20],
]

ser = serial.Serial(COM_PORT, BAUD, timeout=TIMEOUT)

def write_servo(ser_obj, pin, val):
	ser_obj.write(chr(pin)); ser_obj.write(chr(val))

sleep(2)

coord = [0, 110]

write_servo(ser, 2, 20)

while True:
	usr_in = raw_input(">> ")
	if usr_in == "quit":
		break

	coord = [int(x.strip()) for x in usr_in.split(".")]

	theta = [0, 0]

	theta[1] = asin(coord[0] / float(42))
	coord[1] -= 42 * cos(theta[1])
	theta[0] = asin((50 - coord[1]) / 38)
	theta[1] *= -1

	theta = [x * 180 / pi for x in theta]

	for i, angle in enumerate(theta):
		calib = SERVO_CALIB[i]
		write_servo(ser, i, int(calib[0] + calib[1] * angle))

ser.close()