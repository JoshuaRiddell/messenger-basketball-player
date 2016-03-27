from os.path import join
import numpy as np
import cv2

CROP = [[543, 257], [1413, 808]]

cap = cv2.VideoCapture(0)

cap.set(3, 1920)
cap.set(4, 1080)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
print "codec"
out = cv2.VideoWriter('D:\\basketball_logs\\video.avi', fourcc, 30.0, (870, 551))
print((CROP[1][0] - CROP[0][0], CROP[1][1] - CROP[0][1]))
frame = cap.read()[1]

while(True):
    ret, frame = cap.read()
    frame = frame[CROP[0][1]:CROP[1][1], CROP[0][0]:CROP[1][0]]
    if ret==True:
        # write the flipped frame
        out.write(frame)
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()