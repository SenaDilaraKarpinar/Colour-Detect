#!/usr/bin/env python
import cv2
from cv2 import threshold
from cv2 import COLOR_BGR2GRAY
from cv2 import Canny
from matplotlib.pyplot import gray
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import imutils

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    height, width, _ = frame.shape

    cx = int(width/2)
    cy = int(height/2)

    ##pick pixel value
    pixel_center = hsv_frame[cx,cy]
    hue_value = pixel_center[0]

    colour = "UNDEFINED"
    if (0<hue_value<15):
        colour = "RED"
    elif (25<hue_value<30):
        colour = "YELLOW"
    elif (45<hue_value<75):
        colour = "GREEN"
    elif (100<hue_value<140):
        colour = "BLUE"

    pixel_center_bgr = frame[cx,cy]
    b, g, r = int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])
    cv2.putText(frame, colour, (10,70),0,1.5,(b,g,r),2)

    cv2.circle(frame,(cx,cy),5,(25,25,25),3)

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
