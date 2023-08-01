import cv2
import numpy as np
import cv2
import numpy as np
from pymavlink import mavutil
import time
import argparse
import math
import itertools
count = 0
scale = True
real_height = 0.08 #meters
pixel_height = 0

master = mavutil.mavlink_connection('udp:192.168.2.1:14550')

#ensure connection is valid
master.wait_heartbeat()
print("Hi")

#make a function 
#box[0] = (x1, y1) =coord1
#box[1] = (x2, y2) = coord2
#box[0][0] = x1 = coord1[0]
#box[0][1] = y1 = coord1[1]
#box[1][0] = x2 = coord2[0]
#box[1][1] = y2 = coord2[1]

def set_mode(modep):
    #set the desired operating mode
    #why STABILIZE?
    #https://ardupilot.org/dev/docs/apmcopter-programming-advanced.html
    mode = modep
    # get the mode id from the mode_mapping dictionary
    # The code master.mode_mapping()[mode] is accessing a dictionary of mode mappings 
    # stored in the "master" object and returning the value associated with the key mode.
    #https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
    mode_id = master.mode_mapping()[mode]
    #set the mode
    #https://mavlink.io/en/messages/common.html#SET_MODE
    master.mav.set_mode_send(
        master.target_system,#target system
        #https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,#custom_mode is the mode we are setting
        mode_id)#mode_id is the mode we are setting it to (previous code)



def roll(roll_rate):
    mode = 'ACRO'
    # get the mode id from the mode_mapping dictionary
    # The code master.mode_mapping()[mode] is accessing a dictionary of mode mappings 
    # stored in the "master" object and returning the value associated with the key mode.
    #https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
    mode_id = master.mode_mapping()[mode]
    #set the mode
    #https://mavlink.io/en/messages/common.html#SET_MODE
    master.mav.set_mode_send(
        master.target_system,#target system
        #https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,#custom_mode is the mode we are setting
        mode_id)#mode_id is the mode we are setting it to (previous code)
    
    master.mav.set_attitude_target_send(
        time.time(),
        1,
        1,
        0b00000001,
        [0, 0, 0, 0],
        roll_rate,
        0,
        0,
        0
    )
    set_mode("ALT_HOLD")
    rotate = 1



cap = cv2.VideoCapture(0)

# Load the image
while True:
    _ , frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for red color in HSV
    lower_red = np.array([25, 70, 50])
    upper_red = np.array([35, 255, 255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over the contours
    for contour in contours:
        # If the contour is sufficiently large, it might be a red object
        if cv2.contourArea(contour) > 400:
            # Compute the bounding box for the contour and draw it on the frame
            (x, y, w, h) = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            print("hhahahahahha")
            roll(math.pi/2)
        else:
            roll(0)
# Release camera and close windows
cap.release()
cv2.destroyAllWindows()
