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


cap = cv2.VideoCapture(0)

# Load the image
while True:
    _ , frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for red color in HSV
    lower_red = np.array([0, 70, 50])
    upper_red = np.array([10, 255, 255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over the contours
    for contour in contours:
        # If the contour is sufficiently large, it might be a red object
        if cv2.contourArea(contour) > 500:
            # Compute the bounding box for the contour and draw it on the frame
            (x, y, w, h) = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            print("hhahahahahha")

    # Display the image
    cv2.imshow('result', frame)
    cv2.imshow('mask', mask)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release camera and close windows
cap.release()
cv2.destroyAllWindows()
