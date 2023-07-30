import cv2
import numpy as np
from pymavlink import mavutil
import time
import argparse
import math
import itertools

#set mode to stabilize
master = mavutil.mavlink_connection('/dev/ttyACM0', baud = 9600)

#make robot move forward til the gate is detected for a while
cap = cv2.VideoCapture(0)
error_check = 0
scale = True
real_height = 0.08 #meters
pixel_height = 0
detect = False
rotate = 0

def travel_in_x(throttle, distance):
    print("moving forward at "+ str(throttle) + " for " + str(distance))
    time.sleep(3)



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

def adjust_angle_pos(angle):
    radians = math.radians(angle)/2
    roll(radians)
    time.sleep(2)
    roll(0)

def adjust_angle_neg(angle):
    radians = math.radians(angle)/2
    roll(-radians)
    time.sleep(2)
    roll(0)
while True:
    #move forward
    if detect == False:
        #set mode to stabilize or alt_hold?
        manualControl(500, 0, 500, 0)
    #detect red rectangles
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([140, 85, 20])
    upper_red = np.array([183, 360, 255])

    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame, frame, mask=mask1)

    kernel = np.ones((2,2),np.uint8)
    mask1 = cv2.erode(mask1, kernel, iterations = 1)
    mask1 = cv2.dilate(mask1, kernel, iterations = 1)

    contours, hierarchy = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    areas = [cv2.contourArea(cnt) for cnt in contours]
    areas = sorted([(area, cnt) for area, cnt in zip(areas, contours) if area > 150], key=lambda x: x[0], reverse=True)[:3]

    height, width = frame.shape[:2]
    centerx = width // 2
    centery = height // 2
    cv2.circle(frame, (centerx, centery), 10, (154, 20, 125), -1)

    min_diff = float('inf')
    area1 = area2 = None
    rect1 = rect2 = None
    for i in range(len(areas)):
        for j in range(i+1, len(areas)):
            larger_area = max(areas[i][0], areas[j][0])
            diff = abs(areas[i][0] - areas[j][0])
            if diff < 0.35 * larger_area:
                if diff < min_diff:
                    min_diff = diff
                    area1, area2 = areas[i][0], areas[j][0]
                    rect1, rect2 = areas[i][1], areas[j][1]

    centroids = []
    for area, cnt in areas:
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        width, height = rect[1]
        pixel_height = height #pixels
        if cnt is rect1 or cnt is rect2:
            error_check+=1
            if error_check==4:
                detect = True
                if rotate == 0: 
                    roll(math.pi)
                    time.sleep(4)
                roll(0)
            
            cv2.drawContours(frame,[box],0,(255,0,0),2)
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centroids.append((cX, cY))
            cv2.circle(frame, (cX, cY), 10, (0, 0, 255), -1)
        else:
            cv2.drawContours(frame,[box],0,(0,255,0),2)
            box = sorted(box, key=lambda x: x[0])
            # Calculate the distances between all four sides
            distances = [((p1, p2), np.linalg.norm(np.array(p1)-np.array(p2))) for p1, p2 in itertools.combinations(box, 2)]

            # Sort the distances and get the two pairs with the smallest distances
            shortest_pairs = sorted(distances, key=lambda x: x[1])[:2]

            # Calculate the midpoints of the shortest sides
            mid_points = [((p1[0]+p2[0])//2, (p1[1]+p2[1])//2) for (p1, p2), _ in shortest_pairs]

            # Calculate the slope of the line
            dy = mid_points[1][1] - mid_points[0][1]
            dx = mid_points[1][0] - mid_points[0][0]
            slope = dy / dx if dx != 0 else float('inf')
            yintercept = mid_points[1][1] - slope * mid_points[1][0]

            # Calculate the points at the edges of the frame
            if slope != float('inf'):
                y1 = int(slope * (0 - mid_points[0][0]) + mid_points[0][1])
                y2 = int(slope * (frame.shape[1] - 1 - mid_points[0][0]) + mid_points[0][1])
                edge_points = [(0, y1), (frame.shape[1] - 1, y2)]
            else:
                edge_points = [(mid_points[0][0], 0), (mid_points[0][0], frame.shape[0] - 1)]
            # Calculate the angle in radians
            angle_rad = math.atan2(-dy, dx)              
            # Convert to degrees
            angle_deg = math.degrees(angle_rad)
            # Adjust the range to be from 0 to 180
            if angle_deg < 0:
                angle_deg += 180
            while True:
                if angle_deg > 112:
                    adjust_angle_pos(abs(angle_deg - 90))
                elif angle_deg < 78:
                    adjust_angle_neg(abs(angle_deg - 90))
                else:
                    break
            angle_str = "Angle: {:.2f} degrees".format(angle_deg)
            # Display the angle on the frame
            cv2.putText(frame, angle_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            print("slope: ", slope)
            #0 = right, 1 = left, 2 = aligned
            direction = None
            if centery > slope*centerx + yintercept and slope < 0:
                print("on RIGHT of the object")
            elif centery > slope*centerx + yintercept and slope > 0:
                print("on LEFT of the object")
            elif centery < slope*centerx + yintercept and slope < 0:
                print("LEFTTTTTTTTT")
            elif centery < slope*centerx + yintercept and slope > 0:
                print("RIGHTTTTTTTTT")
            elif centerx < mid_points[1][0] and centerx < mid_points[0][0]:
                print("LEFT")
            elif centerx > mid_points[1][0] and centerx > mid_points[0][0]:
                print("RIGHT")
            else:
                print("aligned")  

            #Draw the line
            cv2.line(frame, edge_points[0], edge_points[1], (255,0,0), 2)
            


    distance = None
    if len(centroids) == 2:
        cv2.line(frame, centroids[0], centroids[1], (255,105,180), 2)
        distance = math.sqrt((centroids[0][0] - centroids[1][0])**2 + (centroids[0][1] - centroids[1][1])**2)
        if scale != None:
            #coversion needs to be meters per pixel
            scale = real_height / pixel_height
            distance = distance * scale *100 #*100 = centimeters

    print(f"Area 1: {area1} pixels, Area 2: {area2} pixels, Distance: {distance}")

    cv2.imshow('result', frame)
    cv2.imshow('mask', mask1)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release camera and close windows
cap.release()
cv2.destroyAllWindows()

