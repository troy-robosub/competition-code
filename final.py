import cv2
import numpy as np
from pymavlink import mavutil
import time
import argparse
import math
import itertools
#############################################################################################################################################################################
# Create the connection
master = mavutil.mavlink_connection('udp:192.168.2.1:14550')

#ensure connection is valid
master.wait_heartbeat()
print("Hi")

def check_direction():
    if centery > slope*centerx + yintercept and slope < 0:   
        direction = 1
        print("right")
    elif centery > slope*centerx + yintercept and slope > 0:
        direction = 0
        print("left")
    elif centery < slope*centerx + yintercept and slope < 0:
        direction = 0
        print("left")
    elif centery < slope*centerx + yintercept and slope > 0:
        direction = 1
        print("right")
    elif centerx < mid_points[1][0] and centerx < mid_points[0][0]:
        direction = 0
        print("left")
    elif centerx > mid_points[1][0] and centerx > mid_points[0][0]:
        direction = 1
        print("right")
    else:
        direction = 2
        print("aligned")
    return direction
def move_right():
    manualControl(0,500,0,0)

def move_left():
    manualControl(0,-500,0,0)

#Description of pitch,roll, yaw

################################################################################################################################################
#                                |                                                  |                                                          #
#                                |                                                  |                         0 yaw                            #
#                                |                                                  |                           ^                              #
#                                |                       0 roll                     |    |\                     |                    /\        #
#         \                      |         |\             ^             /|          |      \  -yaw              |           +yaw    /          #
#         \\\     +pitch         |           \            |            /            |       \                   |                  /           #
#           \                    |  +roll     \           |           /    -roll    |       ____--\        |---------|        /--____          #
#            \    ____________   |             \  (-----\----/----)  /              |    \--       \       |         |       /       --/       #
#             \  |            |  |              \(-------\--/------)/               |     \         \      |         |      /         /        #
#  <===========  |   0 pitch  |  |              (---------\/--------)               |      \         \     |         |     /         /         #
#             /  |____________|  |              (---------/\--------)               |       \         \    |         |    /         /          #
#            /                   |               (-------/--\------)                |        \         \   |         |   /         /           #
#           /                    |                (-----/----\----)                 |         \         \  |         |  /         /            #
#         ///    -pitch          |                                                  |          \   ____--\ |_________| /--____   /             #
#         /                      |                                                  |           \--                           --/              #
#                                |                                                  |                                                          #
#                                |                                                  |                                                          #
#            (SIDE VIEW)         |                   (BACK VIEW)                    |                       (TOP VIEW)                         #
################################################################################################################################################

#                        STABILIZE
##############################################################
#    Stabilize mode allows you to fly the sub manually,      #
#  but it will automatically level the roll and pitch axis   #
##############################################################

'''
MANUAL: This mode allows the sub to be flown manually, but it will automatically level the roll and pitch axis.
STABILIZE: This mode allows the sub to be flown manually, but it will automatically level the roll and pitch axis.
DEPTH HOLD: This mode maintains the sub's depth at a constant value.
ALTITUDE HOLD: This mode maintains the sub's altitude at a constant value.
POSITION HOLD: This mode maintains the sub's position at a constant value.
AUTO: This mode allows the sub to follow a pre-programmed mission plan.
MISSION: This mode allows the sub to follow a pre-programmed mission plan.
LOITER: This mode maintains the sub's position and altitude at a constant value.
RETURN TO LAUNCH: This mode returns the sub to its launch location and lands it.
LAND: This mode lands the sub at its current location.
TAKEOFF: This mode takes off from the sub's current location.
FOLLOW: This mode follows a target.
OFFBOARD: This mode allows the sub to be controlled by an external computer.
ACRO: This mode allows the sub to be flown in acrobatic mode.
SURFACE: This mode brings the sub to the surface.
FLIP: This mode flips the sub.
FLOWHOLD: This mode maintains the sub's position using optical flow.
DRIFT: This mode allows the sub to drift with the current.
SPORT: This mode allows the sub to be flown in sport mode.
THROW: This mode throws the sub.
AUTOTUNE: This mode tunes the sub's PID parameters automatically.
AVOID_ADSB: This mode avoids collisions with other aircraft using ADS-B.
GUIDED: This mode allows the sub to be guided to a specific location.
INITIALISING: This mode is used during initialization.
QSTABILIZE: This mode allows a quadrotor to be flown in stabilize mode.
QHOVER: This mode allows a quadrotor to hover.
QLOITER: This mode allows a quadrotor to loiter.
QLAND: This mode allows a quadrotor to land.
QRTL: This mode allows a quadrotor to return to launch.
QAUTOTUNE: This mode tunes a quadrotor's PID parameters automatically.
QACRO: This mode allows a quadrotor to be flown in acrobatic mode.
THERMAL: This mode allows the sub to follow a thermal.
SHUT_DOWN: This mode shuts down the sub.
REBOOT: This mode reboots the sub.
SENSORS: This mode tests the sub's sensors.
'''

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


#Function to provide Manual Controls. For example sendng this with a certain set of numbers will simulate moving the joystick forward.
def manualControl(x, y, z, r):

    # master.mav.<command>(args) is the syntax to communivate with pixhawk
    # manual_control_send uses the mavlink message MANUAL_CONTROL. The _send is used to indicate that the message is being sent
    # messages are meant to be sent, so most messages will be in the format PIXHAWK_MESSAGE_send. recv is not a suffix.
    
    master.mav.manual_control_send(

        #this means the pixhawk you are connected to (sets target system to "master")
        master.target_system,


        x,  # pitch for sub, back/forward on joystick [-1000,1000], respectively
        y,  # roll for sub, left/right on joystick [-1000,1000], respectively
        z,  # thrust for sub, slider on joystick [0,1000]
        r,  # yaw for sub, clockwise/counterclockwise on joystick [-1000,1000], respectively
        0)  # buttons


#Function
def getDepth():
    #set initial depth variable = 0
    depth = 0
    #infinite loop
    while True:
        #recv_match() is for capturing messages with particular names or field values
        #without arguments, it will just look for the next message without specifications
        #so msg is the variable used to store messages
        msg = master.recv_match()
        #if there is not a message,
        if not msg:
            # continue to the next iteration of loop, which means check if there is a new message
            continue
        #If the message type in message is VFR_HUD,
        #https://mavlink.io/en/messages/common.html#VFR_HUD
        #Basically if the message is a metric typically displayed on a HUD for the sub,
        if msg.get_type() == 'VFR_HUD':
            #set a variable data to the string version of the msg
            data = str(msg)
            #data will come as smth like 11:12:13:14:15:16:17:18:19:20
            try:
                #data will be split into a list, so [11,12,13,14,15,16,17,18,19,20]
                data = data.split(":")
                #6th item of list will be split by commas so.. [1,6] and then the first item will be returned so 1
                #depth = 1 in this example  
                depth = data[5].split(",")[0]
            #when depth can't be split into all of the above, returns as empty
            except:
                print('')
            #print the depth out
            print("Current Depth: ", depth)
        #as soon as the depth is detected, and isn't 0 (which is what the fucntion sets it to)
        if not depth == 0:
            #break the infinite loop
            break
    #return the detected depth
    return float(depth)

# very similar to getDepth
# only change is the data split, which means the messages are being in sent in such a way that it is like
# value,thing1 : value,thing2 : value,thing3 :  ... 
#so like data[1] after the split with : would return value,thing2
# which would then be split again and only return value
def get_velocity():
    velocity = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                speed = data[2].split(",")[0]
            except:
                print('')

            velocity = float(speed)
        if not velocity == 0:
            break

    return velocity

#similar to getDepth and getVelocity
def get_heading():
    heading = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                heading = data[3].split(",")[0]
            except:
                print('')

            heading = float(heading)
            break

    return heading


#Function
def goDepth(depth):
    #set the desired operating mode
    #why STABILIZE?
    #https://ardupilot.org/dev/docs/apmcopter-programming-advanced.html
    mode = 'STABILIZE'
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
    
    #set the depth you are currently at
    current_depth = abs(getDepth())

    #if the current depth is greater than the desired depth
    if current_depth > depth:
        #run this 1000000 times (because manual control is one small shift at a time)
        for i in range(1000000):
            #set thrust to 700 (upward)
            manualControl(0,0,700, 0)
            #get the current depth at each small shift
            current_depth = abs(getDepth())
            #if the current depth is less than the desired depth (within 5%)
            if current_depth < depth *0.95:
                #break the loop and stop
                break
        #print the depth reached
        print("REACHED DESIRED Depth: ", getDepth())

        #get mode to DEPTH_HOLD
        mode = 'DEPTH_HOLD'
        #get the mode id
        mode_id = master.mode_mapping()[mode]
        #set the mode via message
        master.mav.set_mode_send(
            master.target_system,#target
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,#set mode to the one specified
            mode_id) #mode_id that we want to set

    #if the current depth is less than the desired depth
    else:
        for i in range(1000000):
            #set thrust to 300 (downward) *wait why is it 300? and not -300? (diff for z)
            manualControl(0,0,300, 0)
            #rest is self-explanatory
            current_depth = abs(getDepth())

            if current_depth > depth *0.95:
                break
        print("REACHED DESIRED Depth: ", getDepth())
        #self-explanatory
        mode = 'DEPTH_HOLD'
        mode_id = master.mode_mapping()[mode]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
    


    
    

#Function
def travel_in_x(xThrottle, distanceTravel):
    #set mode to STABILIZE (prevents too much external movement) KEEPS PITCH AND ROLL STABLE SO WHEN CHANGING XTHROTTLE, MOVES FORWARD!! 
    #would probably work for roll as well  (vertical movement)
    #self-explanatory
    mode = 'STABILIZE'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    #print mode
    print("<<<<<<MODE CHANGED TO ", mode, ">>>>>>")
    #start timer
    start = time.time()
    #create an array to store velocity values
    velocity_array = []
    #distance is initially 0
    distance = 0
    for i in range(10000000):
        #set pitch to xThrottle (forward), no thrust because xthrottle is forward due to STABILIZE (no changes in pitch)
        #notice no iteration
        manualControl(xThrottle, 0, 500, 0)
        #get the elapsed time
        elapsed = time.time() - start
        #get the velocity at each iteration
        velocity_array.append(get_velocity())
        #get the average velocity
        average_velocity = sum(velocity_array) / len(velocity_array)
        #calculate the distance traveled
        distance = elapsed * average_velocity
        #print the distance
        print("RECORDED DISTANCE: ", distance)
        #if the distance traveled is greater than the desired distance(within 5%)
        if distance > 0.95*distanceTravel:
            break
    #print the distance reached
    print("REACHED DESIRED DISTANCE: ", distance)
    #self-explanatory, hold altitude
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

#Function
def rotateClockwise(degrees):
    #hold altitude and send message
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    #the original heading of the vehicle.
    #what is heading? the compass direction in which the craft's nose is pointing
    #https://mavlink.io/en/messages/common.html#ATTITUDE
    start_heading = get_heading()
    #run this 10000000 times
    for i in range(10000000):
        #get current heading, usually after small shift in yaw
        current_heading = get_heading()
        #calculate the difference in rotation by degrees
        rotation = abs(start_heading-current_heading)
        #rotate clockwise, no thrust
        manualControl(0, 0, 500, 250)
        #if the desired degrees rotated
        # is greater than the desired rotation (within 4%), stop
        if rotation > 0.96 * degrees:
            break
    #print the rotation reached
    print("ROTATED: ", rotation)
    #hold altitude
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

#similar to rotateClockwise but -250 instead of 250 (yaw)
def rotateCounterClockwise(degrees):
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    start_heading = get_heading()
    for i in range(10000000):
        current_heading = get_heading()
        print("Current heading: " , current_heading)
        rotation = abs(start_heading-current_heading)
        manualControl(0, 0, 500, -250)
        if rotation > 0.96 * degrees:
            break

    print("ROTATED: ", rotation)
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

# turn to and maintain heading that you give
def maintainHeading(heading):
    #hold altitude
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    #get the current heading
    start_heading = get_heading()
    #calculate the difference in headings
    angle = start_heading - heading
    #if the angle is negative, rotate clockwise (increase heading)
    if angle < 0: 
        rotateClockwise(abs(angle))
    #if the angle is positive, rotate counter-clockwise (decrease heading)
    else: 
        rotateCounterClockwise(angle)

#############################################################################################################################################################################
#ARM THE SUB DUMMY
master.arducopter_arm()
y = 0.5 #meters for # of feet we want to go down.
goDepth(y)

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

set_mode("ALT_HOLD")
count =0
while True:
    #move forward
    if detect == False:
        count +=1
        #set mode to stabilize or alt_hold?
        if count < 25:
            manualControl(500, 0, 500, 0)
    #detect red rectangles
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([160, 100, 20])
    upper_red = np.array([180, 360, 255])

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
            set_mode("ALT_HOLD")
            while True:
                direction = check_direction()
                if direction == 0:
                    while direction ==0:
                        move_right()
                        direction = check_direction()
                    break
                elif direction == 1:
                    while direction == 1:
                        move_left()
                        direction = check_direction()
                    break
                else:
                    break

            for i in range(0,3):
                move_left()  

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
