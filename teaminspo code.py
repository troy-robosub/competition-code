#??
from sys import _current_frames
#pymavlink to communicate with pixhawk
from pymavlink import mavutil
#timing
import time
#??
import argparse
#math operations
import math


# Create the connection
master = mavutil.mavlink_connection('udp:192.168.2.1:14550')

#ensure connection is valid
master.wait_heartbeat()
print("Hi")

#ensure connection is valid
master.wait_heartbeat()
#############################################################################################################################################################################

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
                print("success1")
            #when depth can't be split into all of the above, returns as empty
            except:
                print('flop2')
            #print the depth out
            print("Current Depth: ", depth)
        #as soon as the depth is detected, and isn't 0 (which is what the fucntion sets it to)
        if not depth == 0:
            print("flop3")
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
        mode = 'ALT_HOLD'
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
        mode = 'ALT_HOLD'
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
    mode = 'ALT_HOLD'
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
        #set pitch to xThrottle (forward), no thrust (0-1000) because xthrottle is forward due to STABILIZE (no changes in pitch)
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
    
        

        

    
    


#wait for the connection
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")
#self-explanatory
boot_time = time.time()
master.wait_heartbeat()
print("<<<<<<<HEARTBEAT RECEIVED>>>>>>")
master.arducopter_arm()

#get velocity
print("velocity:")
print(get_velocity())

#test travel_in_x
print("traveling in x")
travel_in_x(700,0,500,0)

#test traveling in y direction
print("traveling in y")
manualControl(0,700,500,0)

#print DEPTH
print("depth:")
print(getDepth())

#rotate
print("rotatation")
rotateClockwise()
rotateClockwise()
print("done")



