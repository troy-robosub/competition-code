#pymavlink to communicate with pixhawk
from pymavlink import mavutil
#timing
import time
#math operations
import math


def set_mode(modep):
    mode = modep
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system, #target system
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, #custom_mode is the mode we are setting
        mode_id) #mode_id is the mode we are setting it to (previous code)
    print("<<<<<<MODE CHANGED TO ", mode, ">>>>>>")

def manualControl(x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        x,  # pitch for sub, back/forward on joystick [-1000,1000], respectively
        y,  # roll for sub, left/right on joystick [-1000,1000], respectively
        z,  # thrust for sub, slider on joystick [0,1000]
        r,  # yaw for sub, clockwise/counterclockwise on joystick [-1000,1000], respectively
        0)  # buttons
    time.sleep(0.3)


#check for velocity in Qgroundcontrol
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
                print(data, "velocity")
                speed = data[2].split(",")[0]
            except:
                print('')

            velocity = float(speed)
        if not velocity == 0:
            break

    return velocity

def getPressure():
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'SCALED_PRESSURE2':
            data = str(msg)
            try:
                data = data.split(":")
                print(data, "pressure")
                pressure = data[3].split(",")[0]
            except:
                print('')
            pressure = float(pressure)
        return pressure
    
def getDepth(pressure):
    pressure = getPressure()
    P = pressure * 100
    g = 9.80665
    p = 1023.6
    depth = P/(p*g)
    if pressure >= 0:
        return depth
    elif pressure < 0:
        depth = -depth
        return depth

#check for heading in Qgroundcontrol
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

#dependent on getDepth()
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
    
#dependent on get_velocity()
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
    print("<<<<<<MODE CHANGED TO ", mode, ">>>>>> travel_in_x")
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

#dependent on get_heading()
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
    
        
master = mavutil.mavlink_connection('udp:192.168.2.1:14550') # Create the connection


print("<<<<<<WAITING FOR CONNECTION>>>>>>")
master.wait_heartbeat() #ensure connection is valid
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")


master.arducopter_arm()


print("DESCENDINGGGGGGGGG")
for i in range(0,10):
    manualControl(0,0,300,0)

set_mode("ALT_HOLD")
print("Hold for 7 seconds")
time.sleep(7)


print("traveling in y")
for i in range(0,5):
    manualControl(0,700,500,0)

print("moving forward? after 7 seconds")
time.sleep(7)
travel_in_x(700, 1)

time.sleep(5)

print("descending")
goDepth(0.4)
print("finished")

time.sleep(5)
master.arducopter_disarm()


