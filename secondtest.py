#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
usage: python square_off.py --connect <*connection_string>
This script connects to the drone and waits until armed. When armed it will takeoff
to 3m altitude, then navigate a 10x10 meter square. At each corner of the square the drone
will wait for 5 seconds.
"""

from __future__ import print_function

import math
import time
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Size of square in meters
SQUARE_SIZE = 10
# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 5 # Target altitude is now 15 feet up
# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
# Variable to keep track of if joystick to arm has returned to center
rcin_4_center = False

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    inityawmsg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(inityawmsg)
    
    time.sleep(1)
    print("Getting the gimbal in place!")
    # # Setting the gimbal
    # gimbalmsg = vehicle.message_factory.command_long_encode(
    #     0, 0,    # target system, target component
    #     mavutil.mavlink.gimbal_manager_set_pitchyaw, #command
    #     0,          # gimbal ID
    #     -0.785,     # param 1, pitch in radians
    #     0,          # param 2, yaw in radians
    #     NaN,        # param 3 pitch rate
    #     0, 	    # param 4, yaw rate
    #     0, 0, 0)    # param 5 ~ 7 not used
    # # send command to vehicle
    # vehicle.send_mavlink(gimbalmsg)


    # delay to wait until yaw of copter is at desired yaw angle and gimbal is set to appropriate bearing
    time.sleep(3)

# Set up option parsing to get connection string and mission plan file
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# aquire connection_string
connection_string = args.connect

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string')

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

print('Succesfully connected to vehicle')

"""
Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
has returned to center for two consecutive seconds.
"""
@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)


if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle.mode = VehicleMode("ALT_HOLD")

# Wait for pilot before proceeding
print('Waiting for safety pilot to arm...')

# Wait until safety pilot arms drone
while not vehicle.armed:
    time.sleep(1)

print('Armed...')
vehicle.mode = VehicleMode("GUIDED")

if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:

    rcin_4_center_once = False
    rcin_4_center_twice = False
    while not rcin_4_center_twice:
        if rcin_4_center:
            if rcin_4_center_once:
                rcin_4_center_twice = True
            else:
                rcin_4_center_once = True
        else:
            rcin_4_center_once = False
        time.sleep(1)
    
    # Takeoff to short altitude
    print("Taking off!")
    vehicle.simple_takeoff(TARGET_ALTITUDE)  # Take off to target altitude

    while True:
         # Break just below target altitude.
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            print("About to break out of takoff while loop")
            break
        time.sleep(0.5)
    # yaw north
    condition_yaw(0)
    


def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]


# Put code for what to do once in the air:

"""
##Set up these values before flying!!!!
"""
run = 1
setpoint_x = 0
setpoint_y = 0
setpoint = [setpoint_x, setpoint_y]
Kp = 0.01
Ki = 0               
Kd = 0
integral = [0.0, 0.0]
error_prior = [0.0, 0.0]
iteration_time = .1
current = [0, 0]
bias = 0
hover_thrust = 0.5
trackerWorking = True


import cv2
import sys
import time

controlVehicle = False 

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
 
 
#     # Set up tracker.
tracker = cv2.TrackerCSRT_create()
# Read video
video = cv2.VideoCapture(0) # for using CAM

# Exit if video not opened.
if not video.isOpened():
    print("Could not open video")
    
    print('Landing')
    if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
        # Land Copter
        vehicle.mode = VehicleMode("LAND")

    # Stay connected to vehicle until landed and disarmed
    while vehicle.armed:
        time.sleep(1)

    print("Done!")

    # Close vehicle object before exiting script
    vehicle.close()
    
    sys.exit()


#0 Read first frame.
"""ok0, frame0 = video.read()
if not ok0:
    print ('Cannot read video file')
    sys.exit()                                  # Does script crash during this command??
# Define an initial bounding box
bbox0 = (287, 23, 86, 320)

time.sleep(3)

# Read second frame.
ok1, frame1 = video.read()
if not ok1:
    print ('Cannot read video file')
    sys.exit()                                  # Does script crash during this command??
# Define an initial bounding box
bbox1 = (287, 23, 86, 320)

time.sleep(3)
"""
# Read third frame.
ok, frame = video.read()
if not ok:
    print ('Cannot read video file')
    sys.exit()                                  # Does script crash during this command??
# Define an initial bounding box
bbox = (287, 43, 154, 320) #changed box size


# Uncomment the line below to select a different bounding box
#bbox = cv2.selectROI(frame, False)
#cv2.destroyAllWindows()

# Initialize tracker with first frame and bounding box
ok = tracker.init(frame, bbox)

def trackFrame(video, bbox):
    ok, frame = video.read()
      
    # Update tracker
    ok, bbox = tracker.update(frame)

    # Draw bounding box
    if ok:
        # Tracking success
        trackerWorking = True
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        object_location_x = int((2*bbox[0] + bbox[2]) / 2)
        object_location_y = int((2*bbox[1] + bbox[3]) / 2)
        # cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        dimensions = frame.shape
        x_image_width = dimensions[1]
        y_image_width = dimensions[0]
        x_error = x_image_width / 2 - object_location_x
        y_error = y_image_width / 2 - object_location_y
        
        cv2.imshow("Tracking", frame)


    else :
        # Tracking failure
        print("Tracking error! Lost object from frame!")


    return x_error, y_error

target_yaw = 0 #math.degrees(90)
print("Control begin in three seconds...")
time.sleep(3)

try:
    while run:
        try:
            x_val, y_val = trackFrame(video, bbox)
            current = [x_val, y_val]
        
        except:
            print("Bounding Box not Found!")
            trackerWorking = False
            # put some code here to STOP if tracker lost the object
            # RTL for now
            output[0] = 0
            output[1] = 0
            new_quat = to_quaternion(output[0], output[1], target_yaw)
            print("Output 0: ", output[0])
            print("Output 1: ", output[1])
            # http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-attitude-target
            msg = vehicle.message_factory.set_attitude_target_encode(
                0, # time_boot_ms
                1, # target system
                1, # target component
                0b00000100, # This mask makes us ignore yaw rate. Visual Tracker commented this
                new_quat, # attitude (quaternion)
                0, # roll rate
                0, # pitch rate
                0, # yaw rate
                hover_thrust  # thrust (0-1 where 0.5 is no vertical velocity)
            )
            vehicle.send_mavlink(msg)

            
           
        print(current)
        
        # calculate PID
        error = [a_i - b_i for a_i, b_i in zip(setpoint, current)]
        integral = [a_i + (b_i * iteration_time) for a_i, b_i in zip(integral, error)]
        derivative = [(a_i - b_i) / iteration_time for a_i, b_i in zip(error, error_prior)]
        output = [Kp*a_i + Ki*b_i + Kd*c_i + bias for a_i, b_i, c_i in zip(error, integral, derivative)]
        
        if (trackerWorking == True):
            # generate mavlink message to send attitude setpoint
            new_quat = to_quaternion(output[0], output[1], target_yaw)
            print("Output 0: ", output[0])
            print("Output 1: ", output[1])
            # http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-attitude-target
            msg = vehicle.message_factory.set_attitude_target_encode(
                0, # time_boot_ms
                1, # target system
                1, # target component
                0b00000100, # This mask makes us ignore yaw rate. Visual Tracker commented this
                new_quat, # attitude (quaternion)
                0, # roll rate
                0, # pitch rate
                0, # yaw rate
                hover_thrust  # thrust (0-1 where 0.5 is no vertical velocity)
            )
            
            if(controlVehicle):
                print("About to send a quarternion command")
                vehicle.send_mavlink(msg)             # This is commented for initial testing, we don't want the drone to be controlled at first
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                controlVehicle = True
                print("controlVehicle now true")
                
                
        else:
            # Return to launch if lost the object
            vehicle.mode = VehicleMode("RTL")

            
        time.sleep(iteration_time)
except KeyboardInterrupt:
    print('exiting')
    video.release()
    pass



# Then this is Sichitu's code for landing

print('Landing')
if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
    # Land Copter
    vehicle.mode = VehicleMode("LAND")



# Stay connected to vehicle until landed and disarmed
while vehicle.armed:
    time.sleep(1)

print("Done!")

# Close vehicle object before exiting script
vehicle.close()
