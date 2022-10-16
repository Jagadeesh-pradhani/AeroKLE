from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import numpy as np
import geopy.distance
from classify import eye
# Set up option parsing to get connection string
import argparse


def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 921600
    print("\nConnecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

vehicle = connectMyCopter()

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def drop_parcel():
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        9,  # servo number
        2000,  # servo position between 1000 and 2000
        0, 0, 0, 0, 0)  # param 3 ~ 7 not used
    print("dropping parcel...")
    # send command to vehicle
    vehicle.send_mavlink(msg)
    print("parcel dropped...")

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)

def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def get_dstance(cord1, cord2):
    # return distance n meter
    return (geopy.distance.geodesic(cord1, cord2).km) * 1000

def goto_location(to_lat, to_long):
    curr_alt = vehicle.location.global_relative_frame.alt

    # set to location (lat, lon, alt)
    to_lat = to_lat
    to_lon = to_long
    to_alt = curr_alt

    to_pont = LocationGlobalRelative(to_lat, to_lon, to_alt)
    vehicle.simple_goto(to_pont, groundspeed=1)

    to_cord = (to_lat, to_lon)
    while True:
        a, b, c = eye()
        if c != 0:   # object found
            curr_lat1 = vehicle.location.global_relative_frame.lat
            curr_lon1 = vehicle.location.global_relative_frame.lon
            detected_pont = LocationGlobalRelative(curr_lat1, curr_lon1, to_alt)
            vehicle.simple_goto(detected_pont, groundspeed=1)  # goto detected point
            x = target()   # go to target location using function target 
            if x != 0:     # if x == 0, then target isn't found hence continue navigation
                break
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)
        distance = get_dstance(curr_cord, to_cord)
        print("distance ramaining {}".format(distance))
        if distance <= 2:
            print("Reached within 2 meters of target location...")
            break
        time.sleep(1)

def to_angles():
    a, b, c = eye()

    if c == 0:
        return 0

    d = math.sqrt((a - 320) * (a - 320) + (b - 240) * (b - 240))  # distance between drone and target

    if a < 320 and b < 240:  # get angle theta made to target
        theta = math.degrees(math.asin((320 - a) / d))
    elif a > 320 and b < 240:
        theta = 360 - math.degrees(math.asin((a - 320) / d))
    elif a < 320 and b > 240:
        theta = 90 + math.degrees(math.asin((b - 240) / d))
    elif a > 320 and b > 240:
        theta = 180 + math.degrees(math.asin((a - 320) / d))

    return theta

def target():
    a, b, c = eye()             # check again to confirm detection
    if c == 0:
        return 0

    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt
    current = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(current, groundspeed=1)
    time.sleep(5)

    condition_yaw(180, True)
    theta = to_angles()
    condition_yaw(theta, True)

    while True:             # loop to reach target.
        a, b, c = eye()             # keep on checking until the target is reached
        d = math.sqrt((a - 320) * (a - 320) + (b - 240) * (b - 240))
        send_ned_velocity(FRONT, 0, 0)
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)

        if d <= 2:
            send_ned_velocity(0, 0, 0)
            print("Reached within 2 meters of target location...")
            break
        time.sleep(1)

    while True:
        send_ned_velocity(0, 0, DOWN)
        curr_alt = vehicle.location.global_relative_frame.alt
        if curr_alt >= decend_alt - 1 and curr_alt <= decend_alt + 1:
            print("Reached target altitude")
            break
        time.sleep(1)

    time.sleep(2)
    drop_parcel()
    time.sleep(2)

    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

    exit()

def mission():
    arm_and_takeoff(takeoff_alt)
    goto_location(25.806476, 86.778428)
    goto_location(25.806476, 86.778428)
    """Number of wavepoints"""
    print("Target Not Found, Returning To launch")
    vehicle.mode = VehicleMode("RTL")

""" Define Required altitude in m """
takeoff_alt = 30
decend_alt = 20

""" Define Speed in m/s"""
FRONT = 2
BACK = -2
RIGHT = 2
LEFT = -2
UP = -0.5
DOWN = 0.5

mission()

vehicle.close()


