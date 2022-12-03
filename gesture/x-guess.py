from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import numpy as np
import geopy.distance
from functions import geta, getb, getc

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

def condition_yaw(heading, relative):
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

def send_ned_velocity(x, y, z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        x, y, z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    
    if x == 0 and y == 0 and z == 0:
        vehicle.send_mavlink(msg)
        return
    
    while True:
        a = geta()
        vehicle.send_mavlink(msg)
        if a < 125 and a > 115:
            break

def set(a):
    if a != 120 and a > 120:
        send_ned_velocity(1, 0, 0)
        send_ned_velocity(0, 0, 0)
    elif a != 120 and a < 120:
        send_ned_velocity(-1, 0, 0)
        send_ned_velocity(0, 0, 0)

def guess():
    while True:
        c = getc()
        if c == 1:
            c = getc()
            if c == 1:
                print("Detected")
                a = geta()
                b = getb()
                print(a, b)
                set(a)          # initially lock to midpoint
                print(a, b)
                for i in range(10):
                    a = geta()
                    if a != 120:
                        set(a)
                        print("set to mid")
                    else:
                        print("not set")



def mission():
    arm_and_takeoff(5)
    time.sleep(2)
    guess()

mission()
