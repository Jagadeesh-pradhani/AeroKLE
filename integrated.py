#!/usr/bin/env python

from __future__ import print_function
import device_patches
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import argparse
import geopy.distance
from classify import eye
import cv2
import os
import sys, getopt
import signal
import time
from edge_impulse_linux.image import ImageImpulseRunner


# connect to drone
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600
    print("\nConnecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

    # connect to drone

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

def get_dstance(cord1, cord2):
    # return distance n meter
    return (geopy.distance.geodesic(cord1, cord2).km) * 1000

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
    time.sleep(1)

def change_alt(to_alt):
    while True:
        send_ned_velocity(0, 0, DOWN)
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= to_alt - 1 and vehicle.location.global_relative_frame.alt <= to_alt + 1:
            print("Reached target altitude")
            break
        time.sleep(1)
    send_ned_velocity(0, 0, 0)


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

def drop_parcel():
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        9,  # servo number
        1000,  # servo position between 1000 and 2000
        0, 0, 0, 0, 0)  # param 3 ~ 7 not used
    print("dropping parcel...")
    # send command to vehicle
    vehicle.send_mavlink(msg)
    print("parcel dropped...")


''' Vision Integration'''


runner = None

show_camera = True
if (sys.platform == 'linux' and not os.environ.get('DISPLAY')):
    show_camera = False
def now():
    return round(time.time() * 1000)



def sigint_handler(sig, frame):
    print('Interrupted')
    if (runner):
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

model = "86.eim"
dir_path = os.path.dirname(os.path.realpath(__file__))
modelfile = os.path.join(dir_path, model)


front = 2  # speed in m/s
back = -2
right = 2
left = -2
UP = -0.5
DOWN = 0.5

def eye():
    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']
            videoCaptureDeviceId = int(0)  # if not automatically detect add id here inside bracket...,
            camera = cv2.VideoCapture(videoCaptureDeviceId)
            ret = camera.read()[0]
            if ret:
                backendName = camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) in port %s selected." % (backendName, h, w, videoCaptureDeviceId))
                camera.release()
            else:
                raise Exception("Couldn't initialize selected camera.")

            next_frame = 0  # limit to ~10 fps here

            for res, img in runner.classifier(videoCaptureDeviceId):
                if (next_frame > now()):
                    time.sleep((next_frame - now()) / 1000)

                # print('classification runner response', res)

                if "classification" in res["result"].keys():
                    print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                    for label in labels:
                        score = res['result']['classification'][label]
                        print('%s: %.2f\t' % (label, score), end='')
                    print('', flush=True)

                elif "bounding_boxes" in res["result"].keys():
                    print('Found %d bounding boxes (%d ms.)' % (
                    len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                    for bb in res["result"]["bounding_boxes"]:
                        print('\t%s (%.2f): centroid x=%d y=%d ' % (bb['label'], bb['value'], bb['x'], bb['y']))
                        img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']),
                                            (255, 0, 0), 1)

                if (show_camera):
                    cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    if cv2.waitKey(1) == ord('q'):
                        break

                next_frame = now() + 2  # you can control speed here....
        finally:
            if (runner):
                runner.stop()
        c = len(res["result"]["bounding_boxes"])
        if (c == 1):
            return (bb['x'], bb['y'], len(res["result"]["bounding_boxes"]))
        else:
            return (0, 0, len(res["result"]["bounding_boxes"]))

def to_angles(a, b, c):
    d = math.sqrt((a - 320) * (a - 320) + (b - 240) * (b - 240))

    if a < 320 and b < 240:
        theta = math.degrees(math.asin((320 - a) / d))
    elif a > 320 and b < 240:
        theta = 360 - math.degrees(math.asin((a - 320) / d))
    elif a < 320 and b > 240:
        theta = 90 + math.degrees(math.asin((b - 240) / d))
    elif a > 320 and b > 240:
        theta = 180 + math.degrees(math.asin((a - 320) / d))

    return theta

def goto_location(to_lat, to_long):
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    curr_lat = vehicle.location.global_relative_frame.lat
    curr_lon = vehicle.location.global_relative_frame.lon
    curr_alt = vehicle.location.global_relative_frame.alt

    # set to locaton (lat, lon, alt)
    to_lat = to_lat
    to_lon = to_long
    to_alt = curr_alt

    to_pont = LocationGlobalRelative(to_lat, to_lon, to_alt)
    vehicle.simple_goto(to_pont, groundspeed=1)

    to_cord = (to_lat, to_lon)
    while True:
        a, b, c = eye()
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)
        to_pont = LocationGlobalRelative(cur_lat, cur_lon, to_alt)
        print("curr location: {}".format(curr_cord))
        distance = get_dstance(curr_cord, to_cord)
        print("distance ramaining {}".format(distance))

        if (c == 1):
            vehicle.simple_goto(to_pont, groundspeed=1)
            a, b, c = eye()
            theta = to_angles(a, b, c)   # turn to required angle
            condition_yaw(theta, True)
            while True:
                send_ned_velocity(front, 0, 0)  # make one coordinate of frame zero with respect to target
                if (a <= 0.5):
                    send_ned_velocity(0, 0, 0)
                    break
            a, b, c = eye()  # call again to get new frames
            a = 0
            theta = to_angles(a, b, c)
            condition_yaw(theta, True)  # turn drone to the target location
            while True:
                send_ned_velocity(front, 0, 0)  # make one coordinate of frame zero with respect to target
                if (b <= 0.5):
                    send_ned_velocity(0, 0, 0)
                    break
            # now the Drone Is on Target
            change_alt(20)  # change altitude to 20m
            time.sleep(3)
            drop_parcel()   # Drop parcel
            
            print("Returning to Launch")
            vehicle.mode = VehicleMode("RTL")
            vehicle.close()
            exit()    # exit code

        if distance <= 2:
            print("Reached within 2 meters of target location...")
            break
        time.sleep(1)


''' Main Program'''

condition_yaw(180, True)
def my_mission():
    arm_and_takeoff(10)
    goto_location(15.3676360, 75.1256290)  # goto function stops and drops parcel if target detected
    time.sleep(2)
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

my_mission()

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

print("Mission Completed")
