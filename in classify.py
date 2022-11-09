#!/usr/bin/env python

from __future__ import print_function
import device_patches
import cv2
import os
import sys, getopt
import signal
import time
from edge_impulse_linux.image import ImageImpulseRunner

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import math
import numpy as np
import geopy.distance
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

def get_dstance(cord1, cord2):
    # return distance n meter
    return (geopy.distance.geodesic(cord1, cord2).km) * 1000

def send_ned_velocity(velocity_z, to_alt):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        0, 0, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    if to_alt == 0:
        vehicle.send_mavlink(msg)
        time.sleep(1)
        return

    while True:
        vehicle.send_mavlink(msg)
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= to_alt - 1 and vehicle.location.global_relative_frame.alt <= to_alt + 1:
            print("Reached target altitude")
            break
        time.sleep(1)

def to_coord(lat, lng, theta, distance):
    """
    to_coord a LatLng theta degrees counterclockwise and some
    meters in that direction.
    Notes:
        http://www.movable-type.co.uk/scripts/latlong.html
        0 DEGREES IS THE VERTICAL Y AXIS! IMPORTANT!
    Args:
        theta:    A number in degrees.
        distance: A number in meters.
    Returns:
        A new LatLng.
    """
    theta = np.float32(theta)

    E_RADIUS = 6371000
    delta = np.divide(np.float32(distance), np.float32(E_RADIUS))

    def to_radians(theta):
        return np.divide(np.dot(theta, np.pi), np.float32(180.0))

    def to_degrees(theta):
        return np.divide(np.dot(theta, np.float32(180.0)), np.pi)

    theta = to_radians(theta)
    lat1 = to_radians(lat)
    lng1 = to_radians(lng)

    lat2 = np.arcsin( np.sin(lat1) * np.cos(delta) +
                      np.cos(lat1) * np.sin(delta) * np.cos(theta) )

    lng2 = lng1 + np.arctan2( np.sin(theta) * np.sin(delta) * np.cos(lat1),
                              np.cos(delta) - np.sin(lat1) * np.sin(lat2))

    lng2 = (lng2 + 3 * np.pi) % (2 * np.pi) - np.pi

    return (to_degrees(lat2), to_degrees(lng2))

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

def calc(a, b):
    if a == 320 and b == 240:
        return (0, 0)

    d_pixels = math.sqrt((a - 320) * (a - 320) + (b - 240) * (b - 240))              # distance between drone and target

    # 53.5 -> horizontal FOV obtained from Picamera documentation for 5 mp
    # 41.41 -> vertical FOV obtained from Picamera documentation for 5 mp
    # 62.2 -> for 8 mp
    # 48.8 -> for 8 mp
    a1 = 2 * vehicle.location.global_relative_frame.alt * math.tan(math.radians(62.2/2))    # horizontal distance in 'm' above 30 m
    b1 = 2 * vehicle.location.global_relative_frame.alt * math.tan(math.radians(48.8/2))   # vertical distance in 'm' above 30 m
    m = a1/(640 * 0.0002645833)                                                               # horizontal constant
    n = b1/(480 * 0.0002645833)

    if a < 320:
        if b < 240:
            theta = math.degrees(math.asin((320 - a) / d_pixels))  # get  angle theta made to target
            theta = 360 - theta  # angle in clock wise direction
            x = ((320 - a) * 0.0002645833) * m  # horizontal distance converted to meters
            y = ((240 - b) * 0.0002645833) * n  # vertical distance converted to meters
            d = math.sqrt((x * x) + (y * y))  # total distance to target in meters

        elif b > 240:
            theta = 180 + math.degrees(math.asin((b - 240) / d_pixels))
            x = ((320 - a) * 0.0002645833) * m
            y = ((b - 240) * 0.0002645833) * n
            d = math.sqrt((x * x) + (y * y))

        elif b == 240:
            theta = 270
            x = ((320 - a) * 0.0002645833) * m
            y = 0
            d = math.sqrt((x * x) + (y * y))

    elif a > 320:
        if b < 240:
            theta = math.degrees(math.asin((a - 320) / d_pixels))
            x = ((a - 320) * 0.0002645833) * m
            y = ((240 - b) * 0.0002645833) * n
            d = math.sqrt((x * x) + (y * y))

        elif b > 240:
            theta = 180 - math.degrees(math.asin((a - 320) / d_pixels))
            x = ((a - 320) * 0.0002645833) * m
            y = ((b - 240) * 0.0002645833) * n
            d = math.sqrt((x * x) + (y * y))

        elif b == 240:
            theta = 90
            x = ((a - 320) * 0.0002645833) * m
            y = 0
            d = math.sqrt((x * x) + (y * y))

    elif a == 320:
        if b == 0:
            theta = 0
            x = 0
            y = 240 * 0.0002645833 * n
            d = math.sqrt((x * x) + (y * y))

        elif b == 480:
            theta = 180
            x = 0
            y = ((b - 240) * 0.0002645833) * n
            d = math.sqrt((x * x) + (y * y))

    return (theta, d)

def target(a, b):
    theta, d = calc(a, b)  # function to find angle and distance to target

    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon

    if d != 0:
        lat, lon = to_coord(lat, lon, theta, d)                               # get coordinates of target
        goto_location(lat, lon, 1)                                            # goto target location

    send_ned_velocity(DOWN, 20)                                                   # descend to 20 m
    send_ned_velocity(0, 0)                                                   # stop at 20 m
    time.sleep(2)
    drop_parcel()                                                             # drop parcel
    time.sleep(2)
    print("Mission complete, Returning to Launch")
    vehicle.mode = VehicleMode("RTL")                                         # Return to launch

    vehicle.close()
    exit()


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
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)
        print("curr location: {}".format(curr_cord))
        distance = get_dstance(curr_cord, to_cord)
        print("distance ramaining {}".format(distance))
        if distance <= 2:
            print("Reached within 2 meters of target location...")
            break
        time.sleep(1)

def mla():
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

    model = "64.eim"
    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']
            videoCaptureDeviceId = int(2)  # if not automatically detect add id here inside bracket...,
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

            def detect(to_lat, to_lon):
                for res, img in runner.classifier(videoCaptureDeviceId):
                    if (next_frame > now()):
                        time.sleep((next_frame - now()) / 1000)

                        # print('classification runner response', res)
                    curr_lat = vehicle.location.global_relative_frame.lat
                    curr_lon = vehicle.location.global_relative_frame.lon
                    curr_coord = (curr_lat, curr_lon)
                    to_coor = (to_lat, to_lon)
                    curr  = LocationGlobalRelative(curr_lat, curr_lon, vehicle.location.global_relative_frame.alt)
                    if "classification" in res["result"].keys():
                        print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                        for label in labels:
                            score = res['result']['classification'][label]
                            print('%s: %.2f\t' % (label, score), end='')
                        print('', flush=True)

                    elif "bounding_boxes" in res["result"].keys():
                        print('Found %d bounding boxes (%d ms.)' % (
                            len(res["result"]["bounding_boxes"]),
                            res['timing']['dsp'] + res['timing']['classification']))
                        for bb in res["result"]["bounding_boxes"]:
                            print('\t%s (%.2f): centroid x=%d y=%d ' % (bb['label'], bb['value'], bb['x'], bb['y']))
                            img = cv2.rectangle(img, (bb['x'], bb['y']),
                                                (bb['x'] + bb['width'], bb['y'] + bb['height']),
                                                (255, 0, 0), 1)

                    if (show_camera):
                        cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

                    distance = get_dstance(to_coor, curr_coord)

                    c = len(res["result"]["bounding_boxes"])
                    if (c == 1):
                        vehicle.simple_goto(curr, groundspeed)
                        return (bb['x'], bb['y'], len(res["result"]["bounding_boxes"]))
                        break

                    else:
                        if distance <= 2:
                            return (0, 0, len(res["result"]["bounding_boxes"]))
                            break

            def goto(to_lat, to_lon):
                to_pont = LocationGlobalRelative(to_lat, to_lon, vehicle.location.global_relative_frame.alt)
                vehicle.simple_goto(to_pont, groundspeed=1)
                a, b, c = detect(to_lat, to_lon)
                time.sleep(2)
                if c == 1:
                    a, b, c = detect()
                    target(a, b)

            goto(23.3659, 236.2555)                     # search area
            goto(23.6659, 25.36648)
            print("no target detected : Returning to Launch")
            vehicle.mode = VehicleMode("RTL")

        finally:
            if (runner):
                runner.stop()


def my_mission():
    arm_and_takeoff(30)
    goto_location(25.6898, 82.2646)
    mla()
    time.sleep(2)
    print("no target detected : Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

RIGHT = 2
LEFT = -2
FRONT = 2
BACK = -2
UP = -0.5
DOWN = 0.5
groundspeed = 1.5
