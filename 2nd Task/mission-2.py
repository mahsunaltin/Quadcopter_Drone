# Dronekit Libraries
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
# Image Processing Libraries
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import math

# Connection Libraries
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Drone'a baglan
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)

# Drone'u dengele
print("Changing flight mode to loiter...")
vehicle.mode = VehicleMode("LOITER")

## MAIN:

#camera resolution as pixel

camera_res = [1088, 1088]
camera = PiCamera()
camera.resolution = (camera_res) #max 3280 x 2464
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(camera_res))
time.sleep(0.1)

# similarity parameter
pixely = 1156

inorout = 'out'
hayt = 190  ## pixhawk data
## max circle parameters as pixel
maxX = 0
maxY = 0
maxR = 0

## max circle parameters as cm
maxXcm = 0
maxYcm = 0
maxRcm = 0
dista = 0

#center of frame
centerx = 10
centery = 10

#as pixel

circlex = 544
circley = 544

centery = 544 #int(camera_res / 2 )#,_ =  frame.shape
centerx = 544
#centerx= int(round(centerx/2))
#centery= int(round(centery/2))
circler = int(round(centerx/4))

closer = np.zeros([150,150], dtype = int)
cv2.imshow('closer',closer)

cnt = 0

# Motoru calistir yuksel definition
def arm_and_takeoff(aTargetAltitude):

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)


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
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def set_roi(location):
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

    For more information see:
    http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
    """
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,  # command
        0,  # confirmation
        0, 0, 0, 0,  # params 1-4
        location.lat,
        location.lon,
        location.alt
    )
    # send command to vehicle
    vehicle.send_mavlink(msg)


def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(
            newlat, newlon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing


def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    # SET_POSITION_TARGET_GLOBAL_INT
    For more information see: https://pixhawk.ethz.ch/mavlink/

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111111000,  # type_mask (only speeds enabled)
        aLocation.lat*1e7,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt,  # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0,  # X velocity in NED frame in m/s
        0,  # Y velocity in NED frame in m/s
        0,  # Z velocity in NED frame in m/s
        # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0, 0,
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def goto_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        north, east, down,
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0, 0,
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_metres(
            vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance <= targetDistance*0.01:
            print("Reached target")
            break
        time.sleep(2)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0, 0,
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        0,
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x,  # X velocity in NED frame in m/s
        velocity_y,  # Y velocity in NED frame in m/s
        velocity_z,  # Z velocity in NED frame in m/s
        # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0, 0,
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


    print("goruntu isleme")

def coverfinder(img):
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 0.9, 120, param1=75, param2=20, minRadius=7, maxRadius=150)
    global maxR
    global maxX
    global maxY
    global maxRcm
    global maxXcm
    global maxYcm
    global leng
    global dista
    global centerx
    global centery
    global circlex
    global circley
    global circler
    global frame
    global inorout
    
    flipfc = frame
    if circles is not None:
        circles = np.uint16(np.around(circles))
        maxR = 0
        for i in circles[0,:]:
            
            if i[2] == 0:
                print("no circle")
            else:
                pixelR = i[2]
                if pixelR>maxR:
                    maxR = pixelR
                    maxX = i[0]
                    maxY = i[1]
                
            
        #in or out
        if (maxX-circlex)**2+(maxY-circley)**2 <= (circler-maxR)**2:
            inorout = 'in'
        else:
            inorout = 'out'
            
        leng = math.sqrt((maxX-circlex)**2+(maxY-circley)**2)
                
        dista = (hayt*leng/pixely)
        
        maxXcm = (hayt*(maxX-circlex)/pixely)
        maxYcm = (hayt*(maxY-circley)/pixely)
        maxRcm = (hayt*maxR/pixely)
        
        flipfc = frame

    return maxXcm, maxYcm, maxRcm, dista

def setGripper(CloseOpen):
    # Set the ArduPilot gripper
    # Argument CloseOpen is 0 for close and 1 for open

    msg = vehicle.message_factory.command_long_encode(
            0, 1,           # target system, target component
            mavutil.mavlink.MAV_CMD_DO_GRIPPER,  # frame
            0,              # confirmation
            1,              # param 1: Gripper # (ArduPilot only supports 1)
            CloseOpen,      # param 2: Open or Close
            0, 0, 0, 0, 0)  # params 3-7 (not used)
    vehicle.send_mavlink(msg)


# 10 metreye drone'u cikar
arm_and_takeoff(8)
print("Ucuyoruz babaaa...")

# 2 saniye beklet
time.sleep(2)

# Siselerin oldugu hedefe gitmek icin lokasyonlari degistir!!!
pointOfTarget = LocationGlobalRelative(-35.363244, 149.168801, 20)
vehicle.simple_goto(pointOfTarget, groundspeed=10)

# 2 metre'ye kadar alcal
arm_and_takeoff(-6)


print("5m/s hiz ile git...")
vehicle.groundspeed = 5

Xaxis, Yaxis, dista = coverfinder()

if(dista == null):
    print("South 1.7 West 0 'a gitmeye basla...")
    goto(-1.7, 0)

    Xaxis, Yaxis, RMax, dista = coverfinder()

    if(dista == null):
        print("South 1.7 West 0 'a gitmeye basla...")
        goto(-1.7, 0)

        Xaxis, Yaxis, RMax, dista = coverfinder()

        if(dista == null):
            print("North 0 West 1.7 'ye gitmeye basla...")
            goto(0, 1.7)

            Xaxis, Yaxis, RMax, dista = coverfinder()

               if(dista == null):
                    print("North 1.7 West 0 'a gitmeye basla...")
                    goto(1.7, 0)

                    Xaxis, Yaxis, RMax, dista = coverfinder()

                    if(dista == null):
                        print("North 1.7 West 0 'a gitmeye basla...")
                        goto(1.7, 0)

                        Xaxis, Yaxis, RMax, dista = coverfinder()

                        if(dista == null):
                            print("North 0 West 1.7 'ye gitmeye basla...")
                            goto(0, 1.7)

                            Xaxis, Yaxis, RMax, dista = coverfinder()

                            if(dista == null):
                                print("South 1.7 West 0 'e gitmeye basla...")
                                goto(-1.7, 0)

                                Xaxis, Yaxis, RMax, dista = coverfinder()

                                if(dista == null):
                                    print("South 1.7 West 0 'e gitmeye basla...")
                                    goto(-1.7, 0)

                                    Xaxis, Yaxis, RMax, dista = coverfinder()

                                else:
                                    print("Boku yedik!!!!!!!!!")

                            else:
                                goto(Xaxis, Yaxis)
                                # 0.5 metre'ye kadar alcal
                                arm_and_takeoff(-1.5)

                        else:
                            goto(Xaxis, Yaxis)
                            # 0.5 metre'ye kadar alcal
                            arm_and_takeoff(-1.5)

                    else:
                        goto(Xaxis, Yaxis)
                        # 0.5 metre'ye kadar alcal
                        arm_and_takeoff(-1.5)

                else:
                    goto(Xaxis, Yaxis)
                    # 0.5 metre'ye kadar alcal
                    arm_and_takeoff(-1.5)
            else:
                goto(Xaxis, Yaxis)
                # 0.5 metre'ye kadar alcal
                arm_and_takeoff(-1.5)

        else:
            goto(Xaxis, Yaxis)
            # 0.5 metre'ye kadar alcal
            arm_and_takeoff(-1.5)

    else:
        goto(Xaxis, Yaxis)
        # 0.5 metre'ye kadar alcal
        arm_and_takeoff(-1.5)

else:
    goto(Xaxis, Yaxis)
    # 0.5 metre'ye kadar alcal
    arm_and_takeoff(-1.5)

print("Drone'u geri dondur babacim...")
vehicle.mode = VehicleMode("LAND")

# Drone'u kapat
vehicle.close()
