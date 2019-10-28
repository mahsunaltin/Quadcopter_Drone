from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Drone'a baglan
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)


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


setGripper(1)
time.sleep(2)
setGripper(0)
time.sleep(2)
setGripper(1)
