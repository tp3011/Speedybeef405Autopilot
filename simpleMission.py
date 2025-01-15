import time
from dronekit import VehicleMode, LocationGlobal, Command, connect

from pymavlink import mavutil
import threading
import sys
import keyboard

from main import check_GPS_status, arm_vehicle, connection_string, baud_rate, check_safety_param

vehicle = connect(ip = connection_string, baud= baud_rate, wait_ready= True)
print(f"Vehicle connected on {connection_string} with baud rate {baud_rate}")
time.sleep(1)

print("Checking GPS and safety parameters...")
while True:
    check_GPS_status(vehicle)
    check_safety_param(vehicle)
    time.sleep(1)
    break
home_position = [vehicle.location.global_frame.lat,
                 vehicle.location.global_frame.lon]
print("Arming vehicle...")
arm_vehicle(vehicle)

print("Setting simple waypoint flight...")
cmds = vehicle.commands
cmds.clear()

mission_start = vehicle.message_factory.command_long_encode(
        0, 0,  # Target system, target component
        mavutil.mavlink.MAV_CMD_MISSION_START,  # Command ID for mission start
        0,  # Confirmation
        0,  # Start mission from this waypoint index (default: 0)
        0, 0, 0, 0, 0, 0  # Unused parameters
    )


msg = vehicle.message_factory.mission_item_int_encode(
        0, 0,0,  # Target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # INT frame for GPS precision
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff command
        0, 1,  # Confirmation, Auto-continue
        0, 0, 0, 0,  # Unused parameters for takeoff
        int(vehicle.location.global_frame.lat * 1e7),  # Latitude converted to int32
        int(vehicle.location.global_frame.lon * 1e7),  # Longitude converted to int32
        100  # Altitude in meters
    )
cmds.add(msg)
time.sleep(1)
climb_command =vehicle.message_factory.mission_item_int_encode(0, 0, 0,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1,
                        0, 0, 0, 0,
                        int(45.512747696420064*1e7), int( -73.8160586510193 *1e7),
                        50)

cmds.add(climb_command)
init_land_command = vehicle.message_factory.mission_item_int_encode(0, 0, 0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            mavutil.mavlink.MAV_CMD_DO_LAND_START,
                            0, 1,
                            0, 0, 0, 0, 0, 0, 0)
approach_waypoint = vehicle.message_factory.mission_item_int_encode(0, 0,0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1,
                            0, 0, 0, 0,
                            int(45.51689761061211 *1e7), int(-73.7947726427685*1e7), 25)
land_command = vehicle.message_factory.mission_item_int_encode(
                0,0, 0,  # Target system and component
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 1, 0, 0, 0, 0,  # Unused parameters
                int(home_position[0] * 1e7),  # Convert latitude to int32
                int(home_position[1] * 1e7),  # Convert longitude to int32
                0  )       # Altitude as int32


cmds.add(init_land_command)
time.sleep(.5)
cmds.add(approach_waypoint)
time.sleep(.5)
cmds.add(land_command)
time.sleep(1)
cmds.upload()
time.sleep(2)
vehicle.mode = VehicleMode("TAKEOFF")
while vehicle.mode != VehicleMode("TAKEOFF"):
    time.sleep(0.5)

    vehicle.mode = VehicleMode("TAKEOFF")
else: print("Vehicle mode set to TAKEOFF")
time.sleep(4)
vehicle.mode = VehicleMode("AUTO")
while vehicle.mode != VehicleMode("AUTO"):
    time.sleep(0.5)

    vehicle.mode = VehicleMode("AUTO")
else: print("Vehicle mode set to AUTO")
print(f"Number of waypoints: {len(cmds)}")

for i, cmd in enumerate(cmds):
    print(f"Command {i}: {cmd}")
vehicle.send_mavlink(mission_start)

time.sleep(15)
print("shutting down flight")

vehicle.close()
sys.exit(0)
