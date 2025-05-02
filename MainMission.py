import time
from dronekit import VehicleMode, LocationGlobal, Command, connect
from numpy import pi, cos, sin
from DataAcquisition import draw
from pymavlink import mavutil
import threading
import sys
import keyboard

from main import check_GPS_status, arm_vehicle, connection_string, baud_rate, check_safety_param

altitude = 15
r_earth = 6378*10**3 #in meters
displacement = 50.0 #meters
# set waypoint, change mode, payload operation, takeoff, land, kill



def coordonate_change(old_lat, old_lon, heading, dist = displacement):
    dy,dx = calculate_translation(heading, dist)
    new_lat = old_lat + (dy / r_earth) * (180 / pi)
    new_long = old_lon + (dx / r_earth) * (180 / pi) / cos(new_lat * pi / 180)
    return  new_lat, new_long
def calculate_translation(heading, dist):
    dy = dist*cos(heading*pi/180)
    dx = dist*sin(heading*pi/180)
    return dy,dx


def takeoff(v):  # function that sets up take off

    takeoff_cmd = v.message_factory.mission_item_int_encode(
                          0, 0,0,  # Target system, target component
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # INT frame for GPS precision
                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff command
                          0, 1,  # Confirmation, Auto-continue
                          0, 0, 0, 0,  # Unused parameters for takeoff
                          int(vehicle.location.global_frame.lat * 1e7),  # Latitude converted to int32
                          int(vehicle.location.global_frame.lon * 1e7),  # Longitude converted to int32
                          altitude  # Altitude in meters
                      )
    return takeoff_cmd


def transit(v):  # creates a transit command (go from point a to b). This doesnt serve any other purpose than to reposition plane
    #undefined for now
    pass


def payload_drop(v, DLZ_coord):
    print("Setting up payload drop... Please provide target GPS location (decimal) when prompted")
    drop_lat, drop_long = coordonate_change(DLZ_coord[0],DLZ_coord[1],to_heading, dis=1)
    drop_list = []

    drop_waypoint = v.message_factory.mission_item_int_encode(0, 0, 0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  ##alt above sea level
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1,
                            0, 0, 0, 0,
                            int(drop_lat *1e7), int(drop_long *1e7),
                            int(DLZ_coord[2] + 1)) ##DLZ altitude +1meters
    drop_list.append(drop_waypoint)
    servo_command = v.message_factory.command_long_encode(
                            0, 0,  # Target system, target component
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command ID for SET_SERVO
                            0,  # Confirmation
                            6,  # Servo output channel
                            1900,
                            0,0,0,0,0)  # PWM value (1000-2000 µs)
    drop_list.append(servo_command)
    climb_command = v.message_factory.mission_item_int_encode(0, 0, 0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1,
                            0, 0, 0, 0,
                            int(DLZ_coord[0] *1e7), int(DLZ_coord[1] *1e7),
                            altitude)
    drop_list.append(climb_command)
    return drop_list


def payload_delivery(v, DLZ_coord):
    print("Setting up touch-and-go delivery...")
    delivery_list = []
      ## Sets up touch-and-go approach
    [delivery_list.append(x) for x in landing(v, DLZ_coord,to_heading+180, delivery=True)]

    servo_command = v.message_factory.command_long_encode(
                            0, 0,  # Target system, target component
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command ID for SET_SERVO
                            0,  # Confirmation
                            6,  # Servo output channel
                            1900,
                            0,0,0,0,0)  # PWM value (1000-2000 µs)
    delivery_list.append(servo_command)
    climb_command = v.message_factory.mission_item_int_encode(0, 0, 0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1,
                            0, 0, 0, 0,
                            int(DLZ_coord[0] *1e7), int(DLZ_coord[1] *1e7),
                            altitude)
    delivery_list.append(climb_command)
    return delivery_list


def landing(v, land_position,heading, delivery=False ):
    if not delivery:
        app_lat, app_lon = coordonate_change(land_position[0], land_position[1], to_heading+180)
        print("Returning home and landing")
        init_land_command = v.message_factory.mission_item_int_encode(0, 0, 0,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    mavutil.mavlink.MAV_CMD_DO_LAND_START,
                                    0, 1,
                                    0, 0, 0, 0, 0, 0, 0)
        # Will have to adjust lat/long to match better, but general idea
        approach_waypoint = v.message_factory.mission_item_int_encode(0, 0,0,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                    0, 1,
                                    0, 0, 0, heading,
                                    int(app_lat *1e7), int(app_lon *1e7), int(altitude/2))

        #1
        land_command = v.message_factory.mission_item_int_encode(
                        0,0, 0,  # Target system and component
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                        0, 1, 0, 0, 0, 0,  # Unused parameters
                        int(land_position[0] * 1e7),  # Convert latitude to int32
                        int(land_position[1] * 1e7),  # Convert longitude to int32
                        0 )       # Altitude as int32

    else:

        app_lat, app_lon = coordonate_change(land_position[0],land_position[1], to_heading)
        init_land_command = v.message_factory.mission_item_int_encode(0, 0, 0,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    mavutil.mavlink.MAV_CMD_DO_LAND_START,
                                    0, 1,
                                    0, 0, 0, 0, 0, 0, 0)
        # Will have to adjust lat/long to match better, but general idea
        approach_waypoint = v.message_factory.mission_item_int_encode(0, 0, 0,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                    0, 1,
                                    0, 5, 0, 0,
                                    int(app_lat *1e7), int(app_lon*1e7), 25)
        # target alt must match 0 relative to DLZ
        land_command = v.message_factory.mission_item_int_encode(0, 0, 0,
                               mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               mavutil.mavlink.MAV_CMD_NAV_LAND,
                               0, 1,
                               0, 0,0,0,
                               land_position[0], land_position[1], land_position[2])

    land_cmds = [init_land_command, approach_waypoint, land_command]
    return land_cmds


if __name__ == "__main__":
    vehicle = connect(ip=connection_string, baud=baud_rate, wait_ready=True)
    print(f'Vehicle connected on {connection_string} with baud rate {baud_rate}')

    while True:
        GPS_fix = check_GPS_status(vehicle)
        check_safety_param(vehicle)
        break


    arm_vehicle(vehicle)

    print("Plotting mission route...")
    time.sleep(1)
    cmds = []
    cmd = vehicle.commands
    cmd.clear()
    cmd.upload()
    time.sleep(2)

    # getting DLZ coordinates
    print("Please provide target GPS location (decimal) when prompted")
    target_lat = float(input("Please provide target latitude :\n"))
    target_long = float(input("Target longitude:\n"))
    target_alt = float(input("Target altitude (for payload DROP) above takeoff tarmac in meters :\n"))
    target_pos = [target_lat, target_long, target_alt]

    # getting vehicle initial position
    home_position = [vehicle.location.global_frame.lat,
                     vehicle.location.global_frame.lon]  # alt = 0 in plane RELATIVE frame
    to_heading = vehicle.heading
    # inserting Takeoff command in queue
    
    time.sleep(5)
    cmds.append(takeoff(vehicle))
    time.sleep(2)




    print("multiple delivery methods available: press 'a' for drop and 'b' for landing delivery \n")
    while True:
        if keyboard.is_pressed('a'):
            command_list = payload_drop(vehicle, target_pos)
            break
        elif keyboard.is_pressed('b'):
            command_list = payload_delivery(vehicle, target_pos)
            break

    for i in command_list:
        cmds.append(i)
        time.sleep(1)

    time.sleep(2)

    ## should add a section for pick up... but risky if autonomous

    # final landing sequence
    landing_sequence = landing(vehicle, home_position, heading=to_heading)
    for i in landing_sequence:
        cmds.append(i)
        time.sleep(1)
    print("Landing sequence added... Mission route complete and ready for review")
    # setting throttle to 0 after landing
    throttle_cut_command = vehicle.message_factory.command_long_encode(
                                   0, 0,  # Target system, target component
                                   mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # C
                                   0,  # Confirmation
                                   3,  # Servo output channel
                                   0,
                                   0,0,0,0,0)  # PWM value (1000-2000 µs)

    cmds.append(throttle_cut_command)
    batch_size = 2
    for i in range(0, len(command_list), batch_size):
            batch = command_list[i:i + batch_size]
            print(f"Uploading batch {i // batch_size + 1} with {len(batch)} commands...")
            for command in batch:
                cmd.add(command)
                print(command)
            cmd.upload(timeout= 180)
            vehicle.commands.wait_ready()
            time.sleep(2)  # Delay to allow processing
            cmd.clear()




    time.sleep(2)
    ####### Add listener for mission departure

    vehicle.mode = VehicleMode("TAKEOFF")
    while vehicle.mode!= VehicleMode("TAKEOFF"):
        time.sleep(.5)
    else:
        time.sleep(5)
        vehicle.mode = VehicleMode("AUTO")
        while vehicle.mode != VehicleMode("AUTO"):
            time.sleep(.5)

####1
#Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_ALT,mavutil.mavlink.MAV_CMD_NAV_LAND,0, 1, 0, 0,0,0,land_position[0], land_position[1], land_position[2])
