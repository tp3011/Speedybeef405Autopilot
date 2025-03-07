import time
from dronekit import VehicleMode,LocationGlobal, Command
from pymavlink import mavutil
import threading
import sys
import keyboard

from main import check_GPS_status, arm_vehicle, connection_string, baud_rate

#set waypoint, change mode, payload operation, takeoff, land, kill

## le but de kill : en cas de perte de controle, redirige l'avion vers un point loin de la foule. 
######################### PROGRAM HAS TO BE UPDATED TO WORK WITH MAVUTIL 2.0 ####################################
#################################### CURRENTLY NOT UPLOADING MISSION ##################################


def KILL(v):
    
    pass

def takeoff(v): #function that sets up take off for a given pitch angle
    vehicle.parameters["TKOFF_THR_MINACC"] = 0
    vehicle.parameters["ARSPD_USE"] = 0
    
    if not v.mode == VehicleMode("AUTO"):
        v.mode =VehicleMode("AUTO")
        while v.mode != VehicleMode("AUTO"):
            time.sleep(0.05)
    else: print("vehicle in AUTO mode. Setting takeoff parameters")
    pitch_ang = float(input("Set max pitch angle"))
    takeoff_cmd = Command(0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 1,
    pitch_ang, 0, 0, 0,  # param1 = pitch, etc., set as needed
    vehicle.location.global_frame.lat,
    vehicle.location.global_frame.lon,
    50)           # target altitude
    return takeoff_cmd

def transit(v): #creates a transit command (go from point a to b). This doesnt serve any other purpose than to reposition plane
    pass

def payload_drop(v, DLZ_coord):
    print("Setting up payload drop... Please provide target GPS location (decimal) when prompted")
    
    drop_list=[]

    drop_waypoint = Command(0,0,0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_ALT, ##alt above sea level
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0,1,
                            0,0,0,0,
                            DLZ_coord[0],DLZ_coord[1],
                            DLZ_coord[3]+1)
    drop_list.append(drop_waypoint)
    servo_command = Command(0,0,0,
                            mavutil.mavlink.MAV_FRAME_MISSION,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0,1,9,      ##last digit represents servo id (change if required)
                            1900,       ##pwm
                            0, 0, 0, 0, 0)
    drop_list.append(servo_command)
    climb_command = Command(0,0,0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0,1,
                            0,0,0,0,
                            DLZ_coord[0],DLZ_coord[1],
                            50)
    drop_list.append(climb_command)
    return drop_list


def payload_delivery(v, DLZ_coord):
    print("Setting up touch-and-go delivery...")
    delivery_list = []
    landing(v, DLZ_coord, delivery = True)  ## Sets up touch-and-go approach
    delivery_list.append(landing)
    
    servo_command = Command(0,0,0,
                            mavutil.mavlink.MAV_FRAME_MISSION,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0,1,9,      ##last digit represents servo id (change if required)
                            1900,       ##pwm reprensenting OPEN
                            0, 0, 0, 0, 0)
    delivery_list.append(servo_command)
    climb_command = Command(0,0,0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0,1,
                            0,0,0,0,
                            DLZ_coord[0],DLZ_coord[1],
                            50)
    delivery_list.append(climb_command)
    return delivery_list
    

def landing(v, land_position, delivery = False, tkoff_heading = 0):
    
    if not delivery:
        print("Returning home and landing")
        init_land_command = Command(0,0,0,
                               mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               mavutil.mavlink.MAV_CMD_DO_LAND_START,
                               0,1,
                               0,0,0,0,0,0,0)
        #Will have to adjust lat/long to match better, but general idea
        approach_waypoint = Command(0,0,0,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                    0,1,
                                    0,5,0,tkoff_heading,
                                    32.59204,-97.48064, 25)
        
        land_command = Command(0,0,0,
                               mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               mavutil.mavlink.MAV_CMD_NAV_LAND,
                               0,0,0,tkoff_heading,
                               land_position[0],land_position[1],0)
        
    else:
        init_land_command = Command(0,0,0,
                               mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               mavutil.mavlink.MAV_CMD_DO_LAND_START,
                               0,1,
                               0,0,0,0,0,0,0)
        #Will have to adjust lat/long to match better, but general idea
        approach_waypoint = Command(0,0,0,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                    0,1,
                                    0,5,0,0,
                                    32.59204,-97.48064, 25)
        #target alt must match 0 relative to DLZ
        land_command = Command(0,0,0,
                               mavutil.mavlink.MAV_FRAME_GLOBAL_ALT,
                               mavutil.mavlink.MAV_CMD_NAV_LAND,
                               0,1,0,0,
                               land_position[0],land_position[1],land_position[2])
    
    land_cmds = [init_land_command, approach_waypoint, land_command]
    return land_cmds
                                    
        
if __name__ == "__main__":
    vehicle = connect(ip = main.connection_string, baud = main.baud_rate, wait_ready = True)
    print(f'Vehicle connected on {main.connection_string} with baud rate {main.baud_rate}')
    
    while True:
        GPS_fix = main.check_GPS_status(vehicle)
        break
    
    if GPS_fix == True:
        main.arm_vehicle(vehicle)

    print("Plotting mission route...")
    time.sleep(1)
    cmds = vehicle.commands
    cmds.clear()

    #getting DLZ coordinates
    print("Please provide target GPS location (decimal) when prompted")
    target_lat = float(input("Please provide target latitude :\n"))
    target_long = float(input("Target longitude:\n"))
    target_alt = float(input("Target altitude (for payload DROP) above sea level in meters :\n"))
    target_pos = [target_lat, target_long, target_alt]
    
    #getting vehicle initial position (im sure theres a better way)
    home_position = [vehicle.location.global_frame.lat,vehicle.location.global_frame.lon] # alt = 0 in plane RELATIVE frame
    #inserting Takeoff command in queue
    cmds.add(takeoff(vehicle))

    print("multiple delivery methods available: press 'a' for drop and 'b' for landing delivery \n")
    while True:
        if keyboard.is_pressed('a'):
            command_list = payload_drop(vehicle, target_pos)
            break
        elif keyboard.is_pressed('b'):
            command _list = payload_delivery(vehicle, target_pos)
            break
        time.sleep(0.1)
    for i in command_list:
        cmds.add(i)
        print(f"added {i} to queue")

    ## should add a section for pick up... but risky if autonomous


    #final landing sequence
    landing_sequence = landing(vehicle, home_position)
    for i in landing_sequence:
        cmds.add(i)
    finally : print("Landing sequence added... Mission route complete and ready for review")
    #setting throttle to 0 after landing 
    throttle_cut_command = Command(0,0,0,
                            mavutil.mavlink.MAV_FRAME_MISSION,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0,1,3,      ##last digit represents servo id (change if required)
                            0,       ##pwm reprensenting off
                            0, 0, 0, 0, 0)
    cmds.add(throttle_cut_command)
    print(cmds)

"""import time
from dronekit import VehicleMode, LocationGlobal, Command, connect
from numpy import pi, cos, sin

from pymavlink import mavutil
import threading
import sys
import keyboard

from main import check_GPS_status, arm_vehicle, connection_string, baud_rate, check_safety_param

r_earth = 6378*10**3 #in meters
displacement = 75.0 #meters
# set waypoint, change mode, payload operation, takeoff, land, kill

def coordonate_change(old_lat, old_lon, dy, dx):
    new_lat = old_lat + (dy / r_earth) * (180 / pi)
    new_long = old_lon + (dx / r_earth) * (180 / pi) / cos(new_lat * pi / 180)
    return  new_lat, new_long

def takeoff(v):  # function that sets up take off




    takeoff_cmd = v.message_factory.mission_item_int_encode(
                          0, 0,0,  # Target system, target component
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # INT frame for GPS precision
                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff command
                          0, 1,  # Confirmation, Auto-continue
                          0, 0, 0, 0,  # Unused parameters for takeoff
                          int(vehicle.location.global_frame.lat * 1e7),  # Latitude converted to int32
                          int(vehicle.location.global_frame.lon * 1e7),  # Longitude converted to int32
                          25  # Altitude in meters
                      )
    return takeoff_cmd


def transit(
        v):  # creates a transit command (go from point a to b). This doesnt serve any other purpose than to reposition plane
    pass


def payload_drop(v, DLZ_coord):
    print("Setting up payload drop... Please provide target GPS location (decimal) when prompted")

    drop_list = []

    drop_waypoint = v.message_factory.mission_item_int_encode(0, 0, 0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  ##alt above sea level
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1,
                            0, 0, 0, 0,
                            int(DLZ_coord[0] *1e7), int(DLZ_coord[1] *1e7),
                            int(DLZ_coord[2] + 1))
    drop_list.append(drop_waypoint)
    servo_command = v.message_factory.command_long_encode(
                            0, 0,  # Target system, target component
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command ID for SET_SERVO
                            0,  # Confirmation
                            9,  # Servo output channel
                            1900,
                               0,0,0,0,0)  # PWM value (1000-2000 µs)
    drop_list.append(servo_command)
    climb_command = v.message_factory.mission_item_int_encode(0, 0, 0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1,
                            0, 0, 0, 0,
                            int(DLZ_coord[0] *1e7), int(DLZ_coord[1] *1e7),
                            50)
    drop_list.append(climb_command)
    return drop_list


def payload_delivery(v, DLZ_coord):
    print("Setting up touch-and-go delivery...")
    delivery_list = []
    landing(v, DLZ_coord, delivery=True)  ## Sets up touch-and-go approach
    delivery_list.append(landing)

    servo_command = Command(0, 0, 0,
                            mavutil.mavlink.MAV_FRAME_MISSION,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0, 1, 9,  ##last digit represents servo id (change if required)
                            1900,  ##pwm reprensenting OPEN
                            0, 0, 0, 0, 0)
    delivery_list.append(servo_command)
    climb_command = Command(0, 0, 0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1,
                            0, 0, 0, 0,
                            DLZ_coord[0], DLZ_coord[1],
                            50)
    delivery_list.append(climb_command)
    return delivery_list


def landing(v, land_position, delivery=False, heading=0):
    if not delivery:
        
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
                                    int(45.51689761061211 *1e7), int(-73.7947726427685*1e7), 25)

        #1
        land_command = v.message_factory.mission_item_int_encode(
                        0,0, 0,  # Target system and component
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                        0, 1, 0, 0, 0, 0,  # Unused parameters
                        int(land_position[0] * 1e7),  # Convert latitude to int32
                        int(land_position[1] * 1e7),  # Convert longitude to int32
                        0  )       # Altitude as int32

    else:
        ###calculating approach waypoints
        x=displacement*sin(to_heading *pi/180)
        y = displacement*cos(to_heading *pi/180)
        app_lat, app_lon = coordonate_change(land_position[0],land_position[1], x, y)
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
                               mavutil.mavlink.MAV_FRAME_GLOBAL_ALT,
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
    target_lat = 45.51154476553373#float(input("Please provide target latitude :\n"))
    target_long = -73.76447441328253#float(input("Target longitude:\n"))
    target_alt = 25#float(input("Target altitude (for payload DROP) above sea level in meters :\n"))
    target_pos = [target_lat, target_long, target_alt]
    
    # getting vehicle initial position
    home_position = [vehicle.location.global_frame.lat,
                     vehicle.location.global_frame.lon]  # alt = 0 in plane RELATIVE frame
    to_heading = vehicle.heading
    # inserting Takeoff command in queue

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
                                   9,  # Servo output channel
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


    vehicle.mode = VehicleMode("TAKEOFF")
    while vehicle.mode!= VehicleMode("TAKEOFF"):
        time.sleep(.5)
    else:
        time.sleep(5)
        vehicle.mode = VehicleMode("AUTO")
        while vehicle.mode != VehicleMode("AUTO"):
            time.sleep(.5)

####1
#Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_ALT,mavutil.mavlink.MAV_CMD_NAV_LAND,0, 1, 0, 0,0,0,land_position[0], land_position[1], land_position[2])"""
    
