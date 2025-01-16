import time
from dronekit import VehicleMode,LocationGlobal, Command
from pymavlink import mavutil
import threading
import sys
import keyboard

from main import check_GPS_status, arm_vehicle, connection_string, baud_rate

#set waypoint, change mode, payload operation, takeoff, land, kill

######################### PROGRAM HAS TO BE UPDATED TO WORK WITH MAVUTIL 2.0 ####################################
#################################### CURRENTLY NOT UPLOADING MISSION ##################################




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


    
