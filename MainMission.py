import time
from dronekit import VehicleMode,LocationGlobal, Command
from pymavlink import mavutil
import threading
import sys
import keyboard

from main import vehicle

#set waypoint, change mode, payload operation, takeoff, land, kill

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

def payload_drop(v):
    print("Setting up payload drop... Please provide target GPS location (decimal) when prompted")
    target_lat = float(input("Please provide target latitude :\n"))
    target_long = float(input("Target longitude:\n"))
    target_alt = float(input("Height above target altitude (for payload DROP):\n"))
    target_pos = [target_lat, target_long,target_alt]
    drop_list=[]

    drop_waypoint = Command(0,0,0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, ##alt above sea level
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0,1,
                            0,0,0,0,
                            target_lat,target_long,
                            target_alt)
    drop_list.append(drop_waypoint)
    servo_command = Command(0,0,0,
                            mavutil.mavlink.MAV_FRAME_MISSION,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0,1,9,      ##last digit represents servo id (change if required)
                            1900,       ##pwm
                            0, 0, 0, 0, 0)
    drop_list.append(servo_command)
    climb_command = Command(0,0,0,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, ##alt above sea level
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0,1,
                            0,0,0,0,
                            target_lat,target_long,
                            50)
    drop_list.append(climb_command)
    return drop_list


def payload_delivery(v):
    pass

def landing(v, delivery=False, land_position, tkoff_heading =0):
    
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
                               mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               mavutil.mavlink.MAV_CMD_NAV_LAND,
                               0,1,0,0,
                               land_position[0],land_position[1],land_position[2])
    
    land_cmds = [init_land_command, approach_waypoint, land_command]
                                    
        
if __name__ == "__main__":
    cmds = vehicle.commands
    cmds.clear()

    #getting vehicle initial position (imsure theres a better way)
    home_position = [vehicle.location.global_frame.lat,vehicle.location.global_frame.lon]
    #inserting Takeoff command in queue
    cmds.add(takeoff(vehicle))

    print("multiple delivery methods available: press 'a' for drop and 'b' for landing delivery")
    while True:
        if keyboard.is_pressed('a'):
            command_list = payload_drop(vehicle)
            break
        elif keyboard.is_pressed('b'):

            break
        time.sleep(0.1)
    for i in command_list:
        cmds.add(i)
        print(f"added {i} to list ")
