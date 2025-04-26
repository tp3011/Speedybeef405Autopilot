import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading
import sys
import keyboard

connection_string = "COM7"#"udp:127.0.0.1:14551"
baud_rate = 57600
print("This program works by iteration for now. Follow the instructions to complete the arming of the vehicle.\n"
      "Press 'c' at any time to shutdown.")
time.sleep(1)
def check_GPS_status(vehicle):
    while vehicle.gps_0.fix_type == 1:
        print("Waiting for fix...")
        time.sleep(5)
        if vehicle.gps_0.fix_type==2:
            print("GPS signal acquired")
            return True


def check_safety_param(vehicle):
    vehicle.parameters["ARMING_CHECK"]=332
    vehicle.parameters["RTL_ALT"] = int(0)
    vehicle.parameters["ARMING_REQUIRE"] = 0
    vehicle.parameters["WP_RETURN_AFTER"] = 0
    vehicle.parameters["ARSPD_USE"] = 0
    vehicle.parameters['TKOFF_THR_MINACC'] = 0
    vehicle.parameters['TKOFF_THR_DELAY'] = 0
    vehicle.parameters['THR_MAX'] = 80
    vehicle.parameters['TKOFF_THR_MAX'] = 95
    vehicle.parameters["TKOFF_LVL_ALT"] = 10
    vehicle.parameters["TKOFF_LVL_PITCH"] = 15
    vehicle.parameters["PTCH_LIM_MAX_DEG"] = 15
    vehicle.parameters["PTCH_LIM_MIN_DEG"] = -15
    vehicle.parameters["ROLL_LIMIT_DEG"] = 15
    vehicle.parameters["RTL_AUTOLAND"] = 2
    time.sleep(3)

    while vehicle.parameters["ARMING_CHECK"] != 332:
        print("Waiting for arming checks to disable... Press 'c' if UNINTENDED")
        time.sleep(5)
    else:
        print("Arming check disabled... Checking GPS status")
        while vehicle.gps_0.fix_type != 2:
            break
            time.sleep(2)
        else: print("GPS signal acquired. Ready to arm vehicle...")

def check_control_surfaces(vehicle):
    print("Testing control surfaces one by one...\n Testing roll right/left")
    vehicle.channels.overrides['1'] = vehicle.parameters['SERVO1_MAX']
    time.sleep(1)
    vehicle.channels.overrides['1'] = vehicle.parameters['SERVO1_MIN']
    time.sleep(1)
    vehicle.channels.overrides['1'] = vehicle.parameters['SERVO1_TRIM']
    print("Testing pitch up/down")
    vehicle.channels.overrides['2'] = vehicle.parameters['SERVO2_MAX']
    time.sleep(1)
    vehicle.channels.overrides['2'] = vehicle.parameters['SERVO2_min']
    time.sleep(1)
    vehicle.channels.overrides['2'] = vehicle.parameters['SERVO2_TRIM']
    print("Testing rudder")
    vehicle.channels.overrides['4'] = vehicle.parameters['SERVO4_MAX']
    time.sleep(1)
    vehicle.channels.overrides['4'] = vehicle.parameters['SERVO4_min']
    time.sleep(1)
    print("Control surface tests done")




def arm_vehicle(vehicle):

    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode != VehicleMode("AUTO"):
        time.sleep(.5)
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting to arm")
    else:
        print("Vehicle is armed and ready...")

def listen_for_shutdown():
    while True:
        if keyboard.is_pressed("c"):
            print("Exiting program through shutdown key...")
            keyboard.unhook_all_hotkeys()
            try: vehicle.close()
            except NameError:
                print("boo")
            break

    sys.exit(0)


if __name__ == "__main__":
    print("This program works by iteration for now. Follow the instructions to complete the arming of the vehicle.\n"
          "Press 'c' at any time to shutdown.")

    listener_thread = threading.Thread (target=listen_for_shutdown, daemon=True)
    listener_thread.start()
    shutdown_event = threading.Event()
    print("press 'f' to connect to vehicle")
    while not keyboard.is_pressed('f'):
        time.sleep(0.05)

    #code to connect to plane

    print("Confirmation received! Starting connection...")

    vehicle = connect(ip=connection_string, baud = baud_rate, wait_ready=True)
    print("Connection successful")

    print("press 's' to disable safety checks... FOR TESTING ONLY. Press 'c' to exit test")
    keyboard.add_hotkey("s", lambda: check_safety_param(vehicle))
    keyboard.wait()

    print("Press 't' to test control surfaces. Press 'p' to skip testing")
    while True:
        if keyboard.is_pressed('t'):
            check_control_surfaces(vehicle)
            break
        elif keyboard.is_pressed('p'):
            print("Skipping testing")
            break

    print("press 'a' to arm vehicle... MAKE SURE TO COMPLY WITH SAFETY REGULATIONS")
    keyboard.add_hotkey("a", lambda : arm_vehicle(vehicle))














