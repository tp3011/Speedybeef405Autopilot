import time
from dronekit import VehicleMode, LocationGlobal, Command, connect

from pymavlink import mavutil
import threading
import sys
import keyboard

from main import check_GPS_status, arm_vehicle, connection_string, baud_rate, check_safety_param


a = int(12.3456789 * 1e7) #### Modifier les valeurs pour les bonnes coordonnés lat gps
b= int(-98.7654321 * 1e7) #### Modifier les valeurs pour les bonnes coordonnés lon gps

Waypoint = [a,b]
# stop 1 : connection au vehicule necessaire pour entame les prochaines etapes
vehicle = connect(ip=connection_string,baud= baud_rate, wait_ready=True, timeout=90)
print(f"Vehicle connected on {connection_string} with baud rate {baud_rate}")
time.sleep(1)

print("Checking GPS and safety parameters...")
# verifie que GPS a un fix / fonctionne avant tout. verifie ensuite que tout les parametres pertinents au vol sont bien configurer
while True:
    check_GPS_status(vehicle)
    check_safety_param(vehicle)
    time.sleep(1)
    break


print("Arming vehicle...")

# on Active les systemes de vol et permet de mettre l'avion en differents mode (TAKEOFF ou AUTO)
arm_vehicle(vehicle)

print("Setting simple waypoint flight...")
# necessaire de clear la chaine de commandes pour empecher l'envoie de commandes accidentelles
cmds = vehicle.commands  # cmds est une liste []
cmds.clear()

# commande qui dit a l'avion de commencer la mission envoyee (apparemment pas necessaire avec le TAKEOFF mode car ce mode commance la mission
# automatiquement avec la presence du NAV_TAKEOFF command)
mission_start = vehicle.message_factory.command_long_encode(
    0, 0,  # Target system, target component
    mavutil.mavlink.MAV_CMD_MISSION_START,  # Command ID for mission start
    0,  # Confirmation
    0,  # Start mission from this waypoint index (default: 0)
    0, 0, 0, 0, 0, 0  # Unused parameters
)
# on stocke les coordonnees lat & lon de la position initiale sur la piste (pas besoin de alt puisque systeme de reference
# par rapport a l'altitude de Home
home_position = [vehicle.location.global_frame.lat,
                 vehicle.location.global_frame.lon]
# commande qui indique le decollage
msg = vehicle.message_factory.mission_item_int_encode(
    0, 0, 0,  # Target system, target component
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # INT frame for GPS precision
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff command
    0, 1,  # Confirmation, Auto-continue
    0, 0, 0, 0,  # Unused parameters for takeoff
    int(vehicle.location.global_frame.lat * 1e7),  # Latitude converted to int32
    int(vehicle.location.global_frame.lon * 1e7),  # Longitude converted to int32
    5  # Altitude in meters
)
# ajoute le decollage a la chaine et laisse 1 seconde pour assurer l'ajout complet
cmds.add(msg)
time.sleep(1)
# ajout d'une commande climb pour gagner de l'altitude et faire un changement de heading vers un premier waypoint
climb_command = vehicle.message_factory.mission_item_int_encode(0, 0, 0,
                                                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                                0, 1,
                                                                0, 0, 0, 0,
                                                                Waypoint[0],
                                                                Waypoint[1],
                                                            5)

cmds.add(climb_command)
time.sleep(1)
# le landing necessite une commande DO_LAND_START
init_land_command = vehicle.message_factory.mission_item_int_encode(0, 0, 0,
                                                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                                    mavutil.mavlink.MAV_CMD_DO_LAND_START,
                                                                    0, 1,
                                                                    0, 0, 0, 0, 0, 0, 0)


# initie le landing ( NAV_LAND inclut des fonctions de flare up et de slow sink rate pour ralentir la descente juste avant)
land_command = vehicle.message_factory.mission_item_int_encode(
    0, 0, 0,  # Target system and component
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 1, 0, 0, 0, 0,  # Unused parameters
    int(home_position[0] * 1e7),  # Convert latitude to int32
    int(home_position[1] * 1e7),  # Convert longitude to int32
    0)  # Altitude as int32

cmds.add(init_land_command)
time.sleep(.5)

cmds.add(land_command)
time.sleep(1)
# la commande upload permet d'envoyer la serie de commande incluse dans cmds au FC (flight controller) + 2 seconde pour upload
cmds.upload(timeout = 180)
time.sleep(2)

# mets le vehicule en mode TAKEOFF et attend la confirmation de l'avion (repete la commande juste au cas dun refus de la part du FC)
vehicle.mode = VehicleMode("TAKEOFF")
while vehicle.mode != VehicleMode("TAKEOFF"):
    time.sleep(1)

    vehicle.mode = VehicleMode("TAKEOFF")

else:
    print("Vehicle mode set to TAKEOFF")
# Donne 4 seconde de takeoff pour accelerer et get airborne
time.sleep(6)

# Ce mode permet a l'avion de suivre les commandes envoyer a l'aide de cmds.upload()
vehicle.mode = VehicleMode("AUTO")
while vehicle.mode != VehicleMode("AUTO"):
    time.sleep(0.5)

    vehicle.mode = VehicleMode("AUTO")
else:
    print("Vehicle mode set to AUTO")
print(f"Number of waypoints: {len(cmds)}")

for i, cmd in enumerate(cmds):
    print(f"Command {i}: {cmd}")



while True :
    time.sleep(5)
    
print("shutting down flight")
cmds.clear()
cmds.upload()
time.sleep(1)

vehicle.close()
sys.exit(0)

# allo
