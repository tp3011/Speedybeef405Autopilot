import time

import numpy as np
from numpy import cos,sin,pi
#from MainMission import  home_position, target_pos
from matplotlib import pyplot as plt


r_earth = 6378*10**3 #in meters
displacement = 75.0 #meters
def coordonate_change(old_lat, old_lon, heading, dist = displacement):
    dy,dx = calculate_translation(heading, dist)
    new_lat = old_lat + (dy / r_earth) * (180 / pi)
    new_long = old_lon + (dx / r_earth) * (180 / pi) / cos(new_lat * pi / 180)
    return  new_lat, new_long
def calculate_translation(heading, dist):
    dy = dist*cos(heading*pi/180)
    dx = dist*sin(heading*pi/180)
    return dy,dx

def plot_route(DLZ_coord, home):
    to_heading = 1
    TO_lat, TO_lon = home[0],home[1]
    CLB_lat, CLB_lon = coordonate_change(TO_lat, TO_lon, to_heading)
    APP_lat, APP_lon = coordonate_change(DLZ_coord[0],DLZ_coord[1], to_heading)
    DLZ_lat, DLZ_lon = DLZ_coord[0],DLZ_coord[1]
    CLB2_lat,CLB2_lon = coordonate_change(DLZ_lat, DLZ_lon, to_heading+180)
    APP2_lat, APP2_lon = coordonate_change(home[0], home[1], to_heading+180)
    LAND_lat, LAND_lon = home[0], home[1]
    waypoints = {"TO" : (TO_lat,TO_lon), "CLB":(CLB_lat, CLB_lon), "APP":(APP_lat, APP_lon), "DLZ": (DLZ_lat, DLZ_lon),
                     "CLB2":(CLB2_lat,CLB2_lon), "APP2" : (APP2_lat, APP2_lon), "LAND" : (LAND_lat, LAND_lon)}
    return waypoints

def draw():
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    home = [32.610003842833294, -97.4844260785071]
    target = [32.61040650223835, -97.4837747167665]
    dict = plot_route(target, home)
    # Load your PNG image (make sure the path is correct)
    print(dict)
    img = plt.imread('Compe_ground.png')

    # Define the geographic extent of the image [lon_min, lon_max, lat_min, lat_max]
    # (These values should be set based on the area your PNG covers)
    lon_min, lon_max = -97.48481770960645,-97.48366435976318   # example values
    lat_min, lat_max = 32.60903672641797, 32.61144522020954      # example values
    extent = [lon_min, lon_max, lat_min, lat_max]

    plt.figure(figsize=(8, 8))
    # Display the background image with the specified extent
    plt.imshow(img, extent=extent, origin='upper')

    # Extract latitudes and longitudes from the waypoints
    lats = [pt[0] for pt in dict.values()]
    lons = [pt[1] for pt in dict.values()]

    # Plot the route: connecting the waypoints
    plt.plot(lons, lats, marker='0', color='red', linewidth=1.5)

    # Optionally add labels for each waypoint
    for label, (lat, lon) in dict.items():
        plt.text(lon, lat, f' {label}', fontsize=9, color='blue')

    plt.title("Flight Route Overlay")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True)
    plt.show()


