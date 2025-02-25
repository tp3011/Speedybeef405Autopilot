# Speedybeef405Autopilot
Code that sets up, calibrates, arms and launches a delivery and capture type mission involving the ardupilot "plane" software

Pertinent info:
Relies on MAVlink 2.0, which might necessitate an adjustement to command function. Otherwise, will receive a "GCS should send mission item int" warning.
simpleMission provides a valid SITL (Simulation In The Loop) mission to test how Ardupilot will react to new commands. 

To do :
Mission waypoints should be automatically updated according to both DLZ coordinates and Home (takeoff) position as to minimize user input and its associated errors. \n
Have to implement a KILL function that cuts throttle and automatically sends the plane to a safe distance from the takeoff location. 
Connect to SpeedyBee F405 wing with code to send mission wirelessly. (Telemetry or wifi also works; to see which is better)
Obtain key data to display on GCS at a set interval of time, depending on data set (altitude updated in real time, where as gps position every half-second, for example)

Would be interesting to:
Find better GPS module 

