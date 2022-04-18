# Big Blocks 
## Flight Control
- Drones fly super slow right now and waypoints are not smooth or represenative actual uav flight **this is good enough**
- Create splines for drones and parameterize as velocity input commands? 
- Might need a better controller to do this

## Aesthetics 
- Have different color waypoints wrt color of UAV
- Develop a "real world"

## Trajectory Planning
- Take in 3d waypoints 
- take in desired velocity and acceleration limits
- Minimize jerky behaviors of uav
- Use piece wise polynomials?        

# To do 02/16/2022
- [X] Control multiple UAS with simple flight
- [X] Do this by making automated CSV parser to JSON,like PX4
- [X] Test and make sure it can control all UAS with correct namespace
- [X] **Make sure the simple_flight_drone.py listens to the launch file correctly**

# Notes from 02/16/2022
- Can only control up to 14 UAS without some CPU throttling, am I capped by hardware specs?
- Optimize code

# To do
- [x] Fly multiple UAS with waypoint commands to the system 
- [x] See if I can add the hiearchial path planning to UAS

# To do 02/21/2022
- [x] Set up USS Path Planning Service 
    - [x] add uav start location and goal location into reservation table
    - [x] When I begin a path remove the uav's start and goal location from the reservation table 
    - [x] Begin path planning
    - [x] Once path is update the reservation table and add this into the reservation collection 
    - [x] if uav has arrived then we remove it  

# How to set mongodb
```
sudo mongod --dbpath /data/db
```

# To do 02/25/2022 '
- [X] Spawn waypoints for UAS

# To do 03/14/2022
- [X] Refactor PX4 Code
- [X] Include constant acceleration for simpleflight
- [] Yaw Control with Fiducial Tags 
- [X] Get heading of Apriltag
- [X] Get heading of Drone
- [X] Compute Difference then calculate desired 

# To do 04/15/2022
-[] Record rosbag of tag position
-[] Exten Kalman Filter to do a kalman filter sweep on this rosbag recording
-[] Build Q and R matrices to reduce error of kalman filter 

## Notes 3/22/2022
- Had to rotate the camera x and y
- This is a useful link to control yaw for PX4 offboard 
- https://discuss.px4.io/t/px4-rotate-drone-with-lower-angular-velocity-or-yaw-rate/24301
- 

## Port connectiosn to Airsim API
- https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/64
- https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/64

# PX4 configurations 
- launch single_drone.launch
    - this launches the following:
        - airsom_ros
        - mavros gcs_bridge
        - for drone:
            - mavros_gcs_launch for communication protocol
- run PX4_SITL
- run offboard script

# Run Computer Vision Fiducial Tags
- run image proc
- run apriltag detection
- run transformation matrix rtag_drone
    - should use mavros_vision_pose?

# Recording ros stuff
```
cd bagfiles
rosbag record -a
```
# How to replay rosbag


# Map Colors for UAS
- Red = 0
- Orange = 1
- Green = 2
- Blue = 3
- Pink = 4
- Cyan = 5
- Black = 6
- White = 7
- Yellow = 8
- Gold = 9



