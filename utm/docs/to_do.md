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
-  sudo mongod --dbpath /data/db

# To do 02/25/2022 '
- [] Spawn waypoints for UAS
- [] Allow multiple port connections for Airsim API 

## Spawn Waypoints
- Using Airsim setSpawnObject -> spawns the global position in NED 
- Look ahead of waypoints and then once its passed through delete them ?

## Port connectiosn to Airsim API
- https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/64
- https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/64

