# How to Fly Multiple Drones with PX4 and Mavros with Unreal Engine

To control multiple drones in WSL2 Ubuntu terminal first launch 

```
roscore
```

run either:
```
python3 uas_sim_launch.py

rosrun utm uas_px4_main
```

or run:
```
roslaunch utm single_drone.launch

rosrun utm uas_px4_main
```

then you need to:
```
cd PX4-Autopilot
./Tools/sitl_multiple_run.sh 2    # 2 here is the number of vehicles/instances 
```

