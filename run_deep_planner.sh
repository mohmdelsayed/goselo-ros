#!/bin/bash
rosrun rviz_tools_py markers.py &
roslaunch odom_to_trajectory create_trajectory.launch &
rosrun deep_planner sensors_to_map_final.py 
