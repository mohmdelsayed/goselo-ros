#!/bin/bash
rosrun rviz_tools_py markers.py &
rosrun deep_planner yaw_server.py &
roslaunch odom_to_trajectory ekf_create_trajectory.launch &
rosrun deep_planner sensors_to_map_final.py