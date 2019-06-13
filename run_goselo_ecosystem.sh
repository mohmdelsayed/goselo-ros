#!/bin/bash
roslaunch odom_to_trajectory ekf_create_trajectory.launch &
rosrun rviz_tools_py markers.py &
rosrun deep_planner goselo_network.py &
rosrun deep_planner laser_to_directions.py &
rosrun deep_planner sensors_to_map.py