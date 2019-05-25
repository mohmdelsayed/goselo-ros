#!/bin/bash
roslaunch odom_to_trajectory create_trajectory.launch &
rosrun deep_planner sensors_to_map_updated.py 