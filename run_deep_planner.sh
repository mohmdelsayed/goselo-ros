#!/bin/bash

rosrun deep_planner sensors_to_map_updated.py &
roslaunch odom_to_trajectory create_trajectory.launch
#rosrun odom_to_trajectory deep_planner.py


