#!/bin/bash


sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

roslaunch rosbot_description rosbot_rviz_gmapping.launch
