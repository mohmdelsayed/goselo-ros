#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy
import numpy as np
import sys
import json
from math import sqrt
from collections import deque

import time

def goalCallback(data):
        global x_goal
        global y_goal
        global flag
        global path
        global current_value

        if ((abs(x_goal - data.pose.position.x) > 0.001) and  (abs(y_goal - data.pose.position.y) > 0.001)):
                print("flag is true!")
                current_value = []
        else:
                print("Flag is flase")
        x_goal = data.pose.position.x
        y_goal = data.pose.position.y

def callback(data):
        global xAnt
        global yAnt
        global cont
        global flag
        global path
        global current_value

    #To avoid repeating the values, it is found that the received values are differents
        if (abs(xAnt - data.pose.pose.position.x) > 0.0001 and abs(yAnt - data.pose.pose.position.y) > 0.0001):
                current_value.append([data.pose.pose.position.x, data.pose.pose.position.y])
                # print([data.pose.pose.position.x, data.pose.pose.position.y])
                # print(len(current_value))
                my_np = np.array(current_value).flatten()
                my_array_for_publishing = Float32MultiArray(data=my_np)
                
                path = my_array_for_publishing
                cont=cont+1

        #rospy.loginfo("Valor del contador: %i" % cont)
        if (cont>max_append and len(current_value) != 0):
        	current_value.pop(0)
        
        pub.publish(path)

    #Save the last position
        xAnt=data.pose.pose.position.x
        yAnt=data.pose.pose.position.y
        return path




if __name__ == '__main__':
        #Variable initialization
        global xAnt
        global yAnt
        global cont
        global path
        global current_value
        path = Float32MultiArray()
        xAnt=0.0
        yAnt=0.0
        cont=0
        global flag
        flag = False

        global x_goal
        global y_goal
        x_goal = 0.0
        y_goal = 0.0

        current_value = []
        #Node and msg initialization
        rospy.init_node('path_odom_plotter')

        #Rosparams that are set in the launch
        #max size of array pose msg from the path
        if not rospy.has_param("~max_list_append"):
                rospy.logwarn('The parameter max_list_append dont exists')
        max_append = rospy.get_param('~max_list_append')

        if not (max_append > 0):
                rospy.logwarn('The parameter max_list_append not is correct')
                sys.exit()
        pub = rospy.Publisher('/odompath', Float32MultiArray, queue_size=1)


        # path = Path() #creamos el mensaje path de tipo path 

        #Subscription to the topic
        rospy.Subscriber('/odom', Odometry, callback) 
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalCallback) 

        rate = rospy.Rate(30) # 30hz

try:
	while not rospy.is_shutdown():
        	#rospy.spin()
        	rate.sleep()
except rospy.ROSInterruptException:
	pass
