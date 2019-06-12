#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from tf.transformations import euler_from_quaternion
import tf
import math
from std_msgs.msg import Float32MultiArray

import roslib
roslib.load_manifest('deep_planner')
from deep_planner.msg import SetYawAction, SetYawGoal
import copy
import timeit
import threading


class publish_global_plan:

    def __init__(self):
        self.bridge = CvBridge()
        self.curr_X = None
        self.curr_Y = None
        self.object_avoidance_range = 2.0
        self.down_scale = 10

        self.goal_x = None
        self.goal_y = None
        self.the_map =  np.zeros((0,0))
        self.lock = threading.Lock()
        self.obstacles_directions = []

        self.GOSELO_Dir = None
        self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.direction_pub = rospy.Publisher('/dir_corrected', Float32, queue_size=1)
        self.direction_sub = rospy.Subscriber('/goselo_dir', Float32, self.callGOSELO_Dir, queue_size=1)
        self.direction_GOSELO_pub = rospy.Publisher('/goselo_direction', Float32, queue_size=1)
        self.obstacles_sub = rospy.Subscriber('/obstacles', Float32MultiArray, self.callObstacles, queue_size=1)
        self.action_goal_client = actionlib.SimpleActionClient('SetYaw', SetYawAction)
        self.goal_s = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.callbackGoal,queue_size = 1)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.callOdom,queue_size = 1)

    def callOdom(self, data):
        self.curr_X = data.pose.pose.position.x
        self.curr_Y = data.pose.pose.position.y

    def callObstacles(self, data):
        input_data = np.asarray(data.data)
        self.obstacles_directions = input_data
        if (self.GOSELO_Dir != None):
            self.move_base(self.GOSELO_Dir)
        else:
            rospy.logwarn("No GOSELO Direction!")

    def callGOSELO_Dir(self, data):
        self.GOSELO_Dir = data.data

    def move_base(self, route):
        if (self.curr_X != None and self.goal_x != None):
            
            if(self.goal_x - self.curr_X)*(self.goal_x - self.curr_X) + (self.goal_y - self.curr_Y) * (self.goal_y - self.curr_Y) < 0.05:
                print "I REACHED THE GOAL"
                cmd_vel_command = Twist()
                cmd_vel_command.linear.x = 0; 
                cmd_vel_command.angular.z = 0
                self.move_robot.publish(cmd_vel_command)
                self.goal_x = None
                return
            else:
                modified_route = self.object_avoid(route)
                #modified_route = route
                self.direction_pub.publish(modified_route)
                self.direction_GOSELO_pub.publish(route)
                goal = SetYawGoal()
                goal_angle = (modified_route * math.pi / 4.)
                goal.desired_yaw = goal_angle
                # print "goal after processing: ", goal_angle
                # Fill in the goal here
                self.action_goal_client.send_goal(goal)
                self.action_goal_client.wait_for_result()
                cmd_vel_command = Twist()
                cmd_vel_command.linear.x = 0.3
                cmd_vel_command.angular.z = 0
                self.move_robot.publish(cmd_vel_command)
        else:
            rospy.logwarn("No goal specified")
            cmd_vel_command = Twist()
            cmd_vel_command.linear.x = 0; 
            cmd_vel_command.angular.z = 0
            self.move_robot.publish(cmd_vel_command)

    def IsNotColliding(self, route):
        if len(self.obstacles_directions) > 0:
            if route in self.obstacles_directions:
                return False
            else:
                return True
        else:
            return True

    def possible_dirs(self, direction):
        n_directions = 8
        goselo_dirs =  list(np.arange(n_directions))
        direction_index = goselo_dirs.index(direction)
        myList = []
        for i in range(1,50):
            if not goselo_dirs[direction_index - i] in myList:
                myList.append(goselo_dirs[direction_index - i])
            if not goselo_dirs[(direction_index + i) % n_directions] in myList:
                myList.append(goselo_dirs[(direction_index + i) % n_directions])
            if  goselo_dirs[direction_index - i] == goselo_dirs[(direction_index + i) % n_directions]:
                break
        return myList

    def object_avoid(self, route):
        if self.IsNotColliding(route):
            rospy.loginfo('Original route does not collide: ' + str(route))
            return route
        else:
            for c in self.possible_dirs(route):
                if self.IsNotColliding(c):
                    new_route = c
                    rospy.logwarn('Object Avoidance! New Route: ' + str(new_route))
                    return new_route
            rospy.logerr("All routes are colliding")
            return route

    def callbackGoal   (self,data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y


if __name__ == '__main__':
  rospy.init_node('robot_mover')
  listener = tf.TransformListener()
  pgp = publish_global_plan()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()