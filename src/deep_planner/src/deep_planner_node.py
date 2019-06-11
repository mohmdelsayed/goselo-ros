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

caffe_root = '/home/ros/caffe'  # Change this to your path.
sys.path.insert(0, caffe_root + 'python')
import caffe
import math

import roslib
roslib.load_manifest('deep_planner')
from deep_planner.msg import SetYawAction, SetYawGoal
import copy
import timeit
import threading

# CNN
pycaffe_dir = caffe_root + 'python/'
center_only = True
image_dims = [224, 224]
channel_swap =  [0, 1, 2, 3, 4, 5]
model_def = '/home/ros/models/deploy.prototxt'
# model_def = '/home/ros/models/deploy_36.prototxt'

#pretrained_model = sys.argv[ 1 ]
pretrained_model ='/home/ros/models/goselo_invisible.caffemodel'
# pretrained_model ='/home/ros/models/model_36.caffemodel'
caffe.set_mode_gpu()

class publish_global_plan:

    def __init__(self):
        self.bridge = CvBridge()
        self.curr_locX = None
        self.curr_locY = None
        self.predictions = np.zeros((1,1))
        self.object_avoidance_range = 1.25
        self.object_avoidance_window = 70
        self.down_scale = 10
        self.goselo_map = np.zeros((1,1))
        self.goselo_loc = np.zeros((1,1))
        self._size_width = 0
        self._size_height = 0
        self.cell_size = None
        self.angle = 0
        self.curr_map_obstacle = np.zeros((1,1))
        self.goal_x = None
        self.goal_y = None
        self.dir_src = None
        self.prev_dir = None
        self.orientation = None
        self.the_map =  np.zeros((0,0))
        self.lock = threading.Lock()

        self._size_width = None
        self._size_height = None
        self.cell_size = None
        self.origin_x = None
        self.origin_y = None
        self.classifier = caffe.Classifier(model_def, pretrained_model, image_dims=image_dims, mean=None, input_scale=1.0, raw_scale=255.0, channel_swap=channel_swap)

        self.direction_pub = rospy.Publisher('/goselo_dir', Float32, queue_size=1)
        self.action_goal_client = actionlib.SimpleActionClient('SetYaw', SetYawAction)
        self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.curr_map_sub = rospy.Subscriber("/map",OccupancyGrid,self.callbackMap,queue_size = 1)
        self.map_sub = rospy.Subscriber("/goselo_map",Image,self.callback_goselo_map,queue_size = 1)
        self.loc_sub = rospy.Subscriber("/goselo_loc",Image,self.callback_goselo_loc,queue_size = 1)
        self.angle_sub = rospy.Subscriber("/angle",Float32,self.callback_angle,queue_size = 1)
        self.laserscan_sub = rospy.Subscriber("/scan",LaserScan,self.callLaserScan,queue_size = 1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.callback_goal,queue_size = 1) # topic subscribed from RVIZ


    def callLaserScan(self, data):

        if not self.lock.locked():
            self.lock.acquire() 
            try:
                (trans,orientation_q) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            except:
                print "couldn't get right transformaton"
                self.lock.release()
                return

            ranges = data.ranges
            min_ang = data.angle_min
            max_ang = data.angle_max
            inc = data.angle_increment

            orientation_list = [orientation_q[0], orientation_q[1], orientation_q[2], orientation_q[3]]
            _, _, yaw = euler_from_quaternion (orientation_list)
            current_rotation = yaw
            current_x = trans[0]
            current_y = trans[1]
            self.curr_locX = current_x
            self.curr_locY = current_y
            self.orientation = yaw
            my_map = np.zeros(self.the_map.shape)
            
            # measurements in laser scanner frame
            np_ranges = np.array(ranges)
            np_indicies = np.arange(len(ranges))


            indicies = np.isfinite(np_ranges)
            correct_values = np_ranges[indicies]
            correct_indicies = np_indicies[indicies]
            
            X_metric = np.cos(-1*min_ang + current_rotation +  correct_indicies*inc)*correct_values +current_x
            Y_metric = np.sin(-1*min_ang + current_rotation + correct_indicies*inc)*correct_values + current_y
            try:
                X = np.array(np.round((X_metric-self.origin_x)/(self.down_scale*self.cell_size)), dtype=np.uint16)
                Y = np.array(np.round((Y_metric-self.origin_y)/(self.down_scale*self.cell_size)), dtype=np.uint16)
                X_thresholded = X[X<self.the_map.shape[0]]
                Y_thresholded = Y[Y<self.the_map.shape[1]]
                my_map[Y_thresholded,X_thresholded] = 1
                self.curr_map_obstacle = my_map                
                
            except:
                rospy.logwarn("Cannot process laser map")
                self.lock.release()
                return
    

            self.lock.release()

        else:
            print "I am blocked"
            return

    def callbackMap(self,data):

        '''
        # The map data, in row-major order, starting with (0,0).  Occupancy
        # probabilities are in the range [0,100].  Unknown is -1.
        int8[] data
        '''
        self._size_width = data.info.width
        self._size_height = data.info.height
        self.cell_size = data.info.resolution
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y
        self.the_map = np.zeros((data.info.height/self.down_scale, data.info.width/self.down_scale))


    def move_base(self, r_predictions):

        max_index = np.argmax( r_predictions )
        #self.dir_src = prediction
        # print "max_index: ", max_index
        if self.prev_dir == None:
            self.dir_src = max_index
            self.prev_dir = max_index
        if max_index != self.prev_dir and np.abs(r_predictions[0][self.prev_dir] - r_predictions[0][max_index]) < 0.1:
                # print "changing direction from  to :", max_index, self.prev_dir
                self.dir_src = self.prev_dir
        else:
            self.dir_src = max_index
            self.prev_dir = max_index
        #dir_src = prediction
        #print "current direction", dir_src
        # print "Self angle from goal: ", self.angle
        resolution = 45
        n_directions = 8
        ang = 360 - resolution * self.dir_src - self.angle - 90
        while ang < 0:
            ang = ang + 360
        # print "Heading Angle: ", ang

        dir_dst = n_directions - int( round( ( ang % 360) / resolution ) )
        if dir_dst == n_directions:
            dir_dst = 0

        route = dir_dst

        if (self.curr_map_obstacle.shape != (1,1) and self.curr_locX != None and self.curr_locY != None and self.angle != 0 and self._size_width != 0 and self._size_height != 0 and self.cell_size != None and self.goal_x != None and self.goal_y != None):
            
            if(self.goal_x - self.curr_locX)*(self.goal_x - self.curr_locX) + (self.goal_y - self.curr_locY) * (self.goal_y - self.curr_locY) < 5*self.cell_size:
                print "I REACHED THE GOAL"
                cmd_vel_command = Twist()
                cmd_vel_command.linear.x = 0; 
                cmd_vel_command.angular.z = 0
                self.move_robot.publish(cmd_vel_command)
                return
            else:
                modified_route = self.object_avoid(route)
                # self.action_goal_client.wait_for_server()
                goal = SetYawGoal()
                goal_angle = (modified_route * math.pi / 4.)
                self.direction_pub.publish(goal_angle)

                goal.desired_yaw = goal_angle
                # print "goal after processing: ", goal_angle
                # Fill in the goal here
                self.action_goal_client.send_goal(goal)
                self.action_goal_client.wait_for_result()
                cmd_vel_command = Twist()
                cmd_vel_command.linear.x = 0.25
                cmd_vel_command.angular.z = 0
                self.move_robot.publish(cmd_vel_command)
        else:
            rospy.logwarn("No goal specified")

    def IsNotColliding(self, route):

        modified_x = self.curr_locX + int(math.cos( route * math.pi / 4.) * self.object_avoidance_range)
        modified_y = self.curr_locY + int(math.sin( route * math.pi / 4.) * self.object_avoidance_range)
        n_directions = 8
        route = (route + 6) % n_directions 
        xA_ = int(round((modified_x - self.origin_x)/(self.cell_size)))
        yA_ = int(round((modified_y - self.origin_y)/(self.cell_size)))
        yMin = yA_-(self.object_avoidance_window//2)
        yMax = yA_+(self.object_avoidance_window//2)
        xMin = xA_-(self.object_avoidance_window//2)
        xMax = xA_+(self.object_avoidance_window//2)
        interest_region = self.curr_map_obstacle[yMin/self.down_scale:yMax/self.down_scale,xMin/self.down_scale:xMax/self.down_scale].copy()

        my_map = self.curr_map_obstacle.copy()
        my_map[yMin/self.down_scale:yMax/self.down_scale,xMin/self.down_scale:xMax/self.down_scale] = 1
        cv2.imshow( 'Avoidance map', my_map*255)
        cv2.waitKey(1)
        # cv2.imshow( 'Window', interest_region*255)
        # cv2.waitKey(1)

        if np.any(interest_region > 0.5):
            return False
        else:
            return True

    def possible_dirs(self, direction):
        goselo_dirs = [0, 1, 2, 3, 4, 5, 6, 7]
        direction_index = goselo_dirs.index(direction)
        myList = []
        n_directions = 8
        for i in range(1,10):
            if not goselo_dirs[direction_index - i] in myList:
                myList.append(goselo_dirs[direction_index - i])
            if not goselo_dirs[(direction_index + i) % n_directions] in myList:
                myList.append(goselo_dirs[(direction_index + i) % n_directions])
            if  goselo_dirs[direction_index - i] == goselo_dirs[(direction_index + i) % n_directions]:
                break
        return myList

    def object_avoid(self, route):
        print "Entered Object Avoidance"
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

    def callback_goselo_map(self,data):
        #print "Received a GOSELO map"

        try:
            self.goselo_map = self.bridge.imgmsg_to_cv2(data, "bgr8") / 255.
        except CvBridgeError, e:
            print e

    def callback_goselo_loc(self,data):
        #print "Received Goselo Location map"
        try:
            self.goselo_loc = self.bridge.imgmsg_to_cv2(data, "bgr8") / 255.
        except CvBridgeError, e:
            print e
        # predict direction
        if (self.goselo_map.shape == (1,1)) or (self.goselo_loc.shape == (1,1)):
            return
        else:
            #print "I entered classifier"
            self.predictions = self.classifier.predict([np.concatenate([self.goselo_map, self.goselo_loc], 2)], not center_only)
            #print "prediction vector is ", self.predictions

    def callback_angle(self,data):
        self.angle = data.data

        if (self.predictions.shape != (1,1)):
            self.move_base(self.predictions)

    def callback_goal   (self,data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y

if __name__ == '__main__':
  rospy.init_node('goselo_network')
  listener = tf.TransformListener()
  pgp = publish_global_plan()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()