#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import actionlib
caffe_root = '/home/ros/caffe'  # Change this to your path.
sys.path.insert(0, caffe_root + 'python')
import caffe
import math

import roslib
roslib.load_manifest('deep_planner')
from deep_planner.msg import SetYawAction, SetYawGoal


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

        self.curr_map_sub = rospy.Subscriber("/map",OccupancyGrid,self.callback_map,queue_size = 1)
        self.map_sub = rospy.Subscriber("/goselo_map",Image,self.callback_goselo_map,queue_size = 1)
        self.loc_sub = rospy.Subscriber("/goselo_loc",Image,self.callback_goselo_loc,queue_size = 1)
        self.angle_sub = rospy.Subscriber("/angle",Float32,self.callback_angle,queue_size = 1)

        # TODO: remove odometry and use EKF instead
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.callback_odom,queue_size = 1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.callback_goal,queue_size = 1) # topic subscribed from RVIZ

        self.direction_pub = rospy.Publisher('/goselo_dir', Float32, queue_size=1)
        self.action_goal_client = actionlib.SimpleActionClient('SetYaw', SetYawAction)
        self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_flag = 0
        self.current_x = None
        self.current_y = None
        self.predictions = np.zeros((1,1))
        self.object_avoidance_range = 50
        self.object_avoidance_window = 40
        self.down_scale = 2
        self.goselo_map = np.zeros((1,1))
        self.goselo_loc = np.zeros((1,1))
        self._size_width = 0   #added
        self._size_height = 0  #added
        self.cell_size = None  #added
        self.angle = 0 # made it zero instead of none
        self.curr_map = np.zeros((1,1)) #added by mohamed
        self.goal_x = None
        self.goal_y = None
        self.dir_src = None
        self.prev_dir = None
        self.classifier = caffe.Classifier(model_def, pretrained_model, image_dims=image_dims, mean=None, input_scale=1.0, raw_scale=255.0, channel_swap=channel_swap)
        self.prev_avoid_direction = None

    def callback_map(self,data):
        self.curr_map = np.array(data.data).reshape((data.info.height, data.info.width))
        
        self._size_width = data.info.width
        self._size_height = data.info.height
        self.cell_size = data.info.resolution
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y
        self.curr_map[self.curr_map == -1] = 127
        self.curr_map = self.curr_map.astype(np.uint8)
        self.curr_map[self.curr_map == 0] = 255
        self.curr_map = self.curr_map.astype(np.uint8)
        self.curr_map[self.curr_map == 100] = 0
        self.curr_map = self.curr_map.astype(np.uint8)
        print "Received a raw map"



    def move_base(self, r_predictions):

        max_index = np.argmax( r_predictions )
        #self.dir_src = prediction
        print "max_index: ", max_index
        if self.prev_dir == None:
            self.dir_src = max_index
            self.prev_dir = max_index
        if max_index != self.prev_dir and np.abs(r_predictions[0][self.prev_dir] - r_predictions[0][max_index]) < 0.1:
                print "changing direction from  to :", max_index, self.prev_dir
                self.dir_src = self.prev_dir
        else:
            self.dir_src = max_index
            self.prev_dir = max_index
        #dir_src = prediction
        #print "current direction", dir_src
        print "Self angle from goal: ", self.angle
        
        ang = 360 - 45 * self.dir_src - self.angle - 90
        while ang < 0:
            ang = ang + 360
        print "Heading Angle: ", ang

        dir_dst = 8 - int( round( ( ang % 360) / 45. ) )
        if dir_dst == 8:
            dir_dst = 0

        route = [dir_dst]

        if (self.curr_map.shape != (1,1) and self.current_x != None and self.current_y != None and self.angle != 0 and self._size_width != 0 and self._size_height != 0 and self.cell_size != None and self.goal_x != None and self.goal_y != None):
            
            route = self.object_avoid(route[0])

            if(self.goal_x - self.current_x)*(self.goal_x - self.current_x) + (self.goal_y - self.current_y) * (self.goal_y - self.current_y) < 5*self.cell_size:
                print "I REACHED THE GOAL"
                cmd_vel_command = Twist()
                cmd_vel_command.linear.x = 0; 
                cmd_vel_command.angular.z = 0
                self.move_robot.publish(cmd_vel_command)

            else:
                self.action_goal_client.wait_for_server()
                goal = SetYawGoal()
                goal_angle = (route[0] * math.pi / 4.)
                self.direction_pub.publish(goal_angle)

                goal.desired_yaw = goal_angle
                print "goal after processing: ", goal_angle
                # Fill in the goal here
                self.action_goal_client.send_goal(goal)
                # self.action_goal_client.wait_for_result(rospy.Duration.from_sec(1.0))
                self.action_goal_client.wait_for_result()
                cmd_vel_command = Twist()
                cmd_vel_command.linear.x = 0.1
                cmd_vel_command.angular.z = 0
                self.move_robot.publish(cmd_vel_command)
        else:
            print "no goal specified"


    def object_avoid(self, direction):
        # force avoidance
        avoid_flg = False
        route = [direction]

        xA = int(round((self.current_x- self.origin_x)/(self.cell_size)))
        yA = int(round((self.current_y- self.origin_y)/(self.cell_size)))

        xA_ = xA + int(math.cos( route[0] * math.pi / 4. ) *self.object_avoidance_range)
        yA_ = yA + int(math.sin( route[0] * math.pi / 4. ) *self.object_avoidance_range)


        #########################################################
        #map_vis = np.zeros((self.curr_map.shape[0]/self.down_scale,self.curr_map.shape[1]/self.down_scale,3), np.uint8)
        #map_vis[:,:,0] = cv2.resize(self.curr_map, dsize=(self.curr_map.shape[1]/self.down_scale,self.curr_map.shape[0]/self.down_scale), interpolation=cv2.INTER_CUBIC)
        #map_vis[:,:,1] = cv2.resize(self.curr_map, dsize=(self.curr_map.shape[1]/self.down_scale,self.curr_map.shape[0]/self.down_scale), interpolation=cv2.INTER_CUBIC)
        #map_vis[:,:,2] = cv2.resize(self.curr_map, dsize=(self.curr_map.shape[1]/self.down_scale,self.curr_map.shape[0]/self.down_scale), interpolation=cv2.INTER_CUBIC)
        #cv2.circle( map_vis, (xAt, yAt), 8, (0, 0, 255), -1 )
        #cv2.circle( map_vis, (xA_t, yA_t), 8, (0, 0, 255), -1 )
        #cv2.rectangle(map_vis,(xA_t-10,yA_t-10),(xA_t+10,yA_t+10),(0,255,0),3)
        #cv2.circle( map_vis, (xB, yB), 8, (0, 0, 255), -1 )
        #cv2.imshow( 'Current Map Locations', map_vis)
        #cv2.waitKey(1)
        if 0 in self.curr_map[ yA_-(self.object_avoidance_window/2): yA_+(self.object_avoidance_window/2), xA_-(self.object_avoidance_window/2): xA_+(self.object_avoidance_window/2)]:
            print "Entered object avoidance"
            if (self.prev_avoid_direction != None):
                c = self.prev_avoid_direction
                xA_ = xA + int(math.cos( c * math.pi / 4. ) * self.object_avoidance_range)
                yA_ = yA + int(math.cos( c * math.pi / 4. ) * self.object_avoidance_range)
                if 0 not in self.curr_map[yA_-(self.object_avoidance_window/2): yA_+(self.object_avoidance_window/2), xA_-(self.object_avoidance_window/2):xA_+(self.object_avoidance_window/2)]:
                    route = [c]
                    self.prev_avoid_direction = c
                    avoid_flg = True
                    print 'Object Avoidance! Route :', route
                #for c in range(2,8):
                for c in range(route[0],route[0]+8):
                    #c = i+1
                    #if c % 2 == 0:
                        #c = route[ 0 ] + c / 2
                    #else:
                        #c = route[ 0 ] - (c-1) / 2
                    if c < 0:
                        c += 8
                    elif c > 7:
                        c -= 8
                    xA_ = xA + int(math.cos( c * math.pi / 4. ) * self.object_avoidance_range)
                    yA_ = yA + int(math.cos( c * math.pi / 4. ) * self.object_avoidance_range)
                    if 0 not in self.curr_map[ yA_-(self.object_avoidance_window/2): yA_+(self.object_avoidance_window/2), xA_-(self.object_avoidance_window/2):xA_+(self.object_avoidance_window/2)]:
                        route = [c]
                        self.prev_avoid_direction = c
                        avoid_flg = True
                        print 'Object Avoidance! Route :', route
                        break
            else:
                #for c in range(2,8):
                for c in range(route[0],route[0]+8):
                    #c = i+1
                    #if c % 2 == 0:
                        #c = route[ 0 ] + c / 2
                    #else:
                        #c = route[ 0 ] - (c-1) / 2
                    if c < 0:
                        c += 8
                    elif c > 7:
                        c -= 8
                    xA_ = xA + int(math.cos( c * math.pi / 4. ) * self.object_avoidance_range)
                    yA_ = yA + int(math.cos( c * math.pi / 4. ) * self.object_avoidance_range)
                    if 0 not in self.curr_map[ yA_-(self.object_avoidance_window/2): yA_+(self.object_avoidance_window/2), xA_-(self.object_avoidance_window/2):xA_+(self.object_avoidance_window/2)]:
                        route = [c]
                        self.prev_avoid_direction = c
                        avoid_flg = True
                        print 'Object Avoidance! Route :', route
                        break
            print "finished object avoidance"
        else:
            self.prev_avoid_direction = None
        
        return route


    def callback_goselo_map(self,data):
        print "Received a GOSELO map"

        try:
            self.goselo_map = self.bridge.imgmsg_to_cv2(data, "bgr8") / 255.
        except CvBridgeError, e:
            print e

    def callback_goselo_loc(self,data):
        print "Received Goselo Location map"
        try:
            self.goselo_loc = self.bridge.imgmsg_to_cv2(data, "bgr8") / 255.
        except CvBridgeError, e:
            print e
        # predict direction
        if (self.goselo_map.shape == (1,1)) or (self.goselo_loc.shape == (1,1)):
            print "nothing to do. returning .."
            return
        else:
            print "I entered classifier"
            self.predictions = self.classifier.predict([np.concatenate([self.goselo_map, self.goselo_loc], 2)], not center_only)
            print "prediction vector is ", self.predictions

    def callback_angle(self,data):
        self.angle = data.data


    def callback_goal   (self,data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y

    def callback_odom   (self,data):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        if (self.predictions.shape != (1,1)):
            self.move_base(self.predictions)



def main(args):
  pgp = publish_global_plan()
  rospy.init_node('goselo_network', log_level=rospy.DEBUG)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)



