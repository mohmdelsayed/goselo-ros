#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
caffe_root = '/home/ros/caffe'  # Change this to your path.
sys.path.insert(0, caffe_root + 'python')
import caffe
import math

if True:
    _size_width = 1436 #2586
    _size_height = 553 #798
    _grid_cell_size_m = 0.05 #0.0500000007451
else:
    _size_width = 2586
    _size_height = 798
    _grid_cell_size_m = 0.0500000007451

# CNN
pycaffe_dir = caffe_root + 'python/'
center_only = True
image_dims = [224, 224]
channel_swap =  [0, 1, 2, 3, 4, 5] # [2, 1, 0]
model_def = '/home/ros/goselo-ros/src/deep_planner/src/models/deploy.prototxt'

#pretrained_model = sys.argv[ 1 ]
pretrained_model ='/home/ros/goselo-ros/src/deep_planner/src/models/goselo_invisible.caffemodel'
caffe.set_mode_gpu()

# !!!
dirs = 36

class publish_global_plan:

    def __init__(self):
        self.bridge = CvBridge()

        self.curr_map_sub = rospy.Subscriber("/map",OccupancyGrid,self.callback_map,queue_size = 1)
        self.map_sub = rospy.Subscriber("/goselo_map",Image,self.callback_goselo_map,queue_size = 1)
        self.loc_sub = rospy.Subscriber("/goselo_loc",Image,self.callback_goselo_loc,queue_size = 1)
        self.angle_sub = rospy.Subscriber("/angle",Float32,self.callback_angle,queue_size = 1)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.callback_odom,queue_size = 1)
        #self.start_sub = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,self.callback_initial,queue_size = 1) # topic subscribed from RVIZ
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.callback_goal,queue_size = 1) # topic subscribed from RVIZ
        self.global_plan_pub = rospy.Publisher('/global_plan', Path, queue_size=1)
        self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.current_x = None
        self.current_y = None
        self.goselo_map = np.zeros((1,1))
        self.goselo_loc = np.zeros((1,1))
        self._size_width = _size_width   #added
        self._size_height = _size_height  #added
        self. cell_size = _grid_cell_size_m  #added
        self.angle = 0 # made it zero instead of none
        self.curr_map = np.zeros((1,1)) #added by mohamed
        self.goal_x = 0.2
        self.dir_src = -10000 #added by mohamed
        self.goal_y = 0.2
        self.loc = np.zeros( (_size_height, _size_width), dtype=np.int )
        self.classifier = caffe.Classifier(model_def, pretrained_model, image_dims=image_dims, mean=None, input_scale=1.0, raw_scale=255.0, channel_swap=channel_swap)
        self.prev_avoid_direction = None

    def callback_map(self,data):
        self.curr_map = np.reshape(np.array(data.data), (data.info.height, data.info.width) ).astype( np.uint8 ) / 100
        self._size_width = data.info.width
        self._size_height = data.info.height
        self.cell_size = data.info.resolution
    def callback_goselo_map(self,data):
        try:
            self.goselo_map = self.bridge.imgmsg_to_cv2(data, "bgr8") / 255.
        except CvBridgeError, e:
            print e

    def callback_goselo_loc(self,data):
        try:
            self.goselo_loc = self.bridge.imgmsg_to_cv2(data, "bgr8") / 255.
        except CvBridgeError, e:
            print e
        # predict direction
        if (self.goselo_map.shape == (1,1)) or (self.goselo_loc.shape == (1,1)):
            print "nothing to do. returning .."
            return
        else:
            predictions = self.classifier.predict([np.concatenate([self.goselo_map, self.goselo_loc], 2)], not center_only)
            self.dir_src = np.argmax( predictions )
            
            dir_src = self.dir_src
            print "current direction", dir_src

            ang = 360 - 10 * dir_src - self.angle - 90
            while ang < 0:
                ang = ang + 360
            
            dir_dst = 36 - int( round( (ang % 360) / 10. ) )
            if dir_dst == 36:
                dir_dst = 0
            route = [dir_dst]
            print 'Route :', route


            # force avoidance
            avoid_flg = False
            if (self.curr_map.shape != (1,1) and self.current_x != None and self.current_y != None): # removed .all()
                x = self.current_x
                y = self.current_y
                xA = int(round(x/self.cell_size)) + _size_width/2
                yA = int(round(y/self.cell_size)) + _size_height/2
                print xA, yA
                self.loc[ yA ][ xA ] += 1
                xA_ = xA + int(math.cos( route[0] * math.pi / 18. ) * 10)
                yA_ = yA + int(math.sin( route[0] * math.pi / 18. ) * 10)
                if self.curr_map[ yA_ ][ xA_ ] :
                    if (self.prev_avoid_direction != None):
                        c = self.prev_avoid_direction
                        xA_ = xA + int(math.cos( c * math.pi / 18. ) * 10)
                        yA_ = yA + int(math.sin( c * math.pi / 18. ) * 10)
                        if not self.curr_map[ yA_ ][ xA_ ] :
                            route = [c]
                            self.prev_avoid_direction = c
                            avoid_flg = True
                            print 'Object Avoidance! Route :', route
                    else:
                        for c in range(2,36):
                            if c % 2 == 0:
                                c = route[ 0 ] + c / 2
                            else:
                                c = route[ 0 ] - (c-1) / 2
                            if c < 0:
                                c += 36
                            elif c > 35:
                                c -= 36
                            xA_ = xA + int(math.cos( c * math.pi / 18. ) * 10)
                            yA_ = yA + int(math.sin( c * math.pi / 18. ) * 10)
                            if not self.curr_map[ yA_ ][ xA_ ] :
                                route = [c]
                                self.prev_avoid_direction = c
                                avoid_flg = True
                                print 'Object Avoidance! Route :', route
                                break
                else:
                    self.prev_avoid_direction = None
            print "finished object avoidance"


            # test direction path
            _dx = math.cos( route[0] * math.pi / 18. ) * self.cell_size
            _dy = math.sin( route[0] * math.pi / 18. ) * self.cell_size

            cmd_vel = Twist()  #added
            cmd_vel.linear.x = 5*_dx  #added
            cmd_vel.linear.y = 5*_dy  #added
            self.move_robot.publish(cmd_vel)
            print "I published velocity", _dx, _dy


    def callback_angle(self,data):
        self.angle = data.data


    def callback_goal   (self,data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y

    def callback_odom   (self,data):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y


def main(args):
  pgp = publish_global_plan()
  rospy.init_node('publish_global_plan', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)



