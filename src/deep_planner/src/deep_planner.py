#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
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
model_def = '/home/ros/yara_ws/src/deep_planner/src/models/deploy.prototxt'
#pretrained_model = sys.argv[ 1 ]
pretrained_model ='/home/ros/yara_ws/src/deep_planner/src/models/goselo_invisible.caffemodel'
#caffe.set_mode_gpu()

# !!!
dirs = 36

class publish_global_plan:

    def __init__(self):
        self.bridge = CvBridge()

        self.curr_map_sub = rospy.Subscriber("/current_map",OccupancyGrid,self.callback0,queue_size = 1)
        self.map_sub = rospy.Subscriber("/goselo_map",Image,self.callback1,queue_size = 1)
        self.loc_sub = rospy.Subscriber("/goselo_loc",Image,self.callback2,queue_size = 1)
        self.angle_sub = rospy.Subscriber("/angle",Float32,self.callback3,queue_size = 1)
        self.start_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.callback4,queue_size = 1) # topic subscribed from RVIZ
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.callback5,queue_size = 1) # topic subscribed from RVIZ


        self.global_plan_pub = rospy.Publisher('/global_plan', Path, queue_size=1)
        self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goselo_map = np.zeros((1,1))
        self.goselo_loc = np.zeros((1,1))
        #self._size_width = _size_width   #added
        #self._size_height = _size_height  #added
        self. cell_size = _grid_cell_size_m  #added
        self.angle = 0 # made it zero instead of none
        self.curr_map = np.zeros((1,1)) #added by mohamed
        self.goal_x = 0.2
        self.dir_src = -10000 #added by mohamed
        self.goal_y = 0.2
        self.loc = np.zeros( (_size_height, _size_width), dtype=np.int )
        self.classifier = caffe.Classifier(model_def, pretrained_model, image_dims=image_dims, mean=None, input_scale=1.0, raw_scale=255.0, channel_swap=channel_swap)
        self.prev_avoid_direction = None

    def callback0(self,data):
        self.curr_map = np.reshape(np.array(data.data), (data.info.height, data.info.width) ).astype( np.uint8 ) / 100
        #self._size_width = data.info.width
        #self._size_height = data.info.height
        self.cell_size = data.info.resolution
    def callback1(self,data):
        try:
            self.goselo_map = self.bridge.imgmsg_to_cv2(data, "bgr8") / 255.
        except CvBridgeError, e:
            print e

    def callback2(self,data):
        try:
            self.goselo_loc = self.bridge.imgmsg_to_cv2(data, "bgr8") / 255.
        except CvBridgeError, e:
            print e
        # predict direction
        predictions = self.classifier.predict([np.concatenate([self.goselo_map, self.goselo_loc], 2)], not center_only)
        self.dir_src = np.argmax( predictions )

    def callback3(self,data):
        self.angle = data.data

    def callback4(self,data):
        print "started long callback"
        if (self.goselo_map.shape == (1,1)) or (self.goselo_loc.shape == (1,1)):
            print "nothing to do. returning .."
            return

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        #print x, y

        dir_src = self.dir_src
        
        #################
        # !!! debug !!! #
        #################
        
        #dir_src = 27

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
        if (self.curr_map.shape != (1,1)): # removed .all()
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

        #################
        ## REVISE THIS ##
        #################

        cmd_vel = Twist()  #added
        cmd_vel.linear.x = _dx  #added
        cmd_vel.linear.y = _dy  #added
        self.move_robot.publish(cmd_vel)
        print "I published velocity"
        
        ##################################

        '''
        path = Path()
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = "/map"
        loc = PoseStamped()
        loc.pose.position.x = x
        loc.pose.position.y = y
        loc.pose.position.z = 0
        path.poses.append(loc)
        for i in range(210):
            x += _dx
            y += _dy
            if not avoid_flg:
                if self.curr_map[ int(round(y/_grid_cell_size_m)) + _size_height/2 ][ int(round(x/_grid_cell_size_m)) + _size_width/2 ]:
                    # obstacle!
                    break
            if (self.goal_x - x)*(self.goal_x - x) + (self.goal_y - y) * (self.goal_y - y) < _grid_cell_size_m:
                # goal!
                break
            #print self.goal_x, self.goal_y, x, y, (self.goal_x - x)*(self.goal_x - x) + (self.goal_y - y) * (self.goal_y - y)
            loc = PoseStamped()
            loc.pose.position.x = x
            loc.pose.position.y = y
            loc.pose.position.z = 0
            path.poses.append(loc)
        self.global_plan_pub.publish(path)
        '''

        #print "reached end of the long callback"

    def callback5(self,data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y

        #self.goal_x = int(round(x/_grid_cell_size_m)) + _size_width/2
        #self.goal_y = int(round(y/_grid_cell_size_m)) + _size_height/2


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
