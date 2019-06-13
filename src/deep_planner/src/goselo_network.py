#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
caffe_root = '/home/ros/caffe'
sys.path.insert(0, caffe_root + 'python')
import caffe

pycaffe_dir = caffe_root + 'python/'
center_only = True
image_dims = [224, 224]
channel_swap =  [0, 1, 2, 3, 4, 5]

# 8-Directions Model
model_def = '/home/ros/models/deploy.prototxt'
pretrained_model ='/home/ros/models/goselo_invisible.caffemodel'

# 36-Directions Model
# model_def = '/home/ros/models/deploy_36.prototxt'
# pretrained_model ='/home/ros/models/model_36.caffemodel'

caffe.set_mode_gpu()

class publish_global_plan:

    def __init__(self):
        self.n_directions = rospy.get_param('n_directions', 8)
        self.bridge = CvBridge()
        self.goselo_map = np.zeros((1,1))
        self.goselo_loc = np.zeros((1,1))
        self.angle = None

        self.classifier = caffe.Classifier(model_def, pretrained_model, image_dims=image_dims, mean=None, input_scale=1.0, raw_scale=255.0, channel_swap=channel_swap)
        self.direction_pub = rospy.Publisher('/goselo_dir', Float32, queue_size=1)
        self.map_sub = rospy.Subscriber("/goselo_map",Image,self.callback_goselo_map,queue_size = 1)
        self.loc_sub = rospy.Subscriber("/goselo_loc",Image,self.callback_goselo_loc,queue_size = 1)
        self.angle_sub = rospy.Subscriber("/angle",Float32,self.callback_angle,queue_size = 1)


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
        if (self.goselo_map.shape == (1,1)) or (self.goselo_loc.shape == (1,1) or self.angle == None):
            return
        else:
            predictions = self.classifier.predict([np.concatenate([self.goselo_map, self.goselo_loc], 2)], not center_only)
            dir_src = np.argmax( predictions )

            resolution = 360/self.n_directions
            ang = 360 - resolution * dir_src - self.angle - 90
            while ang < 0:
                ang = ang + 360

            dir_dst = self.n_directions - int( round( ( ang % 360) / resolution ) )
            if dir_dst == self.n_directions:
                dir_dst = 0

            route = dir_dst
            self.direction_pub.publish(route)

    def callback_angle(self,data):
        self.angle = data.data

if __name__ == '__main__':
  pgp = publish_global_plan()
  rospy.init_node('goselo_network')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()