#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32

import math
import time
import random
import cv2
import sys
import numpy as np
import os
import scipy
import scipy.ndimage
from cv_bridge import CvBridge, CvBridgeError
#from NamedAtomicLock import NamedAtomicLock
            

target_imsize = (224, 224)

class publish_input_maps:

    def __init__(self):
        self.bridge = CvBridge()
        self.curr_locX = None
        self.curr_locY = None
        self.goal_locX = None
        self.goal_locY = None
        self.current_path = np.zeros((0,0))
        self.path_map = None

        self.curr_map_pub = rospy.Publisher("/current_map",Image,queue_size = 1)
        self.map_pub = rospy.Publisher("/goselo_map",Image,queue_size = 1)
        self.path_sub = rospy.Subscriber("/odompath",Path,self.callPath,queue_size = 1)
        self.loc_pub = rospy.Publisher("/goselo_loc",Image,queue_size = 1)
        self.angle_pub = rospy.Publisher("/angle",Float32,queue_size = 1)

        self.start_s = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,self.callbackStart,queue_size = 1)
        self.goal_s = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.callbackGoal,queue_size = 1)
        self.laser_map = rospy.Subscriber("/map",OccupancyGrid,self.callbackMap,queue_size = 1)

    def callbackStart(self, data):
	self.curr_locX = data.pose.pose.position.x
	self.curr_locY = data.pose.pose.position.y


    def callbackGoal(self, data):
	self.goal_locX = data.pose.position.x
	self.goal_locY = data.pose.position.y


    def callPath(self, data):

        #rospy.loginfo("Got a path of length " + str(len(data.poses)))

        self.current_path = np.zeros((len(data.poses), 2))
        for i in range(self.current_path.shape[0]):
            self.current_path[i,0] = data.poses[i].pose.position.x
            self.current_path[i,1] = data.poses[i].pose.position.y

    def callbackMap(self,data):

        '''
        # The map data, in row-major order, starting with (0,0).  Occupancy
        # probabilities are in the range [0,100].  Unknown is -1.
        int8[] data

        '''
        #if self.curr_locX == None or self.curr_locY == None or self.goal_locX == None or self.goal_locY == None:
         #   print "No location"
        #else:
        rospy.loginfo("Entered callback from /map subscriber")
        rospy.loginfo("Map Size (height, width): " + str(data.info.height) + " " + str(data.info.width))
        rospy.loginfo("Cell Size: " + str(data.info.resolution))
        rospy.loginfo("Map origin: " + str(data.info.origin.position.x) + " " + str(data.info.origin.position.y))

        input_map = np.array(data.data).reshape((data.info.height, data.info.width))
        
        
        input_map[input_map == -1] = 127
        input_map = input_map.astype(np.uint8)
        input_map[input_map == 0] = 255
        input_map = input_map.astype(np.uint8)
        input_map[input_map == 100] = 0
        input_map = input_map.astype(np.uint8)
        
        rospy.loginfo("Numpy Map: " + str(input_map.shape))

        #self.curr_locX = 0.1
        #self.curr_locY = 0.1
        #self.goal_locX = 0.2
        #self.goal_locY = 0.2

        if(self.current_path.shape == (0,0)):
	    rospy.loginfo("I received no Path yet!")

            

        if (self.curr_locX == None) or (self.curr_locY == None) or (self.goal_locX == None) or (self.goal_locY == None):
            print "No current_loc OR No goal_loc"
            return

        self.path_map = np.zeros(input_map.shape)
        print "Current path of length", self.current_path.shape
        for i in range(self.current_path.shape[0]):
            y = int(round((self.current_path[i,0]-data.info.origin.position.x)/data.info.resolution))
            x = int(round((self.current_path[i,1]-data.info.origin.position.y)/data.info.resolution))
            #print "Path x, y", x,y
            self.path_map[x, y] += 1

        _, the_map = cv2.threshold( input_map, 100, 1, cv2.THRESH_BINARY_INV )
        
        
        
        the_map_ = cv2.resize(the_map, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)
        path_map_ = cv2.resize(self.path_map, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)
        input_map_ = cv2.resize(input_map, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)


        cv2.imshow( 'Path Map', path_map_ )
        cv2.waitKey(3)
        cv2.imshow( 'The Current Map', input_map_)
        cv2.waitKey(3)
        cv2.imshow( 'Thresholded Map', the_map_*255 )
        cv2.waitKey(3)


        # #############################################################################################
        # ## Getting current location and goal location as a pixel location in the map ###############
        # #############################################################################################
 
        xA = int(round((self.curr_locX-data.info.origin.position.x)/data.info.resolution))
        yA = int(round((self.curr_locY-data.info.origin.position.y)/data.info.resolution))

        xB = int(round((self.goal_locX-data.info.origin.position.x)/data.info.resolution))
        yB = int(round((self.goal_locY-data.info.origin.position.y)/data.info.resolution))
        
        ## JUST FOR PLOTTING ##
        map_vis = np.zeros((input_map.shape[0],input_map.shape[1],3), np.uint8)
        map_vis[:,:,0] = input_map
        map_vis[:,:,1] = input_map
        map_vis[:,:,2] = input_map
        cv2.circle( map_vis, (xA, yA), 8, (0, 0, 255), -1 )
        cv2.circle( map_vis, (xB, yB), 8, (0, 255, 0), -1 )
        
        for i in range(self.current_path.shape[0]):
            y = int(round((self.current_path[i,0]-data.info.origin.position.x)/data.info.resolution))
            x = int(round((self.current_path[i,1]-data.info.origin.position.y)/data.info.resolution))
            #print "Path x, y", x,y
            map_vis[x, y] += 1

        #RGB_img = cv2.cvtColor(map_vis, cv2.COLOR_BGR2RGB)
        map_vis_ = cv2.resize(map_vis, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)

        cv2.imshow( 'Map Locations', map_vis_)
        cv2.waitKey(3)
        
        
        
        goselo_map, goselo_loc, theta = generate_goselo_maps(xA, yA, xB, yB, input_map, self.path_map, data.info.height, data.info.width)

        angle = Float32()		
        angle.data = theta  #in Radians
        self.angle_pub.publish(angle)

        print "I published the angle"
        print "Goselo map dimensions", goselo_map.shape

        goselo_map = np.array(goselo_map, dtype=np.uint8)
        goselo_loc = np.array(goselo_loc, dtype=np.uint8)  

        gos_map_sent = self.bridge.cv2_to_imgmsg(goselo_map,"bgr8")
        gos_loc_sent = self.bridge.cv2_to_imgmsg(goselo_loc,"bgr8")
        input_map_sent = self.bridge.cv2_to_imgmsg(input_map,"mono8")

        ### Publishing the three maps required to deep planner ###

        self.map_pub.publish(gos_map_sent)
        self.loc_pub.publish(gos_loc_sent)
        self.curr_map_pub.publish(input_map_sent)

        print "Published GOSELO Maps + Input Map \n\n\n"

def generate_goselo_maps(xA, yA, xB, yB, the_map, the_map_pathlog, m, n):

    ###############
    # convert img #
    ###############

    orig_map = np.zeros( (m, n, 4), dtype=np.float )
    orig_map[:,:,0] = np.array(the_map) #/ 255.
    orig_map[:,:,1][yA][xA] = 1 #/ 255.
    orig_map[:,:,2][yB][xB] = 1 #/ 255.
    orig_map[:,:,3] = np.array(the_map_pathlog)
    print "reached after conversion"

    sx = xA; sy = yA; gx = xB; gy = yB
    # get average position between goal and start
    mx = (sx + gx) / 2.
    my = (sy + gy) / 2.
    dx_ = max( mx, n - mx )
    dy_ = max( my, m - my )

    im2 = np.zeros( (int(dy_ * 2), int(dx_ * 2), 4), dtype=np.float )
    im2[ int(dy_-my):int(dy_-my)+m , int(dx_-mx):int(dx_-mx)+ n ] = orig_map
    im2[ im2 == 1 ] = 2
    if gx == sx:
        if gy > sy:
            theta = 90
        else:
            theta = 270
    else:
        theta = math.atan( float(gy-sy) / float(gx-sx) ) * 180 / math.pi
        if gx-sx < 0:
            theta = theta + 180

    print "reached after getting angle"

    im2 = scipy.ndimage.interpolation.rotate( im2, 90+theta )
    im2 = im2.transpose( (2,0,1) )
    L = int(np.sqrt( (gx-sx)*(gx-sx) + (gy-sy)*(gy-sy) ))
    im2 = [im2[0], im2[3]]

    im3 = np.zeros( (target_imsize[0], target_imsize[1], 6), dtype=np.uint8 )
    im3 = im3.transpose( (2,0,1) )
    l = (L+4, 4*L, 8*L)

    print "I rotated the map"

    for n_ in range(2):
        for i in range(3):
            im2_ = np.zeros( (l[ i ], l[ i ]), dtype=np.uint8 )
            y1 = max( 0,  (im2[ n_ ].shape[0]-l[i])/2 )
            y2 = max( 0, -(im2[ n_ ].shape[0]-l[i])/2 )
            x1 = max( 0,  (im2[ n_ ].shape[1]-l[i])/2 )
            x2 = max( 0, -(im2[ n_ ].shape[1]-l[i])/2 )
            dy_ = min( l[i], im2[ n_ ].shape[ 0 ] )
            dx_ = min( l[i], im2[ n_ ].shape[ 1 ] )
            im2_[ y2:y2+dy_, x2:x2+dx_ ] = im2[ n_ ][ y1:y1+dy_, x1:x1+dx_ ]
            im3[ i + n_ * 3 ] = cv2.resize( im2_, im3[ i + n_ * 3 ].shape, interpolation = cv2.INTER_AREA )
            t = time.time()
    im3 = im3 * 0.5
    im3[(im3 > 0)*(im3 <= 1)] = 1

    caffe_input_map = im3.transpose( (1, 2, 0) )
    print "Made the input map"

    print "caffe input map dimensions", caffe_input_map.shape

    goselo_map = caffe_input_map[:,:,0:3]
    goselo_loc = caffe_input_map[:,:,3:6]

    return goselo_map, goselo_loc, theta
    

if __name__ == '__main__':
         
    rospy.init_node('publish_input_maps', anonymous=True, log_level=rospy.DEBUG)
    pim = publish_input_maps()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
    cv2.destroyAllWindows()
