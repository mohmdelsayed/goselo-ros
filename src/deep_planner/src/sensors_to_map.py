#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32, Float32MultiArray
import tf
import math
import time
import cv2
import numpy as np
import scipy
import scipy.ndimage
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
import threading

# ROS includes
import roslib
import rviz_tools_py as rviz_tools

import copy

target_imsize = (224, 224)

class publish_input_maps:

    def __init__(self):
        self.bridge = CvBridge()
        self.map = None
        self.goal_locX = None
        self.goal_locY = None
        self.curr_X = None
        self.curr_Y = None
        self.laser_map = np.zeros((0,0))
        self.the_map = np.zeros((0,0))
        self.path_map = np.zeros((0,0))
        self.down_scale = rospy.get_param('down_scale', 10)
        self.TimeParameter = rospy.get_param('publishing_wait', 10)
        self.previous_wait = 0
        self.lock = threading.Lock()

        self.map_pub = rospy.Publisher("/goselo_map",Image,queue_size = 1)
        self.loc_pub = rospy.Publisher("/goselo_loc",Image,queue_size = 1)
        self.angle_pub = rospy.Publisher("/angle",Float32,queue_size = 1)
    
        self.path_sub = rospy.Subscriber("/odompath",Float32MultiArray,self.callPath,queue_size = 1)
        self.goal_s = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.callbackGoal,queue_size = 1)
        self.laser_map_sub = rospy.Subscriber("/map",OccupancyGrid,self.callbackMap,queue_size = 1)
        self.laserscan_sub = rospy.Subscriber("/scan",LaserScan,self.callLaserScan,queue_size = 1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)

    def callbackGoal(self, data):
        self.goal_locX = data.pose.position.x
        self.goal_locY = data.pose.position.y

    def callLaserScan(self, data):

        if not self.lock.locked():
            self.lock.acquire()             
            try:
                (trans,orientation_q) = listener.lookupTransform('/odom', '/rplidar_link', rospy.Time(0))
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
            self.curr_X = current_x
            self.curr_Y = current_y
            
            #my_map = self.the_map.copy()
            #laser standalone
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
                self.laser_map = my_map
                # cv2.imshow( 'self.laser_map', self.laser_map)
                # cv2.waitKey(1)

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
        self.map = data
        self._size_width = data.info.width
        self._size_height = data.info.height
        self.cell_size = data.info.resolution
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y
        self.the_map = np.zeros((data.info.height/self.down_scale, data.info.width/self.down_scale))
        
    def callback_odom(self,data):
        if (self.goal_locX == None or self.goal_locY == None or self.curr_X == None or self.curr_Y == None):
            rospy.logwarn("No Goal Location or Current Location!")
            return

        if(self.path_map.shape == (0,0)):
            rospy.logwarn("No path map!")
            return
        
        xA = int(round((self.curr_X-self.map.info.origin.position.x)/(self.down_scale*self.map.info.resolution)))
        yA = int(round((self.curr_Y-self.map.info.origin.position.y)/(self.down_scale*self.map.info.resolution)))

        xB = int(round((self.goal_locX-self.map.info.origin.position.x)/(self.down_scale*self.map.info.resolution)))
        yB = int(round((self.goal_locY-self.map.info.origin.position.y)/(self.down_scale*self.map.info.resolution)))


        if (abs(xA-xB) < 0.01 and abs(yA-yB) < 0.01):
            rospy.logwarn("Goal Already Reached!")
            return

        if self.laser_map.shape == (0,0):
            return
        goselo_map, goselo_loc, theta = generate_goselo_maps(xA, yA, xB, yB, self.laser_map, self.path_map, self.map.info.height/self.down_scale, self.map.info.width/self.down_scale)
        
        angle = Float32()		
        angle.data = theta  #in Radians

        # print "Goselo map dimensions", goselo_map.shape
        cv2.imshow( 'GOSELO Map', goselo_map)
        cv2.waitKey(1)
        cv2.imshow( 'GOSELO Self Locations', goselo_loc)
        cv2.waitKey(1)


        goselo_map = np.array(goselo_map, dtype=np.uint8)
        goselo_loc = np.array(goselo_loc, dtype=np.uint8)  


        gos_map_sent = self.bridge.cv2_to_imgmsg(goselo_map,"bgr8")
        gos_loc_sent = self.bridge.cv2_to_imgmsg(goselo_loc,"bgr8")

        ### Publishing the three maps required to deep planner ###
        time_now = rospy.get_rostime().secs

        if (time_now - self.previous_wait >= self.TimeParameter):
            self.map_pub.publish(gos_map_sent)
            self.loc_pub.publish(gos_loc_sent)
            self.angle_pub.publish(angle)
            self.previous_wait = time_now
            rospy.loginfo("Published GOSELO Maps + Input Map + Angle \n")

    def callPath(self, data):

        input_data_len = np.asarray(data.data).shape[0]
        input_data = np.asarray(data.data)
        path_vector = np.reshape(input_data, (input_data_len/3, 3))

        # rospy.loginfo("Got a path of length " + str(input_data_len))
        if (type(self.map) == 'NoneType' or self.the_map.shape == (0,0)):
            rospy.logwarn("Didn't receive path yet!")
            return

        temp = np.zeros(self.the_map.shape)

        if(input_data_len == 0):
            self.path_map = temp
            return

        y = (np.round((path_vector[:,0]-self.map.info.origin.position.x)//(self.map.info.resolution*self.down_scale))).astype(int)
        x = (np.round((path_vector[:,1]-self.map.info.origin.position.y)//(self.map.info.resolution*self.down_scale))).astype(int)
        temp[x, y] += 1
        self.path_map = temp
        # cv2.imshow( 'Path Map', self.path_map)
        # cv2.waitKey(1)



def generate_goselo_maps(xA, yA, xB, yB, the_map, the_map_pathlog, m, n):

    orig_map = np.zeros( (m, n, 4), dtype=np.float )
    orig_map[:,:,0] = np.array(the_map) #/ 255.
    orig_map[:,:,1][yA][xA] = 1 #/ 255.
    orig_map[:,:,2][yB][xB] = 1 #/ 255.
    orig_map[:,:,3] = np.array(the_map_pathlog)
    # print "reached after conversion"

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

    # print "reached after getting angle"

    im2 = scipy.ndimage.interpolation.rotate( im2, 90+theta )
    im2 = im2.transpose( (2,0,1) )
    L = int(np.sqrt( (gx-sx)*(gx-sx) + (gy-sy)*(gy-sy) ))
    im2 = [im2[0], im2[3]]

    im3 = np.zeros( (target_imsize[0], target_imsize[1], 6), dtype=np.uint8 )
    im3 = im3.transpose( (2,0,1) )
    l = (L+4, 4*L, 8*L)

    # print "I rotated the map"

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
    # print "Made the input map"

    # print "caffe input map dimensions", caffe_input_map.shape

    goselo_map = caffe_input_map[:,:,0:3]
    goselo_loc = caffe_input_map[:,:,3:6]

    return goselo_map, goselo_loc, theta
    

if __name__ == '__main__':
         
    rospy.init_node('publish_input_maps', log_level=rospy.INFO)
    listener = tf.TransformListener()

    pim = publish_input_maps()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
    cv2.destroyAllWindows()