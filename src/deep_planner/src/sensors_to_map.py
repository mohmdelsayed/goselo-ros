#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32, Float32MultiArray

import math
import time
import cv2
import numpy as np
import scipy
import scipy.ndimage
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError

# ROS includes
import roslib
import rviz_tools_py as rviz_tools

import copy

target_imsize = (224, 224)

class publish_input_maps:

    def __init__(self):
        self.bridge = CvBridge()
        self.current_path = np.zeros((0,0))
        self.map = None
        self.counter = 0
        self.curr_locX = None
        self.curr_locY = None
        self.goal_locX = None
        self.goal_locY = None
        self.map_to_laser = np.zeros((0,0))
        self.the_map = np.zeros((0,0))
        self.path_map = np.zeros((0,0))
        self.down_scale = 10 # MAPS DOWNSCALING
        self.orientation = None
        self.my_measurements = np.zeros((0,0))

        self.map_pub = rospy.Publisher("/goselo_map",Image,queue_size = 1)
        self.path_sub = rospy.Subscriber("/odompath",Float32MultiArray,self.callPath,queue_size = 1)
        self.loc_pub = rospy.Publisher("/goselo_loc",Image,queue_size = 1)
        self.angle_pub = rospy.Publisher("/angle",Float32,queue_size = 1)
    
        self.odom_sub = rospy.Subscriber("/robot_pose_ekf/odom_combined",PoseWithCovarianceStamped,self.callbackEKF,queue_size = 1)
        self.goal_s = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.callbackGoal,queue_size = 1)
        self.laser_map = rospy.Subscriber("/map",OccupancyGrid,self.callbackMap,queue_size = 1)
        self.laserscan_sub = rospy.Subscriber("/scan",LaserScan,self.callLaserScan,queue_size = 1)


    def callbackEKF(self, data):
        self.curr_locX = data.pose.pose.position.x
        self.curr_locY = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.orientation = yaw
        #         print abs(data.header.stamp - self.laser_time_stamp) < 34000000
        # if (abs(data.header.stamp - self.laser_time_stamp) < 34000000):
        #     print "found suitable stamp!"
        #     self.orientation = yaw

    def callbackGoal(self, data):
        self.goal_locX = data.pose.position.x
        self.goal_locY = data.pose.position.y

    def callLaserScan(self, data):

        min_ang = data.angle_min
        max_ang = data.angle_max
        inc = data.angle_increment
        ranges = data.ranges
        if (self.the_map.shape != (0,0) and type(self.map) != 'NoneType' and type(self.orientation) != 'NoneType'):
            #my_map = self.the_map.copy()
            #laser standalone
            my_map = np.zeros(self.the_map.shape)
            current_rotation = copy.copy(self.orientation)
            current_x = copy.copy(self.curr_locX)
            current_y = copy.copy(self.curr_locY)

            self.my_measurements = np.zeros((len(ranges), 2))
            
            # measurements in laser scanner frame
            #orientation = math.atan2(self.curr_locY-self.map.info.origin.position.y, self.curr_locX - self.map.info.origin.position.x)
            for i, measurement in enumerate(ranges):
                if(measurement != float("inf")): # also check of being in max and min range
                    self.my_measurements[i, 0] = math.cos(-1*min_ang + current_rotation +  i*inc)*measurement +current_x
                    self.my_measurements[i, 1] = math.sin(-1*min_ang + current_rotation + i*inc)*measurement + current_y
                    x = int(round((self.my_measurements[i,0]-self.map.info.origin.position.x)/(self.down_scale*self.map.info.resolution)))
                    y = int(round((self.my_measurements[i,1]-self.map.info.origin.position.y)/(self.down_scale*self.map.info.resolution)))
                    try:
                        my_map[y, x] = 1
                    except: #out of range in map
                        pass
            # org = cv2.resize(my_map, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)
            # cv2.imshow( 'org Scan', (org)*255)
            # cv2.waitKey(1)

            # map_vis_ = cv2.resize(my_map, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)*255
            # myText = 'scanImages/Laser Scan'+str(self.counter)+'.jpg'
            # cv2.imwrite(myText, map_vis_)

            self.counter += 1
            # org = cv2.resize(self.the_map, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)
            # cv2.imshow( 'org Scan', (org)*255)
            # cv2.waitKey(1)
            # cv2.imshow( 'self.path_map', self.path_map)
            # cv2.waitKey(1)
        else:
            rospy.logwarn("Cannot process laser map")


        if (self.curr_locX == None or self.goal_locX == None):
            rospy.logwarn("No Goal Location or Current Location!")
            return

        if(self.path_map.shape == (0,0)):
            rospy.logwarn("No path map!")
            return

        
        xA = int(round((self.curr_locX-self.map.info.origin.position.x)/(self.down_scale*self.map.info.resolution)))
        yA = int(round((self.curr_locY-self.map.info.origin.position.y)/(self.down_scale*self.map.info.resolution)))

        xB = int(round((self.goal_locX-self.map.info.origin.position.x)/(self.down_scale*self.map.info.resolution)))
        yB = int(round((self.goal_locY-self.map.info.origin.position.y)/(self.down_scale*self.map.info.resolution)))


        if (abs(xA-xB) < 0.01 and abs(yA-yB) < 0.01):
            print "Goal Already Reached!"
            return

        goselo_map, goselo_loc, theta = generate_goselo_maps(xA, yA, xB, yB, my_map, self.path_map, self.map.info.height/self.down_scale, self.map.info.width/self.down_scale)

        # plot GOSELO maps for debugging and making sure they change as the robot approaches its goal

        cv2.imshow( 'goselo_map', goselo_map)
        cv2.waitKey(1)
        cv2.imshow( 'goselo_loc', goselo_loc)
        cv2.waitKey(1)
        
        angle = Float32()		
        angle.data = theta  #in Radians
        self.angle_pub.publish(angle)

        # print "I published the angle"
        # print "Goselo map dimensions", goselo_map.shape

        goselo_map = np.array(goselo_map, dtype=np.uint8)
        goselo_loc = np.array(goselo_loc, dtype=np.uint8)  

        gos_map_sent = self.bridge.cv2_to_imgmsg(goselo_map,"bgr8")
        gos_loc_sent = self.bridge.cv2_to_imgmsg(goselo_loc,"bgr8")

        ### Publishing the three maps required to deep planner ###

        self.map_pub.publish(gos_map_sent)
        self.loc_pub.publish(gos_loc_sent)

        # print "Published GOSELO Maps + Input Map \n\n\n"

        
    def callPath(self, data):

        input_data_len = np.asarray(data.data).shape[0]
        input_data = np.asarray(data.data)
        path_vector = np.reshape(input_data, (input_data_len/3, 3))

        #rospy.loginfo("Got a path of length " + str(len(data.poses)))
        if (type(self.map) == 'NoneType' or self.the_map.shape == (0,0) or input_data_len == 0 or type(input_data_len) == 'NoneType'):
            return
        #self.orientation = path_vector[-1, 2]
        self.current_path = path_vector
        temp = np.zeros(self.the_map.shape)

        y = (np.round((self.current_path[:,0]-self.map.info.origin.position.x)//(self.map.info.resolution*self.down_scale))).astype(int)
        x = (np.round((self.current_path[:,1]-self.map.info.origin.position.y)//(self.map.info.resolution*self.down_scale))).astype(int)
        temp[x, y] += 1
        self.path_map = temp

    def callbackMap(self,data):

        '''
        # The map data, in row-major order, starting with (0,0).  Occupancy
        # probabilities are in the range [0,100].  Unknown is -1.
        int8[] data

        '''
        self.map = data
        rospy.loginfo("Entered callback from /map subscriber")
        rospy.loginfo("Map Size (height, width): " + str(self.map.info.height) + " " + str(self.map.info.width))
        rospy.loginfo("Cell Size: " + str(self.map.info.resolution))
        rospy.loginfo("Map origin: " + str(self.map.info.origin.position.x) + " " + str(self.map.info.origin.position.y))

        input_map = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        input_map[input_map == -1] = 127
        input_map = input_map.astype(np.uint8)
        input_map[input_map == 0] = 255
        input_map = input_map.astype(np.uint8)
        input_map[input_map == 100] = 0
        input_map = input_map.astype(np.uint8)

        _, the_map = cv2.threshold(input_map, 100, 1, cv2.THRESH_BINARY_INV )
        # thickening the lines in each map
        kernel = np.ones((8,8), np.uint8)
        the_map = cv2.dilate(the_map,kernel,iterations = 1)

        self.the_map = cv2.resize(the_map, dsize=(the_map.shape[1]/self.down_scale, the_map.shape[0]/self.down_scale), interpolation=cv2.INTER_CUBIC)
    
        rospy.loginfo("input_map shape, the_map shape: " + str(input_map.shape) + " , " + str(self.the_map.shape))

        #self.path_map = cv2.dilate(self.path_map,kernel,iterations = 1)
        #self.path_map = cv2.resize(self.path_map, dsize=(self.path_map.shape[1]/self.down_scale, self.path_map.shape[0]/self.down_scale), interpolation=cv2.INTER_CUBIC)

        

def generate_goselo_maps(xA, yA, xB, yB, the_map, the_map_pathlog, m, n):

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
         
    rospy.init_node('publish_input_maps', log_level=rospy.WARN)
    pim = publish_input_maps()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
    cv2.destroyAllWindows()


        # # #######################################################################
        # # ## Visualizing Purposes Only at the end of map callback ###############
        # # #######################################################################
 
        # xA = int(round((self.curr_locX-data.info.origin.position.x)/(self.down_scale*data.info.resolution)))
        # yA = int(round((self.curr_locY-data.info.origin.position.y)/(self.down_scale*data.info.resolution)))

        # xB = int(round((self.goal_locX-data.info.origin.position.x)/(self.down_scale*data.info.resolution)))
        # yB = int(round((self.goal_locY-data.info.origin.position.y)/(self.down_scale*data.info.resolution)))
        

        # # JUST FOR PLOTTING ##
        # map_vis = np.zeros((input_map.shape[0]/self.down_scale,input_map.shape[1]/self.down_scale,3), np.uint8)
        # map_vis[:,:,0] = cv2.resize(input_map, dsize=(input_map.shape[1]/self.down_scale,input_map.shape[0]/self.down_scale), interpolation=cv2.INTER_CUBIC)
        # map_vis[:,:,1] = cv2.resize(input_map, dsize=(input_map.shape[1]/self.down_scale,input_map.shape[0]/self.down_scale), interpolation=cv2.INTER_CUBIC)
        # map_vis[:,:,2] = cv2.resize(input_map, dsize=(input_map.shape[1]/self.down_scale,input_map.shape[0]/self.down_scale), interpolation=cv2.INTER_CUBIC)
        # cv2.circle( map_vis, (xA, yA), 8, (0, 255, 0), -1 )
        # cv2.circle( map_vis, (xB, yB), 8, (0, 0, 255), -1 )
        
        # for i in range(self.current_path.shape[0]):
        #     y = int(round((self.current_path[i,0]-data.info.origin.position.x)/(self.down_scale*data.info.resolution)))
        #     x = int(round((self.current_path[i,1]-data.info.origin.position.y)/(self.down_scale*data.info.resolution)))
        #     #print "Path x, y", x,y
        #     map_vis[x, y] += 1

        # RGB_img = cv2.cvtColor(map_vis, cv2.COLOR_BGR2RGB)
        # map_vis_ = cv2.resize(map_vis, dsize=(224, 224), interpolation=cv2.INTER_CUBIC)
        # cv2.imshow( 'Map Locations', map_vis_)
        # cv2.waitKey(1)