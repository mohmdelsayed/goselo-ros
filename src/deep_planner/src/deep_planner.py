#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
caffe_root = '/home/ros/caffe'  # Change this to your path.
sys.path.insert(0, caffe_root + 'python')
import caffe
import math



# CNN
pycaffe_dir = caffe_root + 'python/'
center_only = True
image_dims = [224, 224]
channel_swap =  [0, 1, 2, 3, 4, 5]
model_def = '/home/ros/goselo-ros/src/deep_planner/src/models/deploy.prototxt'

#pretrained_model = sys.argv[ 1 ]
pretrained_model ='/home/ros/goselo-ros/src/deep_planner/src/models/goselo_invisible.caffemodel'
caffe.set_mode_gpu()

class publish_global_plan:

    def __init__(self):
        self.bridge = CvBridge()

        self.curr_map_sub = rospy.Subscriber("/map",OccupancyGrid,self.callback_map,queue_size = 1)
        self.map_sub = rospy.Subscriber("/goselo_map",Image,self.callback_goselo_map,queue_size = 1)
        self.loc_sub = rospy.Subscriber("/goselo_loc",Image,self.callback_goselo_loc,queue_size = 1)
        self.angle_sub = rospy.Subscriber("/angle",Float32,self.callback_angle,queue_size = 1)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.callback_odom,queue_size = 1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.callback_goal,queue_size = 1) # topic subscribed from RVIZ

        self.global_plan_pub = rospy.Publisher('/global_plan', Path, queue_size=1)
        #move_base_msgs/MoveBaseActionGoal
        self.action_goal_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_flag = 0
        self.current_x = None
        self.current_y = None
        self.goselo_map = np.zeros((1,1))
        self.goselo_loc = np.zeros((1,1))
        self._size_width = 0   #added
        self._size_height = 0  #added
        self.cell_size = None  #added
        self.angle = 0 # made it zero instead of none
        self.curr_map = np.zeros((1,1)) #added by mohamed
        self.goal_x =-4.0
        self.goal_y = -1.0
        self.dir_src = None
        self.prev_dir = None
        self.classifier = caffe.Classifier(model_def, pretrained_model, image_dims=image_dims, mean=None, input_scale=1.0, raw_scale=255.0, channel_swap=channel_swap)
        self.prev_avoid_direction = None

    def callback_map(self,data):
        self.curr_map = np.reshape(np.array(data.data), (data.info.height, data.info.width) ).astype( np.uint8 ) / 100
        self._size_width = data.info.width
        self._size_height = data.info.height
        self.cell_size = data.info.resolution
        print "Received a raw map"
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
            predictions = self.classifier.predict([np.concatenate([self.goselo_map, self.goselo_loc], 2)], not center_only)
            print "prediction vector is ", predictions

            ## TODOOO #####
            ### RANK THE PREDICTIONS AND SELECT THE FIRST ONES WITH INTERDIFFERENCE 1e-1 OR LESS. THEN, SELECT THE DIRECTION TO BE THE CLOSEST FROM THESE TO PREVIOUS DIRECTION ##########
            #max_index = np.argmax( predictions )
            #print "max_index: ", max_index
            #if self.prev_dir == None:
            #    self.dir_src = max_index
            #    self.prev_dir = max_index
            #elif predictions[0][self.prev_dir] - predictions[0][max_index] < 0.1:
            #    self.dir_src = self.prev_dir
            #else:
            #    self.dir_src = max_index
            #    self.prev_dir = max_index
            self.dir_src = np.argmax( predictions )
            dir_src = self.dir_src
            print "current direction", dir_src
            print "Self angle from goal: ", self.angle

            ang = 360 - 45 * dir_src - self.angle - 90
            while ang < 0:
                ang = ang + 360
            print "Heading Angle: ", ang
            
            dir_dst = 8 - int( round( (ang % 360) / 45. ) )
            if dir_dst == 8:
                dir_dst = 0

            route = [dir_dst]

            # force avoidance
            avoid_flg = False
            if (self.curr_map.shape != (1,1) and self.current_x != None and self.current_y != None and self.angle != 0 and self._size_width != 0 and self._size_height != 0 and self.cell_size != None and self.goal_x != None and self.goal_y != None):
                x_o = self.current_x
                y_o = self.current_y
                xA = int(round(x_o/self.cell_size)) + self._size_width/2
                yA = int(round(y_o/self.cell_size)) + self._size_height/2
                #print xA, yA

                #########################################################
                xA_ = xA + int(math.cos( route[0] * math.pi / 4. ) * self.cell_size)
                yA_ = yA + int(math.sin( route[0] * math.pi / 4. ) * self.cell_size)
                #########################################################

                if self.curr_map[ yA_ ][ xA_ ] :
                    print "Entered object avoidance"
                    if (self.prev_avoid_direction != None):
                        c = self.prev_avoid_direction
                        xA_ = xA + int(math.cos( c * math.pi / 4. ) * self.cell_size)
                        yA_ = yA + int(math.sin( c * math.pi / 4. ) * self.cell_size)
                        if not self.curr_map[ yA_ ][ xA_ ] :
                            route = [c]
                            self.prev_avoid_direction = c
                            avoid_flg = True
                            print 'Object Avoidance! Route :', route
                    else:
                        for c in range(2,8):
                            if c % 2 == 0:
                                c = route[ 0 ] + c / 2
                            else:
                                c = route[ 0 ] - (c-1) / 2
                            if c < 0:
                                c += 8
                            elif c > 7:
                                c -= 8
                            xA_ = xA + int(math.cos( c * math.pi / 4. ) * self.cell_size)
                            yA_ = yA + int(math.sin( c * math.pi / 4. ) * self.cell_size)
                            if not self.curr_map[ yA_ ][ xA_ ] :
                                route = [c]
                                self.prev_avoid_direction = c
                                avoid_flg = True
                                print 'Object Avoidance! Route :', route
                                break
                else:
                    self.prev_avoid_direction = None
                #print "finished object avoidance"

                _dx = math.cos( route[0] * math.pi / 4. ) * self.cell_size
                _dy = math.sin( route[0] * math.pi / 4. ) * self.cell_size
                #path = Path()
                #path.header.stamp = rospy.get_rostime()
                #path.header.frame_id = "/map"
                #loc = PoseStamped()
                #loc.pose.position.x = x
                #loc.pose.position.y = y
                #loc.pose.position.z = 0
                #path.poses.append(loc)

                if self.goal_flag == 0:
                    for i in range(40):
                        x_o += _dx
                        y_o += _dy
                        #print "I am in the loop"
                        if not avoid_flg:
                            #print " I entered 1st if"
                            if self.curr_map[ int(round(y_o/self.cell_size)) + self._size_height/2 ][ int(round(x_o/self.cell_size)) + self._size_width/2 ] > 50:
                                print "current map pixels: ", curr_map[0:20, 0:20]
                                print " I entered 2nd if"
                                print "current map value of u_goal: ", self.curr_map[ int(round(y_o/self.cell_size)) + self._size_height/2 ][ int(round(x_o/self.cell_size)) + self._size_width/2 ]
                                # obstacle!
                                #x -= 10*_dx
                                #y -= 10*_dy
                                #break
                        if(self.goal_x - x_o)*(self.goal_x - x_o) + (self.goal_y - y_o) * (self.goal_y - y_o) < 10*self.cell_size:
                            print " I reached the GOAL"
                            self.goal_flag =1 
                            # goal!
                            break

                    #self.action_goal_client.wait_for_server()

                    goal = MoveBaseGoal()
                    goal.target_pose.pose.position.x = x_o
                    goal.target_pose.pose.position.y = y_o
                    goal.target_pose.pose.position.z = 0.0
                    #euclidean_dist = math.sqrt((_dx**2) + (_dy**2))
                    #euler_x = _dx/euclidean_dist
                    #euler_y = _dy/euclidean_dist
                    if _dx == 0:
                        if _dy > 0:
                            quat_theta = (math.pi)/float(2)
                        else:
                            quat_theta = (3*math.pi)/float(2)
                    else:
                        quat_theta = math.atan( float(_dy) / float(_dx) )
                        if _dx < 0:
                            quat_theta = quat_theta + math.pi
                    #quat_theta = math.atan( float(_dy) / float(_dx) )
                    #quat_x = math.sin(quat_theta/float(2))* euler_x
                    #quat_y = math.sin(quat_theta/float(2))* euler_y
                    quat_x = 0.0
                    quat_y = 0.0
                    quat_z = math.sin(quat_theta/float(2))
                    quat_w = math.cos(quat_theta/float(2))
                    #print "dx and dy: ", euler_x, euler_y
                    print "quat_theta: ", quat_theta
                    #q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
                    #q = Quaternion(*q_angle)
                    goal.target_pose.pose.orientation.x = quat_x
                    goal.target_pose.pose.orientation.y = quat_y
                    goal.target_pose.pose.orientation.z = quat_z
                    goal.target_pose.pose.orientation.w = quat_w
                    
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    # Fill in the goal here
                    print "current location x,y: ", self.current_x, self.current_y
                    print "goal location x,y: ", self.goal_x, self.goal_y
                    print "Ultimate goal x,y: ", x_o, y_o
                    self.action_goal_client.send_goal(goal)
                    print "I sent an action goal"
                    #wait = self.action_goal_client.wait_for_result()
                    #if not wait:
                     #   rospy.logerr("Action server not available!")
                        #rospy.signal_shutdown("Action server not available!")
                    #else:
                     #   result = self.action_goal_client.get_result()
                      #  print("Action Result:", result)
                        #print "Action result: ", 
        

                else:
                    
                    # Fill in the goal here
                    print "current location x,y: ", self.current_x, self.current_y
                    print "goal location x,y: ", self.goal_x, self.goal_y
                    print "I reached the goal"
                    #print "I sent an action goal"


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



