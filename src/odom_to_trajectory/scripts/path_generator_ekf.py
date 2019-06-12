#!/usr/bin/env python
#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
from nav_msgs.msg import Odometry

def goalCallback(data):
        global x_goal
        global y_goal
        global flag
        global current_value

        if ((abs(x_goal - data.pose.position.x) > 0.001) and  (abs(y_goal - data.pose.position.y) > 0.001)):
                print("flag is true!")
                current_value = []
        else:
                print("Flag is flase")
        x_goal = data.pose.position.x
        y_goal = data.pose.position.y

def callback(data):
        global xAnt
        global yAnt
        global cont
        global flag
        global path
        global current_value

        #To avoid repeating the values, it is found that the received values are differents
        if (abs(xAnt - data.pose.pose.position.x) > 0.0001 and abs(yAnt - data.pose.pose.position.y) > 0.0001):                
                orientation_q = data.pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
                pose_to_send = [data.pose.pose.position.x, data.pose.pose.position.y, yaw]
                current_value.append(pose_to_send)
                # print(len(current_value))
                my_np = np.array(current_value).flatten()
                my_array_for_publishing = Float32MultiArray(data=my_np)
                
                path = my_array_for_publishing
                cont=cont+1

        #rospy.loginfo("Valor del contador: %i" % cont)
        if (cont>max_append and len(current_value) != 0):
        	current_value.pop(0)
        
        pub.publish(path)

        #Save the last position
        xAnt=data.pose.pose.position.x
        yAnt=data.pose.pose.position.y
        return path


if __name__ == '__main__':
        #Variable initialization
        global xAnt
        global yAnt
        global cont
        global path
        global current_value
        path = Float32MultiArray()
        xAnt=0.0
        yAnt=0.0
        cont=0
        current_value = []

        global flag
        flag = False

        global x_goal
        global y_goal
        x_goal = 0.0
        y_goal = 0.0

        #Node and msg initialization
        rospy.init_node('path_odom_plotter')

        #Rosparams that are set in the launch file (max size of array pose msg from the path)
        if not rospy.has_param("~max_list_append"):
                rospy.logwarn('The parameter max_list_append dont exists')
        max_append = rospy.get_param('~max_list_append')

        if not (max_append > 0):
                rospy.logwarn('The parameter max_list_append not is correct')
                sys.exit()
        pub = rospy.Publisher('/odompath', Float32MultiArray, queue_size=1)


        #Subscription to the topic
        rospy.Subscriber('/odom', Odometry, callback)
        # rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalCallback) 

        rate = rospy.Rate(30)

try:
	while not rospy.is_shutdown():
        	rate.sleep()
except rospy.ROSInterruptException:
	pass
