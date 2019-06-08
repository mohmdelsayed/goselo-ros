#! /usr/bin/env python

import roslib
roslib.load_manifest('deep_planner')
import rospy
import actionlib
from geometry_msgs.msg import Twist
import math
from deep_planner.msg import SetYawAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from simple_pid import PID

class SetYawServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('SetYaw', SetYawAction, self.execute, False)
    self.server.start()
    self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.odom_sub = rospy.Subscriber("/robot_pose_ekf/odom_combined",PoseWithCovarianceStamped,self.callbackStart,queue_size = 1)
    self.curr_heading = None

  def callbackStart(self, msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    if yaw <= 0:
      yaw = yaw + 2*math.pi
    self.curr_heading = yaw
    


  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    cmd_vel_command = Twist()
    P = 1.5
    I = 0.0
    D = 0.0

    pid = PID(P, I, D, setpoint=goal.desired_yaw)

    if type(self.curr_heading) == 'NoneType':
      return

    while abs((goal.desired_yaw - (self.curr_heading))) > 0.1:
      #print "heading now, desired: ", self.curr_heading, goal.desired_yaw
      control_value = pid(self.curr_heading)
      cmd_vel_command.angular.z = control_value #control_gain * (goal.desired_yaw - (self.curr_heading))
      cmd_vel_command.linear.x = 0.0
      self.move_robot.publish(cmd_vel_command)


    # cmd_vel_command.angular.z = 0
    # cmd_vel_command.linear.x = 0.0
    # self.move_robot.publish(cmd_vel_command)
    #print "succeeded with error: ", abs((goal.desired_yaw - (self.curr_heading)))
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('SetYaw_server')
  server = SetYawServer()
  rospy.spin() 