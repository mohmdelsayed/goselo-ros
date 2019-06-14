#! /usr/bin/env python

import roslib
roslib.load_manifest('deep_planner')
import rospy
import actionlib
from geometry_msgs.msg import Twist
import math
from deep_planner.msg import SetYawAction
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from simple_pid import PID
import numpy as np  
class SetYawServer:
  def __init__(self):

    self.yawThreshold = rospy.get_param('yawThreshold', 0.1)
    self.Pgain = rospy.get_param('Pgain', 1.5)
    self.Igain = rospy.get_param('Igain', 0.0)
    self.Dgain = rospy.get_param('Dgain', 0.0)
    self.linearWhileRotating = rospy.get_param('linearWhileRotating', 0.05)
    
    self.curr_heading = None
    self.server = actionlib.SimpleActionServer('SetYaw', SetYawAction, self.execute, False)
    self.server.start()
    self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.odom_sub = rospy.Subscriber("/odom",Odometry,self.callbackOdom,queue_size = 1)

  def callbackOdom(self, msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    # if yaw <= 0:
    #   yaw = yaw + 2*math.pi
    self.curr_heading = yaw

  def execute(self, goal):

    cmd_vel_command = Twist()
    P = self.Pgain
    I = self.Igain
    D = self.Dgain

    if type(self.curr_heading) == 'NoneType':
      return

    # pid = PID(P, I, D, setpoint=goal.desired_yaw)

    while np.minimum(abs(goal.desired_yaw - self.curr_heading), abs(goal.desired_yaw - self.curr_heading - 2*math.pi)) > self.yawThreshold:
      diff = goal.desired_yaw - self.curr_heading
      if (diff > math.pi):
        diff = diff - 2*math.pi
      elif (diff < -1*math.pi):
        diff = diff + 2*math.pi

      cmd_vel_command.angular.z = self.Pgain*diff
      cmd_vel_command.linear.x = self.linearWhileRotating
      self.move_robot.publish(cmd_vel_command)

    #print "succeeded with error: ", abs((goal.desired_yaw - (self.curr_heading)))
    self.server.set_succeeded()

if __name__ == '__main__':
  rospy.init_node('SetYaw_server')
  server = SetYawServer()
  rospy.spin() 