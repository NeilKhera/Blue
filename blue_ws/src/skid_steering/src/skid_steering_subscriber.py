#!/usr/bin/env python

import math
import rospy
from roboclaw.roboclaw import RoboClaw
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

SPEED = 1000
roboclaws = {
  'left': RoboClaw("/dev/left", 115200),
  'right': RoboClaw("/dev/right", 115200)
}

def listener():
  rospy.init_node('skid_steering_subscriber', anonymous = True)
  pub1 = rospy.Publisher('/blue/lwheel', Int16, queue_size=1)
  pub2 = rospy.Publisher('/blue/rwheel', Int16, queue_size=1)
  rospy.Subscriber('cmd_vel', Twist, drive_callback)
  print('Blue, ready to drive...')

  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    claw_left = roboclaws['left']
    claw_right = roboclaws['right']

    encoder_left = (claw_left.ReadEncM1 + claw_left.ReadEncM2) / 2
    encoder_right = (claw_right.ReadEncM1 + claw_right.ReadEncM2) / 2

    pub1.publish(encoder_left)
    pub2.publish(encoder_right)

def turn_left_wheels(speed):
  claw = roboclaws['left']
  claw.SpeedM1M2(speed, speed)
  
def turn_right_wheels(speed):
  claw = roboclaws['right']
  claw.SpeedM1M2(speed, speed)

def drive_callback(msg):
  turn_left_wheels(int((msg.linear.x - msg.angular.z) * SPEED))
  turn_right_wheels(int((msg.linear.x + msg.angular.z) * SPEED))

if __name__ == '__main__':
  listener()
