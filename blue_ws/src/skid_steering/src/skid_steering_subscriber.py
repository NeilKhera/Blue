#!/usr/bin/env python

import math
import rospy
from roboclaw.roboclaw import RoboClaw
from geometry_msgs.msg import Twist

SPEED = 1000
roboclaws = {
  'left': RoboClaw("/dev/left", 115200),
  'right': RoboClaw("/dev/right", 115200)
}

def listener():
  rospy.init_node('skid_steering_subscriber', anonymous = True)
  rospy.Subscriber('cmd_vel', Twist, drive_callback)
  print('Blue, ready to drive...')
  rospy.spin()

def turn_left_wheels(speed):
  claw = roboclaws['left']
  claw.SpeedM1M2(speed, speed)
  
def turn_right_wheels(speed):
  claw = roboclaws['right']
  claw.SpeedM1(speed)
  claw.SpeedM2(speed)

def drive_callback(msg):
  turn_left_wheels(int((msg.linear.x - msg.angular.z) * SPEED))
  turn_right_wheels(int((msg.linear.x + msg.angular.z) * SPEED))

if __name__ == '__main__':
  listener()
