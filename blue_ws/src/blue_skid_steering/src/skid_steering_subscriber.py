#!/usr/bin/env python

import math
import rospy
import numpy as np
from roboclaw.roboclaw import RoboClaw
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

LINEAR_SPEED = 0.075
ANGULAR_SPEED = 0.25
WHEEL_BASE = 0.47
WHEEL_RADIUS = 0.105
TICKS_METER = 7000
roboclaws = {
  'left': RoboClaw("/dev/left", 115200),
  'right': RoboClaw("/dev/right", 115200)
}

def listener():
  rospy.init_node('skid_steering_subscriber', anonymous = True)

  claw_left = roboclaws['left']
  claw_right = roboclaws['right']
  claw_left.SetEncM1(0)
  claw_left.SetEncM2(0)
  claw_right.SetEncM1(0)
  claw_right.SetEncM2(0)

  pub1 = rospy.Publisher('lwheel', Int16, queue_size=1)
  pub2 = rospy.Publisher('rwheel', Int16, queue_size=1)
  rospy.Subscriber('cmd_vel', Twist, drive_callback)
  print('Blue, ready to drive...')

  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    encoder_left = (claw_left.ReadEncM1()[1] + claw_left.ReadEncM2()[1]) / 2
    encoder_right = (claw_right.ReadEncM1()[1] + claw_right.ReadEncM2()[1]) / 2

    enc_l = np.int16(encoder_left)
    enc_r = np.int16(encoder_right)
    pub1.publish(enc_l)
    pub2.publish(enc_r)

def turn_left_wheels(speed):
  claw = roboclaws['left']
  claw.SpeedM1M2(speed, speed)
  
def turn_right_wheels(speed):
  claw = roboclaws['right']
  claw.SpeedM1M2(speed, speed)

def drive_callback(msg):
  turn_left_wheels(int((((msg.linear.x * LINEAR_SPEED) - (msg.angular.z * ANGULAR_SPEED) * WHEEL_BASE / 2) / WHEEL_RADIUS) * TICKS_METER / 4))
  turn_right_wheels(int((((msg.linear.x * LINEAR_SPEED) + (msg.angular.z * ANGULAR_SPEED) * WHEEL_BASE / 2) / WHEEL_RADIUS) * TICKS_METER / 4))

if __name__ == '__main__':
  listener()
