#!/usr/bin/env python

import math
import rospy

from roboclaw.roboclaw import RoboClaw
from skid_steering.srv import SkidSteer, SkidSteerResponse

class SkidSteering:
  WHEEL_DIA = 0.21
  WHEEL_BASE = 0.47625
  ENC_COUNTS_PER_REV = 1024

  def __init__(self):
    self._roboclaws = {
      'left': RoboClaw("/dev/left", 115200),
      'right': RoboClaw("/dev/right", 115200)
    }

    self.skid_steer_service = rospy.Service('skid_steer', SkidSteer, self._drive_callback)
    print('Blue, ready to drive...')

  def turn_left_wheels(self, speed, distance):
    claw = self._roboclaws['left']
    claw.SpeedDistanceM1M2(speed, distance, speed, distance, 1)
  
  def turn_right_wheels(self, speed, distance):
    claw = self._roboclaws['right']
    claw.SpeedDistanceM1M2(speed, distance, speed, distance, 1)

  def _drive_callback(self, req):
    print('Executing drive...')
    
    arc_radius = req.arc_radius
    arc_length = req.arc_length
    angular_velocity = req.angular_velocity

    distance = (arc_length / (math.pi * self.WHEEL_DIA)) * self.ENC_COUNTS_PER_REV * 4 

    self.turn_left_wheels(int(angular_velocity * (arc_radius - self.WHEEL_BASE / 2)), int(distance))
    self.turn_right_wheels(int(angular_velocity * (arc_radius + self.WHEEL_BASE / 2)), int(distance))

    return SkidSteerResponse('Drive completed...')

if __name__ == '__main__':
  ss = SkidSteering()
  rospy.init_node('skid_steering', anonymous = True)
  rospy.spin()
