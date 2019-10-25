#!/usr/bin/env python

import math
import rospy

from roboclaw.roboclaw import RoboClaw
from skid_steering.srv import SkidSteer, SkidSteerResponse

class SkidSteering:
  SPEED = 500

  WHEEL_DIA = 0.21
  GEAR_RATIO = 0.01
  ENC_COUNTS_PER_REV = 1024
  ENC_COUNTS_PER_TURN = ENC_COUNTS_PER_REV * GEAR_RATIO

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
    
    if req.drive != 0:
      distance = (req.drive / (math.pi * self.WHEEL_DIA)) * self.ENC_COUNTS_PER_REV * 4  
      if req.drive < 0:
        speed = -speed
      self.turn_left_wheels(int(self.SPEED), int(distance))
      self.turn_right_wheels(int(self.SPEED), int(distance))
    elif req.drive != 0:
      print 'not yet implemented'
    
    return SkidSteerResponse('Drive completed...')

if __name__ == '__main__':
  ss = SkidSteering()
  rospy.init_node('skid_steering', anonymous = True)
  rospy.spin()
