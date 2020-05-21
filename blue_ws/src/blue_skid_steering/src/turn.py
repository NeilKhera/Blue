#!/usr/bin/env python

import rospy
from roboclaw.roboclaw import RoboClaw
from nav_msgs.msg import Odometry

Kp = 2500
Ki = 0
Kd = 0

roboclaws = {
  'left': RoboClaw("/dev/left", 115200),
  'right': RoboClaw("/dev/right", 115200)
}

odom = Odometry()
goal = 0.707

def callback(data):
    global odom
    odom = data

def listener():
    rospy.init_node('blue_turn', anonymous=True)
    rospy.Subscriber("/Blue/Odom", Odometry, callback)
    rate = rospy.Rate(10)
    
    prev_err = 0.0
    sum_err = 0.0

    while not rospy.is_shutdown():
        curr = odom.pose.pose.orientation.z
        err = goal - curr
        output = int((err * Kp) + (sum_err * Ki) + (prev_err * Kd))
        
        if rospy.is_shutdown():
            roboclaws['left'].SpeedM1M2(0, 0)
            roboclaws['right'].SpeedM1M2(0, 0)

        roboclaws['left'].SpeedM1M2(-output, -output)
        roboclaws['right'].SpeedM1M2(output, output)

        if rospy.is_shutdown():
            roboclaws['left'].SpeedM1M2(0, 0)
            roboclaws['right'].SpeedM1M2(0, 0)

        prev_err = err
        sum_err = sum_err + err
        rate.sleep()

    roboclaws['left'].SpeedM1M2(0, 0)
    roboclaws['right'].SpeedM1M2(0, 0)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        roboclaws['left'].SpeedM1M2(0, 0)
        roboclaws['right'].SpeedM1M2(0, 0)
        pass

