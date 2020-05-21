#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from roboclaw.roboclaw import RoboClaw

WHEEL_DIA = 0.21
WHEEL_BASE = 0.47625
MOTOR_SPEED = 20
TICKS_PER_REV = 4096

roboclaws = {
    'left': RoboClaw("/dev/left", 115200),
    'right': RoboClaw("/dev/right", 115200)
}

def talker():
    pub = rospy.Publisher('/Blue/Odom', Odometry, queue_size=3)
    rospy.init_node('odometry', anonymous=True)
    rate = rospy.Rate(10)

    claw_left = roboclaws['left']
    claw_right = roboclaws['right']

    prev_enc_left_m1 = claw_left.ReadEncM1()[1]
    prev_enc_left_m2 = claw_left.ReadEncM2()[1]
    prev_enc_right_m1 = claw_right.ReadEncM1()[1]
    prev_enc_right_m2 = claw_right.ReadEncM2()[1]

    curr_x = 0
    curr_y = 0
    curr_theta = 0

    prev_x = 0
    prev_y = 0
    prev_theta = 0
    
    claw_left.SpeedM1M2(100, 100)
    claw_right.SpeedM1M2(-100, -100)

    while not rospy.is_shutdown():

        rate.sleep()

        curr_enc_left_m1 = claw_left.ReadEncM1()[1]
        curr_enc_left_m2 = claw_left.ReadEncM2()[1]
        curr_enc_right_m1 = claw_right.ReadEncM1()[1]
        curr_enc_right_m2 = claw_right.ReadEncM2()[1]
    
        curr_enc_left = float(curr_enc_left_m1 + curr_enc_left_m2) / 2
        curr_enc_right = float(curr_enc_right_m1 + curr_enc_right_m2) / 2

        prev_enc_left = float(prev_enc_left_m1 + prev_enc_left_m2) / 2
        prev_enc_right = float(prev_enc_right_m1 + prev_enc_right_m2) / 2

        theta_left = ((curr_enc_left - prev_enc_left) * (2 * math.pi) / TICKS_PER_REV) # radians
        theta_right = ((curr_enc_right - prev_enc_right) * (2 * math.pi) / TICKS_PER_REV) # radians

        dist_left = theta_left * (WHEEL_DIA / 2)
        dist_right = theta_right * (WHEEL_DIA / 2)
        dist_center = (dist_left + dist_right) / 2

        delta_theta = (dist_right - dist_left) / WHEEL_BASE

        curr_x = prev_x + (dist_center * math.cos(delta_theta))
        curr_y = prev_y + (dist_center * math.sin(delta_theta))
        curr_theta = prev_theta + delta_theta
        print(curr_theta)

        prev_enc_left_m1 = curr_enc_left_m1
        prev_enc_left_m2 = curr_enc_left_m2
        prev_enc_right_m1 = curr_enc_right_m1
        prev_enc_right_m2 = curr_enc_right_m2

        prev_x = curr_x
        prev_y = curr_y
        prev_theta = curr_theta

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(curr_theta / 2)
        quaternion.w = math.cos(curr_theta / 2)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = curr_x
        odom.pose.pose.position.y = curr_y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = 0
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = 0
        
        pub.publish(odom)

    claw_left.SpeedM1M2(00, 00)
    claw_right.SpeedM1M2(-00, -00)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
