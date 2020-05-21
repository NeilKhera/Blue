#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from roboclaw.roboclaw import RoboClaw

WHEEL_DIA = 0.206375
WHEEL_BASE = 0.47625
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
    curr_time = rospy.Time.now()

    prev_x = 0
    prev_y = 0
    prev_theta = 0
    prev_time = rospy.Time.now()

    while not rospy.is_shutdown():

        rate.sleep()

        curr_time = rospy.Time.now()
        time_step = (float(curr_time.secs) + (float(curr_time.nsecs) / 1000000000)) - (float(prev_time.secs) + (float(prev_time.nsecs) / 1000000000))

        curr_enc_left_m1 = claw_left.ReadEncM1()[1]
        curr_enc_left_m2 = claw_left.ReadEncM2()[1]
        curr_enc_right_m1 = claw_right.ReadEncM1()[1]
        curr_enc_right_m2 = claw_right.ReadEncM2()[1]
    
        curr_enc_left = float(curr_enc_left_m1 + curr_enc_left_m2) / 2
        curr_enc_right = float(curr_enc_right_m1 + curr_enc_right_m2) / 2

        prev_enc_left = float(prev_enc_left_m1 + prev_enc_left_m2) / 2
        prev_enc_right = float(prev_enc_right_m1 + prev_enc_right_m2) / 2

        wL = ((curr_enc_left - prev_enc_left) * float(360.0 / TICKS_PER_REV)) / time_step # degrees/sec
        wR = ((curr_enc_right - prev_enc_right) * float(360.0 / TICKS_PER_REV)) / time_step # degrees/sec
        wL = wL * (math.pi / 180) # wheel angular velocity [rad/s]
        wR = wR * (math.pi / 180) # wheel angular velocity [rad/s]

        vL = wL * (WHEEL_DIA / 2)
        vR = wR * (WHEEL_DIA / 2)

        w = (vR - vL) / WHEEL_BASE

        v = (vR + vL) / 2
        vx = v * math.cos(curr_theta)
        vy = v * math.sin(curr_theta)

        k00 = v * math.cos(prev_theta)
        k01 = v * math.sin(prev_theta)
        k02 = w

        k10 = v * math.cos(prev_theta + (time_step / 2) * k02)
        k11 = v * math.sin(prev_theta + (time_step / 2) * k02)
        k12 = w

        k20 = v * math.cos(prev_theta + (time_step / 2) * k12)
        k21 = v * math.sin(prev_theta + (time_step / 2) * k12)
        k22 = w

        k30 = v * math.cos(prev_theta + time_step * k22)
        k31 = v * math.sin(prev_theta + time_step * k22)
        k32 = w

        curr_x = prev_x + (time_step / 6) * (k00 + 2 * (k10 + k20) + k30)
        curr_y = prev_y + (time_step / 6) * (k01 + 2 * (k11 + k21) + k31)
        curr_theta = prev_theta + (time_step / 6) * (k02 + 2 * (k12 + k22) + k32)
        print(curr_theta)

        prev_enc_left_m1 = curr_enc_left_m1
        prev_enc_left_m2 = curr_enc_left_m2
        prev_enc_right_m1 = curr_enc_right_m1
        prev_enc_right_m2 = curr_enc_right_m2

        prev_x = curr_x
        prev_y = curr_y
        prev_theta = curr_theta
        prev_time = curr_time

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(curr_theta / 2)
        quaternion.w = math.cos(curr_theta / 2)

        odom = Odometry()
        odom.header.stamp = curr_time
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = curr_x
        odom.pose.pose.position.y = curr_y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = w
        
        pub.publish(odom)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
