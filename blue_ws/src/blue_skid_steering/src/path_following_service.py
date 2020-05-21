#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from blue_skid_steering import PathSrv, PathSrvResponse
from roboclaw.roboclaw import RoboClaw

odom = Odometry()

roboclaws = {
    'left': RoboClaw("/dev/left", 115200),
    'right': RoboClaw("/dev/right", 115200)
}

def callback(data):
    odom = data

def turn(angle):
    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

    while not (angle - yaw < 0.01 and angle - yaw > -0.01):
        err = angle - yaw
        if err > 0:
            roboclaws['left'].SpeedM1M2(-100, -100)
            roboclaws['right'].SpeedM1M2(100, 100)
        else:
            roboclaws['left'].SpeedM1M2(100, 100)
            roboclaws['right'].SpeedM1M2(-100, -100)

    roboclaws['left'].SpeedM1M2(0, 0)
    roboclaws['right'].SpeedM1M2(0, 0)

def go(dist, x, y):
    odom_x = odom.pose.pose.position.x
    odom_y = odom.pose.pose.position.y

    d = math.sqrt((y - odom_y)*(y - odom_y) + (x - odom_x)*(x - odom_x))
    while (d < dist):
        roboclaws['left'].SpeedM1M2(100, 100)
        roboclaws['right'].SpeedM1M2(100, 100)

    roboclaws['left'].SpeedM1M2(0, 0)
    roboclaws['right'].SpeedM1M2(0, 0)

def drive(req):
    x_prev = 0
    y_prev = 0
    theta_prev = 0

    for i in range(len(req.path.poses)):
        curr_pose = req.path.poses[i]
        x_curr = curr_pose.pose.position.x
        y_curr = curr_pose.pose.position.y

        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        theta_curr = euler[2]

        theta_goal = math.atan2(y_curr - y_prev, x_curr - x_prev)
        dists_delta = math.sqrt((y_curr - y_prev)*(y_curr - y_prev) + (x_curr - x_prev)*(x_curr - x_prev))

        print('Step ' + str(i))
        print('Turning...')
        turn(theta_goal)
        print('Moving...')
        go(dists_delta, x_prev, y_prev)
        print('Turning...')
        turn(theta_curr)
        print('Done\n')

        x_prev = x_curr
        y_prev = y_curr
        theta_prev = theta_curr

    return PathSrvResponse('Done.')

def run():
    rospy.init_node('path_following', anonymous=True)
    rospy.Subscriber("Odom", Odometry, callback)
    rospy.Service('drive', PathSrv, drive)
    print("Ready to drive...")
    rospy.spin()

if __name__ == '__main__':
    run()
