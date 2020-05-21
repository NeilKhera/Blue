cmake_minimum_required(VERSION 2.8.3)
project(blue_odometry)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
