cmake_minimum_required(VERSION 2.8.3)
project(blue_tf_broadcaster)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
