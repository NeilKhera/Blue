cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_smasher)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_smasher src/pointcloud_smasher.cpp)
target_link_libraries(pointcloud_smasher ${catkin_LIBRARIES} ${PCL_LIBRARIES})
