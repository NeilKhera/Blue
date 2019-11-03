cmake_minimum_required(VERSION 2.8.3)
project(nov4demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(nov4map src/nov4map.cpp)
target_link_libraries(nov4map ${catkin_LIBRARIES})
add_dependencies(nov4map nov4demo_generate_messages_cpp)
