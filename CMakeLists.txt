cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_walker)

## Use C++11 compiler
add_compile_options(-std=c++11)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## Declare a catkin package
catkin_package()

## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walker src/main.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(tester launch/test.launch test/main.cpp test/test.cpp)
#  target_link_libraries(tester ${catkin_LIBRARIES})
#endif()
