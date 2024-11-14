cmake_minimum_required(VERSION 3.0.2)
project(mycontroller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  wamv_model
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controller
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ./include/mycontroller/
)


add_executable(test_station_keeping ./src/test_station_keeping.cpp ./src/nmpc_station_keeping.cpp)
target_link_libraries(test_station_keeping ${catkin_LIBRARIES})
target_link_libraries(test_station_keeping /usr/local/lib/libcasadi.so.3.7) 

add_executable(test_trajectory_tracking ./src/test_trajectory_tracking.cpp 
              ./src/nmpc_trajectory_tracking.cpp ./src/single_trajectory.cpp 
              ./src/wamv_model.cpp)
target_link_libraries(test_trajectory_tracking ${catkin_LIBRARIES})
target_link_libraries(test_trajectory_tracking /usr/local/lib/libcasadi.so.3.7) 
