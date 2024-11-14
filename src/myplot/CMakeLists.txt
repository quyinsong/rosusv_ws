cmake_minimum_required(VERSION 3.0.2)
project(myplot)

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
#  LIBRARIES myplot
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ./include/myplot/
)

# 查找python库版本
find_package(PythonLibs 3.8)


# 指定头文件路径
set(PYTHON3.8_INLCUDE_DIRS "/usr/include/python3.8")

# 添加头文件到工程
include_directories(
    ${PYTHON3.8_INLCUDE_DIRS}
)


add_executable(test_plot ./src/test_plot.cpp ./src/comfun.cpp)
target_link_libraries(test_plot ${catkin_LIBRARIES})
target_include_directories(test_plot PRIVATE ${PYTHON2_INCLUDE_DIRS})
target_link_libraries(test_plot ${PYTHON_LIBRARIES})

add_executable(test_plot_ttc ./src/test_plot_ttc.cpp ./src/comfun.cpp
               ./src/single_trajectory.cpp ./src/wamv_model.cpp)
target_link_libraries(test_plot_ttc ${catkin_LIBRARIES})
target_include_directories(test_plot_ttc PRIVATE ${PYTHON2_INCLUDE_DIRS})
target_link_libraries(test_plot_ttc ${PYTHON_LIBRARIES})
