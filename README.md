# rosusv_ws
ROS话题通信初探——基于NMPC的自定义无人艇定点控制与轨迹跟踪控制仿真

# 0 项目结构  
src文件夹共包含三个功能包：  
wamv_model： 用于构建小车模型，创建小车节点test_wamv_model  
myplot：    用于可视化小车运行状态，创建绘图节点test_plot  
mycontroller：用于构建nmpc控制器，创建节点test_station_keeping和test_trajectory_tracking  
在mycontroller中有launch文件夹，其中包含两个launch文件

# 1 项目依赖  
ros-noetic 
eigen3         （用于矩阵运算）  
casadi3.6.3    （linux，C++版本，用于求解NLP问题）  
matplotlibcpp  （用于绘图，由于调用的是python的绘图，因此需要安装python和相应的python绘图库）  

# 2 项目构建  
在主目录下创建文件夹rosusv_ws：mkdir rosusv_ws  
拷贝代码到本地文件夹：         git clone https://github.com/quyinsong/rosusv_ws.git  
在rosusv_ws文件夹下运行：     catkin_make  

# 3 构建可能遇到的问题：

## 找不到自定义消息头文件，比如car_model/states.h，car_model/controls.h  

解决办法：打开wamv_model/CMakeList.txt文件,注释  

add_executable(test_wamv_model ./src/test_wamv_model.cpp ./src/wamv_model.cpp)\
target_link_libraries(test_wamv_model ${catkin_LIBRARIES})\

打开controller文件夹，进入mycontroller/CMakeList.txt文件，注释：

add_executable(test_station_keeping ./src/test_station_keeping.cpp ./src/nmpc_station_keeping.cpp)\
target_link_libraries(test_station_keeping ${catkin_LIBRARIES})\
target_link_libraries(test_station_keeping /usr/local/lib/libcasadi.so.3.7) 

add_executable(test_trajectory_tracking ./src/test_trajectory_tracking.cpp \
              ./src/nmpc_trajectory_tracking.cpp ./src/single_trajectory.cpp \
              ./src/wamv_model.cpp)\
target_link_libraries(test_trajectory_tracking ${catkin_LIBRARIES})\
target_link_libraries(test_trajectory_tracking /usr/local/lib/libcasadi.so.3.7)

打开myplot文件夹，进入myplot/CMakeList.txt文件，注释

add_executable(test_plot ./src/test_plot.cpp ./src/comfun.cpp)\
target_link_libraries(test_plot ${catkin_LIBRARIES})\
target_include_directories(test_plot PRIVATE ${PYTHON2_INCLUDE_DIRS})\
target_link_libraries(test_plot ${PYTHON_LIBRARIES})

add_executable(test_plot_ttc ./src/test_plot_ttc.cpp ./src/comfun.cpp\
               ./src/single_trajectory.cpp ./src/wamv_model.cpp)\
target_link_libraries(test_plot_ttc ${catkin_LIBRARIES})\
target_include_directories(test_plot_ttc PRIVATE ${PYTHON2_INCLUDE_DIRS})\
target_link_libraries(test_plot_ttc ${PYTHON_LIBRARIES})

按照上述流程注释完成以后，执行catkin_make，编译生成msg头文件\
然后将上述注释取消，再执行catkin_make，即可编译成功\

## 找不到casadi头文件
自行安装casadi，参考https://blog.csdn.net/qq_41701758/article/details/131527719?spm=1001.2014.3001.5501

## 找不到matplotlibcpp头文件
git clone https://github.com/lava/matplotlib-cpp\
cp matplotlib-cpp/matplotlibcpp.h /usr/local/include/\

# 4 配置环境变量  
把当前工作空间的环境变量设置到bash中并source bashrc文件使其生效:\  
echo "source ~/rosusv_ws/devel/setup.bash" >> ~/.bashrc \
source ~/.bashrc 

# 5 运行  
首先在一个终端运行roscore  \
(1) 测试定点控制：roslaunch mycontroller test_station_keeping.launch  

![image](https://github.com/user-attachments/assets/b0f26818-cc31-4364-9666-a372811e6fc5)

![image](https://github.com/user-attachments/assets/51503941-003c-43cb-88d4-9d5a52a5aef1)

(2) 测试轨迹跟踪控制：roslaunch mycontroller test_trajectory_tracking.launch  (算法存在问题，不能跟踪)

![image](https://github.com/user-attachments/assets/069947e7-5585-4deb-9684-ae957d7f8e10)

![image](https://github.com/user-attachments/assets/c50dfa7c-4a6a-40de-9984-1bb2ceefa5fe)

# 6 算法参数修改  
(1) 定点控制：NMPC参数：在nmpc_station_keeping.cpp中可修改惩罚矩阵Q和R的数值，m_Q和m_R \ 
    设定期望点：在test_station_keeping.cpp中可修改期望到达的位置  \
(2) 轨迹跟踪：NMPC参数：在nmpc_trajectory_tracking.cpp中可修改惩罚矩阵Q和R的数值，m_Q和m_R \
    设定期望轨迹：在test_trajectory_tracking.cpp中可修改期望轨迹  

# 7 问题  
轨迹跟踪存在问题，不能很好的跟踪轨迹，估计是时序问题

# 8 本人联系方式  
***
