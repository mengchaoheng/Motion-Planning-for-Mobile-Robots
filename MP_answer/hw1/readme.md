https://blog.csdn.net/uuumbrellaaa/article/details/139264608
======

sudo apt update
sudo apt install libpcl-dev

sudo apt update
sudo apt install libpcl-conversions-dev

ADD_COMPILE_OPTIONS(-std=c++14 )




改动：

1.grid_path_searcher/CMakeLists.txt的29行c++11改为c++14:

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror





运行命令：
1.cd ~/mp_ws/


2.catkin_make

3.roscore

4.rviz,
选择配置文件/src/grid_path_searcher/launch/rviz_config/demo.rviz

5.
roslaunch grid_path_searcher demo.launch