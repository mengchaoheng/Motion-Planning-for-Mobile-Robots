1.grid_path_searcher/CMakeLists.txt的34行c++11改为c++14

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

2./demo_node/grid_map_vis报错。找到/src/grid_path_searcher/src/demo_node.cpp的world，去掉前面的斜杠：

​    map_vis.header.frame_id = "world";



运行命令：

1 ”roscore” in the terminal, 

2then run ”rviz” in the new terminal to start rviz for configuration, 

3and finally run ”roslaunch grid_path_searcher demo.launch” in the new terminal, 