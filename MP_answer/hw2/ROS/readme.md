1.grid_path_searcher/CMakeLists.txt的29行c++11改为c++14 :

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

2.找到/src/grid_path_searcher/src/demo_node.cpp的world，去掉前面的斜杠:

​    map_vis.header.frame_id = "world";

运行命令：roslaunch grid_path_searcher demo.launch

点击3D Nav Goal,在rviz上选取合适的终点。

注意，只有A*有效。