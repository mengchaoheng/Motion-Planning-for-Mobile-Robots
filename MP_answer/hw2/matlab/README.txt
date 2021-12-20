Use the script "math.m" as the main entry point.
使用math.m作为主函数入口

A_star: useful functions for A*
           distance.m: This function calculates the distance between any two cartesian coordinates.
	              用于计算笛卡尔坐标系下任意两点的距离（此函数可以用作修改启发函数，例如将距离计算换做曼哈顿距离）

A_star_search.m: This unfinished function is your homework, you can use the functions in the folder A_star, or you can do the whole work yourself.
	作业部分完成A_star_search.m，输出路径点列表，规定格式为n*2*1数组：|x_val | y_val，可以选择使用A_start里写好的函数，也可以独立完整完成所有工作。

-----------------------------------------------------------------------------------------------------------
obstacle_map: This function returns a map contains random distribution obstacles.
                       用于返回一个含有随意分布障碍物的地图（障碍物出现概率可以通过改变obstacle_ratio的数值大小来实现）

visualize_map: This function visualizes the 2D grid map consist of obstacles/start point/target point/optimal path.
                       用于可视化二维的栅格地图，包含障碍物、起点、终点和最优路径点