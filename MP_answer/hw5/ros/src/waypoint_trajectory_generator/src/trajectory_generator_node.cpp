#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>
// Useful customized headers
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

// Param from launch file
    double _vis_traj_width;
    double _Vel, _Acc;
    int    _dev_order, _min_order;

// ros related
    ros::Subscriber _way_pts_sub;
    ros::Publisher  _wp_traj_vis_pub, _wp_path_vis_pub;

// for planning
    int _poly_num1D;
    MatrixXd _polyCoeff;
    VectorXd _polyTime;
    Vector3d _startPos = Vector3d::Zero();
    Vector3d _startVel = Vector3d::Zero();

// declare
    void visWayPointTraj( MatrixXd polyCoeff, VectorXd time);
    void visWayPointPath(MatrixXd path);
    Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t );
    VectorXd timeAllocation( MatrixXd Path);
    void trajGeneration(Eigen::MatrixXd path);
    void rcvWaypointsCallBack(const nav_msgs::Path & wp);

//Get the path points 
void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);
    waypoints.row(0) = _startPos;
    
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];

    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    trajGeneration(waypoints);
}

void trajGeneration(Eigen::MatrixXd path)
{
    TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;
    
    MatrixXd vel = MatrixXd::Zero(2, 3); 
    MatrixXd acc = MatrixXd::Zero(2, 3);

    vel.row(0) = _startVel;

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    _polyTime  = timeAllocation(path);

    // generate a minimum-snap piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

    visWayPointPath(path);

    //After you finish your homework, you can use the function visWayPointTraj below to visulize your trajectory
    visWayPointTraj( _polyCoeff, _polyTime);
}


void testQP()
{

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian; //P or H
    Eigen::VectorXd gradient;  //f or q
    Eigen::SparseMatrix<double> linearMatrix;  //A
    Eigen::VectorXd lowerBound; //l
    Eigen::VectorXd upperBound; //u

//下面的实例来源于 https://ww2.mathworks.cn/help/optim/ug/quadprog.html?s_tid=srchtitle
// 具有线性约束的二次规划
    hessian.resize(2,2);
    hessian.insert(0,0) = 100;
    hessian.insert(1,0) = -100;
    hessian.insert(0,1) = -100;
    hessian.insert(1,1) = 200;
    // cout << "\n迭代访问稀疏矩阵的元素：" << endl;
	// for (int k=0; k<hessian.outerSize(); ++k)
	// 	for (SparseMatrix<double>::InnerIterator it(hessian,k); it; ++it)
	// 	{
	// 		it.value(); // 元素值
    //         std::cout << "it.value():" << std::endl << it.value() << std::endl;
	// 		it.row();   // 行标row index
    //         std::cout << "it.row():" << std::endl << it.row() << std::endl;
	// 		it.col();   // 列标（此处等于k）
    //         std::cout << "it.col():" << std::endl << it.col() << std::endl;
	// 		it.index(); // 内部索引，此处等于it.row()
    //         std::cout << "it.index():" << std::endl << it.index() << std::endl;
	// 	}
    std::cout << "hessian:" << std::endl << hessian << std::endl;

    gradient.resize(2);
    gradient << -2, -6;
    
    std::cout << "gradient:" << std::endl << gradient << std::endl;

    linearMatrix.resize(3,2);
    linearMatrix.insert(0,0) = 10;
    linearMatrix.insert(0,1) = 10;
    linearMatrix.insert(1,0) = -10;
    linearMatrix.insert(1,1) = 20;
    linearMatrix.insert(2,0) = 20;
    linearMatrix.insert(2,1) = 10;
    std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

    lowerBound.resize(3);
    lowerBound << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

    upperBound.resize(3);
    upperBound << 2, 2, 3;
    std::cout << "upperBound:" << std::endl << upperBound << std::endl;

    int NumberOfVariables = 2; //A矩阵的列数
    int NumberOfConstraints = 3; //A矩阵的行数

// 具有线性等式约束的二次规划
//     hessian.resize(2,2);
//     hessian.insert(0,0) = 1;
//     hessian.insert(1,0) = -1;
//     hessian.insert(0,1) = -1;
//     hessian.insert(1,1) = 2;
//     std::cout << "hessian:" << std::endl << hessian << std::endl;

//     gradient.resize(2);
//     gradient << -2, -6;
    
//     std::cout << "gradient:" << std::endl << gradient << std::endl;

//     linearMatrix.resize(1,2);
//     linearMatrix.insert(0,0) = 1;
//     linearMatrix.insert(0,1) = 1;
//     std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

//     lowerBound.resize(1);
//     lowerBound << 0;
//     std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

//     upperBound.resize(1);
//     upperBound << 0;
//     std::cout << "upperBound:" << std::endl << upperBound << std::endl;

//     int NumberOfVariables = 2; //A矩阵的列数
//     int NumberOfConstraints = 1; //A矩阵的行数


//具有线性约束和边界的二次最小化
    // hessian.resize(3,3);
    // hessian.insert(0,0) = 1;
    // hessian.insert(1,0) = -1;
    // hessian.insert(2,0) = 1;
    // hessian.insert(0,1) = -1;
    // hessian.insert(1,1) = 2;
    // hessian.insert(2,1) = -2;
    // hessian.insert(0,2) = 1;
    // hessian.insert(1,2) = -2;
    // hessian.insert(2,2) = 4;
    // std::cout << "hessian:" << std::endl << hessian << std::endl;

    // gradient.resize(3);
    // gradient << 2, -3, 1;
    
    // std::cout << "gradient:" << std::endl << gradient << std::endl;

    // linearMatrix.resize(4,3);
    // linearMatrix.insert(0,0) = 1;
    // linearMatrix.insert(1,0) = 0;
    // linearMatrix.insert(2,0) = 0;
    // linearMatrix.insert(3,0) = 1;

    // linearMatrix.insert(0,1) = 0;
    // linearMatrix.insert(1,1) = 1;
    // linearMatrix.insert(2,1) = 0;
    // linearMatrix.insert(3,1) = 1;

    // linearMatrix.insert(0,2) = 0;
    // linearMatrix.insert(1,2) = 0;
    // linearMatrix.insert(2,2) = 1;
    // linearMatrix.insert(3,2) = 1;
    // std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

    // lowerBound.resize(4);
    // lowerBound << 0, 0, 0, 0.5;
    // std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

    // upperBound.resize(4);
    // upperBound << 1, 1, 1, 0.5;
    // std::cout << "upperBound:" << std::endl << upperBound << std::endl;

    // int NumberOfVariables = 3; //A矩阵的列数
    // int NumberOfConstraints = 4; //A矩阵的行数



    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    //矩阵A为m*n矩阵
    solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
    solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
    // if(!solver.data()->setHessianMatrix(hessian)) return 1;//设置P矩阵
    solver.data()->setHessianMatrix(hessian);
    // if(!solver.data()->setGradient(gradient)) return 1; //设置q or f矩阵。当没有时设置为全0向量
    solver.data()->setGradient(gradient);
    // if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    // if(!solver.data()->setLowerBound(lowerBound)) return 1;//设置下边界
    solver.data()->setLowerBound(lowerBound);
    // if(!solver.data()->setUpperBound(upperBound)) return 1;//设置上边界
    solver.data()->setUpperBound(upperBound);

    // instantiate the solver
    // if(!solver.initSolver()) return 1;
    solver.initSolver();

    Eigen::VectorXd QPSolution;

    // solve the QP problem
    // if(!solver.solve()) return 1;
    solver.solve();

    // get the controller input
    QPSolution = solver.getSolution();

    std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;    

    // return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel",   _Vel,   1.0 );
    nh.param("planning/acc",   _Acc,   1.0 );
    nh.param("planning/dev_order", _dev_order,  3 );
    nh.param("planning/min_order", _min_order,  3 );
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

    //_poly_numID is the maximum order of polynomial
    _poly_num1D = 2 * _dev_order;

    //state of start point
    _startPos(0)  = 0;
    _startPos(1)  = 0;
    _startPos(2)  = 0;    

    _startVel(0)  = 0;
    _startVel(1)  = 0;
    _startVel(2)  = 0;
    
    _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack );

    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);
    // testQP();

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();  
        status = ros::ok();  
        rate.sleep();
    }
    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time)
{        
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "/map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;


    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }

    _wp_traj_vis_pub.publish(_traj_vis);
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "/map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

VectorXd timeAllocation( MatrixXd Path)
{ 
    VectorXd time(Path.rows() - 1);
    
    /*

    STEP 1: Learn the "trapezoidal velocity" of "TIme Allocation" in L5, then finish this timeAllocation function

    variable declaration: _Vel, _Acc: _Vel = 1.0, _Acc = 1.0 in this homework, you can change these in the test.launch

    You need to return a variable "time" contains time allocation, which's type is VectorXd

    The time allocation is many relative timeline but not one common timeline

    */
    // for (int i=0;i<time.rows();++i)
    // {
    //     time(i)=1.0f;
    // }
    double _Vel, _Acc;
    _Vel = 1.0, _Acc = 1.0;
    double t_av = _Vel / _Acc;
    double s_m =  t_av * _Vel;

    time = VectorXd::Ones(Path.rows() - 1);
    for(int k = 0;k < Path.rows()-1;k++){
        Vector3d delta = Path.row(k) - Path.row(k+1);
        double s = std::sqrt(delta.dot(delta));

        if(s <= s_m){
            time(k) = 2.0 * s / _Acc;
        }
        else{
            time(k) = (s - s_m) / _Vel + t_av;
        }
    }
    return time;
}