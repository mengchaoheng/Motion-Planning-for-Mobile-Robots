#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;
#include "OsqpEigen/OsqpEigen.h"
#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

Eigen::SparseMatrix<double> TrajectoryGeneratorWaypoint::getAeq(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &Time)
{
    Eigen::SparseMatrix<double> Aeq;
    Aeq.resize(d_order*2 + m-1 + (m-1)*d_order,m*p_num1d);
    // start p v a j
    for(int k=0;k<d_order;k++)
    {
        // Aeq(k,k)=Factorial(k);
        for(int i=k;i<p_num1d;i++)
        {
            Aeq.insert(k,i)=Factorial(i)/Factorial(i-k) * pow(0,i-k);
        }
    }

    // end p v a j
    for(int k=0;k<d_order;k++)
    {
        for(int i=k;i<p_num1d;i++)
        {
            Aeq.insert(k+d_order,i+(m-1)*p_num1d)=Factorial(i)/Factorial(i-k) * pow(Time(m-1),i-k);
        }
    }

    // position constrain in all middle waypoints
    for(int j=0;j<m-1;j++)
    {
        for(int i=0;i<p_num1d;i++)
        {
            Aeq.insert(j+d_order*2,i+j*p_num1d)= pow(Time(j),i);
        }
    }

    // p v a j constrain in all middle waypoints
    for(int j=0;j<m-1;j++)
    {
        for(int k=0;k<d_order;k++)
        {
            for(int i=k;i<p_num1d;i++)
            {
                Aeq.insert(j*d_order+d_order*2+m-1+k,j*p_num1d+i)=Factorial(i)/Factorial(i-k) * pow(Time(j),i-k);
                
            }
            Aeq.insert(j*d_order+d_order*2+m-1+k,(j+1)*p_num1d+k)=-Factorial(k) ;
        }
    }
    return Aeq;
}
Eigen::VectorXd TrajectoryGeneratorWaypoint::getbeq(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &wayPoints)
{
    VectorXd beq=VectorXd::Zero(d_order*2 + m-1 + (m-1)*d_order);

    // start p v a j
    beq(0)=wayPoints(0);

    // end p v a j
    beq(d_order)=wayPoints(m);

    // position constrain in all middle waypoints
    for(int j=0;j<m-1;j++)
    {
        beq(j+d_order*2)=wayPoints(j+1);
    }

    return beq;
}
// get Q of QP problem
Eigen::SparseMatrix<double> TrajectoryGeneratorWaypoint::getQs(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &Time)
{
    Eigen::SparseMatrix<double> Q;
    Q.resize(m*p_num1d,m*p_num1d);
    for(int j=0;j<m;j++)
    {
        for(int i=d_order;i<p_num1d;i++)
        {
            for(int k=d_order;k<p_num1d;k++)
            {
                Q.insert(j*p_num1d+i,j*p_num1d+k)=Factorial(i)/Factorial(i-d_order) * Factorial(k)/Factorial(k-d_order) / (i-d_order+k-d_order+1) * pow(Time(j), (i-d_order+k-d_order+1)) ;
            }
        }

    }
    return Q;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getQ(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &Time)
{
    MatrixXd Q=MatrixXd::Zero(m*p_num1d,m*p_num1d);
    for(int j=0;j<m;j++)
    {
        for(int i=d_order;i<p_num1d;i++)
        {
            for(int k=d_order;k<p_num1d;k++)
            {
                Q(j*p_num1d+i,j*p_num1d+k)=Factorial(i)/Factorial(i-d_order) * Factorial(k)/Factorial(k-d_order) / (i-d_order+k-d_order+1) * pow(Time(j), (i-d_order+k-d_order+1)) ;
            }
        }

    }
    return Q;
}
Eigen::MatrixXd TrajectoryGeneratorWaypoint::getM(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &Time)
{
    MatrixXd M=MatrixXd::Zero(m*d_order*2,m*p_num1d);
    for(int j=0;j<m;j++)
    {
        //M_K_0
        for(int k=0;k<d_order;k++)
        {
            M(j*d_order*2+k,j*d_order*2+k)=Factorial(k);
        }
        // M_k_T
        for(int k=0;k<d_order;k++)
        {
            for(int i=k;i<p_num1d;i++)
            {
                M(j*d_order*2+d_order+k,j*p_num1d+i)=Factorial(i)/Factorial(i-k)  * pow(Time(j),(i-k)); 
            }
        }

    }
    return M;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getCt(const int m, const int d_order)
{
    MatrixXd Ct=MatrixXd::Zero(m*d_order*2,(m+1)*d_order);
    for(int j=0;j<m;j++)
    {
        //first segment
        if(j==1)
        {
            for(int n=0;n<d_order;n++)
            {
                Ct(n,n)=1;
            }
        }
        // end points from 1 to (n_seg-1)-th segment
        if(j<m-1)
        {
            Ct(j*2*d_order+d_order,j+d_order)=1; // end point's p  = the next piont's p
            for(int n=0;n<d_order-1;n++)
            {
                Ct(j*2*d_order+d_order+1+n,m-1+2*d_order+j*3+n)=1; // end point's v a j is free
            }

        }
        // start points from 2 to n_seg-th segment
        if(j>0)
        {
            Ct(j*2*d_order,j-1+d_order)=1; // start point's p t = the previous point's p
            for(int n=0;n<d_order-1;n++)
            {
                Ct(j*2*d_order+1+n,m-1+2*d_order+(j-1)*3+n)=1; // start point's v a j  = the pre piont's v a j
            }
        }
        // last segment
        if(j==m-1)
        {
            for(int n=0;n<d_order;n++)
            {
                Ct(j*2*d_order+d_order+n,m-1+d_order+n)=1; // end point
            }
        }
    }
    return Ct;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff(m, 3 * p_num1d);
  cout << "p_num1d is:\n" << endl << p_num1d << endl;
//   cout << "d_order is:\n" << endl << d_order << endl;
  // cout << "p_order is:\n" << endl << p_order << endl;
  cout << "m is:\n" << endl << m << endl;
  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  //==================================closed solution=======================================================  
  MatrixXd Q = getQ(m, p_num1d, d_order, Time);
  MatrixXd M = getM(m, p_num1d, d_order, Time);
  MatrixXd Ct = getCt(m, d_order);
  MatrixXd C=Ct.transpose();
  MatrixXd M_inv = M.inverse();
  MatrixXd M_inv_t = M_inv.transpose();
  MatrixXd R=C*M_inv_t*Q*M_inv*Ct;
//   cout << "R is:\n" << endl << R << endl;
  MatrixXd R_fp=R.topRightCorner(m-1+2*d_order,(m-1)*(d_order-1));
//   cout << "R_fp is:\n" << endl << R_fp << endl;
  MatrixXd R_pp=R.bottomRightCorner((m-1)*(d_order-1),(m-1)*(d_order-1));
//   cout << "R_pp is:\n" << endl << R_pp << endl;
  
  /*   Produce the derivatives in X, Y and Z axis directly.  */
  VectorXd P;
  Vector3d start_v = Vel.row(0);
//   cout << "start_v: " << endl << start_v << endl;
  for(int dim = 0; dim < 3; dim++){
      VectorXd wayPoints = Path.col(dim);
    //   cout << "waypoints: " << endl << wayPoints << endl;
      VectorXd d_F = VectorXd::Zero(m-1+2*d_order);

      d_F(0) = wayPoints(0); 
      d_F(1) = start_v(dim);
    //   cout << "d_F(1) is:" << endl << d_F(1) << endl;
      for(int i = 0; i < m - 1; i++ ){
          d_F(d_order + i) = wayPoints(i + 1);
      }
      // pT,n_seg-1
      d_F(m-1+d_order) = wayPoints(m);
    //   cout << "d_F is:" << endl << d_F << endl;

      VectorXd d_P = -1.0 * R_pp.inverse() * R_fp.transpose() * d_F;
    //   cout << "d_P is:" << endl << d_P << endl;

      VectorXd d(d_F.rows() + d_P.rows());
      d << d_F, d_P;
    //   cout << "d is:" << endl << d << endl;

      P = M.inverse() * Ct * d;
    //   cout<< "Dimension " << dim <<" coefficients: "<< endl << P << endl;
      for(int i=0;i<m;i++)
      {
          for(int j=0;j<p_num1d;j++)
          {
              PolyCoeff(i,j+p_num1d*dim)=P(j+i*p_num1d);
          }
      }
    //   cout << "PolyCoeff_closed :" << endl << PolyCoeff << endl;
  }

//======================================USE OSQP====================================================
    

// int NumberOfVariables = m*p_num1d; //A矩阵的列数
// int NumberOfConstraints = d_order*2 + m-1 + (m-1)*d_order; //A矩阵的行数
// /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
// Eigen::SparseMatrix<double> hessian = getQs(m, p_num1d, d_order, Time); //P or H
// // cout << "hessian :" << endl << hessian << endl;

// Eigen::VectorXd gradient=VectorXd::Zero(NumberOfVariables);;  //f or q

// Eigen::VectorXd lowerBound=VectorXd::Zero(NumberOfConstraints); //l
// Eigen::VectorXd upperBound=VectorXd::Zero(NumberOfConstraints); //u

    
// Eigen::VectorXd QPSolution;
// /*   Produce the derivatives in X, Y and Z axis directly.  */
// for(int dim = 0; dim < 3; dim++)
// {
//     VectorXd wayPoints = Path.col(dim);
//     /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
//     Eigen::SparseMatrix<double> linearMatrix = getAeq(m, p_num1d, d_order, Time);
//     // cout << "linearMatrix :" << endl << linearMatrix << endl;
//     VectorXd beq = getbeq(m, p_num1d, d_order, wayPoints);
//     // cout << "beq :" << endl << beq << endl;
//     lowerBound = beq;
//     upperBound = beq;

//     // instantiate the solver
//     OsqpEigen::Solver solver;

//     // // settings
//     solver.settings()->setWarmStart(true);
//     // // set the initial data of the QP solver
//     solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
//     solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
//     solver.data()->setHessianMatrix(hessian);
//     solver.data()->setGradient(gradient);
//     solver.data()->setLinearConstraintsMatrix(linearMatrix);
//     solver.data()->setLowerBound(lowerBound);
//     solver.data()->setUpperBound(upperBound);

//     // // instantiate the solver
//     solver.initSolver();

//     // // solve the QP problem
//     solver.solve();
//     // // get the controller input
//     QPSolution = solver.getSolution();
//     // std::cout << "QPSolution:" << std::endl << QPSolution << std::endl; 
//     for(int i=0;i<m;i++)
//     {
//         for(int j=0;j<p_num1d;j++)
//         {
//             PolyCoeff(i,j+p_num1d*dim)=QPSolution(j+i*p_num1d);
//         }
//     }
//     // cout << "PolyCoeff_QP :" << endl << PolyCoeff << endl;
// }
  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}