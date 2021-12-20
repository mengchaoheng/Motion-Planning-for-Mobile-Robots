#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>
#define USE_QP 1

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}


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



/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    MatrixXd PolyCoeff_QP = MatrixXd::Zero(m, 3 * p_num1d);
    MatrixXd PolyCoeff_closed = MatrixXd::Zero(m, 3 * p_num1d);
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    //==================================closed solution=======================================================
    
    MatrixXd Q = getQ(m, p_num1d, d_order, Time);
    MatrixXd M = getM(m, p_num1d, d_order, Time);
    MatrixXd Ct = getCt(m, d_order);
    MatrixXd C=Ct.transpose();
    MatrixXd M_inv = M.inverse();
    MatrixXd M_inv_t = M_inv.transpose();
    MatrixXd R=C*M_inv_t*Q*M_inv*Ct;
    cout << "R is:\n" << endl << R << endl;
    MatrixXd R_fp=R.topRightCorner(m-1+2*d_order,(m-1)*(d_order-1));
    cout << "R_fp is:\n" << endl << R_fp << endl;
    MatrixXd R_pp=R.bottomRightCorner((m-1)*(d_order-1),(m-1)*(d_order-1));
    cout << "R_pp is:\n" << endl << R_pp << endl;
    
    /*   Produce the derivatives in X, Y and Z axis directly.  */
    VectorXd P;
    for(int dim = 0; dim < 3; dim++){
        VectorXd wayPoints = Path.col(dim);
        cout << "waypoints: " << endl << wayPoints << endl;
        VectorXd d_F = VectorXd::Zero(m-1+2*d_order);

        d_F(0) = wayPoints(0); 
        for(int i = 0; i < m - 1; i++ ){
            d_F(d_order + i) = wayPoints(i + 1);
        }
        // pT,n_seg-1
        d_F(m-1+d_order) = wayPoints(m);
        cout << "d_F is:" << endl << d_F << endl;

        VectorXd d_P = -1.0 * R_pp.inverse() * R_fp.transpose() * d_F;
        cout << "d_P is:" << endl << d_P << endl;

        VectorXd d(d_F.rows() + d_P.rows());
        d << d_F, d_P;
        cout << "d is:" << endl << d << endl;

        P = M.inverse() * Ct * d;
        cout<< "Dimension " << dim <<" coefficients: "<< endl << P << endl;
        for(int i=0;i<m;i++)
        {
            for(int j=0;j<p_num1d;j++)
            {
                PolyCoeff_closed(i,j+p_num1d*dim)=P(j+i*p_num1d);
            }
        }
        cout << "PolyCoeff_closed :" << endl << PolyCoeff_closed << endl;
    }
    //-----------------test-------------------------------------------
    /*
    //-----------------------------------------------------------------
    VectorXd dF_x = VectorXd::Zero(m-1+2*d_order);  
    dF_x(0)=Path(0,0);
    for(int n=0;n<m-1;n++)
    {
        dF_x(n+d_order)=Path(n+1,0);
    }
    dF_x(m-1+d_order)=Path(m,0);

    VectorXd dP_x = -1.0 *R_pp.inverse()*R_fp.transpose()*dF_x;
    // cout << "dF_x is:" << endl << dF_x << endl;
    // cout << "dP_x is:" << endl << dP_x << endl;
    VectorXd d_x=VectorXd::Zero(dF_x.rows() + dP_x.rows());
    d_x << dF_x, dP_x;
    // cout << "d_x is:" << endl << d_x << endl;
    Px=M.inverse()*Ct*d_x;
    //-----------------------------------------------------------------
    VectorXd dF_y = VectorXd::Zero(m-1+2*d_order);  
    dF_y(0)=Path(0,1);
    for(int n=0;n<m-1;n++)
    {
        dF_y(n+d_order)=Path(n+1,1);
    }
    dF_y(m-1+d_order)=Path(m,1);

    VectorXd dP_y = -1.0 *R_pp.inverse()*R_fp.transpose()*dF_y;
    // cout << "dF_y is:" << endl << dF_y << endl;
    // cout << "dP_y is:" << endl << dP_y << endl;
    VectorXd d_y=VectorXd::Zero(dF_y.rows() + dP_y.rows());
    d_y << dF_y, dP_y;
    // cout << "d_y is:" << endl << d_y << endl;
    Py=M.inverse()*Ct*d_y;
    //-----------------------------------------------------------------
    VectorXd dF_z = VectorXd::Zero(m-1+2*d_order);  
    dF_z(0)=Path(0,2);
    for(int n=0;n<m-1;n++)
    {
        dF_z(n+d_order)=Path(n+1,2);
    }
    dF_z(m-1+d_order)=Path(m,2);

    VectorXd dP_z = -1.0 *R_pp.inverse()*R_fp.transpose()*dF_z;
    // cout << "dF_z is:" << endl << dF_z << endl;
    // cout << "dP_z is:" << endl << dP_z << endl;
    VectorXd d_z=VectorXd::Zero(dF_z.rows() + dP_z.rows());
    d_z << dF_z, dP_z;
    // cout << "d_z is:" << endl << d_z << endl;
    Pz=M.inverse()*Ct*d_z;
    //-----------------------------------------------------------------
    
    // 

    for(int i=0;i<m;i++)
    {
        for(int j=0;j<p_num1d;j++)
        {
            PolyCoeff(i,j)=Px(j+i*p_num1d);
            PolyCoeff(i,j+p_num1d)=Py(j+i*p_num1d);
            PolyCoeff(i,j+2*p_num1d)=Pz(j+i*p_num1d);
        }
    }
    */
    //------------------------------------------------------------
    

    //======================================USE OSQP====================================================
    

    int NumberOfVariables = m*p_num1d; //A矩阵的列数
    int NumberOfConstraints = d_order*2 + m-1 + (m-1)*d_order; //A矩阵的行数
    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
    Eigen::SparseMatrix<double> hessian = getQs(m, p_num1d, d_order, Time); //P or H
    cout << "hessian :" << endl << hessian << endl;
    
    Eigen::VectorXd gradient=VectorXd::Zero(NumberOfVariables);;  //f or q

    Eigen::VectorXd lowerBound=VectorXd::Zero(NumberOfConstraints); //l
    Eigen::VectorXd upperBound=VectorXd::Zero(NumberOfConstraints); //u

     
    Eigen::VectorXd QPSolution;
    /*   Produce the derivatives in X, Y and Z axis directly.  */
    for(int dim = 0; dim < 3; dim++)
    {
        VectorXd wayPoints = Path.col(dim);
        /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
        Eigen::SparseMatrix<double> linearMatrix = getAeq(m, p_num1d, d_order, Time);
        cout << "linearMatrix :" << endl << linearMatrix << endl;
        VectorXd beq = getbeq(m, p_num1d, d_order, wayPoints);
        cout << "beq :" << endl << beq << endl;
        lowerBound = beq;
        upperBound = beq;

        // instantiate the solver
        OsqpEigen::Solver solver;

        // // settings
        solver.settings()->setWarmStart(true);
        // // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
        solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(gradient);
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);

        // // instantiate the solver
        solver.initSolver();

        // // solve the QP problem
        solver.solve();
        // // get the controller input
        QPSolution = solver.getSolution();
        std::cout << "QPSolution:" << std::endl << QPSolution << std::endl; 
        for(int i=0;i<m;i++)
        {
            for(int j=0;j<p_num1d;j++)
            {
                PolyCoeff_QP(i,j+p_num1d*dim)=QPSolution(j+i*p_num1d);
            }
        }
        cout << "PolyCoeff_QP :" << endl << PolyCoeff_QP << endl;
    }
    //=====================================================================================
    // choose algorithm

    if(USE_QP==1)
    {
        PolyCoeff=PolyCoeff_QP;
    }
    else
    {
        PolyCoeff=PolyCoeff_closed;
    }
    return PolyCoeff;
}