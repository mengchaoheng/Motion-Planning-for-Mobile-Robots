#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>
class TrajectoryGeneratorWaypoint {
private:
		double _qp_cost;
		Eigen::MatrixXd _Q;
		Eigen::VectorXd _Px, _Py, _Pz;

        uint8_t * data;
        int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
        int GLXYZ_SIZE, GLYZ_SIZE;
        double resolution, inv_resolution;
        double gl_xl, gl_yl, gl_zl;
        double gl_xu, gl_yu, gl_zu;
public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);

        int Factorial(int x);
        Eigen::MatrixXd getQ(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &Time);
        Eigen::SparseMatrix<double> getQs(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &Time);
        Eigen::MatrixXd getM(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &Time);
        Eigen::MatrixXd getCt(const int m, const int d_order);
        Eigen::SparseMatrix<double> getAeq(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &Time);
        Eigen::VectorXd getbeq(const int m, const int p_num1d, const int d_order, const Eigen::VectorXd &wayPoints);

        double getObjective();
        Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getVelPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getAccPoly( Eigen::MatrixXd polyCoeff, int k, double t );
};

#endif
