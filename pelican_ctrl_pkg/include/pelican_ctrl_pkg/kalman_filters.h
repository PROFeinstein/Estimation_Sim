//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#ifndef PELICAN_CTRL_PKG_KALMAN_FILTERS_H
#define PELICAN_CTRL_PKG_KALMAN_FILTERS_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "pelican_ctrl_pkg/pelican_data_types.h"

using namespace Eigen;
using namespace std;

typedef Matrix<double, 2, 3> Matrix23d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 3, 2> Matrix32d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 9, 9> Matrix9d;
typedef Matrix<double, 9, 3> Matrix93d;
typedef Matrix<double, 6, 3> Matrix63d;
typedef Matrix<double, 3, 6> Matrix36d;
typedef Matrix<double, 1, 2> Matrix12d;

class kalman_filters{
private:

    int m_states;
    int m_measurements;

    MatrixXd  P;
    VectorXd  x;
    MatrixXd  A;
    VectorXd  B;
    MatrixXd  R;
    MatrixXd  H;
    MatrixXd  Q;

    VectorXd  x_hat;
    MatrixXd  P_;
    MatrixXd  A_;
    MatrixXd  B_;
    MatrixXd  H_;
    MatrixXd  R_;
    MatrixXd  Q_;

    MatrixXd  H_2;
    MatrixXd  R_2;
    MatrixXd  Q_2;
    MatrixXd  P_2;

    double   m_dt;
    Vector3d euler_rates;
    Vector3d fx(Vector3d xhat, Vector3d rates);
    Matrix3d Ajacob(Vector3d xhat, Vector3d rates);
    

public:
    pose_attitude x_hat_;
    
    kalman_filters(int states, int measurements, double dt);
    kalman_filters(double dt, double std_dev_1, double std_dev_2, double std_dev_3, double std_dev_4);
    kalman_filters(double step);
    kalman_filters();
    virtual  ~kalman_filters(void);
    // Vector6d EulerEKF(Vector3d z_meas, Vector3d rates);
    Vector4d Quat_Pose_Estimator(Vector3d z_meas, Vector3d rates);
    VectorXd LKF(VectorXd z_meas);
    VectorXd LKF_2(VectorXd meas, VectorXd input_u);
    void load_kalman_params(MatrixXd q_, MatrixXd r_);
    void load_kalman_q_params(MatrixXd a_, VectorXd x_, MatrixXd q_, MatrixXd r_, MatrixXd h_, MatrixXd p_);

    void ins_eskf(VectorXd imu);
    void gnss_eskf(VectorXd gps);
    void set_bias(VectorXd bias_A_W);

    void ins_eskf2(VectorXd imu, Matrix3d C_bi);
    void gnss_eskf2(VectorXd gps);    
    // Vector2d LKF_2(Vector2d meas);
    // Vector6d get_euler_values(Vector3d rates);
};

#endif //PELICAN_CTRL_PKG_KALMAN_FILTERS_H
