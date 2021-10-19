//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#include "pelican_ctrl_pkg/kalman_filters.h"

kalman_filters::kalman_filters(int states, int measurements, double dt):m_states(states), m_measurements(measurements), m_dt(dt)
{
    int no_rates = m_states/2;

    x.setOnes(m_states);
    A.setIdentity(m_states, m_states);
    MatrixXd dt_mat(no_rates, no_rates);
    dt_mat.setIdentity();
    A.block(0,no_rates,no_rates, no_rates) = dt_mat  * m_dt;
    P.setIdentity(m_states, m_states);
    H.setZero(m_measurements, m_states);
    MatrixXd meas_ident(m_measurements, m_measurements);
    meas_ident.setIdentity();
    H.block(0, 0, m_measurements, m_measurements) = meas_ident;
}

kalman_filters::kalman_filters(double dt, double std_dev_1, double std_dev_2, double std_dev_3, double std_dev_4):m_dt(dt)
{
	x_hat.setOnes(9);
	P_.setIdentity(9,9);
	A_.setIdentity(9,9);
    // A_ << 1, m_dt, pow(m_dt,2)/2,
    //       0,    1,          m_dt,
    //       0,    0,             1;
	A_.block(0,3,3,3) = Eigen::Matrix3d::Identity() * m_dt;
    A_.block(3,6,3,3) = Eigen::Matrix3d::Identity() * m_dt;
    A_.block(0,6,3,3) = Eigen::Matrix3d::Identity() * pow(m_dt,2)/2;

	B_.setOnes(9,1);
    B_.block(0,0,3,1) = Eigen::Vector3d(pow(m_dt,2)/2, pow(m_dt,2)/2, pow(m_dt,2)/2);
    B_.block(3,0,3,1) = Eigen::Vector3d(m_dt, m_dt, m_dt);
	
    H_.setZero(4,9);
	H_.block(1,6,3,3) = Eigen::Matrix3d::Identity();
    H_(0,2) = 1;
	R_.setZero(4,4);
	R_(0,0) = pow(std_dev_1, 2);
	R_(1,1) = pow(std_dev_2, 2);
	R_(2,2) = pow(std_dev_3, 2);
	R_(3,3) = pow(std_dev_4, 2);

	Q_.setZero(9,9);
    Q_ = B_ * B_.transpose() * pow(0.5, 2);   

    // cout << "xhat= \n" << x_hat << endl;
    // cout << "P= \n" << P_ << endl;
    // cout << "A= \n" << A_ << endl;
    // cout << "B= \n" << B_ << endl;
    // cout << "H= \n" << H_ << endl;
    // cout << "R= \n" << R_ << endl;
    // cout << "Q= \n" << Q_ << endl;
    // cout << endl;
    
}

kalman_filters::kalman_filters(double step):m_dt(step)
{

}
kalman_filters::kalman_filters()
{
m_dt = 0.001;
double acc_noise_std = 784e-6;
double gyr_noise_std = 523e-6;
double bias_acc_std  = 392e-6;
double bias_gyr_std  = 8.7266e-5;

//cout << "Nominal x : \n" << nominal_x << endl;

Q_.setZero(15,15);
Q_.block(3,3,3,3)   = Matrix3d::Identity() * pow(acc_noise_std,2) ;//* m_dt;// pow(m_dt,2);
Q_.block(6,6,3,3)   = Matrix3d::Identity() * pow(gyr_noise_std,2) ;//* m_dt;//pow(m_dt,2);
Q_.block(9,9,3,3)   = Matrix3d::Identity() * pow(bias_acc_std,2)  ;//* m_dt;// * m_dt;
Q_.block(12,12,3,3) = Matrix3d::Identity() * pow(bias_gyr_std,2)  ;//* m_dt;// * m_dt;

//P_.setIdentity(15,15);
P_ = Q_;

H_.setZero(3,15);
H_.block(0,0,3,3) = -Matrix3d::Identity();

double gps_std_dev = 0.25;
R_.setZero(3,3);
R_(0,0) = pow(gps_std_dev,2);
R_(1,1) = pow(gps_std_dev,2);
R_(2,2) = pow(gps_std_dev,2);

x_hat_.position = Vector3d(0,0,0);
x_hat_.velocity = Vector3d(0,0,0);
x_hat_.R_matrix = Matrix3d::Identity();

Q_2.setZero(9,9);
Q_2.block(3,3,3,3)   = Matrix3d::Identity() * pow(acc_noise_std,2) ;//* m_dt;// pow(m_dt,2);
Q_2.block(6,6,3,3)   = Matrix3d::Identity() * pow(bias_acc_std,2)  ;//* m_dt;// * m_dt;
P_2 = Q_2;
H_2.setZero(3,9);
H_2.block(0,0,3,3) = -Matrix3d::Identity();

R_2.setZero(3,3);
R_2(0,0) = pow(gps_std_dev,2);
R_2(1,1) = pow(gps_std_dev,2);
R_2(2,2) = pow(gps_std_dev,2);
}

kalman_filters::~kalman_filters(void) 
{
    cout << "Object kalman_filters destroyed" << endl;
}

void kalman_filters::load_kalman_params(MatrixXd q_, MatrixXd r_)
{
    Q = q_;
    R = r_;
}

void kalman_filters::load_kalman_q_params(MatrixXd a_, VectorXd x_, MatrixXd q_, MatrixXd r_, MatrixXd h_, MatrixXd p_)
{
    Q = q_;
    R = r_;
    A = a_;
    x = x_;
    H = h_;
    P = p_;
}

// Matrix3d kalman_filters::Ajacob(Vector3d xhat, Vector3d rates)
// {
//     Matrix3d A;
//     double phi   = xhat(0);
//     double theta = xhat(1);
//     double p     = rates(0);
//     double q     = rates(1);
//     double r     = rates(2);

//     A << q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta),         q*sin(phi)*pow(sec(theta),2)+r*cos(phi)*pow(sec(theta),2), 0,
//                               -q*sin(phi)-r*cos(phi),                                                                 0, 0,
//          q*cos(phi)*sec(theta)-r*sin(phi)*sec(theta), q*sin(phi)*sec(theta)*tan(theta)+r*cos(phi)*sec(theta)*tan(theta), 0;

//     return Matrix3d::Identity() + A * dt;
// }

Vector3d kalman_filters::fx(Vector3d xhat, Vector3d rates)
{
    double phi   = xhat(0);
    double theta = xhat(1);

    Matrix3d angRates2eulerRates;
    angRates2eulerRates << 1.0, sin(phi)*tan(theta), cos(phi)*tan(theta),
                           0.0, cos(phi)           ,           -sin(phi),
                           0.0, sin(phi)*sec(theta), cos(phi)*sec(theta);

    euler_rates = angRates2eulerRates * rates;
    return xhat + euler_rates * m_dt;
}

// Vector6d kalman_filters::EulerEKF(Vector3d z_meas, Vector3d rates)
// {
//     Matrix3d A_(Ajacob(x, rates));
//     Vector3d xp(fx(x, rates));
//     Matrix3d Pp(A_ * P * A_.transpose() + Q);
//     Matrix3d in(H * Pp * H.transpose() + R);
//     Matrix3d K(Pp * H.transpose() * in.inverse());
//     Vector3d res(z_meas - H * xp);
//     x = xp + K * res;
//     P = Pp - K * H * Pp;

//     Vector6d vec_joined;
//     vec_joined << x(0), x(1), x(2), euler_rates(0), euler_rates(1), euler_rates(2);
//     return vec_joined;
// }

VectorXd kalman_filters::LKF(VectorXd z_meas)
{
    Vector6d xp(A*x);
    Matrix6d Pp(A*P*A.transpose() + Q);
    Matrix3d in(H*Pp*H.transpose() + R);
    Matrix63d K(Pp*H.transpose()*in.inverse());
    x = xp + K * (z_meas - H * xp);
    P = (Matrix6d::Identity() - K * H) * Pp;

    return x;
}

VectorXd kalman_filters::LKF_2(VectorXd meas, VectorXd input_u)
{
    VectorXd xp(A_*x_hat + B_*input_u);
    MatrixXd Pp(A_*P_*A_.transpose() + Q_);
    MatrixXd in(H_*Pp*H_.transpose() + R_);
    MatrixXd K(Pp*H_.transpose()*in.inverse());
    x_hat = xp + K * (meas - H_*xp);
    P_ = (Matrix9d::Identity() - K * H_) * Pp;

    return x_hat;
}

// Vector2d kalman_filters::LKF_2(Vector2d meas)
// {
//     ///Predict
//     x2 = A2*x2 + B2*meas(1);
//     P2 = A2*P2*A2.transpose() + Q2;
//     ///Update
//     double in2 = H2*P2*H2.transpose() + R2;
//     ///K = P/(P+R);
//     Vector2d K2;
//     K2 = P2 * H2.transpose() / in2;
//     double res2 = meas(0) - H2*x2;
//     x2 = x2 + K2 * res2;
//     P2 = (Matrix2d::Identity() - K2*H2)*P2;

//     return x2;
// }

Vector4d kalman_filters::Quat_Pose_Estimator(Vector3d z_meas, Vector3d rates)
{
    Matrix4d w;
    w <<       0, -rates(0), -rates(1), -rates(2),
        rates(0),         0,  rates(2), -rates(1),
        rates(1), -rates(2),         0,  rates(0),
        rates(2),  rates(1), -rates(0),         0;
    Matrix4d A_(Matrix4d::Identity() + m_dt * 1/2 * w);
    Vector4d z(euler2quat(z_meas(0), z_meas(1), z_meas(2)));

    Vector4d xp(A_*x);
    Matrix4d Pp(A_*P*A_.transpose() + Q);
    Matrix4d in(H*Pp*H.transpose() + R);
    Matrix4d K(Pp*H.transpose()*in.inverse());
    x = xp + K * (z - H * xp);
    P = Pp - K * H * Pp;

    return x;
}

// Vector6d kalman_filters::get_euler_values(Vector3d rates)
// {
//     Vector3d euler(quaternion2euler(x));
//     euler_rates << angvel2eulerrates_matrix(euler(0), euler(1)) * rates;
//     Vector6d ang_pos_rates;
//     ang_pos_rates << euler(0),euler(1),euler(2), euler_rates(0), euler_rates(1), euler_rates(2);
//     return ang_pos_rates;
// }
void kalman_filters::set_bias(VectorXd bias_A_W)
{
x_hat_.acc_bias = Vector3d(bias_A_W(0),bias_A_W(1),bias_A_W(2));
x_hat_.gyr_bias = Vector3d(bias_A_W(3),bias_A_W(4),bias_A_W(5));
}

void kalman_filters::ins_eskf(VectorXd imu)
{
    Vector3d acc_meas(imu(0), imu(1), imu(2));//mapping acc_sensor to body coordinates
    Vector3d gyro_meas(imu(3), imu(4), imu(5));

    //position
    x_hat_.position = x_hat_.position + x_hat_.velocity * m_dt + (x_hat_.R_matrix*(acc_meas-x_hat_.acc_bias) + grav_vect) * pow(m_dt,2)/2;

    //Velocity
    x_hat_.velocity = x_hat_.velocity + (x_hat_.R_matrix*(acc_meas-x_hat_.acc_bias) + grav_vect) * m_dt;

    ///Attitude
    x_hat_.R_matrix = x_hat_.R_matrix * (Matrix3d::Identity() + get_skew_matrix(gyro_meas-x_hat_.gyr_bias) * m_dt);  

    // ////////////////////// Error states Jacobian //////////////////////////////////////
    MatrixXd F; F.setIdentity(15,15);
    ////////////////position
    F.block(0,3,3,3)   = Matrix3d::Identity() * m_dt;
    // //////////////velocity
    F.block(3,6,3,3)   =-get_skew_matrix(x_hat_.R_matrix*(acc_meas-x_hat_.acc_bias)+ grav_vect) * m_dt;
    F.block(3,9,3,3)   = x_hat_.R_matrix * m_dt;
    // // // ////////////////attitude
    F.block(6,12,3,3)  = x_hat_.R_matrix * m_dt;
    
    //P_ = F*P_*F.transpose() + G*Q_*G.transpose();
    P_ = F*P_*F.transpose() + Q_;
    //P_ = F * (P_ + 0.5 * Q_) * F.transpose() + 0.5 * Q_;

    //cout << "Orientation Covariance Matrix: \n" << P_.block(0,0,9,9) << endl << endl;

}

void kalman_filters::gnss_eskf(VectorXd gps)
{
    VectorXd e_x;
    e_x.setZero(15);
    
    MatrixXd S(H_*P_*H_.transpose() + R_);
    MatrixXd K(P_*H_.transpose()*S.inverse());
    e_x = K * (gps - x_hat_.position);

    //P_ = (MatrixXd::Identity(15,15) - K * H_) * P_;
    P_ = (MatrixXd::Identity(15,15) - K * H_) * P_ * (MatrixXd::Identity(15,15) - K * H_).transpose() + K*R_*K.transpose();
    //cout << e_x << endl << endl;
    x_hat_.position = x_hat_.position - Vector3d(e_x(0), e_x(1), e_x(2));
    x_hat_.velocity = x_hat_.velocity - Vector3d(e_x(3), e_x(4), e_x(5));
    //x_hat_.R_matrix = RPY_ROT(e_x(6), e_x(7), e_x(8)) * x_hat_.R_matrix;
    x_hat_.R_matrix = (Matrix3d::Identity() - get_skew_matrix(Vector3d(e_x(6), e_x(7), e_x(8)))) * x_hat_.R_matrix;
    x_hat_.acc_bias = x_hat_.acc_bias + Vector3d(e_x(9), e_x(10), e_x(11)); 
    x_hat_.gyr_bias = x_hat_.gyr_bias + Vector3d(e_x(12), e_x(13), e_x(14));
}

void kalman_filters::ins_eskf2(VectorXd imu, Matrix3d C_bi)
{
    Vector3d acc_meas(imu(0), imu(1), imu(2));

    //position
    x_hat_.position = x_hat_.position + x_hat_.velocity * m_dt + (C_bi*(acc_meas-x_hat_.acc_bias) + grav_vect) * pow(m_dt,2)/2;

    //Velocity
    x_hat_.velocity = x_hat_.velocity + (C_bi*(acc_meas-x_hat_.acc_bias) + grav_vect) * m_dt;

    // ////////////////////// Error states Jacobian //////////////////////////////////////
    MatrixXd F; F.setIdentity(9,9);
    ////////////////position
    F.block(0,3,3,3)   = Matrix3d::Identity() * m_dt;
    // //////////////velocity
    F.block(3,6,3,3)   = C_bi * m_dt;
    
    //P_ = F*P_*F.transpose() + G*Q_*G.transpose();

    P_2 = F*P_2*F.transpose() + Q_2;
    //P_ = F * (P_ + 0.5 * Q_) * F.transpose() + 0.5 * Q_;

    //cout << "Orientation Covariance Matrix: \n" << P_.block(0,0,9,9) << endl << endl;

}

void kalman_filters::gnss_eskf2(VectorXd gps)
{
    VectorXd e_x;
    e_x.setZero(9);
    
    MatrixXd S(H_2*P_2*H_2.transpose() + R_);
    MatrixXd K(P_2*H_2.transpose()*S.inverse());
    e_x = K * (gps - x_hat_.position);

    //P_ = (MatrixXd::Identity(15,15) - K * H_) * P_;
    P_2 = (MatrixXd::Identity(9,9) - K * H_2) * P_2 * (MatrixXd::Identity(9,9) - K * H_2).transpose() + K*R_*K.transpose();

    x_hat_.position = x_hat_.position - Vector3d(e_x(0), e_x(1), e_x(2));
    x_hat_.velocity = x_hat_.velocity - Vector3d(e_x(3), e_x(4), e_x(5));
    x_hat_.acc_bias = x_hat_.acc_bias + Vector3d(e_x(6), e_x(7), e_x(8)); 

}