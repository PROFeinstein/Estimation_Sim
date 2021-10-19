//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#ifndef PELICAN_CTRL_PKG_PELICAN_DATA_TYPES_H
#define PELICAN_CTRL_PKG_PELICAN_DATA_TYPES_H

#include <math.h>
#include "eigen3/Eigen/Dense"

using namespace Eigen;

struct pose_attitude{
    Vector3d position;
    Vector3d velocity;
    Matrix3d R_matrix;
    Vector3d acc_bias;
    Vector3d gyr_bias;
};

Matrix3d X_ROT(double roll)
{
    Matrix3d ROT;
    ROT << 1,          0,         0,
           0,  cos(roll), sin(roll),
           0, -sin(roll), cos(roll);
    return ROT;
}

Matrix3d Y_ROT(double pitch)
{
    Matrix3d ROT;
    ROT << cos(pitch), 0, -sin(pitch),
                    0, 1,           0,
           sin(pitch), 0,  cos(pitch);
    return ROT;
}

Matrix3d Z_ROT(double yaw)
{
    Matrix3d ROT;
    ROT << cos(yaw), sin(yaw), 0,
          -sin(yaw), cos(yaw), 0,
                  0,        0, 1;
    return ROT;
}

Matrix3d RPY_ROT(double roll, double pitch, double yaw)
{
    Matrix3d ROT(X_ROT(roll)*Y_ROT(pitch)*Z_ROT(yaw));
    return ROT.transpose();
}

Matrix3d angvel2eulerrates_matrix(double roll, double pitch)
{
    Matrix3d transform;
    transform << 1, sin(roll)*tan(pitch), cos(roll)*tan(pitch),
                 0,            cos(roll),           -sin(roll),
                 0, sin(roll)/cos(pitch), cos(roll)/cos(pitch);
    return transform; 
}

Vector3d dcm2eulerangles(MatrixXd dcm)
{
    Vector3d euler_angles;
    Matrix3d dcm_(dcm.transpose());
    euler_angles << atan2(dcm_(1,2), dcm_(2,2)) , -asin(dcm_(0,2)), atan2(dcm_(0,1), dcm_(0,0));
    return euler_angles;
}

Vector3d quaternion2euler(Vector4d q_)
{
    double roll = (double) atan2(2*(q_(2)*q_(3) + q_(0)*q_(1)), 1-2*(q_(1)*q_(1) + q_(2)*q_(2)));
    double pitch = (double) -asin(2* (q_(1)*q_(3) - q_(0)*q_(2)));
    double yaw = (double) atan2(2*(q_(1)*q_(2) + q_(0)*q_(3)), 1-2*(q_(2)*q_(2) + q_(3)*q_(3)));
    Vector3d euler_angles(roll, pitch, yaw);
    return euler_angles;
}

Vector4d euler2quat(double phi, double theta, double psi)
{
    Vector4d q;
	double sinPhi   = sin(phi/2);
	double sinTheta = sin(theta/2);
	double sinPsi   = sin(psi/2);
	double cosPhi   = cos(phi/2);
	double cosTheta = cos(theta/2);
	double cosPsi   = cos(psi/2);

	q << cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi, sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi,
         cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi, cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi;
	return q;
}

Matrix3d get_skew_matrix(Vector3d temp)
{
    Matrix3d skew_temp; 
    skew_temp <<       0,-temp(2), temp(1),
                 temp(2),       0,-temp(0),
                -temp(1), temp(0),       0;
    return skew_temp; 
}

enum eulerAngles {phi, theta, psi};
double sec(double x){ return 1.0 / cos(x);}
const double acc_gravity = 9.8;
const Vector3d grav_vect(0, 0, -acc_gravity);

#endif //PELICAN_CTRL_PKG_PELICAN_DATA_TYPES_H
