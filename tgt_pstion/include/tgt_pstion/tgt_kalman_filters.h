#ifndef TGT_KALMAN_FILTERS_H
#define TGT_KALMAN_FILTERS_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <math.h>

using namespace std;
using namespace Eigen;

class tgt_kalman_filters
{
private:
  // kalman variables //
  const int x, y, z, n, m;
  const double dt;

  Vector3d Xhat;

  Matrix3d P, F;

  MatrixXd Q;

  MatrixXd R;


public:
    tgt_kalman_filters();
    virtual ~tgt_kalman_filters();
    Vector3d kalman_unscented_estimator(Vector3d& pstion, Vector2d& meas);
    Vector3d kalman_extended_estimator(Vector3d& pstion, Vector2d& meas);
    Vector3d kalman_unscented_estimator_2(Vector3d& pstion, Vector2d& meas);
    Vector3d kalman_extended_estimator_2(Vector3d& pstion, Vector2d& meas);
    void SigmaPoints(MatrixXd& Xi, MatrixXd& W);
    Vector3d fx(MatrixXd x_, double dt);
    Vector2d hx(MatrixXd x_, Vector3d pstion);
    void UT(MatrixXd&, MatrixXd&, MatrixXd&, MatrixXd&, MatrixXd&); //MatrixXd& noiseCov, Vector3d& xm, Matrix3d& cov
    double mod(double a, double m_);

};

#endif // TGT_KALMAN_FILTERS_H
