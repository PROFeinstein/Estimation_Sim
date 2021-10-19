#include "tgt_pstion/tgt_kalman_filters.h"

tgt_kalman_filters::tgt_kalman_filters():x(0), y(1), z(2), n(3), m(2), dt(0.05), Q(3,3), R(2,2)
{
    //Kalman Filter Constructor
  Xhat << 10, 10, 10;

  P = Matrix3d::Identity() * 20;

  F = Matrix3d::Identity();

  //Q = Matrix3d::Identity() * 0.01;
  Q << 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // R << 3.55E-5,  0.0,
  //      0.0,     1.086E-8;
  // R << pow(0.01,2),            0.0,
  //                0.0, pow(0.03135,2);
  R << pow(0.1,2),            0.0,
                0.0, pow(0.06,2); //0.055

  }
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
tgt_kalman_filters::~tgt_kalman_filters()
{

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double tgt_kalman_filters::mod(double a, double m_)
{
     return a - m_*floor(a/m_);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Vector3d tgt_kalman_filters::fx(MatrixXd x_, double dt)
{
  return F * x_ ;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Vector2d tgt_kalman_filters::hx(MatrixXd x_, Vector3d pstion)
{
    Vector3d r(pstion - x_);
    //Vector2d h( mod(atan2(r(y),r(x)),2*M_PI) , atan2(r(z),sqrt(pow(r(x),2)+pow(r(y),2))) );
    Vector2d h( atan(r(y)/r(x)) , atan(r(z)/sqrt(pow(r(x),2)+pow(r(y),2))) );
    return h;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tgt_kalman_filters::SigmaPoints(MatrixXd& Xi, MatrixXd &W)
{
 int kappa = 0;

 Xi.col(0) = Xhat;
 W(0) = kappa / (n+kappa);

 Matrix3d P_;
 P_ = (n+kappa)*P;
 Matrix3d U( P_.llt().matrixU() );

 for (int k = 0; k<n; k++)
     {
      Xi.col(k+1) = Xhat + U.row(k).transpose();
      W(k+1)      = (double)1/(2 * (n+kappa));
     }

 for (int k = 0; k<n; k++)
     {
      Xi.col(n+k+1) = Xhat - U.row(k).transpose();
      W(n+k+1)    = (double)1 / (2*(n+kappa));
     }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tgt_kalman_filters::UT(MatrixXd& Xi, MatrixXd& W, MatrixXd& noiseCov, MatrixXd &xm, MatrixXd &cov)
{

  for (int k=0; k<Xi.cols(); k++)
       xm = xm + W(k)*Xi.col(k);

  for (int k=0; k<Xi.cols(); k++)
      {
       VectorXd temp(Xi.col(k) - xm);
       cov = cov + W(k) * (temp * temp.transpose());
      }
  cov = cov + noiseCov;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Vector3d tgt_kalman_filters::kalman_unscented_estimator(Vector3d& pstion, Vector2d& meas)
  {

    MatrixXd Xi(n, 2*n+1);
    Xi.setZero();
    MatrixXd W(2*n+1, 1);
    W.setZero();
    MatrixXd xp(n,1); xp.setZero();
    MatrixXd Pp(n,n); Pp.setZero();
    MatrixXd zp(m,1); zp.setZero();
    MatrixXd Pz(m,m); Pz.setZero();


    SigmaPoints(Xi, W);

    //cout << Xi <<endl<<endl;//<< W <<endl<<endl;

    MatrixXd fXi;
    fXi.resize(Xi.rows(),Xi.cols());

    for (int k = 0; k < 2*n+1; k++ )
         fXi.col(k) = fx(Xi.col(k), dt);

    UT(fXi, W, Q, xp, Pp);

    MatrixXd hXi;
    hXi.resize(m, Xi.cols());

    for (int k = 0; k < 2*n+1; k++ )
         hXi.col(k) = hx(Xi.col(k), pstion);


    UT(hXi, W, R, zp, Pz);


    MatrixXd Pxz(n,m);
    Pxz.col(0) << 0, 0, 0;
    Pxz.col(1) << 0, 0, 0;

    for (int k = 0; k < 2*n+1; k++ )
        Pxz = Pxz + W(k) * ( fXi.col(k) - xp ) * ( hXi.col(k) - zp ).transpose();

    MatrixXd K;
    K = Pxz * Pz.inverse();

    Xhat = xp + K*(meas - zp);

    P = Pp - K*Pz*K.transpose();

    //cout << "Unscented Kalman Filter :\n" << Xhat <<endl;
    // cout << "Prediction: \n" << zp << endl;
    // cout << "Measurement: \n" << meas << endl;

    return Xhat;

  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Vector3d tgt_kalman_filters::kalman_extended_estimator(Vector3d& pstion, Vector2d& meas)
    {
      Xhat = F * Xhat;
      Vector3d r(pstion - Xhat);

      Vector2d h( atan(r(y)/r(x))  , atan(r(z)/sqrt(pow(r(x),2)+pow(r(y),2))) );

      MatrixXd H(2,3);
      H << -r(y)/(pow(r(x),2)+pow(r(y),2)), r(x)/(pow(r(x),2)+pow(r(y),2)), 0,
          (r(x)*r(z))/(sqrt(pow(r(x),2)+pow(r(y),2))*(pow(r(x),2)+pow(r(y),2)+pow(r(z),2))),
          (r(y)*r(z))/(sqrt(pow(r(x),2)+pow(r(y),2))*(pow(r(x),2)+pow(r(y),2)+pow(r(z),2))),
          -sqrt(pow(r(x),2)+pow(r(y),2))/(pow(r(x),2)+pow(r(y),2)+pow(r(z),2));

      P  = F*P*F.transpose() + Q;
      Matrix2d in( H*P*H.transpose() + R );
      MatrixXd K(3,2);
      K = P * H.transpose() * in.inverse();
      Vector2d res(meas - h);
      Xhat = Xhat + K * res;
      P = (Matrix3d::Identity() - K*H)*P;

    cout << "Extended Kalman Filter :\n" << Xhat <<endl;
    cout << "Prediction: \n" << h << endl;
    cout << "Measurement: \n" << meas << endl;
    
    return Xhat;
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
