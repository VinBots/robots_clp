#include "kalman_filter.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_laser_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
  */
  
  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
   //TODO: update the state by using Extended Kalman Filter equations
   
  //VectorXd z_pred = H_ * x_;
  VectorXd z_pred;
  z_pred = VectorXd(3);
  
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double px2py2 = px*px+py*py;
  
  if (px2py2!=0){
    
  // calculate z_pred vector element = h(x')
  double rho_pred = sqrt(px2py2);
  double phi_pred = atan2(py,px);
  double rhodot_pred = (px * vx + py*vy) / sqrt(px2py2);
  
  
  z_pred << rho_pred, phi_pred,rhodot_pred;

  VectorXd y = z - z_pred;
  y(1) = tools.Angle_Normalizer(y(1));
  //
  
  MatrixXd Hj = tools.CalculateJacobian (x_);
  MatrixXd Hjt = Hj.transpose();
  
    MatrixXd S = Hj * P_ * Hjt + R_radar_;

  MatrixXd Si = S.inverse();
  MatrixXd PHjt = P_ * Hjt;
  MatrixXd K = PHjt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
    //std::cout<<"Identity I : "<< I<<"\n";
    //std::cout<<"K : "<<K<<"\n";
    //std::cout<<"Hj : "<<Hj<<"\n";
    //std::cout<<"P : "<<P_<<"\n";
    P_ = (I - K * Hj) * P_;
  }
  else
  {
    std::cout<<"Division by zero!";
}
}



