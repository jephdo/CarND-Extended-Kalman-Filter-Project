#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */


  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);


  float rho = sqrt(px * px + py * py);


  if (fabs(rho) < 0.001) { 
    cout << "Error while converting vector x_ to polar coordinates: Division by Zero" << endl; 
    rho = 0.001;
  }

  float phi = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;

  // cout << "rho/phi/rhodot: " << rho << " " << phi << " " << rho_dot << endl;
  // cout << px << " " << py << " " << vx << " "<< vy << endl;
  // cout << "rho: " << rho << endl;
  // cout << "phi: " << phi << endl;
  // cout << "rhodot: " << rho_dot << endl << endl;

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;

  // rho needs to be between -pi and pi
  // keep adding/subtracting 2pi until it is
  while (y(1) < -M_PI)
    y(1) += 2 * M_PI;
  while (y(1) > M_PI)
    y(1) -= 2* M_PI;


  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
