#include <iostream>
#include <math.h>
#include <cmath> 
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in) {
  uint32_t size = x_in.size();
  x_ = x_in;
  P_ = P_in;
  F_ = MatrixXd::Identity(size, size);
  Q_ = MatrixXd(size, size);
  I_ = MatrixXd::Identity(size, size);
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
  VectorXd y = z - H * x_;
  UpdateY(y, H, R);
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  VectorXd h_x(3);
  double ro = sqrt(pow(px, 2) + pow(py, 2));
  h_x << ro,
         atan2(py, px),
         (px * vx + py * vy) / ro;

  VectorXd y = z - h_x;
  y[1] = fmod(y[1], (2 * M_PI));

  UpdateY(y, H, R);
}

void KalmanFilter::UpdateY(const Eigen::VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = P_ * Ht * S.inverse();
  
  x_ = x_ + K * y;
  P_ = (I_ - K * H) * P_;
}