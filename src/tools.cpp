#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0) {
      cout << "The estimation vector size should not be zero" << endl;
      return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
      cout << "The estimation vector size should equal ground truth vector size" << endl;
      return rmse;
  }

  for (size_t i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  if (px == 0 && py == 0) {
      cout << "Error - division by zero" << endl;
      return Hj;
  }
  
  double px2py2 = px * px + py * py;
  double px2py2_sqrt = sqrt(px2py2);
  double px_div_sqrt = px / px2py2_sqrt;
  double py_div_sqrt = py / px2py2_sqrt;
  double vxpy = vx * py;
  double vypx = vy * px;
  Hj << px_div_sqrt, py_div_sqrt, 0, 0,
        - py / px2py2, px / px2py2, 0, 0,
        py * (vxpy - vypx) / pow(px2py2, 1.5), px * (vypx - vxpy) / pow(px2py2, 1.5), px_div_sqrt, py_div_sqrt;

  return Hj;
}
