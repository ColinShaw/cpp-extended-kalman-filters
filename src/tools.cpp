#include <iostream>
#include "tools.h"

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  for (int i=0; i<estimations.size(); i++){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse /= (double)estimations.size();
  rmse = rmse.array().sqrt(); 
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x) {

  MatrixXd Hj = MatrixXd(3,4);

  double px = x(0);
  double py = x(1);
  double vx = x(2);
  double vy = x(3);
  double s2 = px*px + py*py;
  double s = sqrt(s2);
  double s3 = s * s2;

  if (s2 < 0.0001) {
    Hj << 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0;
  }
  else {
    Hj << px/s,                py/s,                0.0,  0.0,
          -py/s2,              px/s2,               0.0,  0.0,
          py*(vx*py-vy*px)/s3, px*(vy*px-vx*py)/s3, px/s, py/s;
  }

  return Hj;
}

VectorXd Tools::CalculateHx(const VectorXd &x){

  VectorXd Hx = VectorXd(3);

  double px = x(0);
  double py = x(1);
  double vx = x(2);
  double vy = x(3);

  double phi = sqrt(px*px + py*py);
  double psi = atan2(py, px);
  double phi_dot = (px*vx + py*vy) / phi;

  if (phi < 0.0001) {
    Hx << 0.0, 0.0, 0.0;
  }
  else {
    Hx << phi, psi, phi_dot;
  }

  return Hx;
}
