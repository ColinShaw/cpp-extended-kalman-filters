#include <iostream>
#include "kalman_filter.h"

using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in,  MatrixXd &P_in,  MatrixXd &F_in, MatrixXd &Q_in) { 
  x  = x_in;
  P  = P_in;
  F  = F_in;
  Q  = Q_in;
}

void KalmanFilter::SetDeltaT(double dt) {
  F(0,2) = dt;
  F(1,3) = dt;
}

void KalmanFilter::UpdateQ(double ax, double ay, double dt) {
  double dt2 = dt *dt;
  double dt3 = dt * dt2;
  double dt4 = dt * dt3;

  Q << dt4*ax/4.0, 0.0,            dt3*ax/2.0, 0.0,
       0.0,            dt4*ay/4.0, 0.0,        dt3*ay/2.0,
       dt3*ax/2.0, 0.0,            dt2*ax,     0.0,
       0.0,            dt3*ay/2.0, 0.0,        dt2*ay;
}

void KalmanFilter::Predict() {
  x = F * x;
  P = F * P * F.transpose() + Q;
}

void KalmanFilter::UpdateRadar(const VectorXd &z, const MatrixXd &H, const VectorXd &Hx, const MatrixXd &R) {

  MatrixXd Ht = H.transpose();

  VectorXd y = z - Hx;
  MatrixXd S = H * P * Ht + R;
  MatrixXd K = P * Ht * S.inverse();

  x = x + K * y;
  long xs = x.size();
  MatrixXd I = MatrixXd::Identity(xs, xs);

  P = (I - K * H) * P;
}

void KalmanFilter::UpdateLidar(const VectorXd &z, const MatrixXd &H, const MatrixXd &R) {
  MatrixXd Ht = H.transpose();

  VectorXd y = z - H * x;
  MatrixXd S = H * P * Ht + R;
  MatrixXd K = P * Ht * S.inverse();

  x = x + K * y;
  long xs = x.size();
  MatrixXd I = MatrixXd::Identity(xs, xs);

  P = (I - K * H) * P;
}

void KalmanFilter::PrintDebug() {
  cout << "Step:" << endl;
  cout << "x:" << endl << x << endl;
  cout << "P:" << endl << P << endl;
  cout << endl;
}

VectorXd KalmanFilter::GetX() {
  return x;
}
