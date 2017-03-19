#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;


class KalmanFilter {

private:
  VectorXd x;
  MatrixXd P; 
  MatrixXd F;
  MatrixXd Q;

public:
  KalmanFilter();
  virtual ~KalmanFilter();
  void Init(VectorXd &x_in,  MatrixXd &P_in,  MatrixXd &F_in, MatrixXd &Q_in); 
  void SetDeltaT(double dt);
  void UpdateQ(double ax, double ay, double dt);
  void Predict();
  void UpdateLidar(const VectorXd &z, const MatrixXd &H, const MatrixXd &R);
  void UpdateRadar(const VectorXd &z, const MatrixXd &H, const VectorXd &Hx, const MatrixXd &R);
  void PrintDebug();
  VectorXd GetX();
};

#endif /* KALMAN_FILTER_H_ */
