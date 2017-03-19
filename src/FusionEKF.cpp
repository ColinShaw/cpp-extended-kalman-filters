#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {

  is_initialized_ = false;
  previous_timestamp_ = 0;

  // Measurement covariance
  Rl = MatrixXd(2, 2); 
  Rl << 0.01, 0.0,
        0.0,  0.01;

  Rr = MatrixXd(3, 3);	
  Rr << 0.01, 0.0,    0.0,
        0.0,  1.0e-6, 0.0,
        0.0,  0.0,    0.01;

  // Lidar measurement function
  Hl = MatrixXd(2, 4);	
  Hl << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0;

  // State covariance
  P = MatrixXd(4, 4);
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1000, 0,
       0, 0, 0, 1000;

  // Process covariance (updated later)
  Q = MatrixXd::Zero(4, 4);  

  // Kinematic transform (delta t updated in RUQ later)
  F = MatrixXd::Identity(4, 4);
}

FusionEKF::~FusionEKF() {}


void FusionEKF::Init(const MeasurementPackage &measurement_pack) {
  VectorXd x_in = VectorXd(4);
  previous_timestamp_ = measurement_pack.timestamp_;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    VectorXd radar = measurement_pack.raw_measurements_;
    double rho = radar[0];
    double phi = radar[1];
    double rho_dot = radar[2]; 
    double x = rho * cos(phi);
    double y = rho * sin(phi);
    double x_dot = rho_dot * cos(phi);
    double y_dot = rho_dot * sin(phi);
    x_in << x, y, x_dot, y_dot;
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    VectorXd lidar = measurement_pack.raw_measurements_;
    double x = lidar[0];
    double y = lidar[1];
    x_in << x, y, 0.0, 0.0;
  }

  EKF.Init(x_in, P, F, Q);
  is_initialized_ = true;
}


void FusionEKF::Update(const MeasurementPackage &measurement_pack) {
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  EKF.SetDeltaT(dt);
  EKF.UpdateQ(5.0, 5.0, dt); 
  EKF.Predict();

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    VectorXd x = EKF.GetX();
    MatrixXd Hr = tools.CalculateJacobian(x);
    VectorXd Hx = tools.CalculateHx(x);
    EKF.UpdateRadar(measurement_pack.raw_measurements_, Hr, Hx, Rr);
  }
  else {
    EKF.UpdateLidar(measurement_pack.raw_measurements_, Hl, Rl);
  }
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    Init(measurement_pack);
  }
  else {
    Update(measurement_pack);
  }
}
