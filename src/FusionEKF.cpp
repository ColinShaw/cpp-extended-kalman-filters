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
  Rl << 0.2, 0.0,
        0.0, 0.2;

  Rr = MatrixXd(3, 3);	
  Rr << 3.0, 0.0, 0.0,
        0.0, 7.0, 0.0,
        0.0, 0.0, 1.0;

  // Lidar measurement function
  Hl = MatrixXd(2, 4);	
  Hl << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0;

  // Radar measurement function Jacobian placeholder
  Hr = MatrixXd(3, 4);

  // State covariance
  P = MatrixXd(4, 4);
  P << 100.0, 0.0,   0.0,     0.0,
       0,     100.0, 0.0,     0.0,
       0,     0,     10000.0, 0.0,
       0,     0,     0.0,     10000.0;

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
  EKF.UpdateQ(5000.0, 5000.0, dt); 
  EKF.Predict();

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    Hr = tools.CalculateJacobian(EKF.GetX());
    EKF.Update(measurement_pack.raw_measurements_, Hr, Rr);
  }
  else {
    EKF.Update(measurement_pack.raw_measurements_, Hl, Rl);
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
