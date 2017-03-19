#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {

public:
  FusionEKF();
  virtual ~FusionEKF();
  void Init(const MeasurementPackage &measurement_pack);
  void Update(const MeasurementPackage &measurement_pack);
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  KalmanFilter EKF;

private:
  bool is_initialized_;
  long previous_timestamp_;
  Tools tools;
  MatrixXd Rl;
  MatrixXd Rr;
  MatrixXd Hl;
  MatrixXd F;
  MatrixXd P;
  MatrixXd Q;
};

#endif /* FusionEKF_H_ */
