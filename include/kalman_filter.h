#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
#include <Eigen/Dense>

class KalmanFilter {
public:
  KalmanFilter(){};
  struct Kalman
  {
    //x - state, p -covariance matrix
    //A_d - state transition matrix, B_d - control input Matrix
    Eigen::Matrix3d I, Q_d, A_d, P;
    double R_d, dt, u;
    Eigen::Vector3d x, B_d, C;
    //intermediate values
    Eigen::Vector3d x_pred;
    Eigen::Matrix3d P_pred;
  };

  virtual void InitFilter();
  void UpdateFilter(double y_in[3]);
  Kalman ft;
private:
  void Predict();
  void Correct(double y_in[3]);
};
#endif


#endif //end of KALMAN_FILTER_H
