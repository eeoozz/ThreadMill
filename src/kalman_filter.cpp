#include <kalman_filter.h>

void KalmanFilter::InitImuFilter() {
  imu_f_.I << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  imu_f_.Q_d << 0.00201469, 0.251836,  16.7891,
                0.251836,  33.5782,  2518.36,
                16.7891,  2518.36, 251836.0;
  imu_f_.R_d = 0.0014925;
  imu_f_.A_d << 0, 1, 0,
                0, 0, 1,
                0, 0, 0;
  imu_f_.B_d = {0, 0, 1};
  imu_f_.C = {1, 0, 0};
  imu_f_.dt = 0.01;
  imu_f_.u = 0;
  //initial value
  imu_f_.P << 1, 2, 3,
              2, 4, 6,
              3, 6, 9;
  imu_f_.x = {0, 0, 0};
}

void KalmanFilter::UpdateImu(double y_in) {
  Predict();
  Correct(y_in);
}

void KalmanFilter::Predict() {
  imu_f_.x_pred = imu_f_.A_d * imu_f_.x + imu_f_.B_d * imu_f_.u;
  imu_f_.P_pred = imu_f_.A_d * imu_f_.P * imu_f_.A_d.transpose() + imu_f_.Q_d;
}

void KalmanFilter::Correct(double y_in) {
  Eigen::Vector3d L_k;
  L_k = imu_f_.P_pred * imu_f_.C * (imu_f_.C.transpose() * imu_f_.P_pred * imu_f_.C + imu_f_.R_d);
  //correct and update
  imu_f_.x = imu_f_.x_pred + L_k * (y_in - imu_f_.x_pred[0]);
  imu_f_.P = (imu_f_.I - L_k * imu_f_.C.transpose()) * imu_f_.P_pred;
}

//wrapper functions for c implementation
void* createKalmanImu() {
  KalmanFilter *out(new KalmanFilter);
  return( reinterpret_cast< void* >( out ) );
}

void initKalmanImu( void *kalman_imu) {
  reinterpret_cast< KalmanFilter* >( kalman_imu )->InitImuFilter();
}

void updateKalmanImu(void *kalman_imu, double y_in) {
  reinterpret_cast< KalmanFilter* >( kalman_imu )->UpdateImu(y_in);
}
