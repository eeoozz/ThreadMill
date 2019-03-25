#include <kalman_filter.h>
#include <iostream>

void KalmanFilter::InitFilter() {
  ft.dt = 0.01;
  ft.I << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;
  ft.Q_d << 0, 0, 0,
            0, 0, 0,
            0, 0, 0;
  ft.R_d = 0;
  ft.A_d << 1, ft.dt, 0.5*ft.dt*ft.dt,
                0, 1, ft.dt,
                0, 0, 1;
  std::cout << "A_d:" << ft.A_d << std::endl;
  ft.B_d = {ft.dt*ft.dt*ft.dt/6, ft.dt*ft.dt/2, ft.dt};
  ft.C = {1, 0, 0};
  ft.u = 0;
  //initial value
  ft.P << 0, 0, 0,
          0, 0, 0,
          0, 0, 0;
  ft.x = {0, 0, 0};
}

void KalmanFilter::UpdateFilter(double y_in[3]) {
  KalmanFilter::Predict();
  KalmanFilter::Correct(y_in);
}

void KalmanFilter::Predict() {
  ft.x_pred = ft.A_d * ft.x + ft.B_d * ft.u;
  ft.P_pred = ft.A_d * ft.P * ft.A_d.transpose() + ft.Q_d;
  std::cout << "x predicted:" << ft.x_pred.transpose() << std::endl;
}

void KalmanFilter::Correct(double y_in[3]) {
  Eigen::Vector3d L_k, input{y_in[0], y_in[1], y_in[2]};
  L_k = ft.P_pred * ft.C /(ft.C.transpose() * ft.P_pred * ft.C + ft.R_d);
  //correct and update
  ft.x = ft.x_pred + L_k*(input.transpose()*ft.C - ft.C.transpose()*ft.x_pred);
  ft.P = (ft.I - L_k * ft.C.transpose()) * ft.P_pred;
  std::cout << "L_k:" << L_k.transpose() << std::endl;
  std::cout << "P matrix: " << ft.P << std::endl;
}
