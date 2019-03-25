#include <kalman_imu.h>

void KalmanImu::InitFilter() {
  ft.dt = 0.02;
  ft.I << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;
  ft.Q_d << 0.00201469, 0.251836,  16.7891,
            0.251836,  33.5782,  2518.36,
            16.7891,  2518.36, 251836.0;
  ft.R_d = 0.0014925; //0.0014925
  ft.A_d << 1, ft.dt, 0.5*ft.dt*ft.dt,
            0, 1, ft.dt,
            0, 0, 1;
  ft.B_d = {ft.dt*ft.dt*ft.dt/6, ft.dt*ft.dt/2, ft.dt};
  ft.C = {1, 0, 0};
  ft.u = 0;
  //initial value
  ft.P << 1, 2, 3,
          2, 4, 6,
          3, 6, 9;
  ft.x = {0, 0, 0};
}


//wrapper functions for c implementation
void* createKalmanImu() {
  KalmanImu *out(new KalmanImu);
  return( reinterpret_cast< void* >( out ) );
}

void initKalmanImu( void *kalman_imu) {
  reinterpret_cast< KalmanImu* >( kalman_imu )->InitFilter();
}

void updateKalmanImu(void *kalman_imu, double y_in[]) {
  reinterpret_cast< KalmanImu* >( kalman_imu )->UpdateFilter(y_in);
}
