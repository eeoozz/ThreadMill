#include <kalman_encoder.h>

void KalmanEcd::InitFilter() {
  ft.dt = 0.005;
  ft.I << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;
  ft.Q_d << 4.70683e-13,  2.35342e-10,   6.27578e-8,
            2.35342e-10,   1.25516e-7, 0.0000376547,
            6.27578e-8, 0.0000376547,    0.0150619;
  ft.R_d = 4.851e-8;
  ft.A_d << 1.0000, 0.0050, 0.0000,
            0, 1.0000, 0.0050,
            0, 0, 1.0000;
  ft.B_d = {0, 0, 0.005};
  ft.C = {1, 0, 0};
  ft.u = 0;
  //initial value
  ft.P << 2.24671e-9, 2.24671e-9, 2.12799e-9,
          2.24671e-9, 2.24671e-9, 2.12799e-9,
          2.12799e-9, 2.12799e-9, 0.00141885;
  ft.x = {0, 0, 0};
}


//wrapper functions for c implementation
void* createKalmanEcd() {
  KalmanEcd *out(new KalmanEcd);
  return( reinterpret_cast< void* >( out ) );
}

void initKalmanEcd( void *kalman_ecd) {
  reinterpret_cast< KalmanEcd* >( kalman_ecd )->InitFilter();
}

void updateKalmanEcd(void *kalman_ecd, double y_in[]) {
  reinterpret_cast< KalmanEcd* >( kalman_ecd )->UpdateFilter(y_in);
}
