#include <kalman_ptm.h>

void KalmanPtm::InitFilter() {
  ft.dt = 0.005;
  ft.I << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;
  ft.Q_d << 1.20255e-13, 6.01273e-11, 1.60339e-8,
            6.01273e-11, 3.20679e-8, 0.00000962036,
            1.60339e-8, 0.00000962036, 0.00384815;
  ft.R_d = 1.59623e-23;
  ft.A_d << 1.0000, 0.0050, 0.0000,
            0, 1.0000, 0.0050,
            0, 0, 1.0000;
  ft.B_d = {0, 0, 0.005};
  ft.C = {1, 0, 0};
  ft.u = 0;
  //initial value
  ft.P << 5.9801e-10, 5.9801e-10, 5.5551e-10,
          5.9801e-10, 5.9801e-10, 5.5551e-10,
          5.5551e-10, 5.5551e-10, 0.00050861;
  ft.x = {0, 0, 0};
}


//wrapper functions for c implementation
void* createKalmanPtm() {
  KalmanPtm *out(new KalmanPtm);
  return( reinterpret_cast< void* >( out ) );
}

void initKalmanPtm( void *kalman_ptm) {
  reinterpret_cast< KalmanPtm* >( kalman_ptm )->InitFilter();
}

void updateKalmanPtm(void *kalman_ptm, double y_in[]) {
  reinterpret_cast< KalmanPtm* >( kalman_ptm )->UpdateFilter(y_in);
}
