#ifndef KALMAN_ENCODER_H
#define KALMAN_ENCODER_H

#ifdef __cplusplus
#include <kalman_filter.h>

class KalmanEcd : public KalmanFilter{
public:
  KalmanEcd():KalmanFilter(){};
  virtual void InitFilter();
};
#endif

//wrapper functions for c implementation
#ifdef __cplusplus
extern "C" {
#endif

  void* KalmanEcd_c();
  void initKalmanEcd( void *kalman_ecd);
  void updateKalmanEcd(void *kalman_ecd, double y_in[3]);


#ifdef __cplusplus
}
#endif

#endif //end of KALMAN_ENCODER_H
