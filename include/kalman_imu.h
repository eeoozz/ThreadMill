#ifndef KALMAN_IMU_H
#define KALMAN_IMU_H

#ifdef __cplusplus
#include <kalman_filter.h>

class KalmanImu : public KalmanFilter{
public:
  KalmanImu():KalmanFilter(){};
  virtual void InitFilter();
};
#endif

//wrapper functions for c implementation
#ifdef __cplusplus
extern "C" {
#endif

  void* createKalmanImu();
  void initKalmanImu( void *kalman_imu);
  void updateKalmanImu(void *kalman_imu, double y_in[3]);


#ifdef __cplusplus
}
#endif

#endif //end of KALMAN_IMU_H
