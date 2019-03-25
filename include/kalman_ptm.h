//for potentiometer
#ifndef KALMAN_PTM_H
#define KALMAN_PTM_H

#ifdef __cplusplus
#include <kalman_filter.h>

class KalmanPtm : public KalmanFilter{
public:
  KalmanPtm():KalmanFilter(){};
  virtual void InitFilter();
};
#endif

//wrapper functions for c implementation
#ifdef __cplusplus
extern "C" {
#endif

  void* createKalmanPtm();
  void initKalmanPtm( void *kalman_ptm);
  void updateKalmanPtm(void *kalman_ptm, double y_in[3]);


#ifdef __cplusplus
}
#endif

#endif //end of KALMAN_PTM_H
