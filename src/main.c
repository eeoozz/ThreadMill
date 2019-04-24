#include "afo.h"
#include "kalman_imu.h"
#include "kalman_ptm.h"
#include "kalman_encoder.h"
#include <stdio.h>

int main(){
  /*
    void *temp_afo = AFO_c();
    AfoInit_c(temp_afo,0.005);
    for( int n=0 ; n < 8000 ; ++n )//40s offline simulation
      AfoStep_c(temp_afo,sin(2*M_PI*0.005*n));
*/
    double dt = 0.02;
    void *temp_kalmanImu = KalmanImu_c();
    initKalmanImu(temp_kalmanImu);
    for( size_t n=0 ; n < 2000 ; ++n )//40s offline simulation
    {
      double input[3] = {sin(2*M_PI*dt*n), 0, 0};
      updateKalmanImu(temp_kalmanImu, input);
    }
    return 0;
}
