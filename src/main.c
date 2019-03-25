#include "afo.h"
#include "kalman_filter.h"
#include <stdio.h>

int main(){
    void *temp_afo = createAfo();
    initAfo(temp_afo,0.005);
    for( int n=0 ; n < 8000 ; ++n )//40s offline simulation
      stepAfo(temp_afo,sin(2*M_PI*0.005*n));

    double dt = 0.01;
    void *temp_kalmanImu = createKalmanImu();
    initKalmanImu(temp_kalmanImu);
    for( size_t n=0 ; n < 2000 ; ++n )//40s offline simulation
    {
      updateKalmanImu(temp_kalmanImu, sin(2*M_PI*dt*n));
    }
    return 0;
}
