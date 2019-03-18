#include "afo.h"
#include "kalman_filter.h"
#include <stdio.h>

int main(){
    void *temp_afo = createAfo();
    initAfo(temp_afo,0.005);
    for( int n=0 ; n < 8000 ; ++n )//40s offline simulation
      stepAfo(temp_afo,sin(2*M_PI*0.005*n));
    return 0;
}
