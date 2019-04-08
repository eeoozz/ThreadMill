#include "afo.h"
#include "digital_filters.h"
#include <stdio.h>

int main(){
    double dt = 0.005;
    double A[5];
    //A = (double*)malloc(6* sizeof(double));
    double B[5];
    init_digital_filters();
    param_ff_ctrl(&B, &A);
    double out;
    for( size_t n=0 ; n < 2000 ; ++n )//40s offline simulation
    {
      out = digital_filter_tf(sin(2*M_PI*dt*n), 4,
      	&B, &A, 0);
      //double input[3] = {sin(2*M_PI*dt*n), 0, 0};
      printf("input: %f output: %f \n", sin(2*M_PI*dt*n), out);
    }
    free_digital_filters();
    return 0;
}
