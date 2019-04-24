#ifndef TORQUE_AMPLITUDE_H
#define TORQUE_AMPLITUDE_H

#ifdef __cplusplus
#include <math.h>

class TorqueAmp{
public:
  TorqueAmp(){};
  double* velFactor(double f_afo, double ampl_ang);

  //output of the block
  double outTqAmp[2]; //v_ref, ampl_fact
private:
  double l_leg = 0.85;
  double n_tau = 3;
  double v_lim = 0.7;
};
#endif

//wrapper functions for c implementation
#ifdef __cplusplus
extern "C" {
#endif

  void* TorqueAmp_c();
  double* velFactor_c(void *afo, double f_afo, double ampl_ang);


#ifdef __cplusplus
}
#endif

#endif
