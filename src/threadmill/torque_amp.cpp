#include <torque_amp.h>

double* TorqueAmp::velFactor(double f_afo, double ampl_ang) {
  outTqAmp[0] = 4 * l_leg * sin(ampl_ang) * f_afo;  //v_ref
  outTqAmp[1] = 1 - exp(-n_tau / v_lim * outTqAmp[0]); //ampl_fact
}

//wrapper functions for c implementation
void* TorqueAmp_c() {
   TorqueAmp *tqamp( new TorqueAmp);
   return( reinterpret_cast< void* >( tqamp ) );
}

//currently just assume amplitude factor is 1, might change it in the future
double* velFactor_c(void *tqamp, double f_afo, double ampl_ang) {
  double* out = reinterpret_cast< TorqueAmp* >( tqamp )-> velFactor(f_afo, ampl_ang);
  return out;
}
