#include <afo.h>

int AFO::AfoInit(double dt) {
  x[0] = 0;  //phi
  x[1] = 0.4 * 2 * M_PI;  //omega, initial guessing value: 0.4Hz or 0.8pi/s
  x[2] = 1;  //k_alpha[0]
  this->dt = dt;
  t = 0.0;
  phi_reconstruct = 0.0;
  phi_input = 0.0;
  std::cout << "afo init successful" << std::endl;
  return 0;
}


double* AFO::AfoStep(double input) {
  phi_input = input;
  phi_reconstruct = 0.0;
  stepper.do_step( afo_sys , x , t , dt );
  t += dt;

  //reconstruct
  for (int count = 0; count < 11; count++) {
      phi_reconstruct += x[count+2] * cos(count*x[0]);
      phi_reconstruct += x[count+13] * sin(count*x[0]);
  }
  //update err
  afo_sys.err = phi_input - phi_reconstruct;
  //std::cout << "err:" << afo_sys.err << std::endl;

  //return phi_reconstruct;
  //sig_afo
  double int_part;
  afoOutput[0] = modf(x[0]/(2*M_PI),&int_part);
  //f_afo
  afoOutput[1] = x[1]/(2*M_PI); //omega/2pi
  //ampl_ang
  afoOutput[3] = x[3]/fact_afo; //alpha[1]/fact_afo
  //y_fund
  afoOutput[2] = cos(x[0]) * afoOutput[3];
  //y_rec
  afoOutput[4] = phi_reconstruct;
  return afoOutput;
}

//wrapper functions for c implementation
void* AFO_c() {
   AFO *out( new AFO);
   return( reinterpret_cast< void* >( out ) );
}

void AfoInit_c( void *afo, double dt) {
   int k = reinterpret_cast< AFO* >( afo )->AfoInit(dt);
}

double* AfoStep_c(void *afo, double input) {
  double* phi = reinterpret_cast< AFO* >( afo )->AfoStep(input);
  return phi;
}
