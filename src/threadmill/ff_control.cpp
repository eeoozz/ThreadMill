#include <ff_control.h>

FFControl::FFControl():DigitalFilt(5) {
  //discrete filter number for digital filter
  double A_m[5] = {1.000000000000000e+00, -2.424652148014002e+00, 2.158880722275586e+00,
    -8.40791461848115e-01, 1.21259541582703e-01}; //den
  double B_m[5] = {1.1712603903505594e+01, -2.1610718269080696e+01, -1.530926754557027e+00,
    2.1618066591626327e+01, -1.0174328826402933e+01}; //num
  A = A_m;
  B = B_m;

  //for BiQuad
  bq1.set( 11.7126, 0.2604, -11.3522, -1.3869, 0.4504 );
  bq2.set( 1, -1.8759, 0.8962, -1.0377, 0.2692 );
  bqc = bq1 * bq2;
};

double FFControl::dfFFControlOut(double ampl_fact, double F_ref_d) {
  return ampl_fact * Filt(F_ref_d);
}

double FFControl::bqFFControlOut(double ampl_fact, double F_ref_d) {
  return ampl_fact * bqc.step(F_ref_d);
}

//wrapper functions for C implementation
void* FFControl_c() {
  FFControl *out( new FFControl);
  return( reinterpret_cast< void* >( out ) );
}

double  dfFFContrlOut_c(void* ffctrl, double ampl_fact, double F_ref_d) {
  double output = reinterpret_cast< FFControl* >( ffctrl )->dfFFControlOut(ampl_fact, F_ref_d);
  return output;
}

double  bqFFControlOut_c(void* ffctrl, double ampl_fact, double F_ref_d) {
  double output = reinterpret_cast< FFControl* >( ffctrl )->bqFFControlOut(ampl_fact, F_ref_d);
  return output;
}
