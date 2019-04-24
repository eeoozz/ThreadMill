#include <ff_control.h>
#include <iostream> //delettte

FFControl::FFControl() {
  //for BiQuad
  bq1.set( 11.7126, 0.2604, -11.3522, -1.3869, 0.4504 );
  bq2.set( 1, -1.8759, 0.8962, -1.0377, 0.2692 );
  bqc = bq1 * bq2;
}

double FFControl::dfFFControlOut(double ampl_fact, double f_ref_d) {
  f_ref_filt = dgfilter.Filt(f_ref_d);
  return ampl_fact * f_ref_filt * k_ff;
}

double FFControl::bqFFControlOut(double ampl_fact, double f_ref_d) {
  std::cout << "f_ref_d from ff control: " << f_ref_d << std::endl;
  f_ref_filt = bqc.step(f_ref_d);
  return ampl_fact * f_ref_filt * k_ff;
}

//wrapper functions for C implementation
void* FFControl_c() {
  FFControl *out( new FFControl);
  return( reinterpret_cast< void* >( out ) );
}

double  dfFFContrlOut_c(void* ffctrl, double ampl_fact, double f_ref_d) {
  double output = reinterpret_cast< FFControl* >( ffctrl )->dfFFControlOut(ampl_fact, f_ref_d);
  return output;
}

double  bqFFControlOut_c(void* ffctrl, double ampl_fact, double f_ref_d) {
  double output = reinterpret_cast< FFControl* >( ffctrl )->bqFFControlOut(ampl_fact, f_ref_d);
  return output;
}
