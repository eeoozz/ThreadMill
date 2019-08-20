//feedforward control
#ifndef FEEDFORWARD_CONTROL
#define FEEDFORWARD_CONTROL

#ifdef __cplusplus
#include <digital_filter.h>
#include <bi_quad.h>

class FFControl {
public:
  FFControl();
  double dfFFControlOut(double ampl_fact, double f_ref_d);
  double bqFFControlOut(double ampl_fact, double f_ref_d);
  double f_ref_filt;
private:
  BiQuadChain bqc;
  DigitalFilt dgfilter;
  BiQuad bq1;
  BiQuad bq2;
  double k_ff = 1.0;
};
#endif

//wrapper functions for c implementation
#ifdef __cplusplus
extern "C" {
#endif

  void*   FFControl_c();
  double  dfFFControlOut_c(void* ffctrl, double ampl_fact, double f_ref_d);
  double  bqFFControlOut_c(void* ffctrl, double ampl_fact, double f_ref_d);

#ifdef __cplusplus
}
#endif

#endif //end of FEEDFORWARD_CONTROL
