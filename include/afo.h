#ifndef AFO_H
#define AFO_H

#include <math.h>

#ifdef __cplusplus
#include <vector>

#include <boost/numeric/odeint.hpp>

#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/generation.hpp>

typedef std::vector<double> state_type;

class AFO {
 public:
   AFO():x(24, 0){};
   struct AfoSys
   {
       double err = 0.0;

       //constant
       const double eps=10;
       const state_type k_edta = state_type(11,1.0);

       template< class State , class Deriv >
       void operator()( const State &x , Deriv &dxdt , double t) const
       {
         dxdt[0] = x[1]-eps*err*sin(x[0]); //phi
         dxdt[1] = -eps*err*sin(x[0]); //omega
         for (int count = 0; count < 11; count++) {
           dxdt[count+2] = k_edta[count]*cos(count*x[0])*err; //k_alpha
           dxdt[count+13] = k_edta[count]*sin(count*x[0])*err;  //k_beta
         }
       }
   };
   int AfoInit(double dt);
   double* AfoStep(double input);

   AfoSys afo_sys;
   state_type x; //phi, omega, k_alpha(12), k_beta(12)
   double phi_input, phi_reconstruct, t, dt;
 private:
   double afoOutput[5] = {0, 0, 0, 0, 0}; //sig_afo, f_afo, y_fund, ampl_ang, y_rec
   boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;
   float fact_afo = 4.0;
 };
 #endif

//wrapper functions for c implementation
#ifdef __cplusplus
extern "C" {
#endif

  void* AFO_c();
  void AfoInit_c( void *afo, double dt );
  double* AfoStep_c(void *afo, double input);


#ifdef __cplusplus
}
#endif


#endif/*AFO_H*/
