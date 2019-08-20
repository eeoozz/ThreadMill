#include "afo.h"
#include "kalman_imu.h"
#include "kalman_ptm.h"
#include "kalman_encoder.h"
#include <stdio.h>
#include "imu_udp.h"
#include "torque_amp.h"
#include "ff_control.h"

int main(){
  double freq_imu = 0.005;
  //imu
  void *imuComm = ImuUdp_c();
  imuSend_c(imuComm,"Q");
  float out_imu = 0;
  //afo
  void *afo = AFO_c();
  AfoInit_c(afo,freq_imu);
  double * out_afo;
  //torque amplitude
  void *tqamp = TorqueAmp_c();
  double * out_tqamp;
  //feedforward control
  void *ffcontrol = FFControl_c();
  double u_ff;
  double F_ref_d;

  for (int count = 0; count < 2000; count ++) {
  //while (outImu > -9) {
    //out_imu = imuListen_c(imuComm);
    //simulate imu now
    out_imu = (sin(2 *M_PI * count * freq_imu)+1)*0.5;

    out_afo = AfoStep_c(afo,out_imu);

    out_tqamp = velFactor_c(tqamp, out_afo[1], out_afo[3]);

    F_ref_d = (sin(2 * M_PI * count * freq_imu)+1)*50 ;
    u_ff = dfFFControlOut_c(ffcontrol,out_tqamp[1], F_ref_d); //input: ampl_fact, F_ref_d
    printf("%f %f %f %f %f\n",count*freq_imu, out_imu, out_afo[1], out_tqamp[1], u_ff);
  }

/*
    double dt = 0.02;
    void *temp_kalmanImu = KalmanImu_c();
    initKalmanImu(temp_kalmanImu);
    for( size_t n=0 ; n < 2000 ; ++n )//40s offline simulation
    {
      double input[3] = {sin(2*M_PI*dt*n), 0, 0};
      updateKalmanImu(temp_kalmanImu, input);
    }
    */
  return 0;
}
