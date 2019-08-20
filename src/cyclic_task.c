#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <ctype.h>
#include <math.h>
#include "model526.h"

#include "afo.h"
#include "kalman_imu.h"
#include "kalman_ptm.h"
#include "kalman_encoder.h"
#include <stdio.h>
#include "imu_udp.h"
#include "torque_amp.h"
#include "ff_control.h"

/******* Functions for Real-Time task setup *********/
struct period_info {
  struct timespec next_period;
  long period_ns;
};

struct timespec curtime;

static void inc_period(struct period_info *pinfo)
{
  pinfo->next_period.tv_nsec += pinfo->period_ns;

  while (pinfo->next_period.tv_nsec >= 1000000000) {
    /* timespec nsec overflow */
    pinfo->next_period.tv_sec++;
    pinfo->next_period.tv_nsec -= 1000000000;
  }
}

static void periodic_task_init(struct period_info *pinfo)
{
  /* for simplicity, hardcoding a 1ms period */
  pinfo->period_ns = 10000000;

  clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

static void wait_rest_of_period(struct period_info *pinfo)
{
  inc_period(pinfo);

  /* for simplicity, ignoring possibilities of signal wakes */
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

/******* Functions for Real-Time task setup *********/


/******* Real-Time Thread *********/
void *thread_func(void *data)
{
  struct period_info pinfo;

  printf("Initializing Real-Time Task...\n");
  periodic_task_init(&pinfo);
  printf("Done.\n");
  
  // Initialize ADC
  printf("Initializing DAQ hardware...\n");
  s526_init();
  int32_t ADC_CHANNELS[] = {0, 1, 2, 3, 4, 5, 6, 7};
  const int NUM_ADC_CHANNELS = 8;
  double adc_data[NUM_ADC_CHANNELS];
  s526_adc_init(ADC_CHANNELS, NUM_ADC_CHANNELS);
  printf("Done.\n");
  
  //initialize exo control
  double freq_imu = 0.005;
  int count = 0;
  //imu
  int outImu = 0;
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

  while (outImu > -9) {
  /*
    // Read ADC
    s526_adc_read(ADC_CHANNELS, NUM_ADC_CHANNELS, adc_data);

    // Print ADC data
    printf("ADC Data :: [");
    for(int j=0; j<NUM_ADC_CHANNELS; j++)
    {
      printf(" %.2f ", j, adc_data[j]);
    }
    printf("]\n");

    // Print the time
    clock_gettime(CLOCK_MONOTONIC, &curtime);
    printf("Current time: %d s and %f ms.\n", curtime.tv_sec, curtime.tv_nsec/1000000.0);
  */
    out_imu = (sin(2 *M_PI * count * freq_imu)+1)*0.5;

    out_afo = AfoStep_c(afo,out_imu);

    out_tqamp = velFactor_c(tqamp, out_afo[1], out_afo[3]);

    F_ref_d = (sin(2 * M_PI * count * freq_imu)+1)*50 ;
    u_ff = dfFFControlOut_c(ffcontrol,out_tqamp[1], F_ref_d); //input: ampl_fact, F_ref_d
    printf("%f %f %f %f %f\n",count*freq_imu, out_imu, out_afo[1], out_tqamp[1], u_ff);
    
    
    wait_rest_of_period(&pinfo);
  }

  return NULL;
}
/******* Real-Time Thread *********/

int main(int argc, char* argv[])
{
  struct sched_param param;
  pthread_attr_t attr;
  pthread_t thread;
  int ret;

  /* Lock memory */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
    printf("mlockall failed: %m\n");
    exit(-2);
  }

  /* Initialize pthread attributes (default values) */
  ret = pthread_attr_init(&attr);
  if (ret) {
    printf("init pthread attributes failed\n");
    goto out;
  }

  /* Set a specific stack size  */
  ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
  if (ret) {
    printf("pthread setstacksize failed\n");
    goto out;
  }

  /* Set scheduler policy and priority of pthread */
  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret) {
    printf("pthread setschedpolicy failed\n");
    goto out;
  }
  param.sched_priority = 80;
  ret = pthread_attr_setschedparam(&attr, &param);
  if (ret) {
    printf("pthread setschedparam failed\n");
    goto out;
  }
  /* Use scheduling parameters of attr */
  ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (ret) {
    printf("pthread setinheritsched failed\n");
    goto out;
  }

  /* Create a pthread with specified attributes */
  ret = pthread_create(&thread, &attr, thread_func, NULL);
  if (ret) {
    printf("create pthread failed\n");
    goto out;
  }

  /* Join the thread and wait until it is done */
  ret = pthread_join(thread, NULL);
  if (ret)
    printf("join pthread failed: %m\n");

 out:
  return ret;
}
