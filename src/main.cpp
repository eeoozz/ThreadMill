#include <afo.h>
#include <kalman_imu.h>
#include <kalman_ptm.h>
#include <kalman_encoder.h>
#include <math.h>
#include <fstream>
#include <random>
#include <sys_param.h>
#include <model_param.h>
#include <imu_udp.h>
#include <torque_amp.h>
#include <ff_control.h>

int main( int argc , char **argv )
{
  //afo*************************************************************************************
  /*
    //write the data to a file
    std::ofstream myfile;
    myfile.open("data.dat");

    AFO afoOb;
    afoOb.AfoInit(0.005);
    for( size_t n=0 ; n < 8000 ; ++n )//40s offline simulation
    {
      afoOb.AfoStep(sin(2*M_PI*0.005*n));

      myfile << afoOb.t << "   " << afoOb.phi_input << "   " << afoOb.phi_reconstruct << std::endl;
      std::cout << "input = " << afoOb.phi_input << ";reconstruct=" << afoOb.phi_reconstruct << ";err = " << afoOb.afo_sys.err << std::endl;
    }
    myfile.close();
    */

    //kalman***********************************************************************************
    /*
    //write the data to a file
    std::ofstream myfile;
    myfile.open("data.dat");

    // Define random generator with Gaussian distribution
    const double mean = 0.0;
    const double stddev = 0.1;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    KalmanImu kalmanImu;
    kalmanImu.InitFilter();
    std::cout  << kalmanImu.ft.x.transpose() << std::endl;
    for( size_t n=0 ; n < 2000 ; ++n )//40s offline simulation
    {
      double input[3] = {sin(2*M_PI*kalmanImu.ft.dt*n), 0, 0};//+ dist(generator);
      std::cout << "input: " << input[0] << std::endl;
      kalmanImu.UpdateFilter(input);

      std::cout << " output: " << kalmanImu.ft.x.transpose() << std::endl;
      std::cout << std::endl;
      myfile << n*kalmanImu.ft.dt << "   " << input[0] << "   " << kalmanImu.ft.x[0] << " " << kalmanImu.ft.x[1]
             << " " << kalmanImu.ft.x[2] << std::endl;
    }

    ModelParam hk(true);
    myfile.close();
    return 0;
    */


    std::ofstream myfile;
    myfile.open("test.dat");

    //int count = 0;
    double freq_imu = 0.005;

    //imu communication*************************************************************************
    ImuUdp imuComm;
    imuComm.imuSend("Q");
    float outImu = 0;
    //afo
    AFO afoOb;
    afoOb.AfoInit(freq_imu);
    double * outAfo;
    //torque amplitude
    TorqueAmp tqamp;
    double * outTq;
    //feedforward control

    FFControl ffcontrol;
    double u_ff;
    double F_ref_d;

    for (int count = 0; count < 2000; count ++) {
    //while (outImu > -9) {
      std::cout << "*************************" << std::endl;
      //outImu = imuComm.imuListen();
      //simulate imu now
      outImu = (sin(2 *M_PI * count * freq_imu)+1)*0.5;

      outAfo = afoOb.AfoStep(outImu);  //output: sig_afo, f_afo, y_fund, ampl_ang, y_rec
      
      outTq = tqamp.velFactor(outAfo[1], outAfo[3]); //input: f_afo, ampl_ang    output: v_ref, ampl_fact

      //F_ref_d = (sin(2 * M_PI * count * freq_imu) + 1) * 50;
      F_ref_d = (sin(2 * M_PI * count * freq_imu)+1)*50 ;
      u_ff = ffcontrol.dfFFControlOut(outTq[1], F_ref_d); //input: ampl_fact, F_ref_d

      std::cout << (float)count * freq_imu << " " << outImu << " " << outAfo[1] << " " << outTq[1] << " " << u_ff << std::endl;
      myfile << (float)count * freq_imu << " " << outImu << " " << outAfo[1] << " " << outTq[1] << " " << u_ff << std::endl;
      //std::cout << (float)count * freq_imu << " " << F_ref_d << " " << ffcontrol.f_ref_filt << std::endl;
      //myfile << (float)count * freq_imu << " " << F_ref_d << " " << ffcontrol.f_ref_filt << std::endl;
    }

    myfile.close();
    return 0;
}
