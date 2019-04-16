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

    //imu communication*************************************************************************
    ImuUdp imuComm;
    imuComm.imuSend("Q");
    imuComm.imuListen();
    return 0;
}
