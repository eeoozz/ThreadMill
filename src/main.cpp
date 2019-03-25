#include <afo.h>
#include <kalman_filter.h>
#include <math.h>
#include <fstream>
#include <random>

int main( int argc , char **argv )
{
  /*
    //write the data to a file
    std::ofstream myfile;
    myfile.open("data.dat");

    AFO afoOb;
    afoOb.Init(0.005);
    for( size_t n=0 ; n < 8000 ; ++n )//40s offline simulation
    {
      afoOb.DoStep(sin(2*M_PI*0.005*n));

      myfile << afoOb.t << "   " << afoOb.phi_input << "   " << afoOb.phi_reconstruct << std::endl;
      std::cout << "input = " << afoOb.phi_input << ";reconstruct=" << afoOb.phi_reconstruct << ";err = " << afoOb.afo_sys.err << std::endl;
    }
    myfile.close();
    */
    //write the data to a file
    std::ofstream myfile;
    myfile.open("data.dat");

    double dt = 0.01;
    // Define random generator with Gaussian distribution
    const double mean = 0.0;
    const double stddev = 0.1;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    KalmanFilter kalmanImu;
    kalmanImu.InitImuFilter();
    std::cout  << kalmanImu.imu_f_.x.transpose() << std::endl;
    for( size_t n=0 ; n < 2000 ; ++n )//40s offline simulation
    {
      double input = sin(2*M_PI*dt*n);//+ dist(generator);
      std::cout << "input: " << input << std::endl;
      kalmanImu.UpdateImu(input);

      std::cout << " output: " << kalmanImu.imu_f_.x.transpose() << std::endl;
      std::cout << std::endl;
      myfile << n*0.01 << "   " << input << "   " << kalmanImu.imu_f_.x[0] << " " << kalmanImu.imu_f_.x[1]
             << " " << kalmanImu.imu_f_.x[2] << std::endl;
    }
    myfile.close();
    return 0;
}
