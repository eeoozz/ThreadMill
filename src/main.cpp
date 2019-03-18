#include <afo.h>
#include <kalman_filter.h>
#include <math.h>
#include <fstream>

int main( int argc , char **argv )
{
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
    return 0;
}
