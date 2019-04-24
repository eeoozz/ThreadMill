

#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <complex>
#include <fstream>
#include "bi_quad.h"

#include "digital_filter.h"

//biquad definition
// Example: 3th order Butterworth LP (w_c = 0.1*f_nyquist)
BiQuadChain bqc;
BiQuad bq1( 11.7126, 0.2604, -11.3522, -1.3869, 0.4504 );
BiQuad bq2( 1, -1.8759, 0.8962, -1.0377, 0.2692 );

int main()
{

    std::ofstream myfile;
    myfile.open("data.dat");
    double dt = 0.005;
    std::cout << "dt = " << dt << std::endl;

//biquad
    bqc = bq1 * bq2;

    // Find the poles of the filter
    std::cout << "Filter poles" << std::endl;
    std::vector< std::complex<double> > poles = bqc.poles();
    for( size_t i = 0; i < poles.size(); i++ )
        std::cout << "\t"  << poles[i] << std::endl;

    // Find the zeros of the filter
    std::cout << "Filter zeros" << std::endl;
    std::vector< std::complex<double> > zeros = bqc.zeros();
    for( size_t i = 0; i < poles.size(); i++ )
        std::cout << "\t" << zeros[i] << std::endl;

    // Is the filter stable?
    std::cout << "This filter is " << (bqc.stable() ? "stable" : "instable") << std::endl;
//

//digital filter
    DigitalFilt digiFilter;
//

    // Output the step-response
    for( int n = 0; n < 2000; n++ ) {
      /*
        //digital filter
        double out = digiFilter.Filt(sin(2*M_PI*dt*n));
        std::cout << (double)(n*dt) << "   " << sin(2*M_PI*dt*n) << "   " << out << std::endl;
        myfile << (double)(n*dt) << "   " << sin(2*M_PI*dt*n) << "   " << out << std::endl;
      */

        //biquad
        double out = bqc.step(sin(2*M_PI*dt*n));
        std::cout << (double)(n*dt) << "   " << sin(2*M_PI*dt*n) << "   " << out << std::endl;
        myfile << (double)(n*dt) << "   " << sin(2*M_PI*dt*n) << "   " << out << std::endl;

             }

    myfile.close();
    // Done0
    return EXIT_SUCCESS;

}
