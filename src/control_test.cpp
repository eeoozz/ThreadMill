

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
    double A[5] = {1.000000000000000e+00, -2.424652148014002e+00, 2.158880722275586e+00,
      -8.40791461848115e-01, 1.21259541582703e-01}; //den
    double B[5] = {1.1712603903505594e+01, -2.1610718269080696e+01, -1.530926754557027e+00,
      2.1618066591626327e+01, -1.0174328826402933e+01}; //num
    double * A_ptr = A;
    double * B_ptr = B;
    DigitalFilt digiFilter (A_ptr, B_ptr, 5);
//

    // Output the step-response
    for( int n = 0; n < 2000; n++ ) {
        //digital filter
        double out = digiFilter.Filt(sin(2*M_PI*dt*n));
        std::cout << (double)(n*dt) << "   " << sin(2*M_PI*dt*n) << "   " << out << std::endl;
        myfile << (double)(n*dt) << "   " << sin(2*M_PI*dt*n) << "   " << out << std::endl;

        /*
        biquad
        std::cout << (double)(n*dt) << "   " << sin(2*M_PI*dt*n) << "   " << bqc.step(sin(2*M_PI*dt*n)) << std::endl;
        myfile << (double)(n*dt) << "   " << sin(2*M_PI*dt*n) << "   " << bqc.step(sin(2*M_PI*dt*n)) << std::endl;
        */
             }

    myfile.close();
    // Done0
    return EXIT_SUCCESS;

}
