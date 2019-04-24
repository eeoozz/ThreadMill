#include <digital_filter.h>
#include <iostream>

DigitalFilt::DigitalFilt():A_dg(5,0), B_dg(5, 0), input(5, 0), output(5, 0) {
  A_dg = {1.000000000000000e+00, -2.424652148014002e+00, 2.158880722275586e+00,
    -8.40791461848115e-01, 1.21259541582703e-01};
  B_dg = {1.1712603903505594e+01, -2.1610718269080696e+01, -1.530926754557027e+00,
    2.1618066591626327e+01, -1.0174328826402933e+01};
}

DigitalFilt::DigitalFilt(std::vector<double> &A_m, std::vector<double> &B_m):A_dg(5,0), B_dg(5, 0), input(5, 0), output(5, 0) {
  A_dg = A_m;
  B_dg = B_m;
}

double DigitalFilt::Filt(double in) {
  input[0] = in;
  output[0] = 0;
  //calculate the current output
  for (int count = 0; count < size; count++) {
    output[0] += B_dg[count]*input[count];
		if (count >= 1)
			output[0] -= A_dg[count]*output[count];
  }
  //update the data series by moving each input/output to the right (filo)
  for (int count = size-1; count >= 1; count--) {
		input[count] = input[count - 1];
		output[count] = output[count - 1];
	}

  return output[0];
}
