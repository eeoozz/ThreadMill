#include <digital_filter.h>
#include <iostream>

DigitalFilt::DigitalFilt(double * A, double * B, int size) {
  this-> A = A;
  this-> B = B;
  this-> size = size;
  input = new double[size];
  output = new double[size];
  //initialize input and output data series with all 0
  for (int count = 0; count < size; count ++) {
    input[count] = 0;
    output[count] = 0;
  }
  std::cout << "input " << input[0] << std::endl;
  std::cout << "output " << output[0] << std::endl;
}

double DigitalFilt::Filt(double in) {
  input[0] = in;
  output[0] = 0;
  //calculate the current output
  for (int count = 0; count < size; count++) {
    output[0] += B[count]*input[count];
		if (count >= 1)
			output[0] -= A[count]*output[count];
  }
  //update the data series by moving each input/output to the right (filo)
  for (int count = size-1; count >= 1; count--) {
		input[count] = input[count - 1];
		output[count] = output[count - 1];
	}

  return output[0];
}
