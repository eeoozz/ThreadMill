#ifndef DIGITAL_FILTER_H
#define DIGITAL_FILTER_H

#include <Eigen/Dense>

class DigitalFilt{
public:
  DigitalFilt() {};
  DigitalFilt(double * A, double * B, int size);
  double Filt(double in);
  double * A;
  double * B;
  double * input;
  double * output;
  int size;
  //double DigitalFiltTF(double input);
private:
};

#endif
