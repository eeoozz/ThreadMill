#ifndef DIGITAL_FILTER_H
#define DIGITAL_FILTER_H

#include <Eigen/Dense>

class DigitalFilt{
public:
  DigitalFilt();
  DigitalFilt(std::vector<double> &A_m, std::vector<double> &B_m);
  double Filt(double in);
  //double (* A)[5] = &(double[5]){1, 1, 1, 1, 1};
  std::vector<double> A_dg;
  std::vector<double> B_dg;
  std::vector<double> input;
  std::vector<double> output;
  int size = 5;
  //double DigitalFiltTF(double input);
};

/*
class DigitalFilt{
public:
  DigitalFilt();
  DigitalFilt(double * A_m, double * B_m, int size);
  double Filt(double in);
  void Test();
  //double (* A)[5] = &(double[5]){1, 1, 1, 1, 1};
  double * A_dg;
  double * B_dg;
  double * input;
  double * output;
  int size;
  //double DigitalFiltTF(double input);
private:
  double A_default[5] = {1.000000000000000e+00, -2.424652148014002e+00, 2.158880722275586e+00,
    -8.40791461848115e-01, 1.21259541582703e-01}; //den
  double B_default[5] = {1.1712603903505594e+01, -2.1610718269080696e+01, -1.530926754557027e+00,
    2.1618066591626327e+01, -1.0174328826402933e+01}; //num
};
*/
#endif
