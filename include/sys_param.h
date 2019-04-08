#ifndef SYS_PARAM_H
#define SYS_PARAM_H

#include <Eigen/Dense>
#include <cmath>

#define G 9.81

enum cable_type {C_SHOR_J = 0, C_LONG_J};

struct MassSysParam {
  double J1, J2, J3, k_tau1, k_tau2, k_tau3, fact_motor, fact_hum, m_h, b_h, k_h;
  Eigen::Vector3d m, k, b;
};

class MassSys {
public:
  MassSys(bool useLinsea, cable_type ctype) {
    //numerical values
    sys.J1 = 5.517e-6;
    sys.J2 = 7.406e-6;
    sys.k_tau1 = 0.29;

    double p = 2e-3; //ballscrew pitch (m)
    double k2 = 24000; //linear spring (N/m)
    double rc = 0.03; //joint radius exo (m)

    double N_per_nm = 2*M_PI/p;
    sys.fact_motor = pow(N_per_nm,2);

    double m1 = sys.fact_motor * sys.J1;
    double m2 = sys.fact_motor * sys.J2;
    double k1 = sys.fact_motor * sys.k_tau1;

    //leg shank impedance data
    double i_body_c = 11.0; // human body inertia moment about mediolateral axis kg m^2
    double m_body = 70; //kg
    double h_body = 1.7; //m
    double ratio_ie_ih = 0.05;

    double m_sh = 0.061 * m_body; //shank mass
    double l_sh = 0.285 * h_body; //shank length
    double com_sh = 0.606 * l_sh;
    double r_gyr_sh = 0.735 * l_sh;

    //human parameters
    double i_h = m_sh * pow(r_gyr_sh,2);
    double beta_h = 2.0; //(Nms/rad)
    double kappa_h = 2/M_PI * m_sh * G * com_sh; //(Nm/s)

    //exo parameters
    double i_e = ratio_ie_ih * i_h;
    sys.fact_hum = ((useLinsea)? 1/pow(rc,2) : 2/pow(rc,2));
    double m3 =sys.fact_hum * i_e;

    sys.m = {m1,m2, m3};

    double k3;

    if (ctype == C_SHOR_J) {
      sys.b = {0, 4.4, 704.4};
      k3 = 0;
    }
    if (ctype == C_LONG_J) {
      sys.b = {0, 0, 843};
      k3 = 0.003376;
    }

    sys.k = {k1, k2, k3};

    //human parameters, reflected
    sys.m_h = sys.fact_hum * i_h;
    sys.b_h = sys.fact_hum * beta_h;
    sys.k_h = sys.fact_hum * kappa_h;

    //additional computed parameters
    sys.J3 = m3/sys.fact_motor;
    double k_tau3 = k3/sys.fact_motor;

    //low-force spring carriage impedance reflected onto motor shaft
    sys.J2 = m2/sys.fact_motor;
    double k_tau2 = k2/sys.fact_motor;

  };

  MassSysParam sys;
//private:

};

#endif //end of SYS_PARAM_H
