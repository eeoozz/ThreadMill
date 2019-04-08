#ifndef MODEL_PARAM_H
#define MODEL_PARAM_H

#include <sys_param.h>

struct StateSpaceModel {
  Eigen::Matrix4d F_xv13, P_xv_d_o;
  Eigen::Vector4d G_xv13;
  Eigen::Matrix<double, 1, 4> H_y_xv13;
};

class ModelParam : public MassSys{
public:
  ModelParam(bool disturb_on) : MassSys(true, C_LONG_J) {
    int N_u = ((disturb_on)? 2:1);
    Eigen::Matrix2d F_x, F_v;

    //m12 = m1 + m2; k23 = k2; b23 = b2
    double m12 = sys.m[0] + sys.m[1];
    F_x << -sys.k[1]/m12, sys.k[1]/m12, //-k23/m12, k23/m12
           sys.k[1]/sys.m[2], -sys.k[1]/sys.m[2]-sys.k[2]/sys.m[2]; //k23/m3, -k23/m3-k3/m3
    F_v << -sys.b[1]/m12, sys.b[1]/m12, //-b23/m12, b23/m12
           sys.b[1]/sys.m[2], -sys.b[1]/sys.m[2]-sys.b[2]/sys.m[2]; //b23/m3, -b23/m3-b3/m3

    Eigen::MatrixXd G_u;
    if (disturb_on) {
      G_u.resize(2,2);
      G_u << 1/m12, 0,
             0, 1/sys.m[2];
           }
    else {
      G_u.resize(2,1);
      G_u << 1/m12,
             0;
           }

    //state space model
    ssm.F_xv13.topLeftCorner(2,2) = Eigen::MatrixXd::Zero(2,2);
    ssm.F_xv13.topRightCorner(2,2) = Eigen::MatrixXd::Identity(2,2);
    ssm.F_xv13.bottomLeftCorner(2,2) = F_x;
    ssm.F_xv13.bottomRightCorner(2,2) = F_v;
    std::cout << "F_xv13: " << ssm.F_xv13 << std::endl;


    Eigen::MatrixXd G_xv;
    if (disturb_on) {
      G_xv.resize(4,2);
      G_xv.topRows(2) = Eigen::MatrixXd::Zero(2,2);
      G_xv.bottomRows(2) = G_u;
    }
    else {
      G_xv.resize(4,1);
      G_xv.topRows(2) = Eigen::MatrixXd::Zero(2,1);
      G_xv.bottomRows(2) = G_u;
    }

    Eigen::Vector4d G_xv13_d = G_xv.col(1), temp = {1, -1, 0, 0};
    ssm.G_xv13 = G_xv.col(0);
    int N_xv13 = 4;
    ssm.H_y_xv13 = sys.k[1] * temp.transpose();
    Eigen::Vector4d L_y_xv13 = ssm.H_y_xv13.transpose() / (ssm.H_y_xv13 * ssm.H_y_xv13.transpose());

    ssm.P_xv_d_o = Eigen::Matrix4d::Identity();

  }

  StateSpaceModel ssm;

};
#endif //end of MODEL_PARAM_H
