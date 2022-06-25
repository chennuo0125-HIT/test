#pragma once

#include <ceres/ceres.h>
#include "utils.hpp"

class PoseLocalParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> p(x);
    Eigen::Map<const Eigen::Quaterniond> q(x + 3);
    Eigen::Map<const Eigen::Vector3d> dp(delta);
    Eigen::Map<const Eigen::Vector3d> daa(delta + 3);
    Eigen::Quaterniond dq = deltaQ(daa);

    Eigen::Map<Eigen::Vector3d> res_p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> res_q(x_plus_delta + 3);

    res_p = p + dp;
    res_q = (q * dq).normalized();

    return true;
  }

  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
  }

  virtual int GlobalSize() const { return 7; }
  virtual int LocalSize() const { return 6; }
};
