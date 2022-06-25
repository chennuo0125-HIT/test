/*
 * @Author: chennuo0125@163.com
 */
#pragma once
#include <ceres/ceres.h>

#include "utils.hpp"
#include "local_parameters.hpp"

class ReprojectFactor : public ceres::SizedCostFunction<2, 7, 3> {
 public:
  ReprojectFactor(double mea_x, double mea_y) : mea_x_(mea_x), mea_y_(mea_y) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Eigen::Vector3d> tcw(parameters[0]);
    Eigen::Map<const Eigen::Quaterniond> qcw(parameters[0] + 3);

    return true;
  }

  double mea_x_;
  double mea_y_;
};