/*
 * @Author: chennuo0125@163.com
 */
#pragma once
#include <string.h>
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

class ReprojectFactor : public ceres::SizedCostFunction<2, 7, 3> {
 public:
  ReprojectFactor(double mea_x, double mea_y, double fx, double fy, double cx,
                  double cy)
      : mea_x_(mea_x), mea_y_(mea_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Eigen::Vector3d> tcw(parameters[0]);
    Eigen::Map<const Eigen::Quaterniond> qcw(parameters[0] + 3);
    Eigen::Map<const Eigen::Vector3d> pw(parameters[1]);

    Eigen::Matrix3d Rcw(qcw);
    Eigen::Vector3d pc = Rcw * pw + tcw;
    Eigen::Vector3d npc = pc / pc(2);
    Eigen::Matrix<double, 2, 3> K;
    K << fx_, 0.0, cx_, 0.0, fy_, cy_;
    Eigen::Vector2d z = K * npc;

    Eigen::Map<Eigen::Vector2d> res(residuals);
    Eigen::Vector2d mea(mea_x_, mea_y_);
    res = z - mea;

    if (jacobians) {
      Eigen::Matrix3d d_npc_pc;
      double inv_z = 1.0 / pc(2);
      d_npc_pc << inv_z, 0.0, -pc(0) * inv_z * inv_z, 0.0, inv_z,
          -pc(1) * inv_z * inv_z, 0.0, 0.0, 0.0;
      Eigen::Matrix<double, 2, 3> jac_cam = K * d_npc_pc;

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jac_pose(
            jacobians[0]);
        jac_pose.setZero();
        jac_pose.block(0, 0, 2, 3) = jac_cam;
        jac_pose.block(0, 3, 2, 3) = jac_cam * (-Rcw * hat(pw));
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_point(
            jacobians[1]);
        jac_point.setZero();
        jac_point = jac_cam * Rcw;
      }
    }

    return true;
  }

  double fx_, fy_, cx_, cy_;
  double mea_x_;
  double mea_y_;
};

class CeresBa {
 public:
  CeresBa(const SimulationData &sim_data) : sim_data_(sim_data) {}

  void solve() {
    std::cout << "************** solve ba by ceres **************" << std::endl;

    ceres::Problem problem;
    ceres::LocalParameterization *pose_local_parameterization =
        new PoseLocalParameterization();

    // generate values to optimize
    double **poses = new double *[sim_data_.pose_num_];
    for (size_t i = 0; i < sim_data_.pose_num_; i++) {
      poses[i] = new double[7];
      memcpy(poses[i], sim_data_.poses_[i], 7 * sizeof(double));
    }
    double **points = new double *[sim_data_.point_num_];
    for (size_t i = 0; i < sim_data_.point_num_; i++) {
      points[i] = new double[3];
      memcpy(points[i], sim_data_.noise_points_[i], 3 * sizeof(double));
    }

    // set pose vertex
    for (size_t i = 0; i < sim_data_.pose_num_; i++) {
      problem.AddParameterBlock(poses[i], 7, pose_local_parameterization);
      if (i < 2)
        problem.SetParameterBlockConstant(poses[i]);
    }

    // set point vertex
    for (size_t i = 0; i < sim_data_.point_num_; i++) {
      if (!sim_data_.valid_points[i])
        continue;
      problem.AddParameterBlock(points[i], 3);
    }

    // set edges
    for (size_t i = 0; i < sim_data_.pose_num_; i++) {
      const std::list<size_t> &point_idxs = sim_data_.idxs_in_pose_[i];
      const std::list<std::pair<double, double>> &meas =
          sim_data_.meas_in_pose_[i];
      auto pit = point_idxs.begin();
      auto mit = meas.begin();
      for (; pit != point_idxs.end() && mit != meas.end(); pit++, mit++) {
        if (!sim_data_.valid_points[*pit])
          continue;
        ReprojectFactor *f = new ReprojectFactor(
            mit->first, mit->second, sim_data_.focal_length_,
            sim_data_.focal_length_, sim_data_.cx_, sim_data_.cy_);
        problem.AddResidualBlock(f, NULL, poses[i], points[*pit]);
      }
    }

    // optimize
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 200;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    // calculate error
    double error = 0.0;
    size_t valid_num = 0;
    for (size_t i = 0; i < sim_data_.point_num_; i++) {
      if (!sim_data_.valid_points[i])
        continue;
      Eigen::Map<const Eigen::Vector3d> p(sim_data_.points_[i]);
      Eigen::Map<const Eigen::Vector3d> cp(points[i]);
      Eigen::Vector3d diff = cp - p;
      error += sqrt(diff.dot(diff));
      valid_num++;
    }
    error /= valid_num;

    std::cout << std::endl;
    std::cout << "point error before optimize: " << sim_data_.getNoiseError()
              << std::endl;
    std::cout << "point error  after optimize: " << error << std::endl;

    // release memory
    for (size_t i = 0; i < sim_data_.point_num_; i++) delete[] points[i];
    delete[] points;
    for (size_t i = 0; i < sim_data_.pose_num_; i++) delete[] poses[i];
    delete[] poses;
  }

  const SimulationData &sim_data_;
  CeresBa() = delete;
};