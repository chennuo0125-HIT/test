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

class ReprojectFactor : public ceres::SizedCostFunction<2, 7, 3, 4> {
 public:
  ReprojectFactor(double mea_x, double mea_y) : mea_x_(mea_x), mea_y_(mea_y) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Eigen::Vector3d> tcw(parameters[0]);
    Eigen::Map<const Eigen::Quaterniond> qcw(parameters[0] + 3);
    Eigen::Map<const Eigen::Vector3d> pw(parameters[1]);
    double fx = parameters[2][0];
    double fy = parameters[2][1];
    double cx = parameters[2][2];
    double cy = parameters[2][3];

    return true;
  }

  double mea_x_;
  double mea_y_;
};

class CeresBa {
 public:
  CeresBa(const SimulationData &sim_data) : sim_data_(sim_data) {}

  void solve() {
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
    double *cam_param = new double[4];
    cam_param[0] = sim_data_.focal_length_;
    cam_param[1] = sim_data_.focal_length_;
    cam_param[2] = sim_data_.cx_;
    cam_param[3] = sim_data_.cy_;

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

    // set cam param vertex
    problem.AddParameterBlock(cam_param, 4);
    problem.SetParameterBlockConstant(cam_param);

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
        ReprojectFactor *f = new ReprojectFactor(mit->first, mit->second);
        problem.AddResidualBlock(f, NULL, poses[i], points[*pit], cam_param);
      }
    }

    // optimize
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 10;
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
    delete[] cam_param;
  }

  const SimulationData &sim_data_;
  CeresBa() = delete;
};