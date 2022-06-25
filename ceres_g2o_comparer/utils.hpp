/*
 * @Author: chennuo0125@163.com
 */

#pragma once
#include <iostream>
#include <vector>
#include <list>
#include <math.h>
#include <limits.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// transform vector3 to inverse transpose mat3x3
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> hat(
    const Eigen::MatrixBase<Derived> &q) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
  ans << typename Derived::Scalar(0), -q(2), q(1), q(2),
      typename Derived::Scalar(0), -q(0), -q(1), q(0),
      typename Derived::Scalar(0);
  return ans;
}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(
    const Eigen::MatrixBase<Derived> &theta) {
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

class SimulationData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimulationData(size_t pose_num, size_t point_num, double pixel_noise)
      : pose_num_(pose_num), point_num_(point_num), pixel_noise_(pixel_noise) {
    poses_ = new double *[pose_num_];
    for (size_t i = 0; i < pose_num_; i++)
      poses_[i] = new double[7];  // x, y, z, qx, qy, qz, qw

    points_ = new double *[point_num_];
    noise_points_ = new double *[point_num_];
    for (size_t i = 0; i < point_num_; i++) {
      points_[i] = new double[3];  // x, y, z
      noise_points_[i] = new double[3];
    }

    valid_points = new bool[point_num];

    idxs_in_pose_.resize(pose_num_);
    meas_in_pose_.resize(pose_num_);

    srand(time(NULL));
    generateData();
  }

  ~SimulationData() {
    if (poses_) {
      for (size_t i = 0; i < pose_num_; i++) {
        if (poses_[i])
          delete[] poses_[i];
      }
      delete[] poses_;
    }

    if (points_) {
      for (size_t i = 0; i < point_num_; i++) {
        if (points_[i])
          delete[] points_[i];
      }
      delete[] points_;
    }

    if (noise_points_) {
      for (size_t i = 0; i < point_num_; i++) {
        if (noise_points_[i])
          delete[] noise_points_[i];
      }
      delete[] noise_points_;
    }

    if (valid_points)
      delete[] valid_points;
  }

  static double uniformRand(double lb, double ub) {
    return lb + ((double) std::rand() / (RAND_MAX + 1.0)) * (ub - lb);
  }

  static double gaussRand(double mean, double sigma) {
    double y, r2;
    do {
      double x = -1.0 + 2.0 * uniformRand(0.0, 1.0);
      y = -1.0 + 2.0 * uniformRand(0.0, 1.0);
      r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);
    return mean + sigma * y * sqrt(-2.0 * log(r2) / r2);
  }

  void generateData() {
    // generate true poses
    for (size_t i = 0; i < pose_num_; i++) {
      poses_[i][0] = i * 0.04 - 1.0;  // x
      poses_[i][1] = 0.0;             // y
      poses_[i][2] = 0.0;             // z
      poses_[i][3] = 0.0;             // qx
      poses_[i][4] = 0.0;             // qy
      poses_[i][5] = 0.0;             // qz
      poses_[i][6] = 1.0;             // qw
    }

    // generate true points
    for (size_t i = 0; i < point_num_; i++) {
      points_[i][0] = (uniformRand(0.0, 1.0) - 0.5) * 3;
      points_[i][1] = uniformRand(0.0, 1.0) - 0.5;
      points_[i][2] = uniformRand(0.0, 1.0) + 3.0;
    }

    // generate noise points
    for (size_t i = 0; i < point_num_; i++) {
      noise_points_[i][0] = points_[i][0] + gaussRand(0.0, pixel_noise_);
      noise_points_[i][1] = points_[i][1] + gaussRand(0.0, pixel_noise_);
      noise_points_[i][2] = points_[i][2] + gaussRand(0.0, pixel_noise_);
    }

    // mark valid points
    Eigen::Matrix3d K;
    K << focal_length_, 0.0, cx_, 0.0, focal_length_, cy_, 0.0, 0.0, 1.0;
    for (size_t i = 0; i < point_num_; i++) {
      Eigen::Map<const Eigen::Vector3d> pw(points_[i]);
      size_t num_obs = 0;
      for (size_t j = 0; j < pose_num_; j++) {
        Eigen::Map<const Eigen::Quaterniond> q(poses_[j] + 3);
        Eigen::Matrix3d Rcw(q);
        Eigen::Map<const Eigen::Vector3d> tcw(poses_[j]);
        Eigen::Vector3d pc = Rcw * pw + tcw;
        if (pc(2) <= std::numeric_limits<double>::epsilon())
          continue;
        pc = pc / pc(2);
        Eigen::Vector3d z = K * pc;
        if (z(0) >= 0 && z(0) < width_ && z(1) >= 0 && z(1) < height_)
          num_obs++;
      }

      if (num_obs >= 2)
        valid_points[i] = true;
      else
        valid_points[i] = false;
    }

    // record points' idx locate in poses
    for (size_t i = 0; i < pose_num_; i++) {
      Eigen::Map<const Eigen::Quaterniond> q(poses_[i] + 3);
      Eigen::Matrix3d Rcw(q);
      Eigen::Map<const Eigen::Vector3d> tcw(poses_[i]);
      for (size_t j = 0; j < point_num_; j++) {
        if (!valid_points[j])
          continue;
        Eigen::Map<const Eigen::Vector3d> pw(points_[j]);
        Eigen::Vector3d pc = Rcw * pw + tcw;
        if (pc(2) <= std::numeric_limits<double>::epsilon())
          continue;
        pc = pc / pc(2);
        Eigen::Vector3d z = K * pc;
        if (z(0) >= 0 && z(0) < width_ && z(1) >= 0 && z(1) < height_) {
          idxs_in_pose_[i].push_back(j);

          std::pair<double, double> noise_z;
          noise_z.first = z(0) + gaussRand(0.0, pixel_noise_);
          noise_z.second = z(1) + gaussRand(0.0, pixel_noise_);
          meas_in_pose_[i].push_back(noise_z);
        }
      }
    }
  }

  double getNoiseError() {
    size_t valid_num = 0;
    double error = 0.0;
    for (size_t i = 0; i < point_num_; i++) {
      if (!valid_points[i])
        continue;
      Eigen::Map<const Eigen::Vector3d> p(points_[i]);
      Eigen::Map<const Eigen::Vector3d> np(noise_points_[i]);
      Eigen::Vector3d diff = np - p;
      error += diff.dot(diff);
      valid_num++;
    }

    return error / valid_num;
  }

  // simulation pose and points
  size_t pose_num_, point_num_;
  double pixel_noise_;
  double **poses_;         // true poses
  double **points_;        // true points
  double **noise_points_;  // noise points based on true points
  bool *valid_points;      // tag for points are valid

  // idxs of points in fov of camera
  std::vector<std::list<size_t>> idxs_in_pose_;

  // meas of points in pose
  std::vector<std::list<std::pair<double, double>>> meas_in_pose_;

  // camera parameter
  double focal_length_{1000.0};
  double cx_{320.0};
  double cy_{240.0};
  int width_{640};
  int height_{480};

  SimulationData() = delete;
  SimulationData(const SimulationData &) = delete;
  SimulationData &operator=(const SimulationData &) = delete;
};