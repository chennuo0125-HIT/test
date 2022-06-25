/*
 * @Author: chennuo0125@163.com
 */

#pragma once
#include <stdint.h>
#include <iostream>
#include <unordered_set>

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "utils.hpp"

#if defined G2O_HAVE_CHOLMOD
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
#else
G2O_USE_OPTIMIZATION_LIBRARY(eigen);
#endif

G2O_USE_OPTIMIZATION_LIBRARY(dense);

class G2oBA {
 public:
  G2oBA(const SimulationData &sim_data) : sim_data_(sim_data) {}

  void solve() {
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::string solver_name = "lm_fix6_3";
    g2o::OptimizationAlgorithmProperty solver_property;
    optimizer.setAlgorithm(
        g2o::OptimizationAlgorithmFactory::instance()->construct(
            solver_name, solver_property));

    Eigen::Vector2d principal_point(sim_data_.cx_, sim_data_.cy_);
    g2o::CameraParameters *cam_params =
        new g2o::CameraParameters(sim_data_.focal_length_, principal_point, 0.);
    cam_params->setId(0);
    if (!optimizer.addParameter(cam_params)) {
      assert(false);
    }

    // set pose vertex
    for (size_t i = 0; i < sim_data_.pose_num_; i++) {
      Eigen::Map<Eigen::Quaterniond> q(sim_data_.poses_[i] + 3);
      Eigen::Map<Eigen::Vector3d> t(sim_data_.poses_[i]);
      g2o::SE3Quat pose(q, t);
      g2o::VertexSE3Expmap *v_se3 = new g2o::VertexSE3Expmap();
      v_se3->setId(int(i));
      if (i < 2)
        v_se3->setFixed(true);
      v_se3->setEstimate(pose);
      optimizer.addVertex(v_se3);
    }

    // set points vertex
    for (size_t i = 0; i < sim_data_.point_num_; i++) {
      if (!sim_data_.valid_points[i])
        continue;
      Eigen::Map<Eigen::Vector3d> point(sim_data_.noise_points_[i]);
      g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
      int point_id = i + (int) sim_data_.pose_num_;
      v_p->setId(point_id);
      v_p->setMarginalized(true);
      v_p->setEstimate(point);
      optimizer.addVertex(v_p);

      Eigen::Map<Eigen::Vector3d> point1(sim_data_.points_[i]);
    }

    // set project edge
    for (size_t i = 0; i < sim_data_.pose_num_; i++) {
      const std::list<size_t> &point_idxs = sim_data_.idxs_in_pose_[i];
      const std::list<std::pair<double, double>> &meas =
          sim_data_.meas_in_pose_[i];
      auto pit = point_idxs.begin();
      auto mit = meas.begin();
      for (; pit != point_idxs.end() && mit != meas.end(); pit++, mit++) {
        if (!sim_data_.valid_points[*pit])
          continue;
        int pt_vertex_id = *pit + sim_data_.pose_num_;
        int pose_vertex_id = i;
        g2o::EdgeProjectXYZ2UV *e = new g2o::EdgeProjectXYZ2UV();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertices().find(pt_vertex_id)->second));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertices().find(pose_vertex_id)->second));
        Eigen::Vector2d mea(mit->first, mit->second);
        e->setMeasurement(mea);
        e->information() = Eigen::Matrix2d::Identity();
        e->setParameterId(0, 0);
        optimizer.addEdge(e);
      }
    }

    optimizer.initializeOptimization();
    optimizer.setVerbose(true);

    std::cout << "Performing full BA:" << std::endl;
    optimizer.optimize(10);

    // calculate error after optimized
    size_t valid_num = 0;
    double error = 0.0;
    for (size_t i = 0; i < sim_data_.point_num_; i++) {
      if (!sim_data_.valid_points[i])
        continue;
      int point_id = i + (int) sim_data_.pose_num_;
      g2o::VertexPointXYZ *v_p = dynamic_cast<g2o::VertexPointXYZ *>(
          optimizer.vertices().find(point_id)->second);
      Eigen::Map<const Eigen::Vector3d> p(sim_data_.points_[i]);
      Eigen::Vector3d diff = v_p->estimate() - p;
      error += sqrt(diff.dot(diff));
      valid_num++;
    }
    error /= valid_num;

    std::cout << std::endl;
    std::cout << "point error before optimize: " << sim_data_.getNoiseError()
              << std::endl;
    std::cout << "point error  after optimize: " << error << std::endl;
  }

  const SimulationData &sim_data_;

  G2oBA() = delete;
};