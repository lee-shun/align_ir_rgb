/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: align_ir_rgb_g2o.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-04-13
 *
 *   @Description:
 *
 *******************************************************************************/

#include "align_ir_rgb/align_ir_rgb_g2o.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

int main(int argc, char** argv) {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<12, 1>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(
          g2o::make_unique<BlockSolverType>(
              g2o::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  // to be estimated r1-r9, t1-t3
  double r_t[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1};
  RotationAndTrans init_rot_trans = RotationAndTrans(r_t);

  // vertex
  RoteTransVertex* v = new RoteTransVertex;
  v->setEstimate(init_rot_trans);
  v->setId(0);
  optimizer.addVertex(v);

  // edges
  for (int i = 0; i < 1; ++i) {
    double pixel_ir[3] = {0, 0, 1}, z = 1;
    AlignErrEdge* edge = new AlignErrEdge(pixel_ir, z);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(Eigen::Vector3d(0, 0, 1));
    edge->setInformation(Eigen::Matrix3d::Identity());
    optimizer.addEdge(edge);
  }

  std::cout << "start optimization" << std::endl;
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "solve time cost = " << time_used.count() << " seconds. "
            << std::endl;

  // 输出优化值
  RotationAndTrans r_t_estimate = v->estimate();

  return 0;
}
