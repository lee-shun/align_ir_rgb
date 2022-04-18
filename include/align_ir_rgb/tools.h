/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: tools.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-04-18
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_ALIGN_IR_RGB_TOOLS_H_
#define INCLUDE_ALIGN_IR_RGB_TOOLS_H_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <array>

#include <Eigen/Core>

std::vector<std::string> GetSubFolders(const std::string path_name);

struct EachMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<EachMeasurement> Ptr;

 public:
  double distance_{0.0};
  std::array<Eigen::Vector3d, 4> homo_ir_pixel_pos_;
  std::array<Eigen::Vector3d, 4> homo_rgb_pixel_pos_;

 public:
  static EachMeasurement::Ptr CreateFromFolder(const std::string folder_name) {
    std::string matches_file_name = folder_name + "/matches.txt";

    EachMeasurement::Ptr p_measurement = std::make_shared<EachMeasurement>();

    // STEP: 1 read distance, ir, and rgb

    // STEP: 2 create new data
    p_measurement->distance_ = 0.0;

    return p_measurement;
  }

  void PrintData() const {
    std::cout << "distace: " << distance_ << std::endl;

    for (int i = 0; i < 4; ++i) {
      std::cout << "ir and rgb match :" << i << std::endl;
      std::cout << "ir: " << homo_ir_pixel_pos_.at(i) << std::endl;
      std::cout << "rgb: " << homo_rgb_pixel_pos_.at(i) << std::endl;
    }
  }
};

#endif  // INCLUDE_ALIGN_IR_RGB_TOOLS_H_
