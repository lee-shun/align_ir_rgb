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
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <array>

#include <Eigen/Core>

/**
 * get sub folers name under given path
 * */
std::vector<std::string> GetSubFolders(const std::string path_name);

/**
 * locate the file to the line number
 * */
std::ifstream& SeekToLine(std::ifstream& in, const uint16_t line_nbr);

/**
 * data at each dir represents each measurement
 * */
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
    std::ifstream matches_fin_;
    matches_fin_.open(matches_file_name);
    if (!matches_fin_) {
      std::cerr << "can not open " << matches_file_name
                << ", no such file or directory!" << std::endl;
      return nullptr;
    }

    // STEP: 1 read distance, ir, and rgb
    auto new_measurement = std::make_shared<EachMeasurement>();

    // distance

    // ir data -> line 1
    SeekToLine(matches_fin_, 1);
    for (int i = 0; i < 4; ++i) {
      Eigen::Vector3d homo_pixel;
      if (!GetOneHomePiexl(matches_fin_, &homo_pixel)) return nullptr;
      new_measurement->homo_ir_pixel_pos_.at(i) = homo_pixel;
    }

    // rgb_data -> line 2
    SeekToLine(matches_fin_, 2);
    for (int i = 0; i < 4; ++i) {
      Eigen::Vector3d homo_pixel;
      if (!GetOneHomePiexl(matches_fin_, &homo_pixel)) return nullptr;
      new_measurement->homo_rgb_pixel_pos_.at(i) = homo_pixel;
    }

    return new_measurement;
  }

  void PrintData() const {
    std::cout << "distace: " << distance_ << std::endl;

    for (int i = 0; i < 4; ++i) {
      std::cout << "ir and rgb match :" << i << std::endl;
      std::cout << "ir: " << homo_ir_pixel_pos_.at(i) << std::endl;
      std::cout << "rgb: " << homo_rgb_pixel_pos_.at(i) << std::endl;
    }
  }

 private:
  static bool GetOneHomePiexl(std::ifstream& in, Eigen::Vector3d* homo_pixel) {
    std::string pixel_tmp;
    std::vector<double> piexl_pair_tmp;

    for (int i = 0; i < 2; ++i) {
      if (!getline(in, pixel_tmp, ',')) {
        return false;
      }
      piexl_pair_tmp.push_back(std::stod(pixel_tmp));
    }

    (*homo_pixel) << piexl_pair_tmp[0], piexl_pair_tmp[1], 1.0;

    return true;
  }
};

#endif  // INCLUDE_ALIGN_IR_RGB_TOOLS_H_
