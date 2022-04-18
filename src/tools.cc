/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: tools.cc
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

#include "align_ir_rgb/tools.h"
#include <boost/filesystem.hpp>

std::vector<std::string> GetSubFolders(const std::string path_name) {
  boost::filesystem::path p(
      path_name);  //<- The path you want to get sub-folders of

  boost::filesystem::directory_iterator end_itr;

  // cycle through the directory
  std::vector<std::string> dirs;
  for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr) {
    if (is_directory(itr->path())) {
      std::cout << "sub-dir: " << itr->path().string() << std::endl;
      dirs.push_back(itr->path().string());
    }
  }

  return dirs;
}
