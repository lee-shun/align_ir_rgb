/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_files_reading.cc
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

#include <iostream>
#include "boost/filesystem.hpp"

int main(int argc, char *argv[]) {
  boost::filesystem::path p(
      "/home/ls/align_dataset");  //<- The path you want to get sub-folders of

  boost::filesystem::directory_iterator end_itr;

  // cycle through the directory
  std::vector<std::string> dirs;
  for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr) {
    if (is_directory(itr->path())) {
      std::cout << "sub-dir: " << itr->path().string() << std::endl;
      dirs.push_back(itr->path().string());
    }
  }

  return 0;
}
