//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Matthieu Zins
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//=========================================================================

#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>


#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "calibDepthPose.h"
#include "calibParameters.h"

namespace fs = std::filesystem;
using namespace CalibrationDepthPose;

std::istream& operator >>(std::istream &is, Eigen::Isometry3d& pose)
{
  double qw, qx, qy, qz, x, y, z;
  is >> qw >> qx >> qy >> qz >> x >> y >> z;
  pose = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);
  return is;
}

int main(int argc, char* argv[])
{

  std::string filename("/home/matt/dev/CalibrationDepthPose/data/dataset.txt");

  std::ifstream dataset_file(filename);
  std::string pose_filename;
  dataset_file >> pose_filename;
  std::vector<std::string> pc_files;
  std::string line;
  while (dataset_file >> line)
  {
    pc_files.push_back(line);
  }



  dataset_file.close();

  std::vector<CalibrationDepthPose::Pointcloud::Ptr> pointclouds;
  std::vector<Eigen::Isometry3d> poses;
  for (auto const& s : pc_files)
  {
    CalibrationDepthPose::Pointcloud::Ptr pc(new CalibrationDepthPose::Pointcloud());
    if (pcl::io::loadPLYFile(s, *pc) != 0)
    {
      throw std::runtime_error("Could not load point cloud: " + s);
    }
    pointclouds.emplace_back(pc);
  }

  std::ifstream pose_file(pose_filename);
  Eigen::Isometry3d pose;
  while (pose_file >> pose)
  {
    poses.emplace_back(pose);
  }
  pose_file.close();

  if (pointclouds.size() != poses.size())
  {
    throw std::runtime_error("Not the same number of pointclouds ("
                             + std::to_string(pointclouds.size())
                             + ") and poses ("
                             + std::to_string(poses.size()) + ")");
  }


  CalibParameters params;
  params.distanceType = DistanceType::POINT_TO_PLANE;
  params.matchingMaxDistance = 0.1;
  params.matchingPlaneDiscriminatorThreshold = 0.8;
  params.matchingRequiredNbNeighbours = 10;

  std::cout << "Start calibration" << std::endl;
  Eigen::Isometry3d calib = Eigen::Translation3d(0.1, 0, 0) * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  CalibDepthPose calibration(pointclouds, poses, calib);
  calibration.getMatchingMatrix().addMatch(0, 1, true);
  calibration.getMatchingMatrix().addMatch(1, 2, true);
  calibration.getMatchingMatrix().addMatch(2, 3, true);
  calibration.getMatchingMatrix().addMatch(3, 0, true);
  calib = calibration.calibrate(10, &params);

  for (int i = 0; i < pointclouds.size(); ++i)
  {
    transform(pointclouds[i], poses[i] * calib);
    pcl::io::savePLYFile("pc_" + std::to_string(i) + "_world.ply", *pointclouds[i]);
  }


  return 0;
}
