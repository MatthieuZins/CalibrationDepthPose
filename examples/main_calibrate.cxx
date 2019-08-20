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
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdlib.h>
#include <time.h>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <yaml-cpp/yaml.h>

#include "calibDepthPose.h"
#include "calibParameters.h"
#include "exampleUtils.h"

namespace fs = std::filesystem;
using namespace CalibrationDepthPose;


int main(int argc, char* argv[])
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  if (argc < 3)
  {
    std::cerr << "Usage:\n\t calibrate dataset_file configuration_file\n" << std::endl;
    return -1;
  }


  // Load calibration parameters
  YAML::Node config = YAML::LoadFile(argv[2]);
  auto parameters = config["calibration_parameters"];
  int nb_iterations = parameters["nb_iterations"].as<int>();
  auto init_guess = parameters["calibration_initial_guess"];
  Eigen::Isometry3d estimatedCalib = extractIsometry(init_guess);

  CalibParameters params;
  params.matchingMaxDistance
      = parameters["matching_max_distance"].as<double>();
  params.matchingPlaneDiscriminatorThreshold
      = parameters["matching_plane_discriminator_threshold"].as<double>();
  params.matchingRequiredNbNeighbours
      = parameters["matching_required_nb_neighbours"].as<int>();
  if (parameters["distance_type"].as<std::string>() == "POINT_TO_POINT")
    params.distanceType = DistanceType::POINT_TO_POINT;
  else
    params.distanceType = DistanceType::POINT_TO_PLANE;



  // Load dataset
  std::string filename(argv[1]);
  std::ifstream dataset_file(filename);
  if (!dataset_file.is_open())
  {
    throw std::runtime_error("Could not read the dataset file: " + filename);
  }
  std::string pose_filename;
  dataset_file >> pose_filename;
  std::vector<std::string> pc_files;
  std::string line;
  while (dataset_file >> line)
  {
    pc_files.push_back(line);
  }
  dataset_file.close();


  // Load pointclouds
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

  // Load poses
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

  // Generate random colors for outputs (one for each point cloud)
  std::vector<Eigen::Vector3i> colors;
  for (size_t i = 0; i < pointclouds.size(); ++i)
  {
    colors.emplace_back(rand() % 255, rand() % 255, rand() % 255);
  }


  std::cout << "\nStart calibration" << std::endl;
  // Estimation of the calib
  CalibDepthPose calibration(pointclouds, poses, estimatedCalib);
  // Set the pairs of pointclouds used for matching
  for (size_t i = 0; i < pointclouds.size(); ++i)
  {
    calibration.getMatchingMatrix().addMatch(i, (i + 1) % pointclouds.size(), true);
  }

  // Calibration loop
  for (int iter = 0; iter < nb_iterations; ++iter)
  {
    // Ouput the retransformed pointclouds with the current calibration estimate
    ColoredPointcloud::Ptr concat(new ColoredPointcloud);
    for (size_t i = 0; i < pointclouds.size(); ++i)
    {
      auto transformed_pc = transform(pointclouds[i], poses[i] * estimatedCalib);
      auto colored = colorizePointCloud(transformed_pc, colors[i][0], colors[i][1], colors[i][2]);
      *concat += *colored;
    }
    pcl::io::savePLYFile("colored_" + std::to_string(iter) + ".ply", *concat);

    calibration.calibIteration(&params);
    estimatedCalib = calibration.getCurrentCalibration();
  }


  // Output the retransformed pointclouds with the final calibration estimate
  ColoredPointcloud::Ptr concat(new ColoredPointcloud);
  for (size_t i = 0; i < pointclouds.size(); ++i)
  {
    auto transformed_pc = transform(pointclouds[i], poses[i] * estimatedCalib);
    auto colored = colorizePointCloud(transformed_pc, colors[i][0], colors[i][1], colors[i][2]);
    *concat += *colored;
  }
  pcl::io::savePLYFile("colored_" + std::to_string(nb_iterations) + ".ply", *concat);

  std::cout << "Estimated calibration: " << estimatedCalib << std::endl;

  return 0;
}