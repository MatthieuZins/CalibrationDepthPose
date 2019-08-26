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

#ifndef EXAMPLEUTILS_H
#define EXAMPLEUTILS_H

#include <pcl/point_cloud.h>
#include <yaml-cpp/yaml.h>

#include <CalibrationDepthPose/pclUtils.h>

template <class Type>
constexpr double TO_RADIANS(const Type& x) {
  return static_cast<double>(x) *  0.01745329251;
}

template <class T>
constexpr double TO_DEGREES(const T& x) {
  return static_cast<double>(x) * 57.2957795131;
}

using ColoredPoint = pcl::PointXYZRGB;
using ColoredPointcloud = pcl::PointCloud<ColoredPoint>;


std::ostream& operator <<(std::ostream& os, Eigen::Isometry3d const& pose);

/// Add Gaussian noise to point cloud
void addNoise(CalibrationDepthPose::Pointcloud::Ptr pc, double stddev);


std::istream& operator >>(std::istream &is, Eigen::Isometry3d& pose);


/// Colorize the point cloud with rgb
ColoredPointcloud::Ptr colorizePointCloud(CalibrationDepthPose::Pointcloud::Ptr pc,
                                          unsigned char r,
                                          unsigned char b,
                                          unsigned char g);

/// Extract an isometry transform from a YAML node
Eigen::Isometry3d extractIsometry(YAML::Node const& node);


/// Return a string representation of the isometry using Euler angle (Rx Ry Rz) for the orietation
std::string printEulerAngleIsometry(Eigen::Isometry3d const& pose);

#endif // EXAMPLEUTILS_H
