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


#ifndef PCLUTILS_H
#define PCLUTILS_H

#include <Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace CalibrationDepthPose
{

using Point = pcl::PointXYZ;
using Pointcloud = pcl::PointCloud<Point>;
using KDTree = pcl::KdTreeFLANN<Point>;

/**
 * @brief voxelizePointcloud
 * @param pc input pointcloud
 * @param voxelSize
 */
void voxelGridFilter(Pointcloud::Ptr& pc, double voxelSize);


/**
 * @brief transform in place a pointcloud with an rigi transform
 * @param pc [in] pointcloud
 * @param H [in] rigid transform
 */
void transformInPlace(Pointcloud::Ptr& pc, Eigen::Isometry3d const& H);


/**
 * @brief transform a pointcloud with an rigi transform
 * @param pc [in] pointcloud
 * @param H [in] rigid transform
 */
Pointcloud::Ptr transform(Pointcloud::Ptr pc, Eigen::Isometry3d const& H);

} // namespace CalibrationDepthPose

#endif // PCLUTILS_H
