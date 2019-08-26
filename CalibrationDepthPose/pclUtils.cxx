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


#include <CalibrationDepthPose/pclUtils.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>


namespace CalibrationDepthPose
{

void voxelGridFilter(Pointcloud::Ptr& pc, double voxelSize)
{
  pcl::VoxelGrid<Point> vox_grid;
  vox_grid.setInputCloud(pc);
  vox_grid.setLeafSize(voxelSize, voxelSize, voxelSize);
  Pointcloud::Ptr tempCloud (new Pointcloud);
  vox_grid.filter(*tempCloud);
  pc.swap(tempCloud);
}


void transformInPlace(Pointcloud::Ptr &pc, const Eigen::Isometry3d &H)
{
  Pointcloud::Ptr tempCloud (new Pointcloud);
  pcl::transformPointCloud(*pc, *tempCloud, H.matrix().cast<float>());
  pc.swap(tempCloud);
}


Pointcloud::Ptr transform(Pointcloud::Ptr pc, const Eigen::Isometry3d &H)
{
  Pointcloud::Ptr tempCloud (new Pointcloud);
  pcl::transformPointCloud(*pc, *tempCloud, H.matrix().cast<float>());
  return tempCloud;
}

} // namespace CalibrationDepthPose
