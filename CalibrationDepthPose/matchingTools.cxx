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


#include "matchingTools.h"

#include <pcl/common/transforms.h>

#include <CalibrationDepthPose/calibParameters.h>

namespace CalibrationDepthPose
{

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
matchPointClouds(Pointcloud::Ptr pc1, Pointcloud::Ptr pc2,
                 const Eigen::Isometry3d &pose1, const Eigen::Isometry3d &pose2,
                 CalibParameters const* params)
{
  Pointcloud::Ptr pc1_world(new Pointcloud);
  Pointcloud::Ptr pc2_world(new Pointcloud);
  // Transform both point clouds to world
  pcl::transformPointCloud(*pc1, *pc1_world, pose1.matrix().cast<float>());
  pcl::transformPointCloud(*pc2, *pc2_world, pose2.matrix().cast<float>());

  KDTree::Ptr kdtree2_world(new KDTree);
  kdtree2_world->setInputCloud(pc2_world);

  // Match the world point clouds
  double matchingMaxSquareDist = params->matchingMaxDistance * params->matchingMaxDistance;
  std::vector<Eigen::Vector3d> pts1, pts2, normals;
  if (params->distanceType == DistanceType::POINT_TO_POINT)
  {
    for (size_t i = 0; i < pc1_world->size(); ++i)
    {
      auto j = matchPointToPoint(pc1_world->points[i], kdtree2_world, matchingMaxSquareDist);
      if (j == -1)
        continue;
      pts1.emplace_back(pc1->points[i].x, pc1->points[i].y, pc1->points[i].z);
      pts2.emplace_back(pc2->points[j].x, pc2->points[j].y, pc2->points[j].z);
      normals.emplace_back(0, 0, 0);
    }
  }
  else if (params->distanceType == DistanceType::POINT_TO_PLANE)
  {
    for (size_t i = 0; i < pc1_world->size(); ++i)
    {
      auto [res, point, normal]
          = matchPointToPlane(pc1_world->points[i], kdtree2_world, matchingMaxSquareDist,
          params->matchingRequiredNbNeighbours, params->matchingPlaneDiscriminatorThreshold);
      if (res)
      {
        pts1.emplace_back(pc1->points[i].x, pc1->points[i].y, pc1->points[i].z);
        pts2.emplace_back(pose2.inverse() * point);
        normals.emplace_back(pose2.rotation().inverse() * normal);
      }
    }
  }
  return std::make_tuple(std::move(pts1), std::move(pts2), std::move(normals));
}


int matchPointToPoint(Point const& p1, KDTree::Ptr kdtree2, double maxSquareDistance)
{
  std::vector<int> nearest_index;
  std::vector<float> nearest_sqdist;
  kdtree2->nearestKSearch(p1, 1, nearest_index, nearest_sqdist);
  if (nearest_sqdist.front() <= maxSquareDistance)
    return nearest_index.front();
  else
    return -1;
}


std::tuple<bool, Eigen::Vector3d, Eigen::Vector3d>
        matchPointToPlane(Point const& p1, KDTree::Ptr kdtree2,
                          double maxSquareDistance, int requiredNbNeighbours,
                          double planeDiscriminatorThreshold)
{
  std::vector<int> nearest_index;
  std::vector<float> nearest_sqdist;
  kdtree2->nearestKSearch(p1, requiredNbNeighbours, nearest_index, nearest_sqdist);

  if (nearest_sqdist.front() > maxSquareDistance || nearest_sqdist.back() > maxSquareDistance)
  {
    return std::make_tuple(false, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  Eigen::MatrixX3d points(requiredNbNeighbours, 3);
  for (size_t i = 0; i < nearest_index.size(); ++i)
  {
    Point const& p = kdtree2->getInputCloud()->points[nearest_index[i]];
    points(i, 0) = p.x;
    points(i, 1) = p.y;
    points(i, 2) = p.z;
  }
  Eigen::Vector3d mean = points.colwise().mean().transpose();

  auto [v1, v2, v3, normal] = analyzePlane(points);
  double planeDiscriminator = (v2 - v3) / v1; // this discriminator is in range [0, 1]
                                              // (it should tend towards 1 for planes)

  if (planeDiscriminator >= planeDiscriminatorThreshold)
  {
    return std::make_tuple(true, mean, normal);
  }

  return std::make_tuple(false, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

} // namespace CalibrationDepthPose
