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


#ifndef CALIBTOOLS_H
#define CALIBTOOLS_H

#include <Eigen/Dense>

#include <CalibrationDepthPose/pclUtils.h>


namespace CalibrationDepthPose
{

class CalibParameters;

/**
 * @brief matchPointClouds This function computes the matches between two pointclouds
 * @param pc1 [in] first point cloud
 * @param pc2 [in] second point cloud
 * @param pose1 [in] pose of the first point cloud (= transform to world)
 * @param pose2 [in] pose of the second point cloud (= transform to world)
 * @param params [in] global parameters
 * @param reciprocal [in] choose to match also the second point cloud to the first one
 * @return a tuple [list of points, list of points, list of normals] containing the matchs.
 */
std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
matchPointClouds(Pointcloud::Ptr pc1, Pointcloud::Ptr pc2,
                 const Eigen::Isometry3d &pose1, const Eigen::Isometry3d &pose2,
                 const CalibParameters *params);


/**
 * @brief matchPointToPoint This function matches a point to another point
 * @param p1 [in] input point
 * @param kdtree2 [in] kdtree of the target point cloud
 * @param maxDistance [in] max distance to be a match
 * @return the index of the matched point
 */
int matchPointToPoint(Point const& p1, KDTree::Ptr kdtree2, double maxSquareDistance);


/**
 * @brief matchPointToPlane This function matches a point to a plane
 * @param p1 [in] input point
 * @param kdtree2 [in] kdtree of the target point cloud
 * @param maxDistance [in] max distanceto be a match
 * @param requiredNbNeighbours [in] number of points to fit the plane
 * @param planeDiscriminatorThreshold [in] discriminator threshold to decide if it is a plane
 * @return a tuple [boolean result, point matched, normal of the plane]
 */
std::tuple<bool, Eigen::Vector3d, Eigen::Vector3d>
matchPointToPlane(Point const& p1, KDTree::Ptr kdtree2, double maxSquareDistance,
                  int requiredNbNeighbours, double planeDiscriminatorThreshold);


/**
 * @brief analyzePlane This function analyzes a set of points with a PCA
 * @param points
 * @return 3 eigen values and the eigen vector corresponding to the lowest eigen value
 */
inline std::tuple<double, double, double, Eigen::Vector3d>
analyzePlane(Eigen::MatrixX3d const& points)
{
  Eigen::Vector3d mean = points.colwise().mean();
  Eigen::MatrixX3d centered = points.rowwise() - mean.transpose();
  Eigen::MatrixXd cov = centered.transpose() * centered;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
  Eigen::Vector3d D = eig.eigenvalues();              // eigen values
  Eigen::Matrix3d V = eig.eigenvectors();             // eigen vectors

  // Sort eigen values in decreasing order
  std::vector<unsigned int> D_idx = {0, 1, 2};
  std::sort(D_idx.begin(), D_idx.end(), [&D](unsigned int i0, unsigned int i1)
  { return D(i0) > D(i1); });

  return std::make_tuple(D[D_idx[0]], D[D_idx[1]], D[D_idx[2]], V.col(D_idx[2]));
}

} // namespace CalibrationDepthPose

#endif // CALIBTOOLS_H
