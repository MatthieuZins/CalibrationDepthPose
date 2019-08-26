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

#ifndef CALIBDEPTHPOSE_H
#define CALIBDEPTHPOSE_H

#include <Eigen/Dense>

#include <CalibrationDepthPose/eigenUtils.h>
#include <CalibrationDepthPose/matchingMatrix.h>
#include <CalibrationDepthPose/pclUtils.h>


namespace CalibrationDepthPose
{

class CalibParameters;

struct CalibrationRaw
{
  double qw, qx, qy, qz, x, y, z;

  Eigen::Isometry3d toIsometry3d() const {
    return Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);
  }

  void fromIsometry3d(Eigen::Isometry3d const& h) {
    Eigen::Quaterniond q(h.rotation());
    qw = q.w(); qx = q.x(); qy = q.y(); qz = q.z();
    x = h.translation().x();
    y = h.translation().y();
    z = h.translation().z();
  }

  double* data() {
    return reinterpret_cast<double*>(this);
  }
};

std::ostream& operator <<(std::ostream& os, CalibrationRaw const& calib);



class CalibDepthPose
{
public:
  CalibDepthPose(const std::vector<Pointcloud::Ptr> &pointclouds,
                 const Isometry3d_vector &poses,
                 const Eigen::Isometry3d &initial_calib);

   Eigen::Isometry3d calibrate(int nbIterations, CalibParameters *params);

  MatchingMatrix& getMatchingMatrix() {
    return m_matchMatrix;
  }

  void calibIteration(CalibParameters *params);

  Eigen::Isometry3d getCurrentCalibration() const {
    return m_calib.toIsometry3d();
  }

private:
  std::vector<Pointcloud::Ptr> m_pointclouds;     // list of pointclouds
  Isometry3d_vector m_poses;         // list of poses
  MatchingMatrix m_matchMatrix;
  CalibrationRaw m_calib;     // current estimate of the calibration
                              // (it is the pose of the camera in the other sensor frame)
};

} // namespace CalibrationDepthPose

#endif // CALIBDEPTHPOSE_H
