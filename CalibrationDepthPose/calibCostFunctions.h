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

#ifndef CALIBCOSTFUNCTIONS_H
#define CALIBCOSTFUNCTIONS_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>


namespace CalibrationDepthPose
{

/**
 * @brief The CalibrationResiduals struct Residual function for matching
 * The distances are computed in camera 2 frame, which means that pt is transformed to camera 2 frame
 */
struct CalibrationResiduals
{
public:
  /**
   * @brief CalibrationResiduals
   * @param pt1 [in] first point in camera 1 frame
   * @param pt2 [in] second point in camera 2 frame
   * @param normal [in] normal at pt2 in camera 2 frame ([0, 0, 0] if point-to-point distance)
   * @param pose1ToPose2 [in] transform from pose1 to pose2
   */
  CalibrationResiduals(Eigen::Vector3d const& pt1 ,Eigen::Vector3d const& pt2,
                       Eigen::Vector3d const& normal, Eigen::Isometry3d const& pose1ToPose2)
    : X(pt1), Y(pt2), N(normal), H_pose2_pose1(pose1ToPose2)
  {}

  template <typename T>
  bool operator ()(const T* const calib, T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Xj, Yj, Nj;
    Xj << T(X.x()), T(X.y()), T(X.z());
    Yj << T(Y.x()), T(Y.y()), T(Y.z());
    Nj << T(N.x()), T(N.y()), T(N.z());

    Eigen::Quaterniond H_pose2_pose1_rotation(H_pose2_pose1.rotation());
    Eigen::Quaternion<T> Hj_pose2_pose1_rotation(T(H_pose2_pose1_rotation.w()),
                                                 T(H_pose2_pose1_rotation.x()),
                                                 T(H_pose2_pose1_rotation.y()),
                                                 T(H_pose2_pose1_rotation.z()));
    Eigen::Transform<T, 3, Eigen::Isometry>
        Hj_pose2_pose1 = Eigen::Translation<T, 3>(T(H_pose2_pose1.translation().x()),
                                                  T(H_pose2_pose1.translation().y()),
                                                  T(H_pose2_pose1.translation().z()))
        * Hj_pose2_pose1_rotation;

    Eigen::Transform<T, 3, Eigen::Isometry>
        calib_j = Eigen::Translation<T, 3>(T(calib[4]), T(calib[5]), T(calib[6]))
        * Eigen::Quaternion<T>(T(calib[0]), T(calib[1]), T(calib[2]), T(calib[3]));


    Eigen::Matrix<T, 3, 1> Xj_cam2 = calib_j.inverse() * Hj_pose2_pose1 * calib_j * Xj;
    if (N.x() == 0 && N.y() == 0 && N.z() == 0)
    {
      // compute point to point distance
      residuals[0] = Xj_cam2[0] - Yj[0];
      residuals[1] = Xj_cam2[1] - Yj[1];
      residuals[2] = Xj_cam2[2] - Yj[2];
    }
    else {
      // compute point to plane distance
      residuals[0] = (Xj_cam2 - Yj).dot(Nj);
      residuals[1] = T(0);
      residuals[2] = T(0);
    }

    return true;
  }


  static ceres::CostFunction* Create(Eigen::Vector3d const& pt1 ,Eigen::Vector3d const& pt2,
                                     Eigen::Vector3d const& normal,
                                     Eigen::Isometry3d const& pose1ToPose2)
  {
    return (new ceres::AutoDiffCostFunction<CalibrationResiduals, 3, 7>(
              new CalibrationResiduals(pt1, pt2, normal, pose1ToPose2)));
  }


private:
  Eigen::Vector3d X;
  Eigen::Vector3d Y;
  Eigen::Vector3d N;
  Eigen::Isometry3d H_pose2_pose1; // pose2.inv * pose1

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} // namespace CalibrationDepthPose


#endif // CALIBCOSTFUNCTIONS_H
