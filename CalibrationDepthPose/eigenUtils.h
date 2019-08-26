#ifndef EIGENUTILS_H
#define EIGENUTILS_H

#include <vector>

#include <Eigen/Dense>

namespace CalibrationDepthPose
{

// force to use eigen's aligned allocator, otherwise it can cause crashes
using Isometry3d_vector
  = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> >;

} // namespace CalibrationDepthPose

#endif // EIGENUTILS_H
