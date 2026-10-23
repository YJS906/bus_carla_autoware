// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef AUTOWARE__PATH_OPTIMIZER__UTILS__ROAD_CLEARANCE_HPP_
#define AUTOWARE__PATH_OPTIMIZER__UTILS__ROAD_CLEARANCE_HPP_
#include <algorithm>
#include <cmath>

namespace autoware::path_optimizer
{
// Retain the original margin in bends; smoothly reduce only the additional
// clearance on straight reference paths. Vehicle dimensions are unchanged.
inline double roadClearance(double curvature, double straight, double curve)
{
  const double fraction = std::clamp((std::abs(curvature) - 0.002) / 0.005, 0.0, 1.0);
  return straight + fraction * (curve - straight);
}
}  // namespace autoware::path_optimizer
#endif  // AUTOWARE__PATH_OPTIMIZER__UTILS__ROAD_CLEARANCE_HPP_
