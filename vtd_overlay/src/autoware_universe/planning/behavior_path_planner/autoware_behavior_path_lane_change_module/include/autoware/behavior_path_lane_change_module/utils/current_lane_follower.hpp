// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CURRENT_LANE_FOLLOWER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CURRENT_LANE_FOLLOWER_HPP_

#include <autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/is_empty.hpp>
#include <cmath>

namespace autoware::behavior_path_planner::utils::lane_change
{
// A separated source-lane follower is responsible for following ego's braking.
// It must not veto a merge solely because its constant-speed forecast catches
// ego in the lane ego is leaving. Target-lane rear vehicles remain RSS targets.
template<class CurrentPolygon, class TargetPolygon>
bool is_separated_current_lane_follower(
  const path_safety_checker::ExtendedPredictedObject & object,
  const geometry_msgs::msg::Pose & ego, const double rear_overhang,
  const CurrentPolygon & current, const TargetPolygon & target)
{
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  const auto label = object.classification.label;
  if (label != Classification::CAR && label != Classification::TRUCK &&
      label != Classification::BUS && label != Classification::TRAILER) return false;
  if (object.initial_polygon.outer().size() < 4 ||
      boost::geometry::is_empty(current) || boost::geometry::is_empty(target) ||
      boost::geometry::disjoint(object.initial_polygon, current) ||
      !boost::geometry::disjoint(object.initial_polygon, target)) return false;
  const auto yaw = tf2::getYaw(ego.orientation);
  if (std::cos(tf2::getYaw(object.initial_pose.orientation) - yaw) < std::cos(M_PI / 6)) {
    return false;
  }
  for (const auto & p : object.initial_polygon.outer()) {
    const auto longitudinal =
      (p.x() - ego.position.x) * std::cos(yaw) + (p.y() - ego.position.y) * std::sin(yaw);
    if (longitudinal >= -rear_overhang - 1.0) return false;
  }
  // A source-lane vehicle cutting into the target lane is not a simple follower.
  for (const auto & prediction : object.predicted_paths) {
    for (const auto & point : prediction.path) {
      if (!boost::geometry::disjoint(point.poly, target)) return false;
    }
  }
  return true;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
#endif
