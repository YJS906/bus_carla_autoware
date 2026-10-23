// Copyright 2023 TIER IV, Inc.
//
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

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/footprints.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
Polygon2d translate_polygon(const Polygon2d & polygon, const double x, const double y)
{
  Polygon2d translated_polygon;
  const boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translation(x, y);
  boost::geometry::transform(polygon, translated_polygon, translation);
  return translated_polygon;
}

Polygon2d create_footprint(const geometry_msgs::msg::Pose & pose, const Polygon2d & base_footprint)
{
  const auto angle = tf2::getYaw(pose.orientation);
  return translate_polygon(
    autoware_utils::rotate_polygon(base_footprint, angle), pose.position.x, pose.position.y);
}

namespace
{
struct PathProjection
{
  double arc{};
  double distance{std::numeric_limits<double>::infinity()};
  double yaw{};
};

PathProjection project_on_path(
  const double x, const double y, const std::vector<geometry_msgs::msg::Pose> & path)
{
  PathProjection result;
  double arc = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1].position;
    const auto & b = path[i].position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-6) continue;
    const double t = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / (length * length), 0.0, 1.0);
    const double distance = std::hypot(x - a.x - t * dx, y - a.y - t * dy);
    if (distance < result.distance) result = {arc + t * length, distance, std::atan2(dy, dx)};
    arc += length;
  }
  return result;
}

bool is_separated_follower(
  const geometry_msgs::msg::Pose & pose, const Polygon2d & current_footprint,
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<geometry_msgs::msg::Pose> & path,
  const DrivableAreaExpansionParameters & params)
{
  if (path.size() < 2 || params.vehicle_info.vehicle_width_m <= 0) return false;
  const auto object = project_on_path(pose.position.x, pose.position.y, path);
  const auto ego = project_on_path(ego_pose.position.x, ego_pose.position.y, path);
  // Match the heading of the road at the following vehicle, not ego's turning
  // heading. A parallel vehicle in a neighboring lane can also be fully behind
  // ego; its merge risk is checked by the unchanged temporal/RSS planners.
  // Oncoming and crossing headings never qualify for this historical exception.
  if (std::cos(tf2::getYaw(pose.orientation) - object.yaw) < std::cos(M_PI / 6.0)) return false;
  for (const auto & vertex : current_footprint.outer()) {
    const auto point = project_on_path(vertex.x(), vertex.y(), path);
    if (point.arc >= ego.arc - params.vehicle_info.rear_overhang_m) return false;
  }
  return true;
}

MultiPolygon2d create_object_footprints_impl(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const DrivableAreaExpansionParameters & params, const geometry_msgs::msg::Pose * ego_pose,
  const std::vector<geometry_msgs::msg::Pose> & path_poses)

{
  using behavior_path_planner::utils::path_safety_checker::filter::velocity_filter;

  MultiPolygon2d footprints;
  if (!params.object_exclusion.exclude_dynamic && !params.object_exclusion.exclude_static) {
    return footprints;
  }

  auto get_base_footprint = [&](const auto & object) {
    const auto front = object.shape.dimensions.x / 2 + params.object_exclusion.front_offset;
    const auto rear = -object.shape.dimensions.x / 2 - params.object_exclusion.rear_offset;
    const auto left = object.shape.dimensions.y / 2 + params.object_exclusion.left_offset;
    const auto right = -object.shape.dimensions.y / 2 - params.object_exclusion.right_offset;
    Polygon2d footprint;
    footprint.outer() = {
      Point2d{front, left}, Point2d{front, right}, Point2d{rear, right}, Point2d{rear, left},
      Point2d{front, left}};
    return footprint;
  };

  for (const auto & object : objects.objects) {
    const auto base_footprint = get_base_footprint(object);
    if (params.object_exclusion.exclude_dynamic) {
      using Classification = autoware_perception_msgs::msg::ObjectClassification;
      const bool road_vehicle = std::any_of(
        object.classification.begin(), object.classification.end(), [](const auto & c) {
          return c.probability > 0.5 && (c.label == Classification::CAR ||
            c.label == Classification::TRUCK || c.label == Classification::BUS ||
            c.label == Classification::TRAILER);
        });
      if (ego_pose) {
        const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
        const auto current = create_footprint(pose, base_footprint);
        if (road_vehicle && is_separated_follower(pose, current, *ego_pose, path_poses, params)) {
          // This car is behind the entire rear safety envelope. Letting it narrow
          // the historical road corridor makes the optimizer rewrite the path
          // already driven by ego. Keep its current shape and full predictions
          // in the original object message for collision/RSS checks instead.
          continue;
        }
        footprints.push_back(current);  // includes the full existing safety margins
      }
      for (const auto & path : object.kinematics.predicted_paths) {
        const double step = path.time_step.sec + path.time_step.nanosec * 1e-9;
        for (size_t i = 0; i < path.path.size(); ++i) {
          // Road geometry is a near-term envelope, not the union of every
          // possible junction branch over the full prediction horizon.
          // Temporal collision/RSS modules still receive the original message.
          // Pedestrians, unknown objects and invalid time steps stay conservative.
          constexpr double geometry_prediction_horizon = 3.0;
          if (ego_pose && road_vehicle && step > 0.0 &&
              i * step > geometry_prediction_horizon) break;
          footprints.push_back(create_footprint(path.path[i], base_footprint));
        }
      }
      continue;
    }

    if (
      params.object_exclusion.exclude_static &&
      velocity_filter(
        object.kinematics.initial_twist_with_covariance.twist,
        -std::numeric_limits<double>::epsilon(), params.object_exclusion.stopped_obj_vel_th)) {
      footprints.push_back(
        create_footprint(object.kinematics.initial_pose_with_covariance.pose, base_footprint));
    }
  }
  return footprints;
}
}  // namespace

MultiPolygon2d create_object_footprints(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const DrivableAreaExpansionParameters & params)
{
  return create_object_footprints_impl(objects, params, nullptr, {});
}

MultiPolygon2d create_object_footprints(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const DrivableAreaExpansionParameters & params, const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<geometry_msgs::msg::Pose> & path_poses)
{
  return create_object_footprints_impl(objects, params, &ego_pose, path_poses);
}
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
