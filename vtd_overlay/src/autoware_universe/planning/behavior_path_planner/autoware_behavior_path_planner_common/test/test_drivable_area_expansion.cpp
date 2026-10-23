// Copyright 2023-2024 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/drivable_area_expansion.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/path_projection.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/map_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/types.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <memory>

using autoware::behavior_path_planner::drivable_area_expansion::LineString2d;
using autoware::behavior_path_planner::drivable_area_expansion::Point2d;
using autoware::behavior_path_planner::drivable_area_expansion::Segment2d;
constexpr auto eps = 1e-9;

TEST(DrivableAreaExpansionProjection, PointToSegment)
{
  using autoware::behavior_path_planner::drivable_area_expansion::point_to_segment_projection;

  {
    Point2d query(1.0, 1.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 1.0, eps);
    EXPECT_NEAR(projection.point.x(), 1.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    Point2d query(-1.0, 1.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, std::sqrt(2), eps);
    EXPECT_NEAR(projection.point.x(), 0.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    Point2d query(11.0, 1.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, std::sqrt(2), eps);
    EXPECT_NEAR(projection.point.x(), 10.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    Point2d query(5.0, -5.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 5.0, eps);
    EXPECT_NEAR(projection.point.x(), 5.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    Point2d query(5.0, -5.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(0.0, -10.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 5.0, eps);
    EXPECT_NEAR(projection.point.x(), 0.0, eps);
    EXPECT_NEAR(projection.point.y(), -5.0, eps);
  }
  {
    Point2d query(5.0, 5.0);
    Segment2d segment(Point2d(2.5, 7.5), Point2d(7.5, 2.5));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 0.0, eps);
    EXPECT_NEAR(projection.point.x(), 5.0, eps);
    EXPECT_NEAR(projection.point.y(), 5.0, eps);
  }
  {
    Point2d query(0.0, 0.0);
    Segment2d segment(Point2d(2.5, 7.5), Point2d(7.5, 2.5));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, std::sqrt(50), eps);
    EXPECT_NEAR(projection.point.x(), 5.0, eps);
    EXPECT_NEAR(projection.point.y(), 5.0, eps);
  }
}

TEST(DrivableAreaExpansionProjection, PointToLinestring)
{
  using autoware::behavior_path_planner::drivable_area_expansion::point_to_linestring_projection;

  LineString2d ls = {
    Point2d(0.0, 0.0), Point2d(10.0, 0.0), Point2d(10.0, 10.0), Point2d(0.0, 10.0),
    Point2d(5.0, 5.0)};
  {
    Point2d query(0.0, 0.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_NEAR(projection.arc_length, 0.0, eps);
    EXPECT_NEAR(projection.distance, 0.0, eps);
    EXPECT_NEAR(projection.projected_point.x(), 0.0, eps);
    EXPECT_NEAR(projection.projected_point.y(), 0.0, eps);
  }
  {
    Point2d query(2.0, 1.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_NEAR(projection.arc_length, 2.0, eps);
    EXPECT_NEAR(projection.distance, 1.0, eps);
    EXPECT_NEAR(projection.projected_point.x(), 2.0, eps);
    EXPECT_NEAR(projection.projected_point.y(), 0.0, eps);
  }
  {
    Point2d query(0.0, 5.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_NEAR(projection.arc_length, 30.0 + std::sqrt(2.5 * 2.5 * 2), eps);
    EXPECT_NEAR(projection.distance, std::sqrt(2.5 * 2.5 * 2), eps);
    EXPECT_NEAR(projection.projected_point.x(), 2.5, eps);
    EXPECT_NEAR(projection.projected_point.y(), 7.5, eps);
  }
}

TEST(DrivableAreaExpansionProjection, LinestringToPoint)
{
  using autoware::behavior_path_planner::drivable_area_expansion::linestring_to_point_projection;

  LineString2d ls = {
    Point2d(0.0, 0.0), Point2d(10.0, 0.0), Point2d(10.0, 10.0), Point2d(0.0, 10.0),
    Point2d(5.0, 5.0)};
  for (auto arc_length = 0.0; arc_length <= 10.0; arc_length += 1.0) {
    const auto projection = linestring_to_point_projection(ls, arc_length, 0.0);
    EXPECT_NEAR(projection.first.x(), arc_length, eps);
    EXPECT_NEAR(projection.first.y(), 0.0, eps);
    EXPECT_NEAR(projection.second.x(), arc_length, eps);
    EXPECT_NEAR(projection.second.y(), 0.0, eps);
  }
  for (auto arc_length = 11.0; arc_length <= 20.0; arc_length += 1.0) {
    const auto projection = linestring_to_point_projection(ls, arc_length, 0.0);
    EXPECT_NEAR(projection.first.x(), 10.0, eps);
    EXPECT_NEAR(projection.first.y(), arc_length - 10.0, eps);
    EXPECT_NEAR(projection.second.x(), 10.0, eps);
    EXPECT_NEAR(projection.second.y(), arc_length - 10.0, eps);
  }
  for (auto arc_length = 21.0; arc_length <= 30.0; arc_length += 1.0) {
    const auto projection = linestring_to_point_projection(ls, arc_length, 0.0);
    EXPECT_NEAR(projection.first.x(), 10.0 + (20 - arc_length), eps);
    EXPECT_NEAR(projection.first.y(), 10.0, eps);
    EXPECT_NEAR(projection.second.x(), 10.0 + (20 - arc_length), eps);
    EXPECT_NEAR(projection.second.y(), 10.0, eps);
  }
}

TEST(DrivableAreaExpansionProjection, InverseProjection)
{
  using autoware::behavior_path_planner::drivable_area_expansion::linestring_to_point_projection;
  using autoware::behavior_path_planner::drivable_area_expansion::point_to_linestring_projection;

  LineString2d ls = {
    Point2d(0.0, 0.0), Point2d(10.0, 0.0), Point2d(10.0, 10.0), Point2d(0.0, 10.0),
    Point2d(5.0, 5.0)};

  for (auto x = 0.0; x < 10.0; x += 0.1) {
    for (auto y = 0.0; x < 10.0; x += 0.1) {
      Point2d p(x, y);
      const auto projection = point_to_linestring_projection(p, ls);
      const auto inverse =
        linestring_to_point_projection(ls, projection.arc_length, projection.distance);
      EXPECT_NEAR(inverse.second.x(), p.x(), eps);
      EXPECT_NEAR(inverse.second.y(), p.y(), eps);
    }
  }
}

TEST(DrivableAreaExpansionProjection, expand_drivable_area)
{
  autoware::behavior_path_planner::drivable_area_expansion::DrivableAreaExpansionParameters params;
  autoware::behavior_path_planner::drivable_area_expansion::PredictedObjects dynamic_objects;
  lanelet::LaneletMapPtr empty_lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  autoware_map_msgs::msg::LaneletMapBin map =
    autoware::experimental::lanelet2_utils::to_autoware_map_msgs(empty_lanelet_map_ptr);
  autoware::route_handler::RouteHandler route_handler(map);
  lanelet::ConstLanelets path_lanes = {};
  autoware::behavior_path_planner::drivable_area_expansion::PathWithLaneId path;
  {  // Simple path with Y=0 and X = 0, 1, 2
    autoware::behavior_path_planner::drivable_area_expansion::PathWithLaneId::_points_type::
      value_type p;
    p.point.pose.position.x = 0.0;
    p.point.pose.position.y = 0.0;
    path.points.push_back(p);
    p.point.pose.position.x = 1.0;
    path.points.push_back(p);
    p.point.pose.position.x = 2.0;
    path.points.push_back(p);
  }
  {  // Left bound at Y = 1, Right bound at Y = -1
    autoware::behavior_path_planner::drivable_area_expansion::Point pl;
    autoware::behavior_path_planner::drivable_area_expansion::Point pr;
    pl.y = 1.0;
    pr.y = -1.0;
    pl.x = 0.0;
    pr.x = 0.0;
    path.left_bound.push_back(pl);
    path.right_bound.push_back(pr);
    pl.x = 1.0;
    pr.x = 1.0;
    path.left_bound.push_back(pl);
    path.right_bound.push_back(pr);
    pl.x = 2.0;
    pr.x = 2.0;
    path.left_bound.push_back(pl);
    path.right_bound.push_back(pr);
  }
  {  // parameters
    params.enabled = true;
    params.object_exclusion.exclude_dynamic = false;
    params.avoid_linestring_dist = 0.0;
    params.avoid_linestring_types = {};
    params.max_expansion_distance = 0.0;  // means no limit
    params.max_path_arc_length = 0.0;     // means no limit
    params.resample_interval = 1.0;
    // 2m x 4m ego footprint
    params.vehicle_info.front_overhang_m = 0.0;
    params.vehicle_info.wheel_base_m = 2.0;
    params.vehicle_info.vehicle_width_m = 2.0;
  }
  autoware::behavior_path_planner::PlannerData planner_data;
  planner_data.drivable_area_expansion_parameters = params;
  planner_data.dynamic_object =
    std::make_shared<autoware::behavior_path_planner::drivable_area_expansion::PredictedObjects>(
      dynamic_objects);
  planner_data.self_odometry = std::make_shared<nav_msgs::msg::Odometry>();
  planner_data.route_handler =
    std::make_shared<autoware::route_handler::RouteHandler>(route_handler);
  autoware::behavior_path_planner::drivable_area_expansion::expand_drivable_area(
    path, std::make_shared<autoware::behavior_path_planner::PlannerData>(planner_data));
  // unchanged path points
  ASSERT_EQ(path.points.size(), 3ul);
  for (auto i = 0.0; i < path.points.size(); ++i) {
    EXPECT_NEAR(path.points[i].point.pose.position.x, i, eps);
    EXPECT_NEAR(path.points[i].point.pose.position.y, 0.0, eps);
  }
  // straight path: no expansion
  // expanded left bound
  ASSERT_EQ(path.left_bound.size(), 3ul);
  EXPECT_NEAR(path.left_bound[0].x, 0.0, eps);
  EXPECT_NEAR(path.left_bound[0].y, 1.0, eps);
  EXPECT_NEAR(path.left_bound[1].x, 1.0, eps);
  EXPECT_NEAR(path.left_bound[1].y, 1.0, eps);
  EXPECT_NEAR(path.left_bound[2].x, 2.0, eps);
  EXPECT_NEAR(path.left_bound[2].y, 1.0, eps);
  // expanded right bound
  ASSERT_EQ(path.right_bound.size(), 3ul);
  EXPECT_NEAR(path.right_bound[0].x, 0.0, eps);
  EXPECT_NEAR(path.right_bound[0].y, -1.0, eps);
  EXPECT_NEAR(path.right_bound[1].x, 1.0, eps);
  EXPECT_NEAR(path.right_bound[1].y, -1.0, eps);
  EXPECT_NEAR(path.right_bound[2].x, 2.0, eps);
  EXPECT_NEAR(path.right_bound[2].y, -1.0, eps);

  // add some curvature
  path.points[1].point.pose.position.y = 0.5;

  autoware::behavior_path_planner::drivable_area_expansion::expand_drivable_area(
    path, std::make_shared<autoware::behavior_path_planner::PlannerData>(planner_data));
  // expanded left bound
  for (const auto & p : path.left_bound) {
    EXPECT_GT(p.y, 1.0);
  }
  // Clockwise/right turn: only the outer (left) bound is expanded.
  for (const auto & p : path.right_bound) {
    EXPECT_NEAR(p.y, -1.0, eps);
  }
}

TEST(DrivableAreaExpansion, ExpandsOuterBoundOnlyForRightTurns)
{
  using autoware::behavior_path_planner::drivable_area_expansion::Expansion;
  using autoware::behavior_path_planner::drivable_area_expansion::calculate_expansion_distances;

  const auto calculate = [](const double curvature) {
    Expansion expansion;
    expansion.left_bound_indexes = {0U};
    expansion.right_bound_indexes = {0U};
    expansion.left_projections.resize(1U);
    expansion.right_projections.resize(1U);
    expansion.left_projections.front().distance = 0.75;
    expansion.right_projections.front().distance = 0.75;
    expansion.min_lane_widths = {2.0};
    expansion.curvatures = {curvature};
    calculate_expansion_distances(expansion, {1.0, 1.0}, {1.0, 1.0});
    return expansion;
  };

  const auto right_turn = calculate(-0.1);
  EXPECT_DOUBLE_EQ(right_turn.left_distances.at(0), 0.5);
  EXPECT_DOUBLE_EQ(right_turn.right_distances.at(0), 0.0);

  const auto left_turn = calculate(0.1);
  EXPECT_DOUBLE_EQ(left_turn.left_distances.at(0), 0.25);
  EXPECT_DOUBLE_EQ(left_turn.right_distances.at(0), 0.25);
}

TEST(DrivableAreaExpansion, WideBusKeepsClearanceThroughStraightTurnExit)
{
  using namespace autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::msg::Pose> poses(3);
  std::vector<geometry_msgs::msg::Point> left(2), right(2);
  for (size_t i = 0; i < poses.size(); ++i) poses[i].position.x = i * 5.0;
  left[0].y = left[1].y = 1.75;
  right[0].y = right[1].y = -1.75;
  left[1].x = right[1].x = 10.0;
  DrivableAreaExpansionParameters params;
  params.vehicle_info.vehicle_width_m = 4.04;
  params.vehicle_info.wheel_base_m = 5.63;
  params.vehicle_info.front_overhang_m = 1.68;
  params.extra_width = 0.6;
  auto expanded = calculate_expansion(poses, left, right, {0.0, 1e-8, -1e-8}, params);
  for (const auto width : expanded.min_lane_widths) EXPECT_NEAR(width, 4.64, 1e-5);
  // A real curb on the right must shift all extra space toward the inner lane.
  calculate_expansion_distances(expanded, {2.0, 2.0}, {0.0, 0.0});
  EXPECT_NEAR(expanded.left_distances[0], 1.14, 1e-5);
  EXPECT_DOUBLE_EQ(expanded.right_distances[0], 0.0);
  // If both sides are blocked, do not fabricate space beyond those limits.
  calculate_expansion_distances(expanded, {0.0, 0.0}, {0.0, 0.0});
  EXPECT_DOUBLE_EQ(expanded.left_distances[0], 0.0);
  EXPECT_DOUBLE_EQ(expanded.right_distances[0], 0.0);
  // Preserve the previous straight-road behavior for a car that fits its lane.
  params.vehicle_info.vehicle_width_m = 2.0;
  const auto car = calculate_expansion(poses, left, right, {0.0, 0.0, 0.0}, params);
  for (const auto width : car.min_lane_widths) EXPECT_DOUBLE_EQ(width, 0.0);
}

TEST(DrivableAreaExpansion, LateralConnectorDoesNotPinItsInternalEndpoint)
{
  using namespace autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<Point> bound(4);
  bound[0].x = 0.0; bound[0].y = 5.25;
  bound[1].x = 10.0; bound[1].y = 5.25;
  bound[2].x = 10.0; bound[2].y = 1.75;
  bound[3].x = 20.0; bound[3].y = 1.75;
  std::vector<Pose> path(2);
  path[1].position.x = 30.0;
  SegmentRtree protected_lines;
  protected_lines.insert(Segment2d{Point2d{0.0, 5.25}, Point2d{10.0, 5.25}});
  DrivableAreaExpansionParameters params;
  params.avoid_linestring_dist = 0.1;
  params.max_expansion_distance = 3.5;
  const auto limits = calculate_maximum_distance(bound, protected_lines, {}, params, LEFT, path);
  EXPECT_DOUBLE_EQ(limits[0], 0.0);
  EXPECT_DOUBLE_EQ(limits[1], 0.0);
  EXPECT_NEAR(limits[2], 3.4, 1e-6);
  // A curved approach changes the projection direction. A small tilt must not
  // recreate the pinned endpoint while the true road edge remains protected.
  auto tilted_path = path;
  tilted_path[1].position.y = 3.0;
  const auto tilted =
    calculate_maximum_distance(bound, protected_lines, {}, params, LEFT, tilted_path);
  EXPECT_DOUBLE_EQ(tilted[1], 0.0);
  EXPECT_NEAR(tilted[2], 3.4, 1e-6);
  // A protected line in the middle of the connector still blocks expansion.
  protected_lines.insert(Segment2d{Point2d{9.0, 3.0}, Point2d{11.0, 3.0}});
  const auto blocked = calculate_maximum_distance(bound, protected_lines, {}, params, LEFT, path);
  EXPECT_LE(blocked[2], 1.15 + 1e-6);
  // No relaxation for a normal longitudinal road boundary.
  std::vector<Point> straight(2);
  straight[0].y = straight[1].y = 5.25;
  straight[1].x = 10.0;
  const auto fixed = calculate_maximum_distance(straight, protected_lines, {}, params, LEFT, path);
  EXPECT_DOUBLE_EQ(fixed[0], 0.0);
  EXPECT_DOUBLE_EQ(fixed[1], 0.0);
}

TEST(VerifiedStraightMargin, RequiresProofAndKeepsObjectsOtherLevelsAndGapsProtected)
{
  using namespace autoware::behavior_path_planner::drivable_area_expansion;
  lanelet::LaneletMap map;
  lanelet::Point3d a(9001, 0.0, -1.75, 0.0), b(9002, 20.0, -1.75, 0.0);
  lanelet::LineString3d edge(9003, {a, b});
  edge.attributes()["type"] = "road_border";
  map.add(edge);
  Point p; p.x = 10.0; p.y = -1.75;
  auto check = [&](const std::vector<Polygon2d> & objects,
                   const double height = 0.0) {
    auto point = p; point.z = height;
    std::vector<Point> bound{point};
    apply_verified_straight_margins(bound, map, objects);
    return bound.front().y;
  };
  EXPECT_DOUBLE_EQ(check({}), -1.75);  // No blanket road-border override.
  edge.attributes()["bus:margin_source"] = "carla_paved_straight_v1";
  edge.attributes()["bus:outward_sign"] = "-1";
  a.attributes()["bus:margin:9003"] = "0.65";
  b.attributes()["bus:margin:9003"] = "0.65";
  EXPECT_NEAR(check({}), -2.4, 1e-9);
  EXPECT_DOUBLE_EQ(check({}, 10.0), -1.75);  // Different road level.
  Polygon2d object;
  object.outer() = {{9.0, -2.5}, {9.0, -2.0}, {11.0, -2.0}, {11.0, -2.5}, {9.0, -2.5}};
  EXPECT_DOUBLE_EQ(check({object}), -1.75);
  b.attributes()["bus:margin:9003"] = "0";
  EXPECT_DOUBLE_EQ(check({}), -1.75);  // No interpolation across unverified gaps.
}

TEST(VerifiedStraightMargin, SharedPaintedDividerUsesPathSideWithoutChangingRouting)
{
  using namespace autoware::behavior_path_planner::drivable_area_expansion;
  lanelet::LaneletMap map;
  lanelet::Point3d a(9101, 0.0, 0.0, 0.0), b(9102, 20.0, 0.0, 0.0);
  a.attributes()["bus:margin:9103"] = "0.5";
  b.attributes()["bus:margin:9103"] = "0.5";
  lanelet::LineString3d edge(9103, {a, b});
  edge.attributes()["type"] = "line_thin";
  edge.attributes()["subtype"] = "solid";
  edge.attributes()["lane_change"] = "no";
  edge.attributes()["carla:bus_boundary_source"] = "native_nonjunction_edges_v1";
  edge.attributes()["bus:margin_source"] = "carla_paved_straight_v2";
  edge.attributes()["bus:outward_sign"] = "0";
  map.add(edge);
  Point p; p.x = 10.0;
  Pose pose; pose.position.x = 10.0; pose.position.y = -1.75;
  std::vector<Point> right_side{p};
  apply_verified_straight_margins(right_side, map, {}, {pose});
  EXPECT_DOUBLE_EQ(right_side.front().y, 0.5);
  pose.position.y = 1.75;
  std::vector<Point> left_side{p};
  apply_verified_straight_margins(left_side, map, {}, {pose});
  EXPECT_DOUBLE_EQ(left_side.front().y, -0.5);
  EXPECT_EQ(edge.attributeOr("lane_change", std::string{}), "no");
  edge.attributes()["type"] = "curbstone";
  std::vector<Point> physical_divider{p};
  apply_verified_straight_margins(physical_divider, map, {}, {pose});
  EXPECT_DOUBLE_EQ(physical_divider.front().y, 0.0);
}


TEST(DrivableAreaExpansion, ConnectorWidthAllocationKeepsPhysicalEdgeAndOpensInternalVertex)
{
  using namespace autoware::behavior_path_planner::drivable_area_expansion;
  Expansion expansion;
  expansion.min_lane_widths = {5.44};
  expansion.curvatures = {0.0};
  expansion.left_bound_indexes = {0};
  expansion.right_bound_indexes = {1};
  expansion.left_projections.resize(1);
  expansion.right_projections.resize(1);
  expansion.left_projections[0].distance = 1.75;
  expansion.right_projections[0].distance = 1.75;
  // Centre divider is fixed; right boundary joins a fixed road edge to an
  // internal dashed line with 3.4 m of measured clearance to that edge.
  calculate_expansion_distances(expansion, {0.0, 0.0}, {0.0, 0.0, 3.4, 3.4});
  EXPECT_DOUBLE_EQ(expansion.right_distances[1], 0.0);
  EXPECT_NEAR(expansion.right_distances[2], 1.94, 1e-6);
  EXPECT_DOUBLE_EQ(expansion.left_distances[0], 0.0);
  // An object or protected line may close that space on the next update.
  calculate_expansion_distances(expansion, {0.0, 0.0}, {0.0, 0.0, 0.0, 0.0});
  for (const auto distance : expansion.right_distances) EXPECT_DOUBLE_EQ(distance, 0.0);
}

TEST(DrivableAreaExpansion, SharedVertexKeepsLargerSweptWidthRequest)
{
  using namespace autoware::behavior_path_planner::drivable_area_expansion;
  Expansion expansion;
  expansion.min_lane_widths = {5.44, 4.04};
  expansion.curvatures = {0.0, 0.0};
  expansion.left_bound_indexes = {0, 0};
  expansion.right_bound_indexes = {0, 0};
  expansion.left_projections.resize(2);
  expansion.right_projections.resize(2);
  for (auto & p : expansion.left_projections) p.distance = 1.75;
  for (auto & p : expansion.right_projections) p.distance = 1.75;
  calculate_expansion_distances(expansion, {0.0, 0.0}, {3.4, 3.4});
  EXPECT_NEAR(expansion.right_distances[0], 1.94, 1e-6);
  EXPECT_NEAR(expansion.right_distances[1], 1.94, 1e-6);
}
