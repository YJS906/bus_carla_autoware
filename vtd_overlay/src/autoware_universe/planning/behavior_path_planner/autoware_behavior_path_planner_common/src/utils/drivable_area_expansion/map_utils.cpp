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

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/map_utils.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
SegmentRtree extract_uncrossable_segments(
  const lanelet::LaneletMap & lanelet_map, const Point & ego_point,
  const DrivableAreaExpansionParameters & params)
{
  SegmentRtree uncrossable_segments_in_range;
  LineString2d line;
  const auto ego_p = Point2d{ego_point.x, ego_point.y};
  const auto linestrings = [&]() {
    if (params.max_path_arc_length <= 0.0) {
      return lanelet::ConstLineStrings3d{
        lanelet_map.lineStringLayer.begin(), lanelet_map.lineStringLayer.end()};
    }
    const auto range = params.max_path_arc_length;
    const lanelet::BoundingBox2d search_area{
      lanelet::BasicPoint2d{ego_point.x - range, ego_point.y - range},
      lanelet::BasicPoint2d{ego_point.x + range, ego_point.y + range}};
    return lanelet_map.lineStringLayer.search(search_area);
  }();

  for (const auto & ls : linestrings) {
    if (has_types(ls, params.avoid_linestring_types)) {
      line.clear();
      for (const auto & p : ls) line.push_back(Point2d{p.x(), p.y()});
      for (auto segment_idx = 0LU; segment_idx + 1 < line.size(); ++segment_idx) {
        Segment2d segment = {line[segment_idx], line[segment_idx + 1]};
        if (
          params.max_path_arc_length <= 0.0 ||
          boost::geometry::distance(segment, ego_p) < params.max_path_arc_length) {
          uncrossable_segments_in_range.insert(segment);
        }
      }
    }
  }
  return uncrossable_segments_in_range;
}

bool has_types(
  const lanelet::ConstLineString3d & ls,
  const std::vector<DrivableAreaExpansionParameters::LinestringType> & types)
{
  constexpr auto no_type = "";
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
  const auto subtype = ls.attributeOr(lanelet::AttributeName::Subtype, no_type);
  const auto matches_type = [&](const auto & t) { return t.matches(type, subtype); };
  return (type != no_type && std::find_if(types.begin(), types.end(), matches_type) != types.end());
}

void apply_verified_straight_margins(
  std::vector<Point> & bound, const lanelet::LaneletMap & map,
  const std::vector<Polygon2d> & objects, const std::vector<Pose> & path_poses)
{
  if (bound.empty()) return;
  double min_x = bound.front().x, max_x = min_x;
  double min_y = bound.front().y, max_y = min_y;
  for (const auto & p : bound) {
    min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
  }
  const auto lines = map.lineStringLayer.search(lanelet::BoundingBox2d{
    lanelet::BasicPoint2d{min_x - 1.0, min_y - 1.0},
    lanelet::BasicPoint2d{max_x + 1.0, max_y + 1.0}});
  for (auto & p : bound) {
    // Straight-road limits, body-length context and tapering are proven in
    // the native map metadata. Do not toggle this physical space based on a
    // changing planned-path curvature (for example a nearby offset goal).
    bool matched = false;
    for (const auto & ls : lines) {
      const auto source = ls.attributeOr("bus:margin_source", std::string{});
      const bool painted_divider = source == "carla_paved_straight_v2" &&
        ls.attributeOr("type", std::string{}) == "line_thin" &&
        ls.attributeOr("subtype", std::string{}) == "solid" &&
        ls.attributeOr("carla:bus_boundary_source", std::string{}) == "native_nonjunction_edges_v1";
      if ((source != "carla_paved_straight_v1" && source != "carla_paved_straight_v2") ||
          (!painted_divider && ls.attributeOr("type", std::string{}) != "road_border")) continue;
      double sign = ls.attributeOr("bus:outward_sign", 0.0);
      if ((!painted_divider && std::abs(sign) != 1.0) || (painted_divider && sign != 0.0)) continue;
      const auto key = "bus:margin:" + std::to_string(ls.id());
      for (size_t i = 1; i < ls.size(); ++i) {
        const auto & a = ls[i - 1]; const auto & b = ls[i];
        const double dx = b.x() - a.x(), dy = b.y() - a.y();
        const double length = std::hypot(dx, dy);
        if (length < 1e-6) continue;
        const double t = ((p.x - a.x()) * dx + (p.y - a.y()) * dy) / (length * length);
        if (t < 0.0 || t > 1.0 ||
            std::hypot(p.x - a.x() - t * dx, p.y - a.y() - t * dy) > 0.12 ||
            std::abs(p.z - a.z() - t * (b.z() - a.z())) > 0.1) continue;
        // Interpolate only between verified vertices; an unverified endpoint
        // prevents crossing a gap in the mesh proof.
        const double ma = a.attributeOr(key, 0.0), mb = b.attributeOr(key, 0.0);
        if (ma <= 0.0 || mb <= 0.0 || ma > 0.65 || mb > 0.65) continue;
        if (painted_divider) {
          if (ma > 0.5 || mb > 0.5 || path_poses.empty()) continue;
          const auto nearest = std::min_element(path_poses.begin(), path_poses.end(),
            [&](const auto & q, const auto & r) {
              return std::hypot(q.position.x - p.x, q.position.y - p.y) <
                     std::hypot(r.position.x - p.x, r.position.y - p.y);
            });
          const double inside = (dx * (nearest->position.y - p.y) -
                                 dy * (nearest->position.x - p.x)) / length;
          if (std::abs(inside) < 0.25) continue;
          // A shared painted divider has two travel directions. Expand away
          // from the current path, without altering either route centreline.
          sign = inside > 0.0 ? -1.0 : 1.0;
        }
        const double margin = ma + t * (mb - ma);
        Point target = p;
        target.x -= sign * dy / length * margin;
        target.y += sign * dx / length * margin;
        const Segment2d swept{{p.x, p.y}, {target.x, target.y}};
        const bool blocked = std::any_of(objects.begin(), objects.end(), [&](const auto & object) {
          return boost::geometry::distance(swept, object) < 0.01;
        });
        if (!blocked) p = target;
        matched = true;
        break;
      }
      if (matched) break;
    }
  }
}
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
