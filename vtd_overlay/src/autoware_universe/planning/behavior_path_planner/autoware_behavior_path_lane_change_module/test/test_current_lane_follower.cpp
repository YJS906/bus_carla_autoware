// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#include "autoware/behavior_path_lane_change_module/utils/current_lane_follower.hpp"
#include <gtest/gtest.h>

using namespace autoware::behavior_path_planner::utils;
using autoware_utils::Polygon2d;

Polygon2d rectangle(double x0, double x1, double y0, double y1)
{
  Polygon2d p;
  p.outer() = {{x0,y0},{x0,y1},{x1,y1},{x1,y0},{x0,y0}};
  return p;
}

auto follower(double x = -12, double y = 0)
{
  path_safety_checker::ExtendedPredictedObject o;
  o.classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  o.initial_pose.orientation.w = 1.;
  o.initial_pose.position.x = x;
  o.initial_pose.position.y = y;
  o.initial_polygon = rectangle(x-2, x+2, y-1, y+1);
  return o;
}

bool skip(const path_safety_checker::ExtendedPredictedObject & o)
{
  geometry_msgs::msg::Pose ego;
  ego.orientation.w = 1.;
  return lane_change::is_separated_current_lane_follower(
    o, ego, 3., rectangle(-100,100,-1.75,1.75), rectangle(-100,100,1.75,5.25));
}

TEST(CurrentLaneFollower, SeparatedSourceFollowerDoesNotVetoMerge) { EXPECT_TRUE(skip(follower())); }
TEST(CurrentLaneFollower, TargetRearAndStraddlingVehiclesRemainChecked)
{
  EXPECT_FALSE(skip(follower(-12,3.5)));
  EXPECT_FALSE(skip(follower(-12,1.5)));
}
TEST(CurrentLaneFollower, PhysicalRearGapAndAlongsideRemainChecked)
{
  EXPECT_FALSE(skip(follower(-5)));
  EXPECT_FALSE(skip(follower(0)));
  EXPECT_FALSE(skip(follower(12)));
}
TEST(CurrentLaneFollower, OncomingAndVulnerableRoadUsersRemainChecked)
{
  auto o = follower(); o.initial_pose.orientation.w = 0.; o.initial_pose.orientation.z = 1.;
  EXPECT_FALSE(skip(o));
  o = follower(); o.classification.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
  EXPECT_FALSE(skip(o));
}
TEST(CurrentLaneFollower, SourceFollowerPredictedToCutIntoTargetRemainsChecked)
{
  auto o = follower();
  path_safety_checker::PredictedPathWithPolygon prediction;
  prediction.path.emplace_back(2., o.initial_pose, 3., rectangle(-8,-4,2.5,4.5));
  o.predicted_paths.push_back(prediction);
  EXPECT_FALSE(skip(o));
}
