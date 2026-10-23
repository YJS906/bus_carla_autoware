// Copyright 2024 TIER IV, Inc.
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

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

using autoware::test_utils::createPose;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;

constexpr auto epsilon = 1e-6;

TEST(FootprintTest, translate_polygon)
{
  using autoware::behavior_path_planner::drivable_area_expansion::translate_polygon;

  Polygon2d polygon;
  polygon.outer() = {
    Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{1.0, 1.0}, Point2d{0.0, 1.0}, Point2d{0.0, 0.0}};

  Polygon2d translated_polygon = translate_polygon(polygon, 1.0, 2.0);

  ASSERT_EQ(translated_polygon.outer().size(), 5);
  EXPECT_NEAR(translated_polygon.outer().at(0).x(), 1.0, epsilon);
  EXPECT_NEAR(translated_polygon.outer().at(0).y(), 2.0, epsilon);
  EXPECT_NEAR(translated_polygon.outer().at(1).x(), 2.0, epsilon);
  EXPECT_NEAR(translated_polygon.outer().at(1).y(), 2.0, epsilon);
  EXPECT_NEAR(translated_polygon.outer().at(2).x(), 2.0, epsilon);
  EXPECT_NEAR(translated_polygon.outer().at(2).y(), 3.0, epsilon);
  EXPECT_NEAR(translated_polygon.outer().at(3).x(), 1.0, epsilon);
  EXPECT_NEAR(translated_polygon.outer().at(3).y(), 3.0, epsilon);
  EXPECT_NEAR(translated_polygon.outer().at(4).x(), 1.0, epsilon);
  EXPECT_NEAR(translated_polygon.outer().at(4).y(), 2.0, epsilon);
}

TEST(FootprintTest, create_footprint)
{
  using autoware::behavior_path_planner::drivable_area_expansion::create_footprint;

  Polygon2d base_footprint;
  base_footprint.outer() = {
    Point2d{1.0, 1.0}, Point2d{2.0, 1.0}, Point2d{2.0, 2.0}, Point2d{1.0, 2.0}, Point2d{1.0, 1.0}};

  // Condition: without rotation
  auto pose = createPose(1.0, 2.0, 0.0, 0.0, 0.0, 0.0);
  auto footprint = create_footprint(pose, base_footprint);
  ASSERT_EQ(footprint.outer().size(), 5);
  EXPECT_NEAR(footprint.outer().at(0).x(), 2.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(0).y(), 3.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(1).x(), 3.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(1).y(), 3.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(2).x(), 3.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(2).y(), 4.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(3).x(), 2.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(3).y(), 4.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(4).x(), 2.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(4).y(), 3.0, epsilon);

  // Condition: with rotation
  pose = createPose(1.0, 2.0, 0.0, 0.0, 0.0, M_PI_2);
  footprint = create_footprint(pose, base_footprint);
  ASSERT_EQ(footprint.outer().size(), 5);
  EXPECT_NEAR(footprint.outer().at(0).x(), 0.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(0).y(), 3.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(1).x(), 0.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(1).y(), 4.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(2).x(), -1.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(2).y(), 4.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(3).x(), -1.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(3).y(), 3.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(4).x(), 0.0, epsilon);
  EXPECT_NEAR(footprint.outer().at(4).y(), 3.0, epsilon);
}

TEST(FootprintTest, create_object_footprints)
{
  using autoware::behavior_path_planner::drivable_area_expansion::create_object_footprints;

  autoware_perception_msgs::msg::PredictedObjects objects;
  autoware_perception_msgs::msg::PredictedObject object;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;

  // Add a predicted path
  autoware_perception_msgs::msg::PredictedPath path;
  auto pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  path.path.push_back(pose);
  object.kinematics.predicted_paths.push_back(path);

  objects.objects.push_back(object);

  autoware::behavior_path_planner::drivable_area_expansion::DrivableAreaExpansionParameters params;
  params.object_exclusion.exclude_static = false;
  params.object_exclusion.exclude_dynamic = false;
  params.object_exclusion.front_offset = 0.5;
  params.object_exclusion.rear_offset = 0.5;
  params.object_exclusion.left_offset = 0.5;
  params.object_exclusion.right_offset = 0.5;

  // Condition: doesn't avoid dynamic objects
  auto footprints = create_object_footprints(objects, params);
  EXPECT_TRUE(footprints.empty());

  // Condition: single object and single point path
  params.object_exclusion.exclude_dynamic = true;
  footprints = create_object_footprints(objects, params);

  ASSERT_EQ(footprints.size(), 1);
  ASSERT_EQ(footprints.front().outer().size(), 5);

  EXPECT_NEAR(footprints.front().outer().at(0).x(), 2.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(0).y(), 1.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(1).x(), 2.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(1).y(), -1.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(2).x(), -2.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(2).y(), -1.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(3).x(), -2.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(3).y(), 1.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(4).x(), 2.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(4).y(), 1.5, epsilon);
}

TEST(FootprintTest, FollowingPredictionDoesNotEraseEgoCorridor)
{
  using namespace autoware::behavior_path_planner::drivable_area_expansion;
  DrivableAreaExpansionParameters params;
  params.object_exclusion.exclude_dynamic = true;
  params.object_exclusion.front_offset = 0.75;
  params.object_exclusion.rear_offset = 0.75;
  params.object_exclusion.left_offset = 0.75;
  params.object_exclusion.right_offset = 0.75;
  params.vehicle_info.vehicle_width_m = 4.04;
  params.vehicle_info.rear_overhang_m = 3.0;
  const auto ego = createPose(0., 0., 0., 0., 0., 0.);
  const std::vector<geometry_msgs::msg::Pose> lane{
    createPose(-30., 0., 0., 0., 0., 0.), createPose(30., 0., 0., 0., 0., 0.)};
  autoware_perception_msgs::msg::PredictedObjects objects;
  autoware_perception_msgs::msg::PredictedObject object;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = classification.CAR;
  classification.probability = 1.;
  object.classification = {classification};
  object.shape.dimensions.x = 4.;
  object.shape.dimensions.y = 2.;
  object.kinematics.initial_pose_with_covariance.pose = createPose(-10., 0., 0., 0., 0., 0.);
  autoware_perception_msgs::msg::PredictedPath prediction;
  prediction.path = {ego, createPose(10., 0., 0., 0., 0., 0.)};
  prediction.confidence = 1.;
  object.kinematics.predicted_paths = {prediction};
  objects.objects = {object};
  auto footprints = create_object_footprints(objects, params, ego, lane);
  EXPECT_TRUE(footprints.empty());
  EXPECT_DOUBLE_EQ(objects.objects.front().shape.dimensions.x, 4.);
  EXPECT_DOUBLE_EQ(
    objects.objects.front().kinematics.initial_pose_with_covariance.pose.position.x, -10.);
  // The unmodified prediction remains available to collision and lane-change checks.
  EXPECT_EQ(objects.objects.front().kinematics.predicted_paths.front().path.size(), 2U);

  for (const auto & pose : {
      createPose(-4., 3.5, 0., 0., 0., 0.),  // adjacent rear overlaps longitudinal safety gap
      createPose(10., 3.5, 0., 0., 0., 0.),  // adjacent front vehicle
      createPose(-10., 0., 0., 0., 0., 3.141592653589793),  // opposite heading
      createPose(-4., 0., 0., 0., 0., 0.),  // safety envelope overlaps ego rear
      createPose(10., 0., 0., 0., 0., 0.)}) {  // lead vehicle
    objects.objects.front().kinematics.initial_pose_with_covariance.pose = pose;
    EXPECT_EQ(create_object_footprints(objects, params, ego, lane).size(), 3U);
  }
  // A parallel adjacent follower must not rewrite the historical corridor either.
  objects.objects.front().kinematics.initial_pose_with_covariance.pose =
    createPose(-10., 3.5, 0., 0., 0., 0.);
  EXPECT_TRUE(create_object_footprints(objects, params, ego, lane).empty());
  // A turn by ego does not reclassify a straight-following vehicle as crossing.
  objects.objects = {object};
  const std::vector<geometry_msgs::msg::Pose> turning_lane{
    lane.front(), ego, createPose(0., 20., 0., 0., 0., 1.5707963267948966)};
  EXPECT_EQ(create_object_footprints(objects, params,
    createPose(0., 5., 0., 0., 0., 1.5707963267948966), turning_lane).size(), 0U);
  objects.objects.front().classification.front().label = classification.PEDESTRIAN;
  EXPECT_EQ(create_object_footprints(objects, params, ego, lane).size(), 3U);
  // Missing predictions never remove the current physical object.
  objects.objects.front().kinematics.predicted_paths.clear();
  EXPECT_EQ(create_object_footprints(objects, params, ego, lane).size(), 1U);
}

TEST(FootprintTest, RoadGeometryKeepsCurrentAndNearTermCrossingPredictions)
{
  using namespace autoware::behavior_path_planner::drivable_area_expansion;
  DrivableAreaExpansionParameters params;
  params.object_exclusion.exclude_dynamic = true;
  params.vehicle_info.vehicle_width_m = 4.04;
  params.vehicle_info.rear_overhang_m = 3.0;
  const auto ego = createPose(0., 0., 0., 0., 0., 0.);
  const std::vector<geometry_msgs::msg::Pose> lane{
    createPose(-30., 0., 0., 0., 0., 0.), createPose(30., 0., 0., 0., 0., 0.)};
  autoware_perception_msgs::msg::PredictedObjects objects;
  autoware_perception_msgs::msg::PredictedObject object;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = classification.CAR;
  classification.probability = 1.;
  object.classification = {classification};
  object.shape.dimensions.x = 4.; object.shape.dimensions.y = 2.;
  object.kinematics.initial_pose_with_covariance.pose = createPose(10., 0., 0., 0., 0., 1.57);
  autoware_perception_msgs::msg::PredictedPath prediction;
  prediction.time_step.sec = 1;
  for (int i = 0; i < 7; ++i) prediction.path.push_back(createPose(10., i, 0., 0., 0., 1.57));
  object.kinematics.predicted_paths = {prediction}; objects.objects = {object};
  EXPECT_EQ(create_object_footprints(objects, params, ego, lane).size(), 5U);
  EXPECT_EQ(objects.objects.front().kinematics.predicted_paths.front().path.size(), 7U);
  objects.objects.front().classification.front().label = classification.PEDESTRIAN;
  EXPECT_EQ(create_object_footprints(objects, params, ego, lane).size(), 8U);
  objects.objects.front().classification.front().label = classification.CAR;
  objects.objects.front().kinematics.predicted_paths.front().time_step.sec = 0;
  EXPECT_EQ(create_object_footprints(objects, params, ego, lane).size(), 8U);
}
