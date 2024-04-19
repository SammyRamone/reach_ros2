/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach_ros/evaluation/scannability_moveit.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <reach/plugin_utils.h>
#include <reach/utils.h>
#include <yaml-cpp/yaml.h>

namespace reach_ros
{
namespace evaluation
{
ScannabilityMoveIt::ScannabilityMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                       std::string sensor_frame_name, const double min_dist, const double max_dist,
                                       const double opt_dist, const double min_angle, const double max_angle,
                                       const double opt_angle, const double sensor_fov_x, const double sensor_fov_y)
  : model_(model)
  , jmg_(model_->getJointModelGroup(planning_group))
  , sensor_frame_name_(sensor_frame_name)
  , min_dist_(min_dist)
  , max_dist_(max_dist)
  , opt_dist_(opt_dist)
  , min_angle_(min_angle)
  , max_angle_(max_angle)
  , opt_angle_(opt_angle)
  , sensor_fov_x_(sensor_fov_x)
  , sensor_fov_y_(sensor_fov_y)
{
  if (!jmg_)
    throw std::runtime_error("Failed to get joint model group");
}

double ScannabilityMoveIt::calculateScore(const std::map<std::string, double>& pose,
                                          const Eigen::Isometry3d& target) const
{
  // get Cartesian pose of the sensor (in relation to the base) for given joint positions
  std::vector<double> pose_subset = reach::extractSubset(pose, jmg_->getActiveJointModelNames());
  moveit::core::RobotState state(model_);
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();
  const Eigen::Isometry3d& sensor_frame = state.getGlobalLinkTransform(sensor_frame_name_);

  // Normalize all partial scores to [0,1]. scaling factors are choosen manually for the different typical error values
  // Points for being close to optimal distance
  const double distance = utils::distanceBetweenFrames(sensor_frame, target);
  const double distance_score = exp(-100 * pow(utils::distanceBetweenFrames(sensor_frame, target) - opt_dist_, 2));

  // Points for being close to optimal angle
  const double angleToTargetNormal = utils::angleToTargetNormal(sensor_frame, target);
  const double angle_score = exp(-100 * pow(angleToTargetNormal - opt_angle_, 2));

  // Points for being close to image center
  const std::tuple<double, double> anglesToSensorNormal = utils::anglesToSensorNormal(sensor_frame, target);
  const double image_center_score =
      exp(-100 * pow((std::get<0>(anglesToSensorNormal) + std::get<1>(anglesToSensorNormal)) / 2, 2));

  // Check limits
  if (distance < min_dist_ || distance > max_dist_ || angleToTargetNormal < min_angle_ ||
      angleToTargetNormal > max_angle_ || std::get<0>(anglesToSensorNormal) > sensor_fov_x_ ||
      std::get<1>(anglesToSensorNormal) > sensor_fov_y_)
    return 0;
  else
    return (distance_score + angle_score + image_center_score) / 3;
}

reach::Evaluator::ConstPtr ScannabilityMoveItFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto sensor_frame_name = reach::get<std::string>(config, "sensor_frame_name");
  auto min_dist = reach::get<double>(config, "min_dist");
  auto max_dist = reach::get<double>(config, "max_dist");
  auto opt_dist = reach::get<double>(config, "opt_dist");
  auto min_angle = reach::get<double>(config, "min_angle");
  auto max_angle = reach::get<double>(config, "max_angle");
  auto opt_angle = reach::get<double>(config, "opt_angle");
  auto sensor_fov_x = reach::get<double>(config, "sensor_fov_x");
  auto sensor_fov_y = reach::get<double>(config, "sensor_fov_y");

  moveit::core::RobotModelConstPtr model =
      moveit::planning_interface::getSharedRobotModel(reach_ros::utils::getNodeInstance(), "robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ScannabilityMoveIt>(model, planning_group, sensor_frame_name, min_dist, max_dist, opt_dist,
                                              min_angle, max_angle, opt_angle, sensor_fov_x, sensor_fov_y);
}

}  // namespace evaluation
}  // namespace reach_ros
