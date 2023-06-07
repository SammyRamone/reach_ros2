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
#include <yaml-cpp/yaml.h>

namespace reach_ros
{
namespace evaluation
{
ScannabilityMoveIt::ScannabilityMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                       std::string sensor_frame_name, const double min_dist, const double max_dist,
                                       const double opt_dist, const double min_angle, const double max_angle, const double opt_angle,
                                       const double sensor_fov_x, const double sensor_fov_y)
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
  std::vector<double> pose_subset = utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames());
  moveit::core::RobotState state(model_);
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();
  const Eigen::Isometry3d& sensor_frame = state.getGlobalLinkTransform(sensor_frame_name_);
  // vector from sensor to target
  const Eigen::Vector3d& sensor_to_target = target.translation() - sensor_frame.translation();
  const Eigen::Vector3d& target_to_sensor = sensor_frame.translation() - target.translation();

  // compute distance between scanner and target by taking the length of the vector that connects their positions
  double distance = sensor_to_target.norm();

  // get scanning angle by computing the angle between the vector poiting from target to sensor and the targets normal
  // vector
  const Eigen::Vector3d target_z_axis = target.rotation() * Eigen::Vector3d::UnitZ();
  double angle = utils::get_angle_between_vectors(sensor_to_target, target_z_axis);

  // get the angle in x and y direction to see if it fits in the sensor field of view
  const Eigen::Vector3d sensor_frame_yz_plane = sensor_frame.rotation() * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d sensor_frame_xz_plane = sensor_frame.rotation() * Eigen::Vector3d::UnitY();
  double sensor_angle_x = abs(utils::get_angle_between_vectors(sensor_to_target, sensor_frame_yz_plane) - M_PI / 2);
  double sensor_angle_y = abs(utils::get_angle_between_vectors(sensor_to_target, sensor_frame_xz_plane) - M_PI / 2);
  // previous computation does not tell us if we look towards or away from the point. check this by using angle to z
  // axis
  const Eigen::Vector3d sensor_frame_z_axis = sensor_frame.rotation() * Eigen::Vector3d::UnitZ();
  double sensor_angle = utils::get_angle_between_vectors(sensor_to_target, sensor_frame_z_axis);
  if (sensor_angle > M_PI / 2)
  {
    sensor_angle_x = M_PI - sensor_angle_x;
    sensor_angle_y = M_PI - sensor_angle_y;
  }

  // Check limits and increase cost function accordingly. We need to ensure that there is still a clear gradiant
  // in the scoring so that the IK knows in which direction it should search for a valid solution.
  // Still, we can mark invalid solutions with a cost > 1.
  double limit_cost= 0;
  if (distance < min_dist_ || distance > max_dist_ || angle < min_angle_ || angle > max_angle_ || sensor_angle_x > sensor_fov_x_ || sensor_angle_y > sensor_fov_y_)
    return 0;

  // normalize all partial scores to [0,1]. scaling factors are choosen manually for the different typical error values
  // points for being close to optimal distance
  double distance_score = exp(-10 * pow(distance - opt_dist_, 2));
  //std::cout << "Eval" << std::endl << "dsit " << distance_score << std::endl;
  // points for being close to optimal angle
  double angle_score = exp(-2 * pow(angle - opt_angle_, 2));
  //std::cout << "angl " << angle_score << std::endl;
  // points for being close to image center
  double image_center_score = exp(-2 * pow((sensor_angle_x + sensor_angle_y) / 2, 2));
  //std::cout << "im " << image_center_score << std::endl << std::endl;
  return (distance_score + angle_score + image_center_score) / 3 - limit_cost;
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
