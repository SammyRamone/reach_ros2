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
#ifndef REACH_ROS_EVALUATION_SCANNABILITY_MOVEIT_H
#define REACH_ROS_EVALUATION_SCANNABILITY_MOVEIT_H

#include <reach/interfaces/evaluator.h>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
}  // namespace core
}  // namespace moveit

namespace planning_scene
{
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene

namespace reach_ros
{
namespace evaluation
{
class ScannabilityMoveIt : public reach::Evaluator
{
public:
  ScannabilityMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                       std::string sensor_frame_name, const double min_dist, const double max_dist,
                                       const double opt_dist, const double min_angle, const double max_angle, const double opt_angle, const double sensor_fov_x, const double sensor_fov_y);
  double calculateScore(const std::map<std::string, double>& pose, const Eigen::Isometry3d& target) const override;

private:
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* jmg_;
  std::string sensor_frame_name_;
  const double min_dist_;
  const double max_dist_;
  const double opt_dist_;
  const double min_angle_;
  const double max_angle_;
  const double opt_angle_;
  const double sensor_fov_x_;
  const double sensor_fov_y_;

  planning_scene::PlanningScenePtr scene_;
};

struct ScannabilityMoveItFactory : public reach::EvaluatorFactory
{
  reach::Evaluator::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace evaluation
}  // namespace reach_ros

#endif  // REACH_ROS_EVALUATION_SCANNABILITY_MOVEIT_H
