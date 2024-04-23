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
#ifndef REACH_ROS_KINEMATICS_UTILS_H
#define REACH_ROS_KINEMATICS_UTILS_H

#include <Eigen/Dense>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <map>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/bodies.h>

namespace reach
{
class ReachRecord;
}

namespace reach_ros
{
namespace utils
{
moveit_msgs::msg::CollisionObject createCollisionObject(const std::string& mesh_filename,
                                                        const std::string& parent_link, const std::string& object_name);

visualization_msgs::msg::Marker makeVisual(const reach::ReachRecord& r, const std::string& frame, const double scale,
                                           const std::string& ns = "reach",
                                           const Eigen::Vector3f& color = { 0.5, 0.5, 0.5 });

visualization_msgs::msg::InteractiveMarker makeInteractiveMarker(const std::string& id, const reach::ReachRecord& r,
                                                                 const std::string& frame, const double scale,
                                                                 const Eigen::Vector3f& rgb_color = { 0.5, 0.5, 0.5 });

visualization_msgs::msg::Marker makeMarker(const std::vector<geometry_msgs::msg::Point>& pts, const std::string& frame,
                                           const double scale, const std::string& ns = "");

[[deprecated]] std::vector<double> transcribeInputMap(const std::map<std::string, double>& input,
                                                      const std::vector<std::string>& joint_names);

double get_angle_between_vectors(Eigen::Vector3d v1, Eigen::Vector3d v2);

double distanceBetweenFrames(const Eigen::Isometry3d& frame1, const Eigen::Isometry3d& frame2);

double angleToTargetNormal(const Eigen::Isometry3d& sensor_frame, const Eigen::Isometry3d& target_frame);

std::tuple<double, double> anglesToSensorNormal(const Eigen::Isometry3d& sensor_frame,
                                                const Eigen::Isometry3d& target_frame);

class LineOfSightChecker
{
public:
  LineOfSightChecker();
  LineOfSightChecker(moveit::core::RobotModelConstPtr model, collision_detection::WorldPtr world, bool publish_debug_markers_);
  const bool checkLineOfSight(const moveit::core::RobotState& solution_state, const Eigen::Isometry3d& sensor_frame,
                        const Eigen::Isometry3d& target_frame, std::string sensor_frame_name);

private:
  bool decideContact(const collision_detection::Contact& contact) const;
  void create_line_of_sight_cone(const Eigen::Isometry3d& tform_world_to_sensor,
                                 const Eigen::Isometry3d& tform_world_to_target,
                                 shapes::Mesh* m) const;
  void publishDebugMarker(shapes::Mesh *m, const Eigen::Isometry3d& sensor_frame, const Eigen::Isometry3d& target_frame, bool collision);

  std::shared_ptr<collision_detection::CollisionEnvFCL> collision_env_;
  collision_detection::AllowedCollisionMatrix acm_;
  collision_detection::CollisionRequest req_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cone_pub_;
  EigenSTL::vector_Vector3d points_;  // A set of points along the base of the circle
  int cone_sides_;
  double target_radius_;
  bool publish_debug_markers_;
};

/**
 * @brief Returns a singleton ROS2 node for accessing parameters and publishing data
 * @details ROS must be initialized (rclcpp::init, rclpy.init) before calling this method
 */
rclcpp::Node::SharedPtr getNodeInstance();

}  // namespace utils
}  // namespace reach_ros

#endif  // REACH_ROS_KINEMATICS_UTILS_H
