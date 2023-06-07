#ifndef REACH_ROS_IK_BIO_IK_SOLVER_H
#define REACH_ROS_IK_BIO_IK_SOLVER_H

#include <reach_ros/ik/moveit_ik_solver.h>

#include <rclcpp/publisher.hpp>
#include <vector>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <functional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematics_base/kinematics_base.h>

namespace reach_ros
{
namespace ik
{

class MoveItScannabilitySolver : public reach_ros::ik::MoveItIKSolver
{
public:
  MoveItScannabilitySolver(moveit::core::RobotModelConstPtr model, const std::string& planning_group, double dist_threshold, std::string sensor_frame_name, const double min_dist, const double max_dist,
                                       const double opt_dist, const double min_angle_, const double max_angle_, const double opt_angle,
                                       const double sensor_fov_x, const double sensor_fov_y);

  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d& target,
                                           const std::map<std::string, double>& seed) const override;

private:
  double costFunction(const geometry_msgs::msg::Pose& pose_msg, const moveit::core::RobotState& solution_state,
                    const moveit::core::JointModelGroup* jmg, const std::vector<double>& seed_state);
  kinematics::KinematicsBase::IKCostFn cost_fn;

    std::string sensor_frame_name_;
  const double min_dist_;
  const double max_dist_;
  const double opt_dist_;
  const double min_angle_;
  const double max_angle_;
  const double opt_angle_;
  const double sensor_fov_x_;
  const double sensor_fov_y_;
};

struct MoveItScannabilitySolverFactory : public reach::IKSolverFactory
{
  reach::IKSolver::ConstPtr create(const YAML::Node& config) const override;
};
}  // namespace ik
}  // namespace reach_ros

#endif  // REACH_ROS_IK_BIO_IK_SOLVER_H
