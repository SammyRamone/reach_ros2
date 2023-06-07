#include <reach_ros/ik/moveit_scannability_ik_solver.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <reach/plugin_utils.h>
#include <yaml-cpp/yaml.h>

namespace reach_ros
{
namespace ik
{
using namespace std::placeholders;

MoveItScannabilitySolver::MoveItScannabilitySolver(moveit::core::RobotModelConstPtr model,
                                                   const std::string& planning_group, double dist_threshold,
                                                   std::string sensor_frame_name, const double min_dist,
                                                   const double max_dist, const double opt_dist,
                                                   const double min_angle, const double max_angle, const double opt_angle,
                                                   const double sensor_fov_x, const double sensor_fov_y)
  : MoveItIKSolver(model, planning_group, dist_threshold)
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
  cost_fn = std::bind(&MoveItScannabilitySolver::costFunction, this, std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4);
}

std::vector<std::vector<double>> MoveItScannabilitySolver::solveIK(const Eigen::Isometry3d& target,
                                                                   const std::map<std::string, double>& seed) const
{
  moveit::core::RobotState state(model_);

  const std::vector<std::string>& joint_names = jmg_->getActiveJointModelNames();

  std::vector<double> seed_subset = utils::transcribeInputMap(seed, joint_names);
  state.setJointGroupPositions(jmg_, seed_subset);
  state.update();

  // std::cout << "new IK call" << std::endl;
  if (state.setFromIK(jmg_,                    // joint model group
                      { target },              // target pose which we need to transfer the point which we want to scan
                      { sensor_frame_name_ },  // tip frame
                      std::vector<std::vector<double>>(),    // empty consistency limits
                      0.0,                                   // take timeout from config
                      std::bind(&MoveItIKSolver::isIKSolutionValid, this, std::placeholders::_1, std::placeholders::_2,
                                std::placeholders::_3),      // use inherited validity function
                      kinematics::KinematicsQueryOptions(),  // no further options
                      cost_fn))
  {
    std::vector<double> solution;
    state.copyJointGroupPositions(jmg_, solution);

    return { solution };
  }

  return {};
}

bool MoveItIKSolver::isIKSolutionValid(moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
                                       const double* ik_solution) const
{
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();
  const bool joints_in_limits = state->satisfiesBounds();
  const bool colliding = scene_->isStateColliding(*state, jmg->getName(), false);
  const bool too_close =
      (scene_->distanceToCollision(*state, scene_->getAllowedCollisionMatrix()) < distance_threshold_);


  return (joints_in_limits && !colliding && !too_close);
}

double MoveItScannabilitySolver::costFunction(const geometry_msgs::msg::Pose& pose_msg,
                                              const moveit::core::RobotState& solution_state,
                                              const moveit::core::JointModelGroup* jmg,
                                              const std::vector<double>& seed_state)
{
  Eigen::Isometry3d target;
  tf2::fromMsg(pose_msg, target);
  const Eigen::Isometry3d& sensor_frame = solution_state.getGlobalLinkTransform(sensor_frame_name_);
  const Eigen::Vector3d& sensor_to_target = target.translation() - sensor_frame.translation();

  // Compute distance between sensor and target by taking the length of the vector that connects their positions
  double distance = sensor_to_target.norm();

  // get scanning angle by computing the angle between the vector poiting from sensor to target and the targets normal
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
  const double sensor_angle = utils::get_angle_between_vectors(sensor_to_target, sensor_frame_z_axis);
  if (sensor_angle > M_PI / 2)
  {
    sensor_angle_x = M_PI - sensor_angle_x;
    sensor_angle_y = M_PI - sensor_angle_y;
  }

  // normalize all partial scores to [0,1]. scaling factors are choosen manually for the different typical error values
  // points for being close to optimal distance
  const double distance_score = exp(-10 * pow(distance - opt_dist_, 2));
  //  points for being close to optimal angle
  const double angle_score = exp(-2 * pow(angle - opt_angle_, 2));
  //  points for being close to image center
  const double image_center_score = exp(-2 * pow((sensor_angle_x + sensor_angle_y) / 2, 2));
  //  since it is a cost function, we need to take the inverse
  return 1 - (distance_score + angle_score + image_center_score) / 3 + limit_cost;
}

reach::IKSolver::ConstPtr MoveItScannabilitySolverFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");
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

  auto ik_solver = std::make_shared<MoveItScannabilitySolver>(model, planning_group, dist_threshold, sensor_frame_name,
                                                              min_dist, max_dist, opt_dist, min_angle, max_angle, opt_angle,
                                                              sensor_fov_x, sensor_fov_y);

  // Optionally add a collision mesh
  const std::string collision_mesh_filename_key = "collision_mesh_filename";
  const std::string collision_mesh_frame_key = "collision_mesh_frame";
  if (config[collision_mesh_filename_key])
  {
    auto collision_mesh_filename = reach::get<std::string>(config, collision_mesh_filename_key);
    std::string collision_mesh_frame = config[collision_mesh_frame_key] ?
                                           reach::get<std::string>(config, collision_mesh_frame_key) :
                                           ik_solver->getKinematicBaseFrame();

    ik_solver->addCollisionMesh(collision_mesh_filename, collision_mesh_frame);
  }

  // Optionally add touch links
  const std::string touch_links_key = "touch_links";
  if (config[touch_links_key])
  {
    auto touch_links = reach::get<std::vector<std::string>>(config, touch_links_key);
    ik_solver->setTouchLinks(touch_links);
  }

  return ik_solver;
}
}  // namespace ik
}  // namespace reach_ros
