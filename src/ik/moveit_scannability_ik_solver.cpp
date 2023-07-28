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
                                                   std::string sensor_frame_name,
                                                   const double opt_dist, const double opt_angle)
  : MoveItIKSolver(model, planning_group, dist_threshold)
  , sensor_frame_name_(sensor_frame_name)
  , opt_dist_(opt_dist)
  , opt_angle_(opt_angle)
{
  valid_fn = std::bind(&MoveItScannabilitySolver::isIKSolutionValid, this, std::placeholders::_1, std::placeholders::_2,
                                std::placeholders::_3),
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

  kinematics::KinematicsQueryOptions options = kinematics::KinematicsQueryOptions();
  options.return_approximate_solution = true;

  if (state.setFromIK(jmg_,                    // joint model group
                      target,              // target pose which we need to transfer the point which we want to scan
                      sensor_frame_name_,  // tip frame
                      0.0,                                   // take timeout from config
                      valid_fn,
                      options,  // no further options
                      cost_fn))
  {
    std::vector<double> solution;
    state.copyJointGroupPositions(jmg_, solution);

    return { solution };
  }

  return {};
}

bool MoveItScannabilitySolver::isIKSolutionValid(moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
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
  Eigen::Isometry3d target_frame;
  tf2::fromMsg(pose_msg, target_frame);
  const Eigen::Isometry3d& sensor_frame = solution_state.getGlobalLinkTransform(sensor_frame_name_);

  // Normalize all partial scores to [0,1]. scaling factors are choosen manually for the different typical error values
  // Points for being close to optimal distance
  const double distance = utils::distanceBetweenFrames(sensor_frame, target_frame);
  const double distance_score = exp(-1 * pow(utils::distanceBetweenFrames(sensor_frame, target_frame) - opt_dist_, 2));

  // Points for being close to optimal angle
  const double angleToTargetNormal = utils::angleToTargetNormal(sensor_frame, target_frame);
  const double angle_score = exp(-1 * pow(angleToTargetNormal - opt_angle_, 2));

  // Points for being close to image center
  const std::tuple<double, double> anglesToSensorNormal = utils::anglesToSensorNormal(sensor_frame, target_frame);
  const double image_center_score = exp(-1 * pow((std::get<0>(anglesToSensorNormal) +std::get<1>(anglesToSensorNormal)) / 2, 2));

  // Since it is a cost function, we need to take the inverse of the score
  return 1 - (distance_score + angle_score + image_center_score) / 3;

  // If there is some model in between we give scores from 1-2 (or maybe 0.5 to 1 and change the stuff above)
  // additionally to the base 1 cost, we then just take the distance to the target as we will propably have nothing in between
  // but not only distance as we will then look to it from behind. maybe distance and angle
  // but we can not resuse distnace score as this is for optimal distnace not closest distance
  // this must include sitiuations where the sensor is IN a mesh not just a mesh between sensor and target
  // we can not optimize for 0 distance as this might confuse the check of beeing inside a mesh. so better use an epsiolon
}

reach::IKSolver::ConstPtr MoveItScannabilitySolverFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");
  auto sensor_frame_name = reach::get<std::string>(config, "sensor_frame_name");
  auto opt_dist = reach::get<double>(config, "opt_dist");
  auto opt_angle = reach::get<double>(config, "opt_angle");

  moveit::core::RobotModelConstPtr model =
      moveit::planning_interface::getSharedRobotModel(reach_ros::utils::getNodeInstance(), "robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  auto ik_solver = std::make_shared<MoveItScannabilitySolver>(model, planning_group, dist_threshold, sensor_frame_name,
                                                              opt_dist, opt_angle);

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
