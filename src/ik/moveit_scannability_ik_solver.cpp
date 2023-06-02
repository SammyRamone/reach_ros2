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


MoveItScannabilitySolver::MoveItScannabilitySolver(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                               double dist_threshold) : MoveItIKSolver(model, planning_group,
                               dist_threshold)
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

  if (state.setFromIK(jmg_, // joint model group
                      EigenSTL::vector_Isometry3d(), // empty list of targest, as we will only use the cost function
                      std::vector<std::string>(), // empty tips
                      std::vector<std::vector<double>>(), // empty consistency limits
                      0.0, // take timeout from config
                      std::bind(&MoveItIKSolver::isIKSolutionValid, this, std::placeholders::_1, std::placeholders::_2,
                                std::placeholders::_3)), // use inherited validity function
                      kinematics::KinematicsQueryOptions(), // no further options
                      cost_fn) // out of some reason we can not directly put the bind here but need to use a variable instead
  {
    std::vector<double> solution;
    state.copyJointGroupPositions(jmg_, solution);

    return { solution };
  }

  return {};
}

double MoveItScannabilitySolver::costFunction(const geometry_msgs::msg::Pose& pose, const moveit::core::RobotState& solution_state,
                    const moveit::core::JointModelGroup* jmg, const std::vector<double>& seed_state) {
  const auto compute_l2_norm = [](std::vector<double> solution, std::vector<double> start) {
  double sum = 0.0;
  for (size_t ji = 0; ji < solution.size(); ji++)
  {
    double d = solution[ji] - start[ji];
    sum += d * d;
  }
  return sum;
  };
  std::vector<double> proposed_joint_positions;
  solution_state.copyJointGroupPositions(jmg, proposed_joint_positions);
  double cost = compute_l2_norm(proposed_joint_positions, seed_state);
  const double weight = 0.0000005;
  return weight * cost;
}

reach::IKSolver::ConstPtr MoveItScannabilitySolverFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");

  moveit::core::RobotModelConstPtr model =
      moveit::planning_interface::getSharedRobotModel(reach_ros::utils::getNodeInstance(), "robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  auto ik_solver = std::make_shared<MoveItScannabilitySolver>(model, planning_group, dist_threshold);

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
