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
#include <reach_ros/utils.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <reach/types.h>

const static double ARROW_SCALE_RATIO = 6.0;
const static double NEIGHBOR_MARKER_SCALE_RATIO = ARROW_SCALE_RATIO / 2.0;

/**
 * @brief Singleton class for interacting with a ROS network
 */
class ROSInterface
{
public:
  ROSInterface();
  virtual ~ROSInterface();

  rclcpp::Node::SharedPtr node;

private:
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  std::shared_ptr<std::thread> executor_thread;
};

ROSInterface::ROSInterface()
{
  // Ensure ROS is initialized before creating the node/executor
  // Note: we cannot initialize ROS here ourselves with rclcpp::init(0, nullptr) because ROS2 parameters which we want
  // to access are passed in via argv. Since we don't have access to argv outside an executable and can't require
  // users/plugin loaders to pass it around, this function should throw an exception if ROS is not initialized
  if (!rclcpp::ok())
    throw std::runtime_error("ROS must be initialized before accessing the node");

  // Create a node that accepts arbitrary parameters later
  node = std::make_shared<rclcpp::Node>(
      "reach_study_node",
      rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  // Create an executor and add the node to it
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);

  // Create a thread
  executor_thread =
      std::make_shared<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &*executor));
}

ROSInterface::~ROSInterface()
{
  rclcpp::shutdown();
  executor_thread->join();
}

namespace reach_ros
{
namespace utils
{
rclcpp::Node::SharedPtr getNodeInstance()
{
  static std::unique_ptr<ROSInterface> ros;

  // Create an instance of the ROS interface if it doesn't exist yet
  if (!ros)
    ros = std::make_unique<ROSInterface>();

  return ros->node;
}

moveit_msgs::msg::CollisionObject createCollisionObject(const std::string& mesh_filename,
                                                        const std::string& parent_link, const std::string& object_name)
{
  // Create a CollisionObject message for the reach object
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = parent_link;
  obj.id = object_name;
  shapes::ShapeMsg shape_msg;
  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_filename);
  shapes::constructMsgFromShape(mesh, shape_msg);
  obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
  obj.operation = obj.ADD;

  // Assign a default pose to the mesh
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose.position.y = pose.position.z = 0.0;
  pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  obj.mesh_poses.push_back(pose);

  return obj;
}

visualization_msgs::msg::Marker makeVisual(const reach::ReachRecord& r, const std::string& frame, const double scale,
                                           const std::string& ns, const Eigen::Vector3f& color)
{
  static int idx = 0;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = getNodeInstance()->get_clock()->now();
  marker.ns = ns;
  marker.id = idx++;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Transform arrow such that arrow x-axis points along goal pose z-axis (Rviz convention)
  // convert msg parameter goal to Eigen matrix
  Eigen::AngleAxisd rot_flip_normal(M_PI, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_x_to_z(-M_PI / 2, Eigen::Vector3d::UnitY());

  // Transform
  Eigen::Isometry3d goal_eigen = r.goal * rot_flip_normal * rot_x_to_z;

  // Convert back to geometry_msgs pose
  geometry_msgs::msg::Pose msg = tf2::toMsg(goal_eigen);
  marker.pose = msg;

  marker.scale.x = scale;
  marker.scale.y = scale / ARROW_SCALE_RATIO;
  marker.scale.z = scale / ARROW_SCALE_RATIO;

  marker.color.a = 1.0;  // Don't forget to set the alpha!
  if (r.reached)
  {
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
  }
  else
  {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }

  return marker;
}

visualization_msgs::msg::InteractiveMarker makeInteractiveMarker(const std::string& id, const reach::ReachRecord& r,
                                                                 const std::string& frame, const double scale,
                                                                 const Eigen::Vector3f& rgb_color)
{
  visualization_msgs::msg::InteractiveMarker m;
  m.header.frame_id = frame;
  m.name = id;

  // Create a menu entry to display the score
  {
    visualization_msgs::msg::MenuEntry entry;
    entry.command_type = visualization_msgs::msg::MenuEntry::FEEDBACK;
    entry.id = 1;
    entry.parent_id = 0;

    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(4);
    ss << "Score: " << r.score;
    entry.title = ss.str();

    m.menu_entries.push_back(entry);
  }

  // Control
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  // Visuals
  auto visual = makeVisual(r, frame, scale, "reach", rgb_color);
  control.markers.push_back(visual);
  m.controls.push_back(control);

  return m;
}

visualization_msgs::msg::Marker makeMarker(const std::vector<geometry_msgs::msg::Point>& pts, const std::string& frame,
                                           const double scale, const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = getNodeInstance()->get_clock()->now();
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = marker.scale.y = marker.scale.z = scale / NEIGHBOR_MARKER_SCALE_RATIO;

  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 0;
  marker.color.g = 1.0;
  marker.color.b = 0;

  for (std::size_t i = 0; i < pts.size(); ++i)
  {
    marker.points.push_back(pts[i]);
  }

  return marker;
}

std::vector<double> transcribeInputMap(const std::map<std::string, double>& input,
                                       const std::vector<std::string>& joint_names)
{
  if (joint_names.size() > input.size())
    throw std::runtime_error("Seed pose size was not at least as large as the number of joints in the planning group");

  // Pull the joints of the planning group out of the input map
  std::vector<double> joints;
  joints.reserve(joint_names.size());
  for (const std::string& name : joint_names)
  {
    const auto it = input.find(name);
    if (it == input.end())
      throw std::runtime_error("Joint '" + name + "' in the planning group was not in the input map");

    joints.push_back(it->second);
  }

  return joints;
}

double get_angle_between_vectors(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
  return acos(v1.dot(v2) / sqrt(v1.squaredNorm() * v2.squaredNorm()));
}

double distanceBetweenFrames(const Eigen::Isometry3d& frame1, const Eigen::Isometry3d& frame2)
{
  return (frame1.translation() - frame2.translation()).norm();
}

double angleToTargetNormal(const Eigen::Isometry3d& sensor_frame, const Eigen::Isometry3d& target_frame)
{
  // Get the angle between the surface normal of the target and the vector between sensor and target
  // This can be used to check if a sensor can sense the target correctly
  const Eigen::Vector3d target_z_axis = target_frame.rotation() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d& sensor_to_target = target_frame.translation() - sensor_frame.translation();
  return get_angle_between_vectors(sensor_to_target, target_z_axis);
}

std::tuple<double, double> anglesToSensorNormal(const Eigen::Isometry3d& sensor_frame,
                                                const Eigen::Isometry3d& target_frame)
{
  // Get the angle in which the sensor sees the target
  // This can be used to see if a target is in the FOV or in the image center
  const Eigen::Vector3d& sensor_to_target = target_frame.translation() - sensor_frame.translation();
  // Get the angle in x and y direction to see if it fits in the sensor field of view
  const Eigen::Vector3d sensor_frame_yz_plane = sensor_frame.rotation() * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d sensor_frame_xz_plane = sensor_frame.rotation() * Eigen::Vector3d::UnitY();
  double sensor_angle_x = abs(get_angle_between_vectors(sensor_to_target, sensor_frame_yz_plane) - M_PI / 2);
  double sensor_angle_y = abs(get_angle_between_vectors(sensor_to_target, sensor_frame_xz_plane) - M_PI / 2);
  // Previous computation does not tell us if we look towards or away from the point
  // Check this by using angle to z axis
  const Eigen::Vector3d sensor_frame_z_axis = sensor_frame.rotation() * Eigen::Vector3d::UnitZ();
  const double sensor_angle = get_angle_between_vectors(sensor_to_target, sensor_frame_z_axis);
  if (sensor_angle > M_PI / 2)
  {
    sensor_angle_x = M_PI - sensor_angle_x;
    sensor_angle_y = M_PI - sensor_angle_y;
  }
  return { sensor_angle_x, sensor_angle_y };
}

LineOfSightChecker::LineOfSightChecker()
{
  // TODO make this and somehow handle the initialization differently. Maybe with a configure method like the constrain
  // did.
}

LineOfSightChecker::LineOfSightChecker(moveit::core::RobotModelConstPtr model, collision_detection::WorldPtr world)
{
  // create a special collision environment for line of sight checks
  //TODO collision_env_ = std::make_shared<collision_detection::CollisionEnvFCL>(model, world);
  collision_env_ = std::make_shared<collision_detection::CollisionEnvFCL>(model);
  // create matrix to check for collisions between the cone and other objects
  collision_detection::DecideContactFn fn = [this](collision_detection::Contact& contact) {
    return decideContact(contact);
  };
  acm_.setDefaultEntry(std::string("los_cone"), fn);
  // Prepare request for later LOS checks
  req_.contacts = true;
  req_.max_contacts = 1;
  req_.distance = false;
  // TODO would adding a group name to the request speed up computation?

  // TODO could be a config parameter
  cone_sides_ = 4;
  // TODO should be a config parameter
  target_radius_ = 0.01;
  // compute the points on the base circle of the cone that make up the cone sides
  points_.clear();
  double delta = 2.0 * M_PI / static_cast<double>(cone_sides_);
  double a = 0.0;
  for (unsigned int i = 0; i < cone_sides_; ++i, a += delta)
  {
    double x = sin(a) * target_radius_;
    double y = cos(a) * target_radius_;
    points_.push_back(Eigen::Vector3d(x, y, 0.0));
  }
      cone_pub_ =
      reach_ros::utils::getNodeInstance()->create_publisher<visualization_msgs::msg::Marker>("cone_marker", 1);
}

bool LineOfSightChecker::checkLineOfSight(const moveit::core::RobotState& solution_state,
                                          const Eigen::Isometry3d& sensor_frame, const Eigen::Isometry3d& target_frame
                                          )
{
  // Create a cone from sensor to target
  shapes::Mesh* m = create_line_of_sight_cone(sensor_frame, target_frame);
  if (!m)
    throw std::runtime_error("Could not create the visibility mesh.");

  if (true)
  {
    // debug showing of line of sight cone
    visualization_msgs::msg::Marker mk;
    shapes::constructMarkerFromShape(m, mk);
    mk.header.frame_id = "base_link";
    mk.header.stamp = rclcpp::Clock().now();
    mk.id = 1;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
    mk.lifetime = rclcpp::Duration::from_seconds(60);
    // this scale necessary to make results look reasonable
    mk.scale.x = .01;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    cone_pub_->publish(mk);


  visualization_msgs::msg::Marker mka;
  mka.type = visualization_msgs::msg::Marker::ARROW;
  mka.action = visualization_msgs::msg::Marker::ADD;
  mka.color = mk.color;
  mka.pose = mk.pose;

  mka.header = mk.header;
  mka.ns = mk.ns;
  mka.id = 2;
  mka.lifetime = mk.lifetime;
  mka.scale.x = 0.05;
  mka.scale.y = .15;
  mka.scale.z = 0.0;
  mka.points.resize(2);
  Eigen::Vector3d d = target_frame.translation() + target_frame.linear().col(2) * -0.5;
  mka.points[0].x = target_frame.translation().x();
  mka.points[0].y = target_frame.translation().y();
  mka.points[0].z = target_frame.translation().z();
  mka.points[1].x = d.x();
  mka.points[1].y = d.y();
  mka.points[1].z = d.z();
  cone_pub_->publish(mka);

  mka.id = 3;
  mka.color.b = 1.0;
  mka.color.r = 0.0;

  d = sensor_frame.translation() + sensor_frame.linear().col(2) * 0.5;
  mka.points[0].x = sensor_frame.translation().x();
  mka.points[0].y = sensor_frame.translation().y();
  mka.points[0].z = sensor_frame.translation().z();
  mka.points[1].x = d.x();
  mka.points[1].y = d.y();
  mka.points[1].z = d.z();

  cone_pub_->publish(mka);
  }

  // Add the visibility cone to the world
  collision_env_->getWorld()->addToObject("los_cone", shapes::ShapeConstPtr(m), Eigen::Isometry3d::Identity());

  collision_detection::CollisionResult res;
  collision_env_->checkRobotCollision(req_, res, solution_state, acm_); //todo this does not check between world objects

  // remove cone again as it will be at a different pose next time
  collision_env_->getWorld()->removeObject("los_cone");
  std::cout << res.collision << std::endl;
  return !res.collision;
}

bool LineOfSightChecker::decideContact(const collision_detection::Contact& contact) const
{
  // Decide which contacts are allowed and which are not
  // We only want to have information if the cone intersects with something else
  // The cone is created to not intersect with sensor and target by making it a bit smaller
  if (moveit::core::Transforms::sameFrame(contact.body_name_1, "los_cone") ||
      moveit::core::Transforms::sameFrame(contact.body_name_2, "los_cone"))
  {
    return true;
  }
  return false;
}

shapes::Mesh* LineOfSightChecker::create_line_of_sight_cone(const Eigen::Isometry3d& tform_world_to_sensor,
                                                            const Eigen::Isometry3d& tform_world_to_target) const
{
  // This method is based on MoveIt VisibilityConstraint::getVisibilityCone()
  // We create the cone a bit smaller to not intersect with sensor and target shapes

  const Eigen::Vector3d sensor_to_target_normalized =
      (tform_world_to_sensor.translation() - tform_world_to_target.translation()).normalized();
  const Eigen::Vector3d sensor_offset = sensor_to_target_normalized * 0.001;
  Eigen::Isometry3d sp = Eigen::Isometry3d::Identity();
  sp.translation() = tform_world_to_sensor.translation() + sensor_offset;
  sp.linear() = tform_world_to_sensor.rotation();

  // the current pose of the target
  const Eigen::Vector3d target_offset = -sensor_to_target_normalized * 0.001;
  Eigen::Isometry3d tp = Eigen::Isometry3d::Identity();
  tp.translation() = tform_world_to_target.translation() + target_offset;
  tp.linear() = tform_world_to_target.rotation();

  // transform the points on the disc to the desired target frame
  const EigenSTL::vector_Vector3d* points = &points_;
  std::unique_ptr<EigenSTL::vector_Vector3d> temp_points;

  temp_points = std::make_unique<EigenSTL::vector_Vector3d>(points_.size());
  for (std::size_t i = 0; i < points_.size(); ++i)
  {
    temp_points->at(i) = tp * points_[i];
  }
  points = temp_points.get();

  // allocate memory for a mesh to represent the visibility cone
  shapes::Mesh* m = new shapes::Mesh();
  m->vertex_count = cone_sides_ + 2;
  m->vertices = new double[m->vertex_count * 3];
  m->triangle_count = cone_sides_ * 2;
  m->triangles = new unsigned int[m->triangle_count * 3];
  // we do NOT allocate normals because we do not compute them

  // the sensor origin
  m->vertices[0] = sp.translation().x();
  m->vertices[1] = sp.translation().y();
  m->vertices[2] = sp.translation().z();

  // the center of the base of the cone approximation
  m->vertices[3] = tp.translation().x();
  m->vertices[4] = tp.translation().y();
  m->vertices[5] = tp.translation().z();

  // the points that approximate the base disc
  for (std::size_t i = 0; i < points->size(); ++i)
  {
    m->vertices[i * 3 + 6] = points->at(i).x();
    m->vertices[i * 3 + 7] = points->at(i).y();
    m->vertices[i * 3 + 8] = points->at(i).z();
  }

  // add the triangles
  std::size_t p3 = points->size() * 3;
  for (std::size_t i = 1; i < points->size(); ++i)
  {
    // triangle forming a side of the cone, using the sensor origin
    std::size_t i3 = (i - 1) * 3;
    m->triangles[i3] = i + 1;
    m->triangles[i3 + 1] = 0;
    m->triangles[i3 + 2] = i + 2;
    // triangle forming a part of the base of the cone, using the center of the base
    std::size_t i6 = p3 + i3;
    m->triangles[i6] = i + 1;
    m->triangles[i6 + 1] = 1;
    m->triangles[i6 + 2] = i + 2;
  }

  // last triangles
  m->triangles[p3 - 3] = points->size() + 1;
  m->triangles[p3 - 2] = 0;
  m->triangles[p3 - 1] = 2;
  p3 *= 2;
  m->triangles[p3 - 3] = points->size() + 1;
  m->triangles[p3 - 2] = 1;
  m->triangles[p3 - 1] = 2;

  return m;
}

}  // namespace utils
}  // namespace reach_ros
