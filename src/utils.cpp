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

namespace reach_ros
{
namespace utils
{
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
  return acos(v1.dot(v2)/sqrt(v1.squaredNorm() * v2.squaredNorm()));
}

double distanceBetweenFrames(const Eigen::Isometry3d& frame1, const Eigen::Isometry3d& frame2){
  return (frame1.translation() - frame2.translation()).norm();
}

double angleToTargetNormal(const Eigen::Isometry3d& sensor_frame, const Eigen::Isometry3d& target_frame){
  // Get the angle between the surface normal of the target and the vector between sensor and target
  // This can be used to check if a sensor can sense the target correctly
  const Eigen::Vector3d target_z_axis = target_frame.rotation() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d& sensor_to_target = target_frame.translation() - sensor_frame.translation();
  return get_angle_between_vectors(sensor_to_target, target_z_axis);
}

std::tuple<double, double> anglesToSensorNormal(const Eigen::Isometry3d& sensor_frame, const Eigen::Isometry3d& target_frame){
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
  return {sensor_angle_x, sensor_angle_y};
}


}  // namespace utils
}  // namespace reach_ros
