#include <iostream>
#include <pcl_ros/transforms.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

#include "utils.h"
#include "detection_visualizer.h"
#include "cloud_segmentator.h"

using boost::str;
using boost::format;

namespace bachelors_final_project
{
int getNBiggerIndex(size_t size, int n)
{ return size > n - 1 ? n - 1 : size - 1; }


double durationMillis(clock_t &begin)
{
  return (double(clock() - begin) / CLOCKS_PER_SEC) * 1000;
}


void callable(pcl::visualization::PCLVisualizer &viz)
{
  viz.addCoordinateSystem(1.0);
}

Point newPoint(Eigen::Vector3f v)
{
  return Point(v[0], v[1], v[2]);
}

pcl::PointXYZRGB newPointXYZRGB(Point p, uint8_t r, uint8_t g, uint8_t b)
{
  pcl::PointXYZRGB point(r, g, b);
  point.x = p.x;
  point.y = p.y;
  point.z = p.z;
  return point;
}


Eigen::Vector3f newVector3f(Point p)
{
  return Eigen::Vector3f(p.x, p.y, p.z);
}

void setProperties(const CloudPtr &coppied_cloud, CloudPtr &cloud_out, int width, int height)
{
  cloud_out->width = (uint32_t) width;
  cloud_out->height = (uint32_t) height;
  // Change the size of the pointcloud
  cloud_out->points.resize((unsigned long) (width * height));
  cloud_out->sensor_origin_ = coppied_cloud->sensor_origin_;
  cloud_out->sensor_orientation_ = coppied_cloud->sensor_orientation_;
  cloud_out->header = coppied_cloud->header;
}

Eigen::Vector3d castVector3d(Eigen::Vector3f v)
{
  Eigen::Vector3d v1(v[0],v[1],v[2]);
  return v1;
}

bool transformPoint(const std::string &init_frame, const std::string &final_frame, const Eigen::Vector3f &point_in,
                    Point &point_out, uint64_t micro_sec_time, tf::TransformListener &tf_listener)
{
  // Constructor requires seconds
  ros::Time tf_time(micro_sec_time / 1000000.0);
  try
  {
    if (not tf_listener.waitForTransform(final_frame, init_frame, tf_time, TF_DURATION))
      throw tf::TransformException(str(format("Timeout [%gs]") % TF_TIMEOUT));

    tf::StampedTransform stamped_tf;
    tf_listener.lookupTransform(final_frame, init_frame, tf_time, stamped_tf);
    tf::Vector3 tf_point_in(point_in[0],point_in[1],point_in[2]);
    tf::Vector3 point_final_frame = stamped_tf * tf_point_in;

    ROS_DEBUG("TF %s to %s [%g,%g,%g]", init_frame.c_str(), final_frame.c_str(),
              point_final_frame.x(), point_final_frame.y(), point_final_frame.z());

    point_out = Point((float) point_final_frame.x(), (float) point_final_frame.y(), (float) point_final_frame.z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Exception transforming point from frame %s to %s: %s", init_frame.c_str(), final_frame.c_str(),
              ex.what());
    return false;
  }
  return true;
}

bool transformPose(const std::string &init_frame, const std::string &final_frame,
                    geometry_msgs::PoseStamped &pose_in, geometry_msgs::PoseStamped &pose_out,
                    uint64_t micro_sec_time, tf::TransformListener &tf_listener)
{
  // Constructor requires seconds
  ros::Time tf_time(micro_sec_time / 1000000.0);

  pose_in.header.frame_id = init_frame;
  pose_out.header.frame_id = final_frame;
  try
  {
    if (not tf_listener.waitForTransform(final_frame, init_frame, tf_time, TF_DURATION))
      throw tf::TransformException(str(format("Timeout [%gs]") % TF_TIMEOUT));

    tf_listener.transformPose(final_frame,tf_time, pose_in,init_frame, pose_out);
    ROS_DEBUG("TF %s to %s", init_frame.c_str(), final_frame.c_str());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Exception transforming pose from '%s' to '%s': %s", init_frame.c_str(), final_frame.c_str(),
              ex.what());
    return false;
  }
  return true;
}


bool transformPointCloud(const std::string &init_frame, const std::string &final_frame, const CloudPtr &cloud_in,
                         const CloudPtr &cloud_out, const uint64_t micro_sec_time,
                         tf::TransformListener &tf_listener)
{
  // Constructor requires seconds
  ros::Time tf_time(micro_sec_time / 1000000.0);

  try
  {
    if (not tf_listener.waitForTransform(final_frame, init_frame, tf_time, TF_DURATION))
      throw tf::TransformException(str(format("Timeout [%gs]") % TF_TIMEOUT));

    pcl_ros::transformPointCloud(final_frame, tf_time, *cloud_in, init_frame, *cloud_out, tf_listener);

    ROS_DEBUG("Transform pointcloud [%lu] from %s to %s", cloud_out->size(), init_frame.c_str(), final_frame.c_str());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Exception transforming pointcloud from frame %s to %s: %s", init_frame.c_str(), final_frame.c_str(),
              ex.what());
    return false;
  }
  return true;
}

void projectOnPlane(const CloudPtr &cloud, const pcl::ModelCoefficientsPtr &table,
                    const pcl::PointIndices::Ptr &indices, CloudPtr &projected_cloud)
{
  pcl::ProjectInliers<Point> project_inliers = getProjector(cloud, table);
  project_inliers.setIndices(indices);
  project_inliers.filter(*projected_cloud);
}

void projectOnPlane(const CloudPtr &cloud, const pcl::ModelCoefficientsPtr &table, CloudPtr &projected_cloud)
{
  pcl::ProjectInliers<Point> project_inliers = getProjector(cloud, table);
  project_inliers.filter(*projected_cloud);
}

pcl::ProjectInliers<Point> getProjector(const CloudPtr &sensor_cloud, const pcl::ModelCoefficientsPtr &table)
{
  pcl::ProjectInliers<Point> project_inliers;
  project_inliers.setModelType(pcl::SACMODEL_PLANE);
  project_inliers.setInputCloud(sensor_cloud);
  project_inliers.setModelCoefficients(table);
  return project_inliers;
}

Point fourToPoint(Eigen::Vector4f &vector)
{
  return Point(vector[0], vector[1], vector[2]);
}

Point threeToPoint(Eigen::Vector3f &vector)
{
  return Point(vector[0], vector[1], vector[2]);
}

int selectChoice(std::string message)
{
  ROS_INFO("%s", message.c_str());
  int a;
  std::cin >> a;
  return a;
}

float selectChoiceFloat(std::string message)
{
  ROS_INFO("%s", message.c_str());
  float a;
  std::cin >> a;
  return a;
}

void pick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 0.32;
  p.pose.position.y = -0.7;
  p.pose.position.z = 0.5;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
  g.pre_grasp_approach.min_distance = 0.2;
  g.pre_grasp_approach.desired_distance = 0.4;

  g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.25;

  g.pre_grasp_posture.joint_names.resize(1, "r_gripper_joint");
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 1;

  g.grasp_posture.joint_names.resize(1, "r_gripper_joint");
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 0;

  grasps.push_back(g);
  group.setSupportSurfaceName("table");
  group.pick("part", grasps);
}

void place(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 0.7;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.5;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  g.pre_place_approach.direction.vector.z = -1.0;
  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.direction.header.frame_id = "base_footprint";
  g.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";
  g.pre_place_approach.min_distance = 0.1;
  g.pre_place_approach.desired_distance = 0.2;
  g.post_place_retreat.min_distance = 0.1;
  g.post_place_retreat.desired_distance = 0.25;

  g.post_place_posture.joint_names.resize(1, "r_gripper_joint");
  g.post_place_posture.points.resize(1);
  g.post_place_posture.points[0].positions.resize(1);
  g.post_place_posture.points[0].positions[0] = 1;

  loc.push_back(g);
  group.setSupportSurfaceName("table");


  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  //  group.setPathConstraints(constr);
  group.setPlannerId("RRTConnectkConfigDefault");

  group.place("part", loc);
}

void theThing()
{
//  ros::AsyncSpinner spinner(1);
//  spinner.start();

  ROS_INFO("EMPEZO");

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  ROS_INFO("PUBS");
  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroup group("right_arm");
  group.setPlanningTime(5.0);
  ROS_INFO("GROUP");

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint";

  // remove pole
  co.id = "pole";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);
  ROS_INFO("REMOVE POLE");

  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.4;
  co.primitive_poses[0].position.z = 0.85;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);

  ROS_INFO("POLE");

  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.2;
  co.primitive_poses[0].position.z = 0.175;
  pub_co.publish(co);
  ROS_INFO("TABLE");

  co.id = "part";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  moveit_msgs::AttachedCollisionObject aco;
  aco.object = co;
  pub_aco.publish(aco);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

  co.primitive_poses[0].position.x = 0.6;
  co.primitive_poses[0].position.y = -0.7;
  co.primitive_poses[0].position.z = 0.5;
  pub_co.publish(co);
  ROS_INFO("PART");
  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group);
  ROS_INFO("PICK");

  ros::WallDuration(1.0).sleep();

  //place(group);
  ROS_INFO("PLACE");
  selectChoice("press key");
}


} // namespace bachelors_final_project
