//
// Created by ian on 10/6/15.
//

#include "grasp_filter.h"

#include <ros/ros.h>

#include <pcl/common/angles.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include "utils.h"
#include "bounding_box.h"
#include "grasp_filter_utils.h"

using std::string;
using namespace moveit_msgs;
using shape_msgs::SolidPrimitive;
using shape_msgs::Plane;

namespace bachelors_final_project
{

const std::string detection::GraspFilter::GRASPABLE_OBJECT = "graspable object";
const std::string detection::GraspFilter::SUPPORT_TABLE = "table";
const std::string detection::GraspFilter::WRIST_LINK = "r_wrist_roll_link";
const std::string detection::GraspFilter::GRIPPER_JOINT = "r_gripper_joint";
const double detection::GraspFilter::STANDOFF = 0.03;
const float detection::GraspFilter::PREGRASP_DISTANCE = 0.1;

detection::GraspFilter::GraspFilter(ros::NodeHandle &nh, tf::TransformListener &tf_listener) :
    group_("right_arm"),
    display_pub(nh.advertise<DisplayTrajectory>("/move_group/display_planned_path", 1, true)),
    collision_obj_pub(nh.advertise<CollisionObject>("/collision_object", 10)),
    attached_obj_pub(nh.advertise<AttachedCollisionObject>("/attached_collision_object", 10)),
    grasps_marker_(nh.advertise<visualization_msgs::MarkerArray>("grasps_marker", 10)),
    planning_scene_diff_pub(nh.advertise<PlanningScene>("/planning_scene", 1)),
    client_get_scene(nh.serviceClient<GetPlanningScene>("/get_planning_scene")),
    tf_listener_(tf_listener)
{
  ROS_INFO("Frame de referencia: %s", group_.getPlanningFrame().c_str());
  ROS_INFO("End Effector link: %s", group_.getEndEffectorLink().c_str());
}


void detection::GraspFilter::configure(float side_grasp_height, float top_graps_height,
                                       BoundingBoxPtr &obj_bounding_box, BoundingBoxPtr &table_bounding_box)
{
  OBJ_FRAME = obj_bounding_box->OBJ_FRAME;

  Point table_position = table_bounding_box->computePosition2DRobotFrame(tf_listener_);
  Eigen::Vector3f table_size = table_bounding_box->getSizeWithExternHeight(table_position.z);

  // Add the table to supress collisions of the object with it
  addSupportTable(table_position, table_size);

  // Add a box to supress collisions with the object
  Eigen::Vector3f obj_size = obj_bounding_box->getSize3D();
  //addCollisionObject(obj_size, OBJ_FRAME);

  obj_pose_ = obj_bounding_box->computePose3DRobotFrame(tf_listener_);
  side_grasp_center_ = Eigen::Vector3d(0, 0, side_grasp_height);
  top_grasp_center_ = Eigen::Vector3d(0, 0, top_graps_height);

  //updateAllowedCollisionMatrix();
}

void detection::GraspFilter::addSupportTable(Point &pose_pt, Eigen::Vector3f &size)
{
  CollisionObject co = getCollisionObjUpdated(collision_obj_pub, SUPPORT_TABLE, false, attached_obj_pub,
                                              FOOTPRINT_FRAME);

  /* Define a plane as a box to add to the world. */
  co.primitives.push_back(constructPrimitive(size[0] * 0.9f, size[1] * 0.9f, size[2]));
  // A pose for the plane (specified relative to frame_id).
  // We need to divide z by two because we want the table to ocuppy all the space below the real table
  // 0.2796  is the border of the robot
  Eigen::Vector3f pose(fmax(pose_pt.x, 0.2796 + size[0] / 2), pose_pt.y, pose_pt.z * 0.5);
  co.primitive_poses.push_back(newPose(pose));

  collision_obj_pub.publish(co);
}

void detection::GraspFilter::addCollisionObject(Eigen::Vector3f &size, const std::string &frame)
{
  CollisionObject co = getCollisionObjUpdated(collision_obj_pub, GRASPABLE_OBJECT, true, attached_obj_pub,
                                              frame);
  /* Define a box to add to the world. */
  co.primitives.push_back(constructPrimitive(size[0], size[1], size[2]));
  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  co.primitive_poses.push_back(pose);
  collision_obj_pub.publish(co);
  ros::WallDuration(1.0).sleep();
}

/*
void detection::GraspFilter::updateAllowedCollisionMatrix()
{
  GetPlanningScene scene_srv;
  scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

  if (!client_get_scene.call(scene_srv))
  {
    ROS_WARN("Failed to call service /get_planning_scene");
    return;
  }

  AllowedCollisionMatrix currentACM = getCollisionMatrix(scene_srv, false);
  currentACM.entry_names.push_back(GRASPABLE_OBJECT);
  AllowedCollisionEntry entry;
  entry.enabled.resize(currentACM.entry_names.size());

  // Add new row to allowed collision matrix
  //TODO: Make objects allowed to collision only with gripper
  for (int i = 0; i < entry.enabled.size(); i++)
    entry.enabled[i] = true;

  currentACM.entry_values.push_back(entry);

  // Extend the last column of the matrix
  for (int i = 0; i < currentACM.entry_values.size(); i++)
    currentACM.entry_values[i].enabled.push_back(true);

  PlanningScene newSceneDiff;
  newSceneDiff.is_diff = true;
  newSceneDiff.allowed_collision_matrix = currentACM;
  planning_scene_diff_pub.publish(newSceneDiff);

  // To check that the matrix was properly updated
  if (!client_get_scene.call(scene_srv))
    ROS_WARN("Failed to call service /get_planning_scene");
  else
    getCollisionMatrix(scene_srv, true);
}

AllowedCollisionMatrix detection::GraspFilter::getCollisionMatrix(GetPlanningScene scene_srv,
                                                                  bool after)
{
  ROS_INFO("Modified scene!");
  AllowedCollisionMatrix currentACM = scene_srv.response.scene.allowed_collision_matrix;
  std::string case_ = after ? "after " : "before ";
  ROS_WARN("Size of acm_entry %s {names:%lu, values:%lu, entry:%lu}", case_.c_str(), currentACM.entry_names.size(),
           currentACM.entry_values.size(), currentACM.entry_values[0].enabled.size());
  return currentACM;
}
*/

void detection::GraspFilter::filterGraspingPoses(CloudPtr side_grasps, CloudPtr top_grasps)
{
  side_grasps_ = side_grasps;
  top_grasps_ = top_grasps;

  /*ROS_INFO("Filtering %lu side grasps", side_grasps_->size());
  feasible_side_grasps_.reset(new Cloud);
  BOOST_FOREACH(Point &point, side_grasps_->points)
        {
          if (processSample(point, true))
            feasible_side_grasps_->push_back(point);
        }*/

  ROS_INFO("Filtering %lu top grasps", top_grasps_->size());
  feasible_top_grasps_.reset(new Cloud);
  BOOST_FOREACH(Point &point, top_grasps_->points)
  {
    if (processSample(point, false))
      feasible_top_grasps_->push_back(point);
  }
}

/*void detection::GraspFilter::visualizePlan(moveit::planning_interface::MoveGroup::Plan pose_plan)
{
  ROS_INFO("Visualizing plan again:");
  DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = pose_plan.start_state_;
  display_trajectory.trajectory.push_back(pose_plan.trajectory_);
  display_pub.publish(display_trajectory);
  sleep(5.0); // Para alcanzar a ver la trayectoria
}*/

bool detection::GraspFilter::processSample(Point &sample, bool generate_side_grasps)
{
  /*// Define a goal pose
  group_.setPoseTarget(newPose(sample));

  // Plan trayectory
  moveit::planning_interface::MoveGroup::Plan pose_plan;
  bool success = group_.plan(pose_plan);
  ROS_INFO("Visualizando plan 1 (Pose Goal) %s", success ? "" : "FAILED");
  // Sleep to give Rviz time to visualize the plan.
  //sleep(5.0);
  //if (success) visualizePlan(pose_plan);*/

  /*double x = obj_pose_.position.x;
  double y = obj_pose_.position.y;
  double z = obj_pose_.position.z;*/
  group_.setPlanningTime(10.0);
  //TODO: switch r_arm_controllers of call action client directly
  bool success;
  ROS_INFO("Trying to pick");
  // Needed to specify that attached object is allowed to touch table
  group_.setSupportSurfaceName(SUPPORT_TABLE);
  if (generate_side_grasps)
    success = group_.pick(GRASPABLE_OBJECT, generateSideGrasps(sample.x, sample.y, sample.z));
  else
    success = group_.pick(GRASPABLE_OBJECT, generateTopGrasps(sample.x, sample.y, sample.z));

  if (success)
    ROS_INFO("Pick was successful.");

  /* Actual movement

  ros::WallDuration(1.0).sleep();

  ROS_INFO("Placing object in muffin holder");
  success &= place(group);

//  // attach object to muffin holder link (TODO: update pose to current pose of object)
//  aco.link_name = "cup";
//  add_attached_collision_object();

  ROS_INFO("Moving arm to arm_far_away pose");
  group.setNamedTarget("arm_far_away");
  success &= group.move();

  if (success)
  {
    ROS_INFO("Done.");
    return EXIT_SUCCESS;
  }
  else
  {
    ROS_ERROR("One of the moves failed!");
    return EXIT_FAILURE;

  }
   */

  return success;
}

std::vector<moveit_msgs::Grasp> detection::GraspFilter::generateTopGrasps(double x, double y, double z)
{
  const double STRAIGHT_ANGLE_MIN = pcl::deg2rad(-45.0);
  const double ANGLE_MAX = pcl::deg2rad(45.0);
  const double ANGLE_INC = pcl::deg2rad(11.25);

  std::vector<moveit_msgs::Grasp> grasps;

  /* ----- Straight grasps
   * 2. straight grasp (xz-plane of `r_wrist_roll_link` contains z axis of `base_link`):
   *      (x_w, y_w) = position of `r_wrist_roll_link` in `base_link` frame)
   *   - standard: `rpy = (0, *, atan2(y_w, x_w))`
   *   - overhead: `rpy = (pi, *, atan2(y_w, x_w))` */
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, z));
  double yaw = atan2(y, x);
  for (double roll = 0.0; roll <= M_PI; roll += M_PI)
  {
    for (double pitch = STRAIGHT_ANGLE_MIN; pitch <= ANGLE_MAX; pitch += ANGLE_INC)
    {
      transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
      grasps.push_back(tfTransformToGrasp(transform * STANDOFF_TRANS));
    }
  }

  publishGraspsAsMarkerarray(grasps);
  return grasps;
}

/**
 * x, y, z: center of grasp point (the point that should be between the finger tips of the gripper)
 */
std::vector<moveit_msgs::Grasp> detection::GraspFilter::generateSideGrasps(double x, double y, double z)
{
  // How far from the grasp center should the wrist be. X is pointing toward the grasping point
  tf::Transform standoff_trans;
  standoff_trans.setOrigin(tf::Vector3(x, y, 0).normalize() * STANDOFF);
  Eigen::Quaterniond eigen_q;
  // Get the rotation from the standard direction (no rotation), to the oposite grasp point
  eigen_q.setFromTwoVectors(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-x, -y, 0.0));
  standoff_trans.setRotation(tf::Quaternion(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w()).normalize());

  /* Computation: xy-planes of 'r_wrist_roll_link' and 'base_link' are parallel */
  tf::Transform transform(tf::createIdentityQuaternion(), tf::Vector3(x, y, z));
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.push_back(tfTransformToGrasp(transform * standoff_trans));

  publishGraspsAsMarkerarray(grasps);
  return grasps;
}


moveit_msgs::Grasp detection::GraspFilter::tfTransformToGrasp(tf::Transform t)
{
  // Unique identifier for the grasp
  static int i = 0;
  moveit_msgs::Grasp grasp;
  grasp.id = boost::lexical_cast<std::string>(i++);

  tf::Vector3 &origin = t.getOrigin();
  tf::Quaternion rotation = t.getRotation().normalize();

  // The pose of the grasping point
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = OBJ_FRAME;
  pose.header.stamp = ros::Time(0);
  pose.pose.position.x = origin.m_floats[0];
  pose.pose.position.y = origin.m_floats[1];
  pose.pose.position.z = origin.m_floats[2];
  tf::quaternionTFToMsg(rotation, pose.pose.orientation);
  grasp.grasp_pose = pose;

  // Approach in x direction towards the object center
  grasp.pre_grasp_approach.direction.vector.x = 1.0;
  grasp.pre_grasp_approach.direction.header.frame_id = WRIST_LINK;
  grasp.pre_grasp_approach.min_distance = 0.05;
  grasp.pre_grasp_approach.desired_distance = PREGRASP_DISTANCE;
  grasp.pre_grasp_approach.direction.header.stamp = ros::Time(0);

  // Walk away in z direction (footprint frame)
  grasp.post_grasp_retreat.direction.vector.z = 1.0;
  grasp.post_grasp_retreat.min_distance = 0.05;
  grasp.post_grasp_retreat.desired_distance = PREGRASP_DISTANCE;
  grasp.post_grasp_retreat.direction.header.frame_id = FOOTPRINT_FRAME;
  grasp.post_grasp_retreat.direction.header.stamp = ros::Time(0);

  // Walk away in z direction (footprint frame)
  grasp.post_place_retreat = grasp.post_grasp_retreat;

  // Start with open gripper
  grasp.pre_grasp_posture.joint_names.push_back(GRIPPER_JOINT);
  grasp.pre_grasp_posture.points.resize(1);
  grasp.pre_grasp_posture.points[0].positions.push_back(0.08); // Hard limit 0.086

  // Close the gripper in the grasp
  grasp.grasp_posture = grasp.pre_grasp_posture;
  grasp.grasp_posture.points[0].positions[0] = 0.0;

  // Add the collision object
  grasp.allowed_touch_objects.push_back(GRASPABLE_OBJECT);
  return grasp;
}

void detection::GraspFilter::publishGraspsAsMarkerarray(std::vector<moveit_msgs::Grasp> grasps)
{
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.stamp = ros::Time(0);
  marker.header.frame_id = OBJ_FRAME;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.ns = "graspmarker";
  marker.scale.x = 0.025;
  marker.scale.y = 0.0015;
  marker.scale.z = 0.003;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  int i = 0;
  BOOST_FOREACH(moveit_msgs::Grasp &grasp, grasps)
        {
          marker.id = i++;
          marker.pose = grasp.grasp_pose.pose;
          markers.markers.push_back(marker);
        }

  grasps_marker_.publish(markers);
}

CloudPtr detection::GraspFilter::getSideGrasps()
{
  return feasible_side_grasps_;
}

CloudPtr detection::GraspFilter::getTopGrasps()
{
  return feasible_top_grasps_;
}
} // namespace bachelors_final_project
