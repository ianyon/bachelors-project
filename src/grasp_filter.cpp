//
// Created by ian on 10/6/15.
//

#include "grasp_filter.h"

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>

#include <pcl/common/angles.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PickupAction.h>

#include <moveit/move_group_pick_place_capability/capability_names.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <grasp_point_detector.h>

#include "utils.h"
#include "bounding_box.h"
#include "grasp_filter_utils.h"
#include "detection_utils.h"

using std::string;
using namespace moveit_msgs;
using shape_msgs::SolidPrimitive;
using shape_msgs::Plane;

namespace bachelors_final_project
{

const float detection::GraspFilter::PREGRASP_DISTANCE = 0.1;

detection::GraspFilter::GraspFilter(ros::NodeHandle &nh, tf::TransformListener &tf_listener,
                                    moveit::planning_interface::MoveGroup &r_arm_group,
                                    PickupActionPtr &pick_action_client, const string &planner_name) :
    nh_(nh),
    r_arm_group_(r_arm_group),
    gripper_group_("right_gripper"),
    collision_obj_pub(nh.advertise<CollisionObject>("/collision_object", 10)),
    attached_obj_pub(nh.advertise<AttachedCollisionObject>("/attached_collision_object", 10)),
    pose_pub_(nh.advertise<geometry_msgs::PoseArray>("sampled_poses", 10)),
    actual_pose_pub_(nh.advertise<geometry_msgs::PoseStamped>("actual_sampled_pose", 10)),
    tf_listener_(tf_listener),
    pick_action_client_(pick_action_client),
    PLANNER_NAME_(planner_name)
{
  r_arm_group_.setPlanningTime(10.0);
  r_arm_group_.setPlannerId(PLANNER_NAME_);
  ROS_DEBUG_NAMED(DETECTION(), "Frame de referencia: %s", r_arm_group_.getPlanningFrame().c_str());
  ROS_DEBUG_NAMED(DETECTION(), "End Effector link: %s", r_arm_group_.getEndEffectorLink().c_str());
  ROS_DEBUG_NAMED(DETECTION(), "End Effector: %s", r_arm_group_.getEndEffector().c_str());
}

void detection::GraspFilter::configure(float side_grasp_height, float top_graps_height,
                                       BoundingBoxPtr &obj_bounding_box, BoundingBoxPtr &table_bounding_box)
{
  OBJ_FRAME = obj_bounding_box->OBJ_FRAME;

  Point table_position = table_bounding_box->computePosition2DRobotFrame(tf_listener_);
  Vec3f table_size = table_bounding_box->getSizeWithExternHeight(table_position.z);

  // Add the table to supress collisions of the object with it
  addSupportTable(table_position, table_size);

  // Add a box to supress collisions with the object
  Vec3f obj_size = obj_bounding_box->getSize3D();
  addCollisionObject(obj_size, OBJ_FRAME);

  obj_pose_ = obj_bounding_box->computePose3DRobotFrame(tf_listener_);
  side_grasp_center_ = Vec3d(0, 0, side_grasp_height);
  top_grasp_center_ = Vec3d(0, 0, top_graps_height);

  feasible_side_grasps_.clear();
  feasible_top_grasps_.clear();
}

void detection::GraspFilter::addSupportTable(Point &pose_pt, Vec3f &size)
{
  CollisionObject co = getCollisionObjUpdated(collision_obj_pub, SUPPORT_TABLE(), false, attached_obj_pub,
                                              FOOTPRINT_FRAME);

  /* Define a plane as a box to add to the world. */
  co.primitives.push_back(constructPrimitive(size[0], size[1], size[2]));
  // A pose for the plane (specified relative to frame_id).
  // We need to divide z by two because we want the table to ocuppy all the space below the real table
  // 0.2796  is the border of the robot
  Vec3f pose(fmax(pose_pt.x, 0.2796 + size[0] / 2), pose_pt.y, pose_pt.z * 0.5);
  co.primitive_poses.push_back(newPose(pose));

  collision_obj_pub.publish(co);
}

void detection::GraspFilter::addCollisionObject(Vec3f &size, const std::string &frame)
{
  CollisionObject co = getCollisionObjUpdated(collision_obj_pub, GRASPABLE_OBJECT(), true, attached_obj_pub,
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

void detection::GraspFilter::filterGraspingPoses(CloudPtr side_grasps, CloudPtr top_grasps)
{
  side_grasps_ = side_grasps;
  top_grasps_ = top_grasps;

  if (side_grasps_->size() > 0)
  {
    ROS_DEBUG_NAMED(DETECTION(), "Filtering %lu side grasps", side_grasps_->size());
    BOOST_FOREACH(Point &point, side_grasps_->points)
          {
            Grasp grasp;
            if (processSample(point, true, grasp))
              feasible_side_grasps_.push_back(grasp);
          }
    ROS_DEBUG_NAMED(DETECTION(), "%lu feasible SIDE grasps", feasible_side_grasps_.size());
    ROS_INFO_COND_NAMED(feasible_side_grasps_.size() == 0, DETECTION(), "NO feasible SIDE grasps");
  }

  if (top_grasps->size() > 0)
  {
    ROS_DEBUG_NAMED(DETECTION(), "Filtering %lu top grasps", top_grasps_->size());
    BOOST_FOREACH(Point &point, top_grasps_->points)
          {
            Grasp grasp;
            if (processSample(point, false, grasp))
              feasible_top_grasps_.push_back(grasp);
          }
    ROS_DEBUG_NAMED(DETECTION(), "%lu feasible TOP grasps", feasible_top_grasps_.size());
    ROS_INFO_COND_NAMED(feasible_top_grasps_.size() == 0, DETECTION(), "NO feasible TOP grasps");
  }

  // Publish result
  std::vector<Grasp> feasible_ones = feasible_side_grasps_;
  feasible_ones.reserve(feasible_side_grasps_.size() + feasible_top_grasps_.size()); // preallocate memory
  feasible_ones.insert(feasible_ones.end(), feasible_side_grasps_.begin(), feasible_side_grasps_.end());
  feasible_ones.insert(feasible_ones.end(), feasible_top_grasps_.begin(), feasible_top_grasps_.end());
  ROS_ERROR_COND_NAMED(feasible_ones.size() == 0, DETECTION(), "NO FEASIBLE GRASPS");
  publishGrasps(feasible_ones);
}

bool detection::GraspFilter::processSample(Point &sample, bool generate_side_grasps, Grasp &grasp)
{

  if (ros::param::has("/bachelors_final_project/reconfigure"))
  {
    bool reconfigure;
    ros::param::get("/bachelors_final_project/reconfigure", reconfigure);
    if (reconfigure)
    {
      ros::param::set("/bachelors_final_project/reconfigure", false);
      throw ComputeFailedException("Asked to change current object. Stopping grasp filtering");
    }
  }

  bool success;
  ROS_DEBUG_NAMED(DETECTION(), "Trying to pick");
  // Needed to specify that attached object is allowed to touch table
  r_arm_group_.setSupportSurfaceName(SUPPORT_TABLE());

  if (generate_side_grasps)
    grasp = generateSideGrasps(sample.x, sample.y, sample.z);
  else
    grasp = generateTopGrasps(sample.x, sample.y, sample.z);

  success = pick(r_arm_group_, GRASPABLE_OBJECT(), grasp);


  ROS_DEBUG_COND_NAMED(success, DETECTION(), "Pick planning was successful.");

  return success;
}

bool detection::GraspFilter::pick(moveit::planning_interface::MoveGroup &group, const string &object,
                                  Grasp &grasp)
{
  if (!pick_action_client_)
  {
    ROS_ERROR_STREAM("Pick action client not found");
    return false;
  }
  if (!pick_action_client_->isServerConnected())
  {
    ROS_ERROR_STREAM("Pick action server not connected");
    return false;
  }

  PickupGoal goal;
  constructGoal(group, goal, object, PLANNER_NAME_, true);
  goal.possible_grasps.push_back(grasp);
  pick_action_client_->cancelAllGoals();
  pick_action_client_->sendGoal(goal);
  if (!pick_action_client_->waitForResult(ros::Duration(0.5)))
    ROS_DEBUG_STREAM("Pickup action returned early");

  bool plan_succeeded = pick_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  ROS_DEBUG_STREAM_COND(!plan_succeeded,
                        "Fail: " << pick_action_client_->getState().toString() << ": " <<
                        pick_action_client_->getState().getText());

/*  if (plan_succeeded && (selectChoice("Press 1 and enter to pick or 2 to continue...") == 1))
    pickMovement(pick_action_client_, goal);*/

  return plan_succeeded;
}

Grasp detection::GraspFilter::generateTopGrasps(double x, double y, double z)
{

  //TODO   // distance from center point of object to end effector
  //TODO grasp_depth_ = 0.06;// in negative or 0 this makes the grasps on the other side of the object! (like from below)


  // How far from the grasp center should the wrist be. X is pointing toward the grasping point
  tf::Transform standoff_trans;
  standoff_trans.setOrigin(tf::Vector3(x, y, z).normalize() * standoff);
  Eigen::Quaterniond eigen_q;

  // Special case for the short axis
  if (x == 0.0 && y != 0.0)
  {
    Eigen::Quaterniond eigen_aux;
    // Get the rotation from the standard direction (no rotation), to the oposite grasp point
    eigen_aux.setFromTwoVectors(Vec3d(1.0, 0.0, 0.0), Vec3d(0.0, 1.0, 0.0));
    eigen_q.setFromTwoVectors(Vec3d(0.0, 1.0, 0.0), Vec3d(-x, -y, -z));
    eigen_q *= eigen_aux;
  }
  else // Get the rotation from the standard direction (no rotation), to the oposite grasp point
    eigen_q.setFromTwoVectors(Vec3d(1.0, 0.0, 0.0), Vec3d(-x, -y, -z));

  standoff_trans.setRotation(tf::Quaternion(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w()));

  tf::Transform transform(tf::createIdentityQuaternion(), tf::Vector3(x, y, z));
  Grasp grasp = tfTransformToGrasp(transform * standoff_trans);

  actual_pose_pub_.publish(grasp.grasp_pose);
  return grasp;
}

/**
 * x, y, z: center of grasp point (the point that should be between the finger tips of the gripper)
 */
Grasp detection::GraspFilter::generateSideGrasps(double x, double y, double z)
{

  //TODO   // distance from center point of object to end effector
  //TODO grasp_depth_ = 0.06;// in negative or 0 this makes the grasps on the other side of the object! (like from below)


  // How far from the grasp center should the wrist be. X is pointing toward the grasping point
  tf::Transform standoff_trans;
  standoff_trans.setOrigin(tf::Vector3(x, y, 0).normalize() * standoff);
  Eigen::Quaterniond eigen_q;
  // Get the rotation from the standard direction (no rotation), to the oposite grasp point in the plane
  eigen_q.setFromTwoVectors(Vec3d(1.0, 0.0, 0.0), Vec3d(-x, -y, 0.0));
  standoff_trans.setRotation(tf::Quaternion(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w()).normalize());

  /* Computation: xy-planes of 'r_wrist_roll_link' and 'base_link' are parallel */
  tf::Transform transform(tf::createIdentityQuaternion(), tf::Vector3(x, y, z));
  Grasp grasp = tfTransformToGrasp(transform * standoff_trans);

  actual_pose_pub_.publish(grasp.grasp_pose);
  return grasp;
}


Grasp detection::GraspFilter::tfTransformToGrasp(tf::Transform t)
{
  // Unique identifier for the grasp
  static int i = 0;
  Grasp grasp;
  grasp.id = boost::lexical_cast<std::string>(i++);

  // The pose of the grasping point
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = OBJ_FRAME;
  pose.header.stamp = ros::Time(0);
  pose.pose.position.x = t.getOrigin().m_floats[0];
  pose.pose.position.y = t.getOrigin().m_floats[1];
  pose.pose.position.z = t.getOrigin().m_floats[2];
  tf::quaternionTFToMsg(t.getRotation().normalize(), pose.pose.orientation);
  grasp.grasp_pose = pose;

  // Approach in x direction towards the object center
  grasp.pre_grasp_approach.direction.header.frame_id = r_arm_group_.getEndEffectorLink().c_str();
  grasp.pre_grasp_approach.direction.header.stamp = ros::Time(0);
  grasp.pre_grasp_approach.direction.vector.x = 1.0;
  grasp.pre_grasp_approach.min_distance = 0.1;
  //grasp.pre_grasp_approach.min_distance = 0.05;
  grasp.pre_grasp_approach.desired_distance = 0.2;
  //grasp.pre_grasp_approach.desired_distance = PREGRASP_DISTANCE;

  // Walk away in z direction (footprint frame)
  grasp.post_grasp_retreat.direction.header.frame_id = FOOTPRINT_FRAME;
  grasp.post_grasp_retreat.direction.header.stamp = ros::Time(0);
  grasp.post_grasp_retreat.direction.vector.z = 1.0;
  grasp.post_grasp_retreat.min_distance = 0.1;
  //grasp.post_grasp_retreat.min_distance = 0.05;
  grasp.post_grasp_retreat.desired_distance = 0.25;
  //grasp.post_grasp_retreat.desired_distance = PREGRASP_DISTANCE;

  // Walk away in z direction (footprint frame)
  grasp.post_place_retreat = grasp.post_grasp_retreat;

  // Start with open gripper
  grasp.pre_grasp_posture = generateGraspPosture(1.0);
  // Close the gripper in the grasp
  grasp.grasp_posture = generateGraspPosture(0.0);

  // Add the collision object
  grasp.allowed_touch_objects.push_back(GRASPABLE_OBJECT());
  return grasp;
}

void detection::GraspFilter::publishGrasps(std::vector<Grasp> grasps)
{
  geometry_msgs::PoseArray pose_array;
  BOOST_FOREACH(Grasp &grasp, grasps)
        {
          pose_array.poses.push_back(grasp.grasp_pose.pose);
          pose_array.header.frame_id = OBJ_FRAME;
          pose_array.header.stamp = ros::Time(0);
        }
  pose_pub_.publish(pose_array);
}

bool detection::GraspFilter::generateSideSamples(BoundingBoxPtr &obj_bounding_box, float grasp_height)
{
  // grasp_height is in the lower half of the object. grasp_height > 0 if object is smaller than grasp height
  bool result = grasp_height < 0;
  float grasp_height_from_base = (float) (obj_bounding_box->getHeight() / 2) + grasp_height;
  // Or it must be 2 cm lower than obj's height
  result |= grasp_height_from_base  <= obj_bounding_box->getHeight() - 0.02;
  ROS_INFO_COND_NAMED(!result, DETECTION(), "No SIDE grasp generation. Unfeasible SIDE grasps in object ");
  return result;
}

bool detection::GraspFilter::generateTopSamples(BoundingBoxPtr &obj_bounding_box, float height,
                                                bool &generate_mayor_axis, bool &generate_minor_axis)
{
  // Only generate grasps along mayor axis if minor axis is thinner than gripper
  generate_mayor_axis = true || obj_bounding_box->getMinorAxisSize2D() <= 0.09;
  // Only generate grasps along mayor axis if minor axis is thinner than gripper
  generate_minor_axis = true || obj_bounding_box->getMayorAxisSize2D() <= 0.09;
  //Not useful
  bool result = generate_mayor_axis || generate_minor_axis;
  ROS_INFO_COND_NAMED(!result, DETECTION(), "No TOP grasp generation. Unfeasible TOP grasps in object");
  return result;
}

void detection::GraspFilter::moveAllGripperJoints(float v)
{
  robot_state::RobotStatePtr kinematic_state = gripper_group_.getCurrentState();
  const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(
      gripper_group_.getName());

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  BOOST_FOREACH(double &value, joint_values)
        {
          value = v;
        }

  gripper_group_.setJointValueTarget(joint_values);
  moveit::planning_interface::MoveGroup::Plan plan;
  plan.planning_time_ = 2.0;
  bool success = gripper_group_.plan(plan);

  if (success)
    gripper_group_.move();
  else
    ROS_ERROR_NAMED(DETECTION(), "Couldn't plan for gripper");

  ros::WallDuration(2.0).sleep();
}

void detection::GraspFilter::openGripper()
{
  moveAllGripperJoints(1.0);
}

void detection::GraspFilter::closeGripper()
{
  moveAllGripperJoints(0.0);
}

trajectory_msgs::JointTrajectory detection::GraspFilter::generateGraspPosture(float value)
{
  trajectory_msgs::JointTrajectory posture;

  robot_state::RobotStatePtr kinematic_state = gripper_group_.getCurrentState();
  const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(
      gripper_group_.getName());

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  const std::vector<std::string> &actuated_joint_names = gripper_group_.getActiveJoints();
  BOOST_FOREACH(const std::string &name, actuated_joint_names)
        {
          posture.joint_names.push_back(name);
        }
  //r_gripper_motor_screw_joint
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

  //posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
  posture.points.resize(1);
  posture.points[0].positions.resize(posture.joint_names.size(), value); // Hard limit 0.086?
  posture.points[0].time_from_start = ros::Duration(15.0);

  // Only valid in grasp posture
  posture.points[0].effort.resize(posture.joint_names.size(), -1.0);  // Close gently

  return posture;
}
} // namespace bachelors_final_project
