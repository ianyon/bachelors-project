//
// Created by ian on 10/6/15.
//

#include "grasp_filter.h"

#include <ros/ros.h>

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
const float detection::GraspFilter::PREGRASP_DISTANCE = 0.1;
const std::string detection::GraspFilter::LBKPIECE_PLANNER = "LBKPIECEkConfigDefault";
const std::string detection::GraspFilter::RRTCONNECT_PLANNER = "RRTConnectkConfigDefault";

detection::GraspFilter::GraspFilter(ros::NodeHandle &nh, tf::TransformListener &tf_listener) :
    nh_(nh),
    group_("right_arm"),
    display_pub(nh.advertise<DisplayTrajectory>("/move_group/display_planned_path", 1, true)),
    collision_obj_pub(nh.advertise<CollisionObject>("/collision_object", 10)),
    attached_obj_pub(nh.advertise<AttachedCollisionObject>("/attached_collision_object", 10)),
    grasps_marker_(nh.advertise<visualization_msgs::MarkerArray>("grasps_marker", 10)),
    pose_pub_(nh.advertise<geometry_msgs::PoseStamped>("sampled_poses", 10)),
    planning_scene_diff_pub(nh.advertise<PlanningScene>("/planning_scene", 1)),
    client_get_scene(nh.serviceClient<GetPlanningScene>("/get_planning_scene")),
    tf_listener_(tf_listener),
    pick_action_client_(new PickupAction(move_group::PICKUP_ACTION, true)),
    PLANNER_NAME_(RRTCONNECT_PLANNER),
    feasible_top_grasps_(new Cloud),
    feasible_side_grasps_(new Cloud)
{
  group_.setPlanningTime(10.0);
  group_.setPlannerId(PLANNER_NAME_);
  ROS_INFO("Frame de referencia: %s", group_.getPlanningFrame().c_str());
  ROS_INFO("End Effector link: %s", group_.getEndEffectorLink().c_str());

  waitForAction(pick_action_client_, ros::Duration(5.0), move_group::PICKUP_ACTION);
}

void detection::GraspFilter::waitForAction(const PickupActionPtr &action, const ros::Duration &wait_for_server,
                                           const std::string &name)
{
  ROS_DEBUG("Waiting for MoveGroup action server (%s)...", name.c_str());

  // in case ROS time is published, wait for the time data to arrive
  ros::Time start_time = ros::Time::now();
  while (start_time == ros::Time::now())
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  // wait for the server (and spin as needed)
  if (wait_for_server == ros::Duration(0, 0))
  {
    while (nh_.ok() && !action->isServerConnected())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }
  else
  {
    ros::Time final_time = ros::Time::now() + wait_for_server;
    while (nh_.ok() && !action->isServerConnected() && final_time > ros::Time::now())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }

  if (!action->isServerConnected())
    throw std::runtime_error("Unable to connect to action server within allotted time");
  else
    ROS_DEBUG("Connected to '%s'", name.c_str());
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
  addCollisionObject(obj_size, OBJ_FRAME);

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

  if (side_grasps_->size() > 0)
  {
    ROS_INFO("Filtering %lu side grasps", side_grasps_->size());
    feasible_side_grasps_->clear();
    BOOST_FOREACH(Point &point, side_grasps_->points)
          {
            if (processSample(point, true))
              feasible_side_grasps_->push_back(point);
          }
    ROS_INFO("%lu feasible SIDE grasps", feasible_side_grasps_->size());
  }

  if (top_grasps->size() > 0)
  {
    ROS_INFO("Filtering %lu top grasps", top_grasps_->size());
    feasible_top_grasps_->clear();
    BOOST_FOREACH(Point &point, top_grasps_->points)
          {
            if (processSample(point, false))
              feasible_top_grasps_->push_back(point);
          }
    ROS_INFO("%lu feasible TOP grasps", feasible_top_grasps_->size());
  }
}

bool detection::GraspFilter::processSample(Point &sample, bool generate_side_grasps)
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
  ROS_DEBUG("Trying to pick");
  // Needed to specify that attached object is allowed to touch table
  group_.setSupportSurfaceName(SUPPORT_TABLE);
  if (generate_side_grasps)
    success = pick(group_, GRASPABLE_OBJECT, generateSideGrasps(sample.x, sample.y, sample.z));
  else
    success = pick(group_, GRASPABLE_OBJECT, generateTopGrasps(sample.x, sample.y, sample.z));

  ROS_DEBUG_COND(success, "\n\n\n\nPick planning was successful.\n\n\n\n");

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

bool detection::GraspFilter::pick(moveit::planning_interface::MoveGroup &group, const string &object,
                                  const std::vector<Grasp> &grasps)
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
  constructGoal(group, goal, object);
  goal.possible_grasps = grasps;
  goal.planning_options.plan_only = true;
  //pick_action_client_->cancelGoal();
  pick_action_client_->cancelAllGoals();
  pick_action_client_->sendGoal(goal);
  if (!pick_action_client_->waitForResult(ros::Duration(2.0)))
    ROS_WARN_STREAM("Pickup action returned early");

  bool plan_succeeded = pick_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  ROS_WARN_STREAM_COND(!plan_succeeded,
                       "Fail: " << pick_action_client_->getState().toString() << ": " <<
                       pick_action_client_->getState().getText());

  if (plan_succeeded)
  {
    ROS_INFO("Press 1 and enter to pick or 2 to continue...");
    int a;
    std::cin >> a;
    if (a != 1)
      return plan_succeeded;
    ROS_INFO("TRYING REAL PICK ACTION");
    goal.planning_options.plan_only = false;
    //pick_action_client_->cancelGoal();
    //pick_action_client_->cancelAllGoals();
    pick_action_client_->sendGoal(goal);
    if (!pick_action_client_->waitForResult(ros::Duration(2.0)))
      ROS_WARN_STREAM("Pickup action returned early");

    bool move_succeeded = pick_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    ROS_WARN_STREAM_COND(!move_succeeded,
                         "Fail: " << pick_action_client_->getState().toString() << ": " <<
                         pick_action_client_->getState().getText());


    return move_succeeded;
  }

  return plan_succeeded;
}

void detection::GraspFilter::constructGoal(moveit::planning_interface::MoveGroup &group, PickupGoal &goal_out,
                                           const std::string &object)
{
  PickupGoal goal;
  goal.target_name = object;
  goal.group_name = group.getName();
  goal.end_effector = group.getEndEffector();
  goal.allowed_planning_time = group.getPlanningTime();
  goal.support_surface_name = SUPPORT_TABLE;
  goal.planner_id = PLANNER_NAME_;

  if (group.getPathConstraints().name != std::string())
    goal.path_constraints = group.getPathConstraints();

  goal_out = goal;
}

std::vector<Grasp> detection::GraspFilter::generateTopGrasps(double x, double y, double z)
{
  std::vector<Grasp> grasps;

  // How far from the grasp center should the wrist be. X is pointing toward the grasping point
  tf::Transform standoff_trans;
  standoff_trans.setOrigin(tf::Vector3(x, y, z).normalize() * standoff);
  Eigen::Quaterniond eigen_q;

  // Special case for the short axis
  if (x == 0.0 && y != 0.0)
  {
    Eigen::Quaterniond eigen_aux;
    // Get the rotation from the standard direction (no rotation), to the oposite grasp point
    eigen_aux.setFromTwoVectors(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0));
    eigen_q.setFromTwoVectors(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(-x, -y, -z));
    eigen_q *= eigen_aux;
  }
  else
  {
    // Get the rotation from the standard direction (no rotation), to the oposite grasp point
    eigen_q.setFromTwoVectors(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-x, -y, -z));
  }

  standoff_trans.setRotation(tf::Quaternion(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w()));

  tf::Transform transform(tf::createIdentityQuaternion(), tf::Vector3(x, y, z));
  grasps.push_back(tfTransformToGrasp(transform * standoff_trans));

  publishGraspsAsMarkerarray(grasps);
  return grasps;
}

/**
 * x, y, z: center of grasp point (the point that should be between the finger tips of the gripper)
 */
std::vector<Grasp> detection::GraspFilter::generateSideGrasps(double x, double y, double z)
{
  // How far from the grasp center should the wrist be. X is pointing toward the grasping point
  tf::Transform standoff_trans;
  standoff_trans.setOrigin(tf::Vector3(x, y, 0).normalize() * standoff);
  Eigen::Quaterniond eigen_q;
  // Get the rotation from the standard direction (no rotation), to the oposite grasp point in the plane
  eigen_q.setFromTwoVectors(Eigen::Vector3d(1.0, 0.0, 0.0),
                            Eigen::Vector3d(-x, -y, 0.0));
  //-(top_grasp_center_[2] + side_grasp_center_[2])));
  standoff_trans.setRotation(tf::Quaternion(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w()).normalize());

  /* Computation: xy-planes of 'r_wrist_roll_link' and 'base_link' are parallel */
  tf::Transform transform(tf::createIdentityQuaternion(), tf::Vector3(x, y, z));
  std::vector<Grasp> grasps;
  grasps.push_back(tfTransformToGrasp(transform * standoff_trans));

  publishGraspsAsMarkerarray(grasps);
  return grasps;
}


Grasp detection::GraspFilter::tfTransformToGrasp(tf::Transform t)
{
  // Unique identifier for the grasp
  static int i = 0;
  Grasp grasp;
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
  grasp.pre_grasp_posture.points[0].positions.push_back(1.0); // Hard limit 0.086?

  // Close the gripper in the grasp
  grasp.grasp_posture = grasp.pre_grasp_posture;
  grasp.grasp_posture.points[0].positions[0] = 0.0;

  // Add the collision object
  grasp.allowed_touch_objects.push_back(GRASPABLE_OBJECT);
  return grasp;
}

void detection::GraspFilter::publishGraspsAsMarkerarray(std::vector<Grasp> grasps)
{
  BOOST_FOREACH(Grasp &grasp, grasps)
        {
          pose_pub_.publish(grasp.grasp_pose);
        }
}

CloudPtr detection::GraspFilter::getSideGrasps()
{
  return feasible_side_grasps_;
}

CloudPtr detection::GraspFilter::getTopGrasps()
{
  return feasible_top_grasps_;
}

void detection::GraspFilter::setParams(double standoff_)
{
  standoff = standoff_;
}

bool detection::GraspFilter::generateSideGrasps(BoundingBoxPtr &obj_bounding_box, float grasp_height)
{
  return grasp_height / 2 <= obj_bounding_box->getHeight();
}

bool detection::GraspFilter::generateTopGrasps(BoundingBoxPtr &obj_bounding_box, float height)
{
  // Only generate grasps along mayor axis if minor axis is thinner than gripper
  generate_top_grasps_mayor_axis_ = obj_bounding_box->getMinorAxisSize2D() <= 0.09;
  // Only generate grasps along mayor axis if minor axis is thinner than gripper
  generate_top_grasps_minor_axis_ = obj_bounding_box->getMayorAxisSize2D() <= 0.09;
  return generate_top_grasps_mayor_axis_ || generate_top_grasps_minor_axis_;
}

bool detection::GraspFilter::generateTopGraspsMinorAxis()
{
  return generate_top_grasps_minor_axis_;
}

bool detection::GraspFilter::generateTopGraspsMayorAxis()
{
  return generate_top_grasps_mayor_axis_;
}
} // namespace bachelors_final_project
